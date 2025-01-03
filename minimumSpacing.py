import networkx as nx
import schemdraw
import schemdraw.elements as elm
import xml.etree.ElementTree as ET
import numpy as np
from shapely.geometry import LineString, Polygon, Point
import xml.dom.minidom as minidom
import math
import copy  # Added copy module
import sys

# Sample Netlist (Ensure this is your actual netlist)
netlist = """

*Q1 in in 0 N90 W=1u L=90n
*Q2 out in 0 N90 W=1u L=90n
M1 d1 0 out vdd P90 W=1u L=90n
J3 in in d1 N90 W=1u L=90n
Q4 out in d2 N90 W=1u L=90n

Iin vdd in 10u

Vdd vdd D0 C 1.8
Vin out 0 DC 1.8

.dc Iin 0 10u 0.1u

.options savecurrents

.end

"""

# Configuration Parameters
EnlargeSize = 15
max_attempts = 300
routing_method = 1  # Only Manhattan routing is used
auto = 1  # Set to 0 for fixed positions, 1 for automatic layout
shrink_size = 0.05
wire_safe_color = 'green'
wire_danger_color = 'red'
grid_size = 0.1  # Define the grid size
draw_grid_or_not = 0  # Set to 1 to draw grid, 0 to not draw grid
min_wire_spacing = 0.2  # Minimum spacing between parallel wires

# Default Directions and Flips
default_directions = {
    'GND': 'right',  # In this library, the default direction of ground is right
    'R': 'right',
    'C': 'right',
    'L': 'right',
    'D': 'right',
    'V': 'up',
    'I': 'up',
    'S': 'right',
    'J': 'right',
    'Q': 'up',
    'M': 'right',  # Added for MOSFETs
    'E': 'up',
    'H': 'up',
    'F': 'up',
    'G': 'up',
    'node': 'right',  # For node, you can set the default value arbitrarily
    'ground': 'right'  # ground is the same as GND
}

default_flips = {
    'GND': 'none',
    'R': 'none',
    'C': 'none',
    'L': 'none',
    'D': 'none',
    'V': 'none',
    'I': 'none',
    'S': 'none',
    'J': 'none',
    'Q': 'none',
    'M': 'none',  # Added for MOSFETs
    'E': 'none',
    'H': 'none',
    'F': 'none',
    'G': 'none',
    'node': 'none',
    'ground': 'none'
}

# Default Scaling Ratios
default_scaling_ratios = {
    'GND': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'R': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'C': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'L': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'D': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'V': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'I': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'S': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'J': {'vertical_scale': 6/5, 'horizontal_scale': 12/11},       #BBox: xmin=-0.6966666666666667, ymin=-0.6966666666666667, xmax=0.7516666666666666, ymax=0.6966666666666667
    'Q': {'vertical_scale': 300/209, 'horizontal_scale': 600/451}, #BBox: xmin=-0.6966666666666667, ymin=-0.6966666666666667, xmax=0.7516666666666666, ymax=0.6966666666666667
    'M': {'vertical_scale': 6/5, 'horizontal_scale': 90/82},       #BBox: xmin=-1.5, ymin=-1.5, xmax=1.3666666666666667, ymax=0 work for 0.1 if vertical is 0.8
    'E': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'H': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'F': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'G': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'node': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'ground': {'vertical_scale': 1.0, 'horizontal_scale': 1.0}
}

# Component Mapping
component_map = {
    'GND': elm.Ground,
    'R': elm.Resistor,
    'C': elm.Capacitor,
    'L': elm.Inductor2,
    'D': elm.Diode,
    'V': elm.SourceV,
    'I': elm.SourceI,
    'S': elm.Switch,
    'J': elm.JFet,
    'Q': elm.Bjt,
    'M': None,  # Will handle separately based on model
    'E': elm.SourceControlledV,
    'H': elm.SourceControlledV,
    'F': elm.SourceControlledI,
    'G': elm.SourceControlledI,
    'ground': elm.Ground,
    'node': elm.Dot(radius=0.12)
}

def parse_spice_netlist(netlist):
    print("[DEBUG] Starting to parse netlist...")
    components = []
    commands = []
    subcircuits = []
    for line_number, line in enumerate(netlist.strip().split('\n'), start=1):
        original_line = line
        # Remove comments
        if ';' in line:
            line = line.split(';')[0].strip()
        line = line.strip()
        if line.startswith('*'):
            print(f"[DEBUG] Skipping comment line {line_number}: {original_line}")
            continue  # Skip comment lines
        if not line:
            print(f"[DEBUG] Skipping empty line {line_number}")
            continue  # Skip empty lines

        try:
            parts = line.split()
            if not parts:
                print(f"[DEBUG] Line {line_number} became empty after splitting.")
                continue  # Skip if line became empty after splitting

            first_char = parts[0][0].upper()

            if first_char in ['V', 'I', 'R', 'C', 'L', 'D', 'M', 'Q', 'J', 'K', 'S', 'T', 'E', 'F', 'G', 'H']:
                if first_char in ['V', 'I', 'S', 'E', 'F', 'G', 'H']:
                    if len(parts) < 4:
                        raise ValueError(f"Invalid voltage/current source definition: {line}")
                    value = ' '.join(parts[3:])
                    nodes = parts[1:3]
                elif first_char == 'D':
                    if len(parts) < 4:
                        raise ValueError(f"Invalid diode definition: {line}")
                    value = parts[3]
                    nodes = parts[1:3]
                elif first_char == 'J':
                    if len(parts) < 5:
                        raise ValueError(f"Invalid JFET definition: {line}")
                    value = parts[4] if len(parts) > 4 else ''
                    nodes = parts[1:4]
                    components.append({
                        'type': first_char,
                        'id': parts[0],
                        'nodes': nodes,
                        'value': value,
                        'pins': ['drain', 'gate', 'source'],
                        'scale': 1.0  # [MODIFIED] Add default scale
                    })
                    print(f"[DEBUG] Added JFET component: {parts[0]}")
                    continue
                elif first_char == 'Q':
                    if len(parts) < 5:
                        raise ValueError(f"Invalid BJT definition: {line}")
                    value = parts[4] if len(parts) > 4 else ''
                    nodes = parts[1:4]
                    components.append({
                        'type': first_char,
                        'id': parts[0],
                        'nodes': nodes,
                        'value': value,
                        'pins': ['collector', 'base', 'emitter'],
                        'scale': 1.0  # [MODIFIED] Add default scale
                    })
                    print(f"[DEBUG] Added BJT component: {parts[0]}")
                    continue
                elif first_char == 'M':
                    if len(parts) < 6:
                        raise ValueError(f"Invalid MOSFET definition: {line}")
                    id = parts[0]
                    nodes = parts[1:5]  # drain, gate, source, bulk
                    model = parts[5]
                    # Optional parameters
                    optional_params = parts[6:]
                    params_dict = {}
                    for param in optional_params:
                        if '=' in param:
                            key, val = param.split('=', 1)
                            params_dict[key.upper()] = val
                    components.append({
                        'type': first_char,
                        'id': id,
                        'nodes': nodes,
                        'model': model,
                        'parameters': params_dict,
                        'pins': ['drain', 'gate', 'source', 'bulk'],
                        'scale': 1.0  # [MODIFIED] Add default scale
                    })
                    print(f"[DEBUG] Added MOSFET component: {id}")
                    continue
                else:
                    if len(parts) < 4:
                        raise ValueError(f"Invalid component definition: {line}")
                    value = ' '.join(parts[3:]) if len(parts) > 3 else ''
                    nodes = parts[1:3]

                components.append({
                    'type': first_char,
                    'id': parts[0],
                    'nodes': nodes,
                    'value': value,
                    'scale': 1.0  # [MODIFIED] Add default scale
                })
                print(f"[DEBUG] Added component: {parts[0]} of type {first_char}")

            elif first_char == 'X':
                if len(parts) < 4:
                    raise ValueError(f"Invalid subcircuit definition: {line}")
                subcircuits.append({
                    'type': 'X',
                    'id': parts[0],
                    'nodes': parts[1:-1],
                    'subckt': parts[-1]
                })
                print(f"[DEBUG] Added subcircuit: {parts[0]}")
            elif line.startswith('.'):
                commands.append({'command': parts[0][1:], 'params': parts[1:]})
                print(f"[DEBUG] Added command: {parts[0]}")
            else:
                raise ValueError(f"Unknown element or command: {line}")
        except (IndexError, ValueError) as e:
            print(f"[ERROR] Parsing line {line_number}: {original_line}")
            print(f"        {e}")
            continue

    print("[DEBUG] Finished parsing netlist.")
    return components, commands, subcircuits


def netlist_to_xml(components, commands, subcircuits):
    print("[DEBUG] Converting netlist to XML...")
    root = ET.Element("spice_netlist")

    components_elem = ET.SubElement(root, "components")
    for comp in components:
        comp_type = comp['type']
        cid = comp['id']
        comp_elem = ET.SubElement(components_elem, "component", type=comp_type, id=cid)
        # Add nodes
        for i, node in enumerate(comp['nodes'], start=1):
            ET.SubElement(comp_elem, f"node{i}").text = node
        # Add value or model
        if 'model' in comp:
            ET.SubElement(comp_elem, "model").text = comp['model']
        if 'value' in comp:
            ET.SubElement(comp_elem, "value").text = comp['value']
        # Add optional parameters for MOSFETs
        if 'parameters' in comp:
            params_elem = ET.SubElement(comp_elem, "parameters")
            for key, val in comp['parameters'].items():
                ET.SubElement(params_elem, key).text = val
        # Add pins if available
        if 'pins' in comp:
            ET.SubElement(comp_elem, "pins").text = ' '.join(comp['pins'])
        # Set direction and flip
        comp_direction = default_directions.get(comp_type, 'right')
        comp_elem.set("direction", comp_direction)
        comp_flip = default_flips.get(comp_type, 'none')
        comp_elem.set("flip", comp_flip)  # Default no flip
        # [MODIFIED] Add vertical_scale and horizontal_scale from default_scaling_ratios
        scaling = default_scaling_ratios.get(comp_type, {'vertical_scale': 1.0, 'horizontal_scale': 1.0})
        comp_elem.set("vertical_scale", str(scaling.get('vertical_scale', 1.0)))
        comp_elem.set("horizontal_scale", str(scaling.get('horizontal_scale', 1.0)))

        print(f"[DEBUG] Converted component {cid} to XML.")

    subcircuits_elem = ET.SubElement(root, "subcircuits")
    for subckt in subcircuits:
        subckt_elem = ET.SubElement(subcircuits_elem, "subcircuit", id=subckt['id'], subckt=subckt['subckt'])
        for i, node in enumerate(subckt['nodes'], start=1):
            ET.SubElement(subckt_elem, f"node{i}").text = node
        print(f"[DEBUG] Converted subcircuit {subckt['id']} to XML.")

    commands_elem = ET.SubElement(root, "commands")
    for cmd in commands:
        cmd_elem = ET.SubElement(commands_elem, "command", name=cmd['command'])
        cmd_elem.text = ' '.join(cmd['params'])
        print(f"[DEBUG] Converted command {cmd['command']} to XML.")

    print("[DEBUG] Finished converting netlist to XML.")
    return root


def pretty_print_xml(element):
    print("[DEBUG] Pretty printing XML...")
    rough_string = ET.tostring(element, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")


def convert_netlist_to_xml_file(netlist, output_filename="spice_netlist.xml"):
    print("[DEBUG] Starting conversion of netlist to XML file...")
    components, commands, subcircuits = parse_spice_netlist(netlist)
    xml_root = netlist_to_xml(components, commands, subcircuits)
    pretty_xml = pretty_print_xml(xml_root)
    with open(output_filename, "w") as f:
        f.write(pretty_xml)
    print(f"[DEBUG] Netlist converted and saved to {output_filename}.")


def create_graph_from_xml(xml_root):
    print("[DEBUG] Creating graph from XML...")
    components_element = xml_root.find('components')
    if components_element is None or len(components_element) == 0:
        print("[ERROR] No components found in XML.")
        raise ValueError("No components found in XML.")

    G = nx.MultiGraph()

    for component in components_element:
        ctype = component.attrib['type']
        cid = component.attrib['id']

        value_elem = component.find('value')
        model_elem = component.find('model')

        # Separate 'value' and 'model'
        if 'model' in component.attrib:
            model = component.attrib['model']
        else:
            model = ''

        if model_elem is not None and model_elem.text:
            model = model_elem.text.upper()

        if value_elem is not None and value_elem.text:
            value = value_elem.text
        else:
            value = ''

        # Optionally, handle parameters if needed
        parameters = {}
        params_elem = component.find('parameters')
        if params_elem is not None:
            for param in params_elem:
                parameters[param.tag] = param.text

        nodes = []
        i = 1
        while component.find(f'node{i}') is not None:
            node = component.find(f'node{i}').text
            if node:  # Ensure node is not None or empty
                nodes.append(node)
            i += 1

        if not nodes:
            print(f"[WARNING] Component {cid} of type {ctype} has no nodes defined.")
            continue

        # Add component as a node in the graph with 'model' if available
        G.add_node(cid, type=ctype, value=value, model=model, parameters=parameters)

        # Connect component to its nodes
        for node in nodes:
            if node == '0':
                G.add_node(node, type='ground')
            else:
                G.add_node(node, type='node')
            G.add_edge(cid, node)
            print(f"[DEBUG] Connected component {cid} to node {node}.")

    print("[DEBUG] Graph creation from XML completed.")
    return G, components_element


def align_to_grid(position, grid_size=1.0):
    """
    将给定的位置对齐到最近的网格点。

    :param position: 元组，(x, y) 坐标
    :param grid_size: 网格大小
    :return: 对齐后的 (x, y) 坐标
    """
    x, y = position
    aligned_x = round(x / grid_size) * grid_size
    aligned_y = round(y / grid_size) * grid_size
    return (aligned_x, aligned_y)


def draw_grid(d, grid_size=1.0, grid_extent=20):
    """
    在绘图中添加网格线。

    :param d: schemdraw.Drawing 对象
    :param grid_size: 网格大小
    :param grid_extent: 网格的范围（正负方向）
    """
    for x in np.arange(-grid_extent, grid_extent + grid_size, grid_size):
        d.add(elm.Line().at((x, -grid_extent)).to((x, grid_extent)).color('lightgray').linewidth(0.3))
    for y in np.arange(-grid_extent, grid_extent + grid_size, grid_size):
        d.add(elm.Line().at((-grid_extent, y)).to((grid_extent, y)).color('lightgray').linewidth(0.3))
    print(f"[DEBUG] Drawn grid with size {grid_size} and extent {grid_extent}.")


def update_xml_positions_and_directions(xml_root, pos, directions, flips=None, scaling_ratios=None):
    if flips is None:
        flips = {}
    if scaling_ratios is None:
        scaling_ratios = {}
    print("[DEBUG] Updating XML with positions, directions, flips, and scales...")
    components_elem = xml_root.find('components')
    for comp in components_elem:
        cid = comp.attrib['id']
        if cid in pos:
            x, y = pos[cid]
            comp.set('x', str(x))
            comp.set('y', str(y))
            print(f"[DEBUG] Set position for {cid}: x={x}, y={y}")
        if cid in directions:
            comp.set("direction", directions[cid])
            print(f"[DEBUG] Set direction for {cid}: {directions[cid]}")
        if cid in flips:
            comp.set("flip", flips[cid])
            print(f"[DEBUG] Set flip for {cid}: {flips[cid]}")
        if cid in scaling_ratios:
            scaling = scaling_ratios[cid]
            comp.set("vertical_scale", str(scaling.get('vertical_scale', 1.0)))
            comp.set("horizontal_scale", str(scaling.get('horizontal_scale', 1.0)))
            print(f"[DEBUG] Set vertical_scale for {cid}: {scaling.get('vertical_scale', 1.0)}")
            print(f"[DEBUG] Set horizontal_scale for {cid}: {scaling.get('horizontal_scale', 1.0)}")
    nodes_elem = xml_root.find('nodes')
    if nodes_elem is None:
        nodes_elem = ET.SubElement(xml_root, 'nodes')
    for nid in pos:
        if any(comp.attrib['id'] == nid for comp in components_elem):
            continue
        node_elem = next((node for node in nodes_elem if node.attrib.get('id') == nid), None)
        if node_elem is None:
            node_elem = ET.SubElement(nodes_elem, 'node', id=nid)
            print(f"[DEBUG] Added new node element for {nid}.")
        x, y = pos[nid]
        node_elem.set('x', str(x))
        node_elem.set('y', str(y))
        print(f"[DEBUG] Set position for node {nid}: x={x}, y={y}")


def add_anchor_dots(d, all_wires):
    print("[DEBUG] Adding anchor dots where necessary...")
    anchor_wire_count = {}
    for wire in all_wires:
        line = wire['line']
        start_pt = line.coords[0]
        end_pt = line.coords[1]
        anchor_wire_count[start_pt] = anchor_wire_count.get(start_pt, 0) + 1
        anchor_wire_count[end_pt] = anchor_wire_count.get(end_pt, 0) + 1

    added_dots = set()
    for anchor_pos, count in anchor_wire_count.items():
        if count >= 3 and anchor_pos not in added_dots:
            d.add(elm.Dot(radius=0.12).at(anchor_pos))
            print(f"[DEBUG] Added anchor dot at position: {anchor_pos}")
            added_dots.add(anchor_pos)


def build_comp_node_mapping(components):
    """
    Constructs a mapping from component IDs to their node-pin associations.
    """
    print("[DEBUG] Building component-node mapping...")
    comp_node_mapping = {}
    for comp in components:
        ctype = comp.attrib['type']
        cid = comp.attrib['id']
        # Extract nodes
        nodes = []
        i = 1
        while True:
            node = comp.find(f'node{i}')
            if node is not None:
                nodes.append(node.text)
                i += 1
            else:
                break

        if ctype == 'Q':
            if len(nodes) < 3:
                print(f"[DEBUG] Component {cid} of type Q has insufficient nodes.")
                continue
            node_collector = nodes[0]
            node_base = nodes[1]
            node_emitter = nodes[2]
            mapping = {}
            mapping.setdefault(node_collector, []).append('collector')
            mapping.setdefault(node_base, []).append('base')
            mapping.setdefault(node_emitter, []).append('emitter')
            comp_node_mapping[cid] = mapping
            print(f"[DEBUG] Mapping for {cid}: {mapping}")

        elif ctype == 'D':
            if len(nodes) < 2:
                print(f"[DEBUG] Component {cid} of type D has insufficient nodes.")
                continue
            node_anode = nodes[0]
            node_cathode = nodes[1]
            mapping = {}
            mapping.setdefault(node_anode, []).append('anode')
            mapping.setdefault(node_cathode, []).append('cathode')
            comp_node_mapping[cid] = mapping
            print(f"[DEBUG] Mapping for {cid}: {mapping}")

        elif ctype in ['R', 'C', 'L', 'S']:
            if len(nodes) < 2:
                print(f"[DEBUG] Component {cid} of type {ctype} has insufficient nodes.")
                continue
            node_start = nodes[0]
            node_end = nodes[1]
            mapping = {}
            mapping.setdefault(node_start, []).append('start')
            mapping.setdefault(node_end, []).append('end')
            comp_node_mapping[cid] = mapping
            print(f"[DEBUG] Mapping for {cid}: {mapping}")

        elif ctype in ['V', 'I', 'E', 'H', 'F', 'G']:
            if len(nodes) < 2:
                print(f"[DEBUG] Component {cid} of type {ctype} has insufficient nodes.")
                continue
            node_positive = nodes[0]
            node_negative = nodes[1]
            mapping = {}
            mapping.setdefault(node_positive, []).append('positive')
            mapping.setdefault(node_negative, []).append('negative')
            comp_node_mapping[cid] = mapping
            print(f"[DEBUG] Mapping for {cid}: {mapping}")

        elif ctype == 'J':
            if len(nodes) < 3:
                print(f"[DEBUG] Component {cid} of type J has insufficient nodes.")
                continue
            node_drain = nodes[0]
            node_gate = nodes[1]
            node_source = nodes[2]
            mapping = {}
            mapping.setdefault(node_drain, []).append('drain')
            mapping.setdefault(node_gate, []).append('gate')
            mapping.setdefault(node_source, []).append('source')
            comp_node_mapping[cid] = mapping
            print(f"[DEBUG] Mapping for {cid}: {mapping}")

        elif ctype == 'M':
            if len(nodes) < 4:
                print(f"[DEBUG] Component {cid} of type M has insufficient nodes.")
                continue
            node_drain = nodes[0]
            node_gate = nodes[1]
            node_source = nodes[2]
            node_bulk = nodes[3]
            mapping = {}
            mapping.setdefault(node_drain, []).append('drain')
            mapping.setdefault(node_gate, []).append('gate')
            mapping.setdefault(node_source, []).append('source')
            mapping.setdefault(node_bulk, []).append('bulk')
            comp_node_mapping[cid] = mapping
            print(f"[DEBUG] Mapping for {cid}: {mapping}")

        # Add more component types as needed

    print("[DEBUG] Component-node mapping completed.")
    return comp_node_mapping


def get_component_pin(comp, neighbor, comp_info, components_element, pins, pin_mapping):
    """
    Assigns a specific pin of a component to connect to a neighbor node.
    """
    ctype = comp_info.get('type')

    if ctype in ['Q', 'D', 'M']:
        # For BJT (Q), Diode (D), and MOSFET (M), use the specific pin mapping
        if comp in pin_mapping:
            mapping = pin_mapping.get(comp, {})
            if neighbor in mapping and mapping[neighbor]:
                pin = mapping[neighbor].pop(0)
                print(f"[DEBUG] Assigned pin '{pin}' for component '{comp}' to neighbor '{neighbor}'")
                return pin
            else:
                print(f"[DEBUG] No available pins for component '{comp}' connecting to '{neighbor}'")
                return None
    elif ctype in ['R', 'C', 'L', 'S', 'V', 'I', 'E', 'H', 'F', 'G', 'J']:
        # For other components, use the general pin mapping
        if comp in pin_mapping:
            mapping = pin_mapping.get(comp, {})
            if neighbor in mapping and mapping[neighbor]:
                pin = mapping[neighbor].pop(0)
                print(f"[DEBUG] Assigned pin '{pin}' for component '{comp}' to neighbor '{neighbor}'")
                return pin
            else:
                print(f"[DEBUG] No available pins for component '{comp}' connecting to '{neighbor}'")
                return None

    # Default case
    print(f"[DEBUG] Defaulting to 'pin' for component '{comp}' connecting to '{neighbor}'")
    return 'pin'


def get_pin_position(pins, node, pin_name, grid_size=1.0):
    try:
        pos = pins[node][pin_name]
        if isinstance(pos, tuple):
            aligned_pos = align_to_grid(pos, grid_size=grid_size)
            return aligned_pos
        elif hasattr(pos, 'x') and hasattr(pos, 'y'):
            aligned_pos = align_to_grid((pos.x, pos.y), grid_size=grid_size)
            return aligned_pos
        else:
            return pos
    except KeyError:
        print(f"[ERROR] Pin '{pin_name}' not found for component/node '{node}'.")
        return None


def route_connection_current_method(start_pos, end_pos, component_boxes, comp, neighbor, all_wires, min_wire_spacing=0.4):
    """
    使用曼哈顿路由方法连接起点和终点，并确保平行线之间的最小间距。
    """
    # Define possible paths: first horizontal then vertical, or first vertical then horizontal
    path1 = [start_pos, (end_pos[0], start_pos[1]), end_pos]
    path2 = [start_pos, (start_pos[0], end_pos[1]), end_pos]

    # Create buffer around existing wires
    existing_buffers = [wire['line'].buffer(min_wire_spacing / 2, cap_style=2, join_style=2) for wire in all_wires]

    def path_ok(path):
        for i in range(len(path) - 1):
            seg = LineString([path[i], path[i + 1]])
            # Check intersection with component bounding boxes
            if is_wire_crossing_components(path[i], path[i + 1], component_boxes, comp, neighbor):
                return False
            # Check intersection with existing wire buffers
            for buf in existing_buffers:
                if seg.intersects(buf):
                    print(f"[DEBUG] Path segment {seg} intersects with existing wire buffer.")
                    return False
        return True

    # Check path1
    if path_ok(path1):
        print(f"[DEBUG] Path1 is OK: {path1}")
        return path1, wire_safe_color

    # Check path2
    if path_ok(path2):
        print(f"[DEBUG] Path2 is OK: {path2}")
        return path2, wire_safe_color

    # If neither path is valid, attempt to find an alternative path or mark as danger
    print(f"[DEBUG] Both paths not OK. Selecting path1 with danger color.")
    return path1, wire_danger_color


def is_wire_crossing_components(start_pos, end_pos, component_boxes, comp, neighbor):
    line = LineString([start_pos, end_pos])
    line_start_point = Point(start_pos)
    line_end_point = Point(end_pos)
    for comp_id, comp_info in component_boxes.items():
        polygon = comp_info['polygon']
        if line.intersects(polygon):
            intersection = line.intersection(polygon)
            if (intersection.equals(line_start_point) or intersection.equals(line_end_point)) and (
                    comp_id == comp or comp_id == neighbor):
                continue
            else:
                print(f"[DEBUG] Wire from {start_pos} to {end_pos} crosses component {comp_id}")
                return True
    return False


def check_overlapping_wires(all_wires):
    print("[DEBUG] Checking for overlapping wires...")
    overlapping = False
    num_wires = len(all_wires)
    for i in range(num_wires):
        line_i = all_wires[i]['line']
        for j in range(i + 1, num_wires):
            line_j = all_wires[j]['line']
            if line_i.equals(line_j) or line_i.contains(line_j) or line_j.contains(line_i):
                overlapping = True
                all_wires[i]['color'] = wire_danger_color
                all_wires[j]['color'] = wire_danger_color
                print(f"[DEBUG] Overlapping wires detected between wire {i} and wire {j}. Marking as danger.")
    return overlapping


def set_element_direction(element, direction):
    print(f"[DEBUG] Setting direction: {direction}")
    if direction == 'up':
        print("[DEBUG] Applying 'up' direction.")
        element.up()
    elif direction == 'down':
        print("[DEBUG] Applying 'down' direction.")
        element.down()
    elif direction == 'left':
        print("[DEBUG] Applying 'left' direction.")
        element.left()
    else:
        print("[DEBUG] Applying 'right' direction (default).")
        element.right()


def rotate_ccw(x, y, cx, cy, theta=0):
    # Convert the angle theta to radians
    rad = math.radians(theta)
    # rotate the point (x, y) counterclockwise by theta degrees around the point (cx, cy)
    x_new = (x - cx) * math.cos(rad) - (y - cy) * math.sin(rad) + cx
    y_new = (x - cx) * math.sin(rad) + (y - cy) * math.cos(rad) + cy
    return x_new, y_new


def flip_component(points, flip_point, mode="horizontal"):
    """
    根据给定点对长方形进行水平或垂直翻转。

    :param points: 长方形的四个顶点 [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]
    :param flip_point: 翻转点 (xf, yf)
    :param mode: 翻转模式 "horizontal" 或 "vertical"
    :return: 翻转后的新顶点 [(x1', y1'), (x2', y2'), (x3', y3'), (x4', y4')]
    """
    xf, yf = flip_point
    flipped_points = []

    for x, y in points:
        if mode == "horizontal":  # 水平翻转
            new_x = 2 * xf - x
            new_y = y
        elif mode == "vertical":  # 垂直翻转
            new_x = x
            new_y = 2 * yf - y
        else:
            new_x = x
            new_y = y
        flipped_points.append((new_x, new_y))

    return flipped_points


def get_component_bbox(component_type,
                       pos=(1, 2),
                       direction='right',
                       shrink_size=0.1, flip='none',
                       vertical_scale=1.0,
                       horizontal_scale=1.0,
                       **kwargs):
    print(
        f"[DEBUG] Getting bounding box for component type '{component_type}' at position {pos} with direction "
        f"'{direction}', flip '{flip}', vertical_scale '{vertical_scale}', and horizontal_scale '{horizontal_scale}'")
    # Handle MOSFETs separately
    if component_type == 'M':
        model = kwargs.get('model', '').upper()
        if model.startswith('P'):
            try:
                comp_class = elm.PFet
                print("[DEBUG] Detected PMOS transistor.")
            except AttributeError:
                print("Error: schemdraw does not have 'PFet' element. Please verify the element name.")
                return
        else:
            try:
                comp_class = elm.NFet
                print("[DEBUG] Detected NMOS transistor.")
            except AttributeError:
                print("Error: schemdraw does not have 'NFet' element. Please verify the element name.")
                return
    else:
        comp_class = component_map.get(component_type, None)
        if comp_class is None:
            raise ValueError(f"Unsupported component type: {component_type}")
        print(f"[DEBUG] Using schemdraw element '{comp_class.__name__}' for component type '{component_type}'.")

    if component_type == 'M':
        # Extract additional parameters if any
        bulk = kwargs.get('bulk', False)
        element = comp_class(bulk=bulk).at(pos)
    else:
        element = comp_class(**kwargs).at(pos)

    # Apply direction
    set_element_direction(element, direction)
    d = schemdraw.Drawing(show=False)
    # Apply scaling
    element.scalex(horizontal_scale).scaley(vertical_scale)
    print(f"[DEBUG] Applied horizontal scale {horizontal_scale} and vertical scale {vertical_scale} to component.")

    element = d.add(element)
    print(f"[DEBUG] Element added to drawing: {element}")

    # Determine reference anchor
    if 'start' in element.anchors:
        ref_anchor_name = 'start'
    elif 'drain' in element.anchors:
        ref_anchor_name = 'drain'
    elif 'base' in element.anchors:
        ref_anchor_name = 'base'
    elif 'bulk' in element.anchors:
        ref_anchor_name = 'bulk'
    else:
        ref_anchor_name = list(element.anchors.keys())[0]

    abs_ref = element.absanchors[ref_anchor_name]
    local_ref = element.anchors[ref_anchor_name]

    if isinstance(local_ref, (tuple, list)):
        xDiff = abs_ref.x - local_ref[0]
        yDiff = abs_ref.y - local_ref[1]
    else:
        xDiff = abs_ref.x - local_ref.x
        yDiff = abs_ref.y - local_ref.y

    bbox_local = element.get_bbox()
    xmin_global = xDiff + bbox_local.xmin * horizontal_scale
    xmax_global = xDiff + bbox_local.xmax * horizontal_scale
    ymin_global = yDiff + bbox_local.ymin * vertical_scale
    ymax_global = yDiff + bbox_local.ymax * vertical_scale

    # Adjust bounding box based on component type
    if component_type in ['GND', 'ground']:
        ymax_global -= shrink_size * vertical_scale
    elif component_type in ['R', 'C', 'L', 'D', 'V', 'I', 'E', 'H', 'F', 'G', 'S']:
        xmin_global += shrink_size * horizontal_scale
        xmax_global -= shrink_size * horizontal_scale
    elif component_type == 'J':
        xmax_global -= shrink_size * horizontal_scale
        ymin_global += shrink_size * vertical_scale
        ymax_global -= shrink_size * vertical_scale
        # Prevent wire crossing the junction field-effect transistor.
        xmin_global -= shrink_size * horizontal_scale * 2
    elif component_type == 'Q':
        xmin_global += shrink_size * horizontal_scale
        ymin_global += shrink_size * vertical_scale
        ymax_global -= shrink_size * vertical_scale
        # Prevent wire crossing the bipolar transistor.
        xmax_global += shrink_size * horizontal_scale * 2
    elif component_type == 'M':
        xmin_global += shrink_size * horizontal_scale
        ymin_global += shrink_size * vertical_scale
        ymax_global -= shrink_size * vertical_scale
        xmax_global -= shrink_size * horizontal_scale  # Changed from += to -= to match user's configuration

    anchors_global = {k: (v.x, v.y) for k, v in element.absanchors.items()}
    bbox = {
        'xmin': xmin_global,
        'ymin': ymin_global,
        'xmax': xmax_global,
        'ymax': ymax_global
    }
    print(f"[DEBUG] Bounding box before rotation: {bbox}")

    # Get all vertices of the rectangle
    vertices = [
        (bbox['xmin'], bbox['ymin']),
        (bbox['xmax'], bbox['ymin']),
        (bbox['xmax'], bbox['ymax']),
        (bbox['xmin'], bbox['ymax'])
    ]

    print(f"[DEBUG] Original vertices: {vertices}")
    (x_center, y_center) = pos
    print(pos)
    print(abs_ref)
    # Rotate each vertex
    if direction == 'up':
        rotated_vertices = [rotate_ccw(x, y, x_center, y_center, 90) for x, y in vertices]
    elif direction == 'left':
        rotated_vertices = [rotate_ccw(x, y, x_center, y_center, 180) for x, y in vertices]
    elif direction == 'down':
        rotated_vertices = [rotate_ccw(x, y, x_center, y_center, 270) for x, y in vertices]
    else:
        rotated_vertices = vertices
    # Find new boundaries
    rotated_x = [v[0] for v in rotated_vertices]
    rotated_y = [v[1] for v in rotated_vertices]

    bbox = {
        'xmin': min(rotated_x),
        'ymin': min(rotated_y),
        'xmax': max(rotated_x),
        'ymax': max(rotated_y)
    }
    print(f"[DEBUG] Bounding box after rotation: {bbox}")

    # Get all vertices of the rectangle
    rotated_vertices = [
        (bbox['xmin'], bbox['ymin']),
        (bbox['xmax'], bbox['ymin']),
        (bbox['xmax'], bbox['ymax']),
        (bbox['xmin'], bbox['ymax'])
    ]

    print(f"[DEBUG] Rotated vertices: {rotated_vertices}")

    if component_type in ['J', 'M']:
        # because the direction is up or down, so the horizontal flip is the same as the vertical flip
        flipped_vertices = flip_component(rotated_vertices, (x_center, y_center), mode=flip)

    elif component_type == 'Q':
        # because the direction is up or down, so the horizontal flip is the same as the vertical flip
        if direction == 'up' or direction == 'down':
            if flip == 'horizontal':
                flipped_vertices = flip_component(rotated_vertices, (x_center, y_center), mode=flip)
                print("flip horizontal")
            elif flip == 'vertical':
                collector = element.absanchors['collector']
                print(collector.x, collector.y)
                flipped_vertices = flip_component(rotated_vertices, (collector.x, collector.y), mode=flip)
                print("flip vertical")
            else:
                flipped_vertices = rotated_vertices
            print("run there-------------------------------------why")
        else:
            if flip == 'horizontal':
                collector = element.absanchors['collector']
                print(collector.x, collector.y)
                flipped_vertices = flip_component(rotated_vertices, (collector.x, collector.y), mode=flip)
                print("work??????????????")
            elif flip == 'vertical':
                flipped_vertices = flip_component(rotated_vertices, (x_center, y_center), mode=flip)
            else:
                flipped_vertices = rotated_vertices

    else:
        # No flip needed for other components
        flipped_vertices = rotated_vertices

    # Find new boundaries after flip
    rotated_x = [v[0] for v in flipped_vertices]
    rotated_y = [v[1] for v in flipped_vertices]

    bbox = {
        'xmin': min(rotated_x),
        'ymin': min(rotated_y),
        'xmax': max(rotated_x),
        'ymax': max(rotated_y)
    }
    print(f"[DEBUG] Bounding box after flip: {bbox}")
    print(f"[DEBUG] Global anchors: {anchors_global}")

    return bbox, element.anchors, anchors_global


def draw_component_or_node(d, elements, pins, node, node_info, x, y, direction, component_boxes, G, flip='none', scaling_ratios=None):
    if scaling_ratios is None:
        scaling_ratios = {}
    print(f"[DEBUG] Drawing component/node: {node} of type {node_info.get('type')}")
    ctype = node_info.get('type')
    if ctype is None:
        ctype = 'node'

    deg = G.degree(node)

    if ctype == 'node':
        if deg >= 3:
            elements[node] = d.add(elm.Dot(radius=0.12).at((x, y)))
            d.add(elm.Label().at((x, y)).label(node, ofst=0.2))
            pins[node] = {'pin': (x, y)}
            print(f"[DEBUG] Drew node '{node}' as a junction (degree={deg}).")
        else:
            d.add(elm.Label().at((x, y)).label(node, ofst=0.2))
            pins[node] = {'pin': (x, y)}
            print(f"[DEBUG] Drew node '{node}' as a label (degree={deg}).")
        return

    if ctype == 'ground':
        ctype_for_bbox = 'GND'
    else:
        ctype_for_bbox = ctype

    print(f"[DEBUG] Component type for bbox: {ctype_for_bbox}")
    try:
        if ctype_for_bbox == 'M':
            # Retrieve model to determine PFet or NFet
            model = node_info.get('model', '').upper()
            if not model:
                # Try to get model from parameters
                model = node_info.get('parameters', {}).get('MODEL', '').upper()
            if model.startswith('P'):
                comp_class = elm.PFet
                print("[DEBUG] Detected PMOS transistor.")
            else:
                comp_class = elm.NFet
                print("[DEBUG] Detected NMOS transistor.")
            # Extract bulk parameter if available
            bulk = 'bulk' in node_info.get('pins', [])
            # Get additional parameters
            params = node_info.get('parameters', {})
            # Remove 'MODEL' from params to avoid passing it to the element
            params = {k: v for k, v in params.items() if k != 'MODEL'}
            # [MODIFIED] Retrieve vertical and horizontal scales
            scale_info = scaling_ratios.get(node, default_scaling_ratios.get(ctype_for_bbox, {'vertical_scale':1.0, 'horizontal_scale':1.0}))
            vertical_scale = scale_info.get('vertical_scale', 1.0)
            horizontal_scale = scale_info.get('horizontal_scale', 1.0)
            bbox, anchors_local, anchors_global = get_component_bbox(
                ctype_for_bbox, pos=(x, y), direction=direction,
                shrink_size=shrink_size, flip=flip,
                vertical_scale=vertical_scale,
                horizontal_scale=horizontal_scale,
                model=model, bulk=bulk, **params
            )
        else:
            # [MODIFIED] Retrieve vertical and horizontal scales
            scale_info = scaling_ratios.get(node, default_scaling_ratios.get(ctype_for_bbox, {'vertical_scale':1.0, 'horizontal_scale':1.0}))
            vertical_scale = scale_info.get('vertical_scale', 1.0)
            horizontal_scale = scale_info.get('horizontal_scale', 1.0)
            bbox, anchors_local, anchors_global = get_component_bbox(
                ctype_for_bbox, pos=(x, y), direction=direction,
                shrink_size=shrink_size, flip=flip,
                vertical_scale=vertical_scale,
                horizontal_scale=horizontal_scale,
                **node_info.get('parameters', {})
            )
    except ValueError as e:
        print(f"[ERROR] {e}")
        return

    label_text = f"{node}\n{node_info.get('value', '')}"

    if ctype_for_bbox == 'M':
        # Instantiate the correct MOSFET type
        if model.startswith('P'):
            try:
                element = elm.PFet(bulk=True, scale=1.0).at((x, y))  # schemdraw does not support separate scaling
            except AttributeError:
                print("Error: schemdraw does not have 'PFet' element. Please verify the element name.")
                return
        else:
            try:
                element = elm.NFet(bulk=True, scale=1.0).at((x, y))  # schemdraw does not support separate scaling
            except AttributeError:
                print("Error: schemdraw does not have 'NFet' element. Please verify the element name.")
                return
    else:
        comp_class = component_map.get(ctype_for_bbox, elm.Dot(radius=0.12))
        if comp_class is None:
            print(f"[ERROR] Unsupported component type: {ctype_for_bbox}")
            return
        # [MODIFIED] Apply horizontal scale using scalex and vertical scale using scaley
        element = comp_class(**node_info.get('parameters', {})).at((x, y))

    # Apply direction
    set_element_direction(element, direction)

    # Apply flip if not MOSFET (handled above)
    if ctype_for_bbox != 'M':
        if direction in ['up', 'down']:
            if flip == 'horizontal':
                element.flip()
                print(f"[DEBUG] Applied horizontal flip to component '{node}'.")
            elif flip == 'vertical':
                element.reverse()
                print(f"[DEBUG] Applied vertical flip to component '{node}'.")
        else:
            if flip == 'horizontal':
                element.reverse()
                print(f"[DEBUG] Applied horizontal flip to component '{node}'.")
            elif flip == 'vertical':
                element.flip()
                print(f"[DEBUG] Applied vertical flip to component '{node}'.")

    # Add label
    element.label(label_text)
    element.scalex(horizontal_scale).scaley(vertical_scale)
    element = d.add(element)
    print(f"[DEBUG] Drew component '{node}' with label '{label_text}'.")

    # Assign pins based on component type
    if ctype_for_bbox in ['V', 'I', 'E', 'H', 'F', 'G']:
        pins[node] = {
            'positive': (element.absanchors['end'].x, element.absanchors['end'].y),
            'negative': (element.absanchors['start'].x, element.absanchors['start'].y)
        }
        print(f"[DEBUG] Assigned 'positive' and 'negative' pins for component '{node}'.")
    elif ctype_for_bbox in ['R', 'C', 'L', 'S']:
        pins[node] = {
            'start': (element.absanchors['start'].x, element.absanchors['start'].y),
            'end': (element.absanchors['end'].x, element.absanchors['end'].y)
        }
        print(f"[DEBUG] Assigned 'start' and 'end' pins for component '{node}'.")
    elif ctype_for_bbox == 'D':
        pins[node] = {
            'anode': (element.absanchors['start'].x, element.absanchors['start'].y),
            'cathode': (element.absanchors['end'].x, element.absanchors['end'].y)
        }
        print(f"[DEBUG] Assigned 'anode' and 'cathode' pins for component '{node}'.")
    elif ctype_for_bbox == 'J':
        pins[node] = {
            'drain': (element.absanchors['drain'].x, element.absanchors['drain'].y),
            'gate': (element.absanchors['gate'].x, element.absanchors['gate'].y),
            'source': (element.absanchors['source'].x, element.absanchors['source'].y)
        }
        print(f"[DEBUG] Assigned 'drain', 'gate', and 'source' pins for component '{node}'.")
    elif ctype_for_bbox == 'Q':
        pins[node] = {
            'collector': (element.absanchors['collector'].x, element.absanchors['collector'].y),
            'base': (element.absanchors['base'].x, element.absanchors['base'].y),
            'emitter': (element.absanchors['emitter'].x, element.absanchors['emitter'].y)
        }
        print(f"[DEBUG] Assigned 'collector', 'base', and 'emitter' pins for component '{node}'.")
    elif ctype_for_bbox == 'M':
        pins[node] = {
            'drain': (element.absanchors['drain'].x, element.absanchors['drain'].y),
            'gate': (element.absanchors['gate'].x, element.absanchors['gate'].y),
            'source': (element.absanchors['source'].x, element.absanchors['source'].y),
            'bulk': (element.absanchors['bulk'].x, element.absanchors['bulk'].y)
        }
        print(f"[DEBUG] Assigned 'drain', 'gate', 'source', and 'bulk' pins for component '{node}'.")
    elif ctype_for_bbox == 'ground':
        pins[node] = {'pin': (x, y)}
        print(f"[DEBUG] Assigned single 'pin' for ground node '{node}'.")
    elif ctype_for_bbox == 'node':
        pins[node] = {'pin': (x, y)}
        print(f"[DEBUG] Assigned single 'pin' for node '{node}'.")
    else:
        pins[node] = {'pin': (x, y)}
        print(f"[DEBUG] Assigned default 'pin' for component/node '{node}'.")

    x_min, y_min, x_max, y_max = bbox['xmin'], bbox['ymin'], bbox['xmax'], bbox['ymax']
    component_boxes[node] = {
        'bbox': bbox,
        'polygon': Polygon([(x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max)]),
        'type': ctype_for_bbox
    }
    print(f"[DEBUG] Bounding box for '{node}': {bbox}")


def draw_connections(d, G, components_element, pins, component_boxes, drawn_edges, routing_method, all_wires,
                     pin_mapping, grid_size=1.0, min_wire_spacing=0.4):
    print("[DEBUG] Starting to draw connections...")
    for comp, neighbor, key in G.edges(keys=True):
        edge_nodes = tuple(sorted([comp, neighbor]))
        # Include key to distinguish multiple edges
        edge_key = edge_nodes + (key,)
        print(f"[DEBUG] Processing edge_key: {edge_key}")
        if edge_key in drawn_edges:
            print(f"[DEBUG] Edge {edge_key} already drawn. Skipping.")
            continue

        comp_info = G.nodes[comp]
        neighbor_info = G.nodes[neighbor]

        comp_pin = get_component_pin(comp, neighbor, comp_info, components_element, pins, pin_mapping)
        print(f"[DEBUG] comp_pin for {comp}: {comp_pin}")
        neighbor_pin = get_component_pin(neighbor, comp, neighbor_info, components_element, pins, pin_mapping)
        print(f"[DEBUG] neighbor_pin for {neighbor}: {neighbor_pin}")
        if comp_pin is None or neighbor_pin is None:
            print(f"[WARNING] Skipping edge {edge_key} due to missing pin assignments.")
            continue

        start_pos = get_pin_position(pins, comp, comp_pin, grid_size=grid_size)
        end_pos = get_pin_position(pins, neighbor, neighbor_pin, grid_size=grid_size)

        print(
            f"[DEBUG] Connecting {comp_pin} of '{comp}' at {start_pos} to {neighbor_pin} of '{neighbor}' at {end_pos}")
        if start_pos is None or end_pos is None:
            print(f"[WARNING] Skipping edge {edge_key} due to missing positions.")
            continue

        selected_path, line_color = route_connection_current_method(
            start_pos, end_pos, component_boxes, comp, neighbor, all_wires, min_wire_spacing=min_wire_spacing
        )

        for i in range(len(selected_path) - 1):
            segment_start = selected_path[i]
            segment_end = selected_path[i + 1]
            # Ensure wires align to grid
            segment_start = align_to_grid(segment_start, grid_size=grid_size)
            segment_end = align_to_grid(segment_end, grid_size=grid_size)
            new_line = LineString([segment_start, segment_end])
            all_wires.append({'line': new_line, 'color': line_color})
            print(f"[DEBUG] Added wire segment from {segment_start} to {segment_end} with color '{line_color}'")

        drawn_edges.add(edge_key)
        print(f"[DEBUG] Finished drawing edge {edge_key}")

    print("[DEBUG] Finished drawing all connections.")


def draw_circuit(G, components_element, xml_root, xml_file, initial_comp_node_mapping, max_attempts=100,
                EnlargeSize=2.5, routing_method=1, auto=1, grid_size=1.0, min_wire_spacing=0.4):
    print("[DEBUG] Starting to draw the circuit...")
    attempt = 0
    success = False

    last_pos = None
    last_all_wires = None

    # Read existing directions, flips, positions, and scales from XML
    positions_in_xml = {}
    directions_in_xml = {}
    flips_in_xml = {}  # Read flip attribute
    scaling_ratios = {}  # Read scale attributes

    components_elem = xml_root.find('components')
    if components_elem is not None:
        for comp in components_elem:
            cid = comp.attrib['id']
            dir_val = comp.attrib.get('direction', 'right')
            flip_val = comp.attrib.get('flip', 'none')  # Read flip attribute
            vertical_scale = float(comp.attrib.get('vertical_scale', '1.0'))
            horizontal_scale = float(comp.attrib.get('horizontal_scale', '1.0'))
            scaling_ratios[cid] = {
                'vertical_scale': vertical_scale,
                'horizontal_scale': horizontal_scale
            }
            directions_in_xml[cid] = dir_val
            flips_in_xml[cid] = flip_val
            if 'x' in comp.attrib and 'y' in comp.attrib:
                positions_in_xml[cid] = (float(comp.attrib['x']), float(comp.attrib['y']))
            print(
                f"[DEBUG] Component '{cid}': direction='{dir_val}', flip='{flip_val}', vertical_scale='{vertical_scale}', horizontal_scale='{horizontal_scale}', position={positions_in_xml.get(cid, 'Not set')}")

    nodes_elem = xml_root.find('nodes')
    if nodes_elem is not None:
        for node in nodes_elem:
            nid = node.attrib['id']
            if 'x' in node.attrib and 'y' in node.attrib:
                positions_in_xml[nid] = (float(node.attrib['x']), float(node.attrib['y']))
                print(f"[DEBUG] Node '{nid}': position=({node.attrib['x']}, {node.attrib['y']})")

    if auto == 1:
        # Automatic layout
        while attempt < max_attempts:
            attempt += 1
            print(f"[DEBUG] Attempt {attempt} (auto=1, automatic layout)")

            original_pos = nx.spring_layout(G)
            # Enlarge layout
            pos = {n: (EnlargeSize * original_pos[n][0], EnlargeSize * (-original_pos[n][1])) for n in G.nodes}

            # Align all positions to grid
            pos = {n: align_to_grid(pos[n], grid_size=grid_size) for n in pos}

            directions = {}
            flips = {}
            scales = {}
            for n in G.nodes:
                directions[n] = directions_in_xml.get(n, 'right')
                flips[n] = flips_in_xml.get(n, 'none')
                scales[n] = scaling_ratios.get(n, default_scaling_ratios.get(G.nodes[n]['type'], {'vertical_scale':1.0, 'horizontal_scale':1.0}))  # Assign scale

            d = schemdraw.Drawing()
            if draw_grid_or_not == 1:
                draw_grid(d, grid_size=grid_size)  # Add grid lines

            elements = {}
            pins = {}
            component_boxes = {}
            drawn_edges = set()
            all_wires = []

            for node in G.nodes:
                node_info = G.nodes[node]
                x, y = pos[node]
                direction = directions.get(node, 'right')
                flip_in_xml = flips.get(node, 'none')
                scale = scales.get(node, {'vertical_scale':1.0, 'horizontal_scale':1.0})  # Assign scale
                print(
                    f"[DEBUG] Drawing node/component '{node}' at ({x}, {y}) with direction '{direction}', flip '{flip_in_xml}', and scale '{scale}'")
                draw_component_or_node(d, elements, pins, node, node_info, x, y, direction, component_boxes, G,
                                       flip=flip_in_xml, scaling_ratios=scales)  # Pass scaling_ratios
                # Update scaling_ratios for later use
                scaling_ratios[node] = scale

            # Use a deep copy of initial_comp_node_mapping
            pin_mapping_copy = copy.deepcopy(initial_comp_node_mapping)

            draw_connections(d, G, components_element, pins, component_boxes, drawn_edges, routing_method,
                             all_wires, pin_mapping_copy, grid_size=grid_size, min_wire_spacing=min_wire_spacing)

            any_red_line = any(wire['color'] == wire_danger_color for wire in all_wires)
            overlapping = check_overlapping_wires(all_wires)

            last_pos = pos
            last_all_wires = all_wires

            if any_red_line or overlapping:
                print("[DEBUG] Overlap or red lines detected. Retrying...")
                continue
            else:
                print(f"[DEBUG] Successful layout found on attempt {attempt}. Drawing wires...")
                for wire_info in all_wires:
                    line = wire_info['line']
                    color = wire_info['color']
                    d.add(elm.Line().at(line.coords[0]).to(line.coords[1]).color(color))
                    print(f"[DEBUG] Drew wire from {line.coords[0]} to {line.coords[1]} with color '{color}'")
                add_anchor_dots(d, all_wires)
                print(f"[DEBUG] Layout successful after {attempt} attempts.")
                update_xml_positions_and_directions(xml_root, pos, directions, flips, scaling_ratios)  # Pass scaling_ratios
                tree = ET.ElementTree(xml_root)
                tree.write(xml_file)
                d.draw()
                success = True
                break

        if not success:
            print("[DEBUG] No suitable layout found after all attempts. Showing last attempt with issues.")
            d = schemdraw.Drawing()
            if last_pos is not None and last_all_wires is not None:
                update_xml_positions_and_directions(xml_root, last_pos, directions_in_xml, flips_in_xml, scaling_ratios)
                tree = ET.ElementTree(xml_root)
                tree.write(xml_file)

                elements = {}
                pins = {}
                component_boxes = {}
                drawn_edges = set()
                for node in G.nodes:
                    node_info = G.nodes[node]
                    x, y = last_pos[node]
                    direction = directions_in_xml.get(node, 'right')
                    flip_in_xml = flips_in_xml.get(node, 'none')
                    scale = scaling_ratios.get(node, {'vertical_scale':1.0, 'horizontal_scale':1.0})  # Assign scale
                    print(
                        f"[DEBUG] Redrawing node/component '{node}' at ({x}, {y}) with direction '{direction}', flip '{flip_in_xml}', and scale '{scale}'")
                    draw_component_or_node(d, elements, pins, node, node_info, x, y, direction, component_boxes, G,
                                           flip=flip_in_xml, scaling_ratios=scaling_ratios)  # Pass scaling_ratios
                    # Update scaling_ratios for later use
                    scaling_ratios[node] = scale

                for wire_info in last_all_wires:
                    line = wire_info['line']
                    color = wire_info['color']
                    d.add(elm.Line().at(line.coords[0]).to(line.coords[1]).color(color))
                    print(f"[DEBUG] Drew wire from {line.coords[0]} to {line.coords[1]} with color '{color}'")
                add_anchor_dots(d, last_all_wires)
                d.draw()

    else:
        # Fixed layout
        print("[DEBUG] Drawing circuit with fixed positions (auto=0)...")
        d = schemdraw.Drawing()
        if draw_grid_or_not == 1:
            draw_grid(d, grid_size=grid_size)  # Add grid lines
        elements = {}
        pins = {}
        component_boxes = {}
        drawn_edges = set()
        all_wires = []

        # Use positions from XML
        pos = positions_in_xml
        directions = directions_in_xml
        flips = flips_in_xml

        for node in G.nodes:
            node_info = G.nodes[node]
            if node not in pos:
                print(f"[WARNING] Node '{node}' 没有定义位置，跳过。")
                continue
            x, y = pos[node]
            direction = directions.get(node, 'right')
            flip_val = flips.get(node, 'none')
            scale = scaling_ratios.get(node, default_scaling_ratios.get(G.nodes[node]['type'], {'vertical_scale':1.0, 'horizontal_scale':1.0}))  # Assign scale
            print(f"[DEBUG] Drawing node/component '{node}' at ({x}, {y}) with direction '{direction}', flip '{flip_val}', and scale '{scale}'")
            draw_component_or_node(d, elements, pins, node, node_info, x, y, direction, component_boxes, G,
                                   flip=flip_val, scaling_ratios=scaling_ratios)  # Pass scaling_ratios
            # Update scaling_ratios for later use
            scaling_ratios[node] = scale

        # Use a deep copy of initial_comp_node_mapping
        pin_mapping_copy = copy.deepcopy(initial_comp_node_mapping)

        draw_connections(d, G, components_element, pins, component_boxes, drawn_edges, routing_method,
                         all_wires, pin_mapping_copy, grid_size=grid_size, min_wire_spacing=min_wire_spacing)

        # Check wire colors and overlaps
        any_red_line = any(wire['color'] == wire_danger_color for wire in all_wires)
        overlapping = check_overlapping_wires(all_wires)

        if any_red_line or overlapping:
            print("[WARNING] 存在红色线条或重叠线条，部分连接可能有问题。")

        print(f"[DEBUG] Drawing wires...")
        for wire_info in all_wires:
            line = wire_info['line']
            color = wire_info['color']
            d.add(elm.Line().at(line.coords[0]).to(line.coords[1]).color(color))
            print(f"[DEBUG] Drew wire from {line.coords[0]} to {line.coords[1]} with color '{color}'")
        add_anchor_dots(d, all_wires)

        # Update positions and directions in XML
        update_xml_positions_and_directions(xml_root, pos, directions, flips, scaling_ratios)
        tree = ET.ElementTree(xml_root)
        tree.write(xml_file)
        d.draw()

    print("[DEBUG] Finished drawing the circuit.")


def main():
    print("[DEBUG] Script started.")
    # Output XML filename
    output_xml = "spice_netlist.xml"

    if auto == 1:
        print("[DEBUG] Converting netlist to XML file...")
        convert_netlist_to_xml_file(netlist, output_xml)
    else:
        print("[DEBUG] Skipping netlist to XML conversion (auto=0). Ensure XML file exists with component positions.")

    try:
        print("[DEBUG] Parsing XML file...")
        tree = ET.parse(output_xml)
    except FileNotFoundError:
        print(f"[ERROR] '{output_xml}' not found. Ensure that the netlist has been converted to XML.")
        sys.exit(1)
    except ET.ParseError as e:
        print(f"[ERROR] Error parsing XML: {e}")
        sys.exit(1)

    xml_root = tree.getroot()
    G, components_element = create_graph_from_xml(xml_root)

    # Build component-node mapping
    initial_comp_node_mapping = build_comp_node_mapping(components_element)

    if auto == 1:
        # Automatic layout
        print("[DEBUG] Starting automatic layout...")
    else:
        # Fixed layout
        print("[DEBUG] Starting fixed layout...")

    # No need to initialize 'd' here as it's handled inside 'draw_circuit'
    # d = schemdraw.Drawing()

    draw_circuit(
        G, components_element, xml_root, output_xml,
        initial_comp_node_mapping,  # Pass initial mapping
        max_attempts=max_attempts,
        EnlargeSize=EnlargeSize,
        routing_method=routing_method,
        auto=auto,
        grid_size=grid_size,  # Add grid_size parameter
        min_wire_spacing=min_wire_spacing  # Add min_wire_spacing parameter
        # default_size_ratio removed as scaling is per-component
    )

    print("[DEBUG] Script finished successfully.")


if __name__ == "__main__":
    main()
