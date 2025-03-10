import networkx as nx
import schemdraw
import schemdraw.elements as elm
import xml.etree.ElementTree as ET
import numpy as np
from shapely.geometry import LineString, Polygon, Point
from shapely.strtree import STRtree
import xml.dom.minidom as minidom

# --------------------------------------------------------------------------------------------
# Below is a comprehensive Python script that:
# 1. Parses a SPICE netlist.
# 2. Converts the parsed data into XML format.
# 3. Uses NetworkX and Schemdraw to attempt to lay out the circuit diagram automatically.
# 4. Tries to route connections between components without overlaps or cross-overs.
# 5. Updates the XML with positional information of components once a suitable layout is found.
#
# This code includes elm.Switch and elm.Bjt in the component_map and handles them similarly
# to other components. The addition of these components does not break any existing functionality.
#
# All code is commented in professional English to aid understanding.
#
# Copy and paste this entire code block into your environment to run.
#
# Requirements:
# - Python 3.x
# - networkx
# - schemdraw
# - shapely
# - lxml or xml.etree (we use xml.etree which is in the standard library)
# --------------------------------------------------------------------------------------------

# Test SPICE netlist used for demonstration:
netlist = """
*Multiple dc sources
v1 1 0 dc 24 
v2 3 0 dc 15 
r1 1 2 10k 
r2 2 3 8.1k 
r3 2 0 4.7k 
.end 


"""

# Adjustable parameters for layout attempts
EnlargeSize = 1.5
max_attempts = 30
routing_method = 1
auto = 0
shrink_size = 0.05

# --------------------------------------------------------------------------------------------
# Component mapping from SPICE element types to Schemdraw elements.
# Now includes 'S' for Switch and 'Q' for Bjt.
# --------------------------------------------------------------------------------------------
component_map = {
    'GND': elm.Ground,
    'R': elm.Resistor,
    'C': elm.Capacitor,
    'L': elm.Inductor2,
    'D': elm.Diode,
    'V': elm.SourceV,
    'I': elm.SourceI,
    'S': elm.Switch,            # Switch element
    'J': elm.JFet,
    'Q': elm.Bjt,               # Bjt element added here
    'E': elm.SourceControlledV,
    'H': elm.SourceControlledV,
    'F': elm.SourceControlledI,
    'G': elm.SourceControlledI,
    'ground': elm.Ground,
    'node': elm.Dot
}

# --------------------------------------------------------------------------------------------
# Function: get_component_bbox
#   Creates a temporary Schemdraw drawing of a component, determines its bounding box, and
#   applies a shrink_size to avoid excessive overlap. It returns both bbox and anchor information.
# --------------------------------------------------------------------------------------------
def get_component_bbox(component_type,
                       pos=(1, 2),
                       direction='right',
                       shrink_size=0.1,
                       **kwargs):
    comp_class = component_map.get(component_type, None)
    if comp_class is None:
        raise ValueError(f"Unsupported component type: {component_type}")

    with schemdraw.Drawing(show=False) as d:
        element = d.add(comp_class(**kwargs).at(pos))
        element.right()

        if 'start' in element.anchors:
            ref_anchor_name = 'start'
        elif 'source' in element.anchors:
            ref_anchor_name = 'source'
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
        xmin_global = xDiff + bbox_local.xmin
        xmax_global = xDiff + bbox_local.xmax
        ymin_global = yDiff + bbox_local.ymin
        ymax_global = yDiff + bbox_local.ymax

        # Adjust bounding box based on component type
        if component_type in ['GND', 'ground']:
            ymax_global -= shrink_size
        elif component_type in ['R', 'C', 'L', 'D', 'V', 'I', 'E', 'H', 'F', 'G', 'S']:
            xmin_global += shrink_size
            xmax_global -= shrink_size
        elif component_type == 'J':
            xmax_global -= shrink_size
            ymin_global += shrink_size
            ymax_global -= shrink_size
        elif component_type == 'Q':
            xmin_global += shrink_size
            ymin_global += shrink_size
            ymax_global -= shrink_size

        anchors_global = {k: (v.x, v.y) for k, v in element.absanchors.items()}
        bbox = {
            'xmin': xmin_global,
            'ymin': ymin_global,
            'xmax': xmax_global,
            'ymax': ymax_global
        }

        return bbox, element.anchors, anchors_global

# --------------------------------------------------------------------------------------------
# Function: parse_spice_netlist
#   Parses a SPICE-format netlist. Extracts components, commands, and subcircuits. Returns lists.
#   Handles various element types, including transistors (J, Q).
# --------------------------------------------------------------------------------------------
def parse_spice_netlist(netlist):
    components = []
    commands = []
    subcircuits = []
    for line in netlist.strip().split('\n'):
        if ';' in line:
            line = line.split(';')[0].strip()
        line = line.strip()
        if line.startswith('*'):
            continue
        if not line.strip():
            continue

        try:
            parts = line.split()
            first_char = parts[0][0].upper()

            if first_char in ['V', 'I', 'R', 'C', 'L', 'D', 'M', 'Q', 'J', 'K', 'S', 'T', 'E', 'F', 'G', 'H']:
                # Parse standard SPICE components
                # Q (Bjt) and J (Jfet) may have more pins.
                if first_char in ['V', 'I', 'S', 'E', 'F', 'G', 'H']:
                    # Typically two-terminal elements
                    value = ' '.join(parts[3:])
                    nodes = parts[1:3]
                elif first_char == 'D':
                    # Diode: Dxxx anode cathode model
                    value = parts[3] if len(parts) > 3 else ''
                    nodes = parts[1:3]
                elif first_char == 'J':
                    # JFET: Jxxx drain gate source [model]
                    if len(parts) < 4:
                        raise ValueError(f"Invalid JFET definition: {line}")
                    value = parts[4] if len(parts) > 4 else ''
                    nodes = parts[1:4]
                    components.append({
                        'type': first_char,
                        'id': parts[0],
                        'nodes': nodes,
                        'value': value,
                        'pins': ['drain', 'gate', 'source']
                    })
                    continue
                elif first_char == 'Q':
                    # BJT: Qxxx collector base emitter [model]
                    if len(parts) < 4:
                        raise ValueError(f"Invalid BJT definition: {line}")
                    value = parts[4] if len(parts) > 4 else ''
                    nodes = parts[1:4]
                    # We'll store pin names for BJT as collector, base, emitter
                    components.append({
                        'type': first_char,
                        'id': parts[0],
                        'nodes': nodes,
                        'value': value,
                        'pins': ['collector', 'base', 'emitter']
                    })
                    continue
                else:
                    # Other 2-pin components like R, C, L
                    if len(parts) < 3:
                        raise ValueError(f"Invalid component definition: {line}")
                    value = parts[3] if len(parts) > 3 else ''
                    nodes = parts[1:3]

                components.append({
                    'type': first_char,
                    'id': parts[0],
                    'nodes': nodes,
                    'value': value
                })
            elif first_char == 'X':
                # Subcircuit call
                if len(parts) < 4:
                    raise ValueError(f"Invalid subcircuit definition: {line}")
                subcircuits.append({
                    'type': 'X',
                    'id': parts[0],
                    'nodes': parts[1:-1],
                    'subckt': parts[-1]
                })
            elif line.startswith('.'):
                # SPICE command line (.tran, .plot, etc.)
                commands.append({'command': parts[0][1:], 'params': parts[1:]})
            else:
                raise ValueError(f"Unknown element or command: {line}")
        except (IndexError, ValueError) as e:
            print(f"Error parsing line: {line}\n{e}")
            continue

    return components, commands, subcircuits

# --------------------------------------------------------------------------------------------
# Function: netlist_to_xml
#   Converts the parsed netlist components, commands, and subcircuits to an XML tree.
# --------------------------------------------------------------------------------------------
def netlist_to_xml(components, commands, subcircuits):
    root = ET.Element("spice_netlist")

    components_elem = ET.SubElement(root, "components")
    for comp in components:
        comp_elem = ET.SubElement(components_elem, "component", type=comp['type'], id=comp['id'])
        for i, node in enumerate(comp['nodes'], start=1):
            ET.SubElement(comp_elem, f"node{i}").text = node
        ET.SubElement(comp_elem, "value").text = comp['value']
        if 'pins' in comp:
            ET.SubElement(comp_elem, "pins").text = ' '.join(comp['pins'])

    subcircuits_elem = ET.SubElement(root, "subcircuits")
    for subckt in subcircuits:
        subckt_elem = ET.SubElement(subcircuits_elem, "subcircuit", id=subckt['id'], subckt=subckt['subckt'])
        for i, node in enumerate(subckt['nodes'], start=1):
            ET.SubElement(subckt_elem, f"node{i}").text = node

    commands_elem = ET.SubElement(root, "commands")
    for cmd in commands:
        cmd_elem = ET.SubElement(commands_elem, "command", name=cmd['command'])
        cmd_elem.text = ' '.join(cmd['params'])

    return root

# --------------------------------------------------------------------------------------------
# Function: pretty_print_xml
#   Formats and returns an XML element as a pretty-printed string.
# --------------------------------------------------------------------------------------------
def pretty_print_xml(element):
    rough_string = ET.tostring(element, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")

# --------------------------------------------------------------------------------------------
# Function: convert_netlist_to_xml_file
#   Parses the netlist, converts to XML, and writes out an XML file.
# --------------------------------------------------------------------------------------------
def convert_netlist_to_xml_file(netlist, output_filename="spice_netlist.xml"):
    components, commands, subcircuits = parse_spice_netlist(netlist)
    xml_root = netlist_to_xml(components, commands, subcircuits)
    pretty_xml = pretty_print_xml(xml_root)
    with open(output_filename, "w") as f:
        f.write(pretty_xml)
    print(pretty_xml)

# --------------------------------------------------------------------------------------------
# Function: create_graph_from_xml
#   Creates a NetworkX graph from the XML. Components and nodes become graph nodes. Edges
#   represent connections between components and their nodes.
# --------------------------------------------------------------------------------------------
def create_graph_from_xml(xml_root):
    components_element = xml_root.find('components')
    if components_element is None or len(components_element) == 0:
        print("No components found in XML.")
        raise ValueError("No components found in XML.")

    G = nx.Graph()

    for component in components_element:
        ctype = component.attrib['type']
        cid = component.attrib['id']
        value = component.find('value').text
        nodes = []
        i = 1
        while component.find(f'node{i}') is not None:
            nodes.append(component.find(f'node{i}').text)
            i += 1

        G.add_node(cid, type=ctype, value=value)
        for node in nodes:
            if node == '0':
                G.add_node(node, type='ground')
            else:
                G.add_node(node, type='node')
            G.add_edge(cid, node)

    return G, components_element

# --------------------------------------------------------------------------------------------
# Function: update_xml_positions
#   Updates the XML with the computed positions (x,y) of components/nodes.
# --------------------------------------------------------------------------------------------
def update_xml_positions(xml_root, pos):
    components_elem = xml_root.find('components')
    for comp in components_elem:
        cid = comp.attrib['id']
        if cid in pos:
            x, y = pos[cid]
            comp.attrib['x'] = str(x)
            comp.attrib['y'] = str(y)

    nodes_elem = xml_root.find('nodes')
    if nodes_elem is None:
        nodes_elem = ET.SubElement(xml_root, 'nodes')
    for nid in pos:
        if any(comp.attrib['id'] == nid for comp in components_elem):
            continue
        node_elem = next((node for node in nodes_elem if node.attrib['id'] == nid), None)
        if node_elem is None:
            node_elem = ET.SubElement(nodes_elem, 'node', id=nid)
        x, y = pos[nid]
        node_elem.attrib['x'] = str(x)
        node_elem.attrib['y'] = str(y)

# --------------------------------------------------------------------------------------------
# Function: add_anchor_dots
#   Adds dots to indicate connection points where 3 or more wires meet.
# --------------------------------------------------------------------------------------------
def add_anchor_dots(d, all_wires):
    anchor_wire_count = {}
    for w in all_wires:
        line = w['line']
        start_pt = line.coords[0]
        end_pt = line.coords[1]
        anchor_wire_count[start_pt] = anchor_wire_count.get(start_pt, 0) + 1
        anchor_wire_count[end_pt] = anchor_wire_count.get(end_pt, 0) + 1

    added_dots = set()
    for anchor_pos, count in anchor_wire_count.items():
        if count >= 3 and anchor_pos not in added_dots:
            d.add(elm.Dot(radius=0.12).at(anchor_pos))
            added_dots.add(anchor_pos)

# --------------------------------------------------------------------------------------------
# Function: draw_circuit
#   Attempts to layout and draw the circuit using Schemdraw and NetworkX.
#   If auto=1, attempts automatic layout multiple times.
#   If auto=0, uses fixed positions from XML and tries to route wires multiple times.
# --------------------------------------------------------------------------------------------
def draw_circuit(G, components_element, xml_root, xml_file, max_attempts=100, EnlargeSize=2.5, routing_method=1, auto=1):
    attempt = 0
    success = False

    last_pos = None
    last_all_wires = None

    positions_in_xml = {}
    components_elem = xml_root.find('components')
    if components_elem is not None:
        for comp in components_elem:
            cid = comp.attrib['id']
            if 'x' in comp.attrib and 'y' in comp.attrib:
                positions_in_xml[cid] = (float(comp.attrib['x']), float(comp.attrib['y']))
    nodes_elem = xml_root.find('nodes')
    if nodes_elem is not None:
        for node in nodes_elem:
            nid = node.attrib['id']
            if 'x' in node.attrib and 'y' in node.attrib:
                positions_in_xml[nid] = (float(node.attrib['x']), float(node.attrib['y']))

    if auto == 1:
        # Automatic layout mode
        while attempt < max_attempts:
            attempt += 1
            print(f"Attempt {attempt} (auto=1, automatic layout)")

            pos = nx.spring_layout(G, scale=EnlargeSize)

            d = schemdraw.Drawing()
            elements = {}
            pins = {}
            component_boxes = {}
            drawn_edges = set()
            all_wires = []

            for node in G.nodes:
                node_info = G.nodes[node]
                x, y = pos[node]
                x, y = (EnlargeSize * x, EnlargeSize * -y)
                if np.isnan(x) or np.isnan(y) or np.isinf(x) or np.isinf(y):
                    print(f"Node {node} has invalid position: ({x}, {y})")
                    continue
                draw_component_or_node(d, elements, pins, node, node_info, x, y, component_boxes, G)

            component_polygons = [comp_info['polygon'] for comp_info in component_boxes.values()]
            spatial_index = STRtree(component_polygons)

            draw_connections(d, G, components_element, pins, component_boxes, drawn_edges, routing_method, spatial_index, all_wires)

            any_red_line = any(wire['color'] == 'red' for wire in all_wires)
            overlapping = check_overlapping_wires(all_wires)

            last_pos = pos
            last_all_wires = all_wires

            if any_red_line or overlapping:
                print(f"Overlap or red lines detected in attempt {attempt}. Retrying...")
                continue
            else:
                for wire_info in all_wires:
                    line = wire_info['line']
                    color = wire_info['color']
                    d.add(elm.Line().at(line.coords[0]).to(line.coords[1]).color(color))
                add_anchor_dots(d, all_wires)
                print(f"Successful layout found after {attempt} attempts.")
                success = True
                update_xml_positions(xml_root, pos)
                tree = ET.ElementTree(xml_root)
                tree.write(xml_file)
                d.draw()
                break

        if not success:
            print(f"No suitable layout found after {max_attempts} attempts. Showing the last attempt with issues.")
            d = schemdraw.Drawing()
            if last_pos is not None and last_all_wires is not None:
                update_xml_positions(xml_root, last_pos)
                tree = ET.ElementTree(xml_root)
                tree.write(xml_file)

                elements = {}
                pins = {}
                component_boxes = {}
                drawn_edges = set()
                for node in G.nodes:
                    node_info = G.nodes[node]
                    x, y = last_pos[node]
                    x, y = (EnlargeSize * x, EnlargeSize * -y)
                    draw_component_or_node(d, elements, pins, node, node_info, x, y, component_boxes, G)

                for wire_info in last_all_wires:
                    line = wire_info['line']
                    color = wire_info['color']
                    d.add(elm.Line().at(line.coords[0]).to(line.coords[1]).color(color))

                add_anchor_dots(d, last_all_wires)
                d.draw()
    else:
        # auto=0: use fixed positions from XML
        if positions_in_xml:
            pos = positions_in_xml
            attempt = 0
            success = False
            last_all_wires = None
            while attempt < max_attempts:
                attempt += 1
                print(f"Attempt {attempt} (auto=0, fixed positions)")

                d = schemdraw.Drawing()
                elements = {}
                pins = {}
                component_boxes = {}
                drawn_edges = set()
                all_wires = []

                for node in G.nodes:
                    node_info = G.nodes[node]
                    if node not in pos:
                        print(f"No position for node {node}")
                        continue
                    x, y = (EnlargeSize * pos[node][0], EnlargeSize * -pos[node][1])
                    if np.isnan(x) or np.isnan(y) or np.isinf(x) or np.isinf(y):
                        print(f"Node {node} invalid: ({x}, {y})")
                        continue
                    draw_component_or_node(d, elements, pins, node, node_info, x, y, component_boxes, G)

                component_polygons = [comp_info['polygon'] for comp_info in component_boxes.values()]
                spatial_index = STRtree(component_polygons)

                draw_connections(d, G, components_element, pins, component_boxes, drawn_edges, routing_method, spatial_index, all_wires)

                any_red_line = any(wire['color'] == 'red' for wire in all_wires)
                overlapping = check_overlapping_wires(all_wires)

                last_all_wires = all_wires

                if any_red_line or overlapping:
                    print(f"Overlap or red lines detected in attempt {attempt}. Retrying...")
                    continue
                else:
                    for wire_info in all_wires:
                        line = wire_info['line']
                        color = wire_info.get('color', 'black')
                        d.add(elm.Line().at(line.coords[0]).to(line.coords[1]).color(color))
                    add_anchor_dots(d, all_wires)
                    print(f"Successful routing found after {attempt} attempts with fixed positions.")
                    success = True
                    tree = ET.ElementTree(xml_root)
                    tree.write(xml_file)
                    d.draw()
                    break

            if not success:
                print(f"No suitable routing after {max_attempts} attempts with fixed positions. Showing last attempt with issues.")
                d = schemdraw.Drawing()
                elements = {}
                pins = {}
                component_boxes = {}
                drawn_edges = set()
                for node in G.nodes:
                    node_info = G.nodes[node]
                    if node not in pos:
                        continue
                    x, y = (EnlargeSize * pos[node][0], EnlargeSize * -pos[node][1])
                    if np.isnan(x) or np.isnan(y) or np.isinf(x) or np.isinf(y):
                        continue
                    draw_component_or_node(d, elements, pins, node, node_info, x, y, component_boxes, G)

                if last_all_wires is not None:
                    for wire_info in last_all_wires:
                        line = wire_info['line']
                        color = wire_info.get('color', 'black')
                        d.add(elm.Line().at(line.coords[0]).to(line.coords[1]).color(color))

                add_anchor_dots(d, last_all_wires or [])
                tree = ET.ElementTree(xml_root)
                tree.write(xml_file)
                d.draw()
        else:
            print("No positions found in XML. Please run with auto=1 first.")

# --------------------------------------------------------------------------------------------
# Function: draw_component_or_node
#   Draws a single component or node at the specified coordinates.
#   Applies the appropriate schemdraw element based on the node type, including BJTs.
# --------------------------------------------------------------------------------------------
def draw_component_or_node(d, elements, pins, node, node_info, x, y, component_boxes, G):
    ctype = node_info.get('type')
    if ctype is None:
        ctype = 'node'

    deg = G.degree(node)  # Get the number of connections

    if ctype == 'node':
        # For a simple node, if degree >=3, show a Dot, else just a label
        if deg >= 3:
            elements[node] = d.add(elm.Dot().at((x, y)))
            d.add(elm.Label().at((x, y)).label(node, ofst=0.2))
            pins[node] = {'pin': (x, y)}
        else:
            d.add(elm.Label().at((x, y)).label(node, ofst=0.2))
            pins[node] = {'pin': (x, y)}
        return

    if ctype == 'ground':
        ctype_for_bbox = 'GND'
    else:
        ctype_for_bbox = ctype

    bbox, anchors_local, anchors_global = get_component_bbox(ctype_for_bbox, pos=(x, y), direction='right', shrink_size=shrink_size)
    label_text = f"{node}\n{node_info.get('value','')}"

    # Handle controlled sources, switches, BJTs similarly
    if ctype == 'E' or ctype == 'H':
        # Controlled voltage source
        elements[node] = d.add(elm.SourceControlledV().at((x, y)).right().label(label_text))
        pins[node] = {'start': elements[node].absanchors['end'],'end': elements[node].absanchors['start']}
    elif ctype == 'F' or ctype == 'G':
        # Controlled current source
        elements[node] = d.add(elm.SourceControlledI().at((x, y)).right().label(label_text))
        pins[node] = {'start': elements[node].absanchors['end'],'end': elements[node].absanchors['start']}
    elif ctype == 'R':
        elements[node] = d.add(elm.Resistor().at((x, y)).right().label(label_text))
        pins[node] = {'start': elements[node].absanchors['start'], 'end': elements[node].absanchors['end']}
    elif ctype == 'C':
        elements[node] = d.add(elm.Capacitor().at((x, y)).right().label(label_text))
        pins[node] = {'start': elements[node].absanchors['start'], 'end': elements[node].absanchors['end']}
    elif ctype == 'L':
        elements[node] = d.add(elm.Inductor2(loops=3).at((x, y)).right().label(label_text))
        pins[node] = {'start': elements[node].absanchors['start'], 'end': elements[node].absanchors['end']}
    elif ctype == 'V':
        elements[node] = d.add(elm.SourceV().at((x, y)).right().label(label_text))
        pins[node] = {'positive': elements[node].absanchors['end'], 'negative': elements[node].absanchors['start']}
    elif ctype == 'I':
        elements[node] = d.add(elm.SourceI().at((x, y)).right().label(label_text))
        pins[node] = {'positive': elements[node].absanchors['end'], 'negative': elements[node].absanchors['start']}
    elif ctype == 'D':
        elements[node] = d.add(elm.Diode().at((x, y)).right().label(label_text))
        pins[node] = {'anode': elements[node].absanchors['start'], 'cathode': elements[node].absanchors['end']}
    elif ctype == 'J':
        # JFET has drain, gate, source anchors
        elements[node] = d.add(elm.JFet().at((x, y)).right().label(label_text))
        pins[node] = {
            'drain': elements[node].absanchors['drain'],
            'gate': elements[node].absanchors['gate'],
            'source': elements[node].absanchors['source']
        }
    elif ctype == 'Q':
        # BJT has collector, base, emitter anchors
        # Schemdraw's Bjt typically defines these anchors by names: 'collector', 'base', 'emitter'
        # We'll map them accordingly.
        elements[node] = d.add(elm.Bjt().at((x, y)).right().label(label_text))
        pins[node] = {
            'collector': elements[node].absanchors['collector'],
            'base': elements[node].absanchors['base'],
            'emitter': elements[node].absanchors['emitter']
        }
    elif ctype == 'ground':
        elements[node] = d.add(elm.Ground().at((x, y)))
        pins[node] = {'pin': (x, y)}
    elif ctype == 'S':
        # Switch element
        elements[node] = d.add(elm.Switch().at((x, y)).right().label(label_text))
        pins[node] = {'start': elements[node].absanchors['start'], 'end': elements[node].absanchors['end']}
    else:
        # For unrecognized component type, draw a Dot as a fallback
        elements[node] = d.add(elm.Dot().at((x, y)).right().label(label_text))
        pins[node] = {'pin': (x, y)}

    x_min, y_min, x_max, y_max = bbox['xmin'], bbox['ymin'], bbox['xmax'], bbox['ymax']
    component_boxes[node] = {
        'bbox': bbox,
        'polygon': Polygon([(x_min, y_min), (x_max, y_min),(x_max, y_max),(x_min, y_max)]),
        'type': ctype
    }

# --------------------------------------------------------------------------------------------
# Function: check_overlapping_wires
#   Checks if any wires are overlapping. If so, marks them red.
# --------------------------------------------------------------------------------------------
def check_overlapping_wires(all_wires):
    overlapping = False
    num_wires = len(all_wires)
    for i in range(num_wires):
        line_i = all_wires[i]['line']
        for j in range(i + 1, num_wires):
            line_j = all_wires[j]['line']
            if line_i.equals(line_j) or line_i.contains(line_j) or line_j.contains(line_i):
                overlapping = True
                all_wires[i]['color'] = 'red'
                all_wires[j]['color'] = 'red'
    return overlapping

# --------------------------------------------------------------------------------------------
# Function: draw_connections
#   Draws connections (wires) between components and nodes in the graph.
#   Attempts a simple routing method (L-shaped paths) and checks for intersections.
# --------------------------------------------------------------------------------------------
def draw_connections(d, G, components_element, pins, component_boxes, drawn_edges, routing_method, spatial_index, all_wires):
    for comp, neighbor in G.edges():
        edge_key = tuple(sorted([comp, neighbor]))
        if edge_key in drawn_edges:
            continue

        comp_info = G.nodes[comp]
        neighbor_info = G.nodes[neighbor]

        comp_pin = get_component_pin(comp, neighbor, comp_info, components_element, pins)
        neighbor_pin = get_component_pin(neighbor, comp, neighbor_info, components_element, pins)

        if comp_pin is None or neighbor_pin is None:
            continue

        start_pos = get_pin_position(pins, comp, comp_pin)
        end_pos = get_pin_position(pins, neighbor, neighbor_pin)

        if start_pos is None or end_pos is None:
            continue

        selected_path, line_color = route_connection_current_method(
            start_pos, end_pos, component_boxes, comp, neighbor, all_wires
        )

        for i in range(len(selected_path) - 1):
            segment_start = selected_path[i]
            segment_end = selected_path[i + 1]
            new_line = LineString([segment_start, segment_end])
            all_wires.append({'line': new_line,'color': line_color})

        drawn_edges.add(edge_key)

# --------------------------------------------------------------------------------------------
# Function: route_connection_current_method
#   Attempts a simple L-shaped routing. If the first path is blocked, tries the second path.
# --------------------------------------------------------------------------------------------
def route_connection_current_method(start_pos, end_pos, component_boxes, comp, neighbor, all_wires):
    path1 = [start_pos, (end_pos[0], start_pos[1]), end_pos]
    path2 = [start_pos, (start_pos[0], end_pos[1]), end_pos]

    def path_ok(path):
        for i in range(len(path)-1):
            if is_wire_crossing_components(path[i], path[i+1], component_boxes, comp, neighbor):
                return False
        for i in range(len(path)-1):
            seg = LineString([path[i], path[i+1]])
            for w in all_wires:
                lw = w['line']
                if seg.equals(lw) or seg.contains(lw) or lw.contains(seg):
                    return False
        return True

    if path_ok(path1):
        return path1, "black"
    if path_ok(path2):
        return path2, "black"

    return path1, "red"

# --------------------------------------------------------------------------------------------
# Function: is_wire_crossing_components
#   Checks if a wire segment crosses any component polygons.
# --------------------------------------------------------------------------------------------
def is_wire_crossing_components(start_pos, end_pos, component_boxes, comp, neighbor):
    line = LineString([start_pos, end_pos])
    line_start_point = Point(start_pos)
    line_end_point = Point(end_pos)
    for comp_id, comp_info in component_boxes.items():
        polygon = comp_info['polygon']
        if line.intersects(polygon):
            intersection = line.intersection(polygon)
            # Allow touching at the starting or ending component itself
            if (intersection.equals(line_start_point) or intersection.equals(line_end_point)) and (comp_id == comp or comp_id == neighbor):
                continue
            else:
                return True
    return False

# --------------------------------------------------------------------------------------------
# Function: get_node_coordinate
#   Retrieves a node's coordinate from the pins dictionary.
# --------------------------------------------------------------------------------------------
def get_node_coordinate(pins, node):
    if node in pins and 'pin' in pins[node]:
        return pins[node]['pin']
    elif node in pins and 'positive' in pins[node]:
        return pins[node]['positive']
    elif node in pins and 'negative' in pins[node]:
        return pins[node]['negative']
    elif node in pins and 'anode' in pins[node]:
        return pins[node]['anode']
    elif node in pins and 'cathode' in pins[node]:
        return pins[node]['cathode']
    elif node in pins and 'drain' in pins[node]:
        return pins[node]['drain']
    elif node in pins and 'gate' in pins[node]:
        return pins[node]['gate']
    elif node in pins and 'source' in pins[node]:
        return pins[node]['source']
    elif node in pins and 'collector' in pins[node]:
        return pins[node]['collector']
    elif node in pins and 'base' in pins[node]:
        return pins[node]['base']
    elif node in pins and 'emitter' in pins[node]:
        return pins[node]['emitter']
    else:
        return None

# --------------------------------------------------------------------------------------------
# Function: get_component_pin
#   Determines which pin of the component corresponds to the neighbor node.
#   This is necessary for directional components or those with multiple pin types (e.g., BJT).
# --------------------------------------------------------------------------------------------
def get_component_pin(comp, neighbor, comp_info, components_element, pins):
    ctype = comp_info.get('type')
    # For generic 2-pin devices (R, C, L, S), we use shortest distance mapping as before.
    if ctype in ['R', 'C', 'L', 'S']:
        comp_pins = ['start', 'end']
        comp_pin_coords = {pin: pins[comp][pin] for pin in comp_pins}

        component = next(c for c in components_element if c.attrib['id'] == comp)
        node1 = component.find('node1').text
        node2 = component.find('node2').text
        nodes = [node1, node2]

        node_coords = {}
        for node in nodes:
            node_coord = get_node_coordinate(pins, node)
            if node_coord:
                node_coords[node] = node_coord

        if node1 not in node_coords or node2 not in node_coords:
            return None  # Cannot determine

        distance1 = np.linalg.norm(np.array(comp_pin_coords['start']) - np.array(node_coords[node1])) + \
                    np.linalg.norm(np.array(comp_pin_coords['end']) - np.array(node_coords[node2]))
        distance2 = np.linalg.norm(np.array(comp_pin_coords['start']) - np.array(node_coords[node2])) + \
                    np.linalg.norm(np.array(comp_pin_coords['end']) - np.array(node_coords[node1]))

        if distance1 <= distance2:
            pin_node_pairs = [('start', node1), ('end', node2)]
        else:
            pin_node_pairs = [('start', node2), ('end', node1)]

        for pin, node in pin_node_pairs:
            if node == neighbor:
                return pin

    elif ctype in ['V', 'I']:
        # Voltage/Current source: node1 -> positive, node2 -> negative
        component = next(c for c in components_element if c.attrib['id'] == comp)
        node1 = component.find('node1').text
        node2 = component.find('node2').text
        if neighbor == node1:
            return 'positive'
        elif neighbor == node2:
            return 'negative'

    elif ctype == 'D':
        # Diode: node1 -> anode, node2 -> cathode
        component = next(c for c in components_element if c.attrib['id'] == comp)
        node1 = component.find('node1').text
        node2 = component.find('node2').text
        if neighbor == node1:
            return 'anode'
        elif neighbor == node2:
            return 'cathode'

    elif ctype == 'J':
        # JFET: node1=drain, node2=gate, node3=source
        component = next(c for c in components_element if c.attrib['id'] == comp)
        node_drain = component.find('node1').text
        node_gate = component.find('node2').text
        node_source = component.find('node3').text
        node_mapping = {node_drain: 'drain', node_gate: 'gate', node_source: 'source'}
        if neighbor in node_mapping:
            return node_mapping[neighbor]

    elif ctype == 'Q':
        # BJT: node1=collector, node2=base, node3=emitter
        component = next(c for c in components_element if c.attrib['id'] == comp)
        node_collector = component.find('node1').text
        node_base = component.find('node2').text
        node_emitter = component.find('node3').text
        node_mapping = {node_collector: 'collector', node_base: 'base', node_emitter: 'emitter'}
        if neighbor in node_mapping:
            return node_mapping[neighbor]

    elif ctype in ['E', 'H', 'F', 'G']:
        # Controlled sources (2-pin): start and end mapping
        if 'start' in pins[comp] and 'end' in pins[comp]:
            component = next(c for c in components_element if c.attrib['id'] == comp)
            node1 = component.find('node1').text
            node2 = component.find('node2').text
            if neighbor == node1:
                return 'start'
            elif neighbor == node2:
                return 'end'
    else:
        # For any unknown type, return 'pin'
        return 'pin'

    return None

# --------------------------------------------------------------------------------------------
# Function: get_pin_position
#   Extracts the pin position from the pins dictionary.
# --------------------------------------------------------------------------------------------
def get_pin_position(pins, comp, pin_name):
    try:
        pos = pins[comp][pin_name]
        if isinstance(pos, schemdraw.util.Point):
            return (pos.x, pos.y)
        else:
            return pos
    except KeyError:
        print(f"Pin {pin_name} not found for component {comp}")
        return None

# --------------------------------------------------------------------------------------------
# Main block to run if executed as a script
# --------------------------------------------------------------------------------------------
if __name__ == "__main__":
    if auto == 1:
        convert_netlist_to_xml_file(netlist, "spice_netlist.xml")

    tree = ET.parse("spice_netlist.xml")
    xml_root = tree.getroot()
    G, components_element = create_graph_from_xml(xml_root)

    draw_circuit(
        G, components_element, xml_root, "spice_netlist.xml",
        max_attempts=max_attempts,
        EnlargeSize=EnlargeSize,
        routing_method=routing_method,
        auto=auto
    )
