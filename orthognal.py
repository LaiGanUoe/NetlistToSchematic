import networkx as nx
import schemdraw
import schemdraw.elements as elm
import xml.etree.ElementTree as ET
import numpy as np
from shapely.geometry import LineString, Polygon, Point, box
from shapely.strtree import STRtree
import shapely.ops
import xml.dom.minidom as minidom
import subprocess
import os

# Define the SPICE netlist, including a JFET
netlist = """
* JFET Constant Current Source
VDD 1 0 DC 15        ; Power supply voltage 15V
J1 1 0 2 JFET_N      ; N-type JFET, drain connected to power supply, gate and source shorted
R1 2 0 10            ; Load resistor 10 Ohms
.END

"""

# Parameters
EnlargeSize = 2
shrink_ratio = 0.08  # Default shrink_ratio
max_attempts = 300
routing_method = 1  # 1: Current method, 2: A* algorithm
auto = 1  # Set to 1 for automatic layout, 0 to read positions from XML

# Shrink ratios per component type
shrink_ratios = {
    'R': 0.08,    # Resistor
    'C': 0.08,    # Capacitor
    'L': 0.08,    # Inductor
    'V': 0.08,    # Voltage source
    'I': 0.08,    # Current source
    'D': 0.08,    # Diode
    'J': 0.168,   # JFET
    'ground': 0.08,
    'node': 0.08
    # Add other component types if needed
}

def parse_spice_netlist(netlist):
    # ... (Your existing parse_spice_netlist function)
    components = []
    commands = []
    subcircuits = []
    for line in netlist.strip().split('\n'):
        # Remove comments starting with ';'
        if ';' in line:
            line = line.split(';')[0].strip()
        line = line.strip()
        # Skip comments starting with '*'
        if line.startswith('*'):
            continue
        # Skip empty lines
        if not line.strip():
            continue

        try:
            parts = line.split()
            first_char = parts[0][0].upper()

            # Parse different types of components based on the first character
            if first_char in ['V', 'I', 'R', 'C', 'L', 'D', 'M', 'Q', 'J', 'K', 'S', 'T', 'E', 'F', 'G', 'H']:
                if first_char == 'V':
                    value = ' '.join(parts[3:])  # Voltage source
                    nodes = parts[1:3]
                elif first_char == 'I':
                    value = ' '.join(parts[3:])  # Current source
                    nodes = parts[1:3]
                elif first_char == 'D':
                    value = parts[3] if len(parts) > 3 else ''
                    nodes = parts[1:3]
                elif first_char == 'J':
                    if len(parts) < 4:
                        raise ValueError(f"Invalid JFET definition: {line}")
                    value = parts[4] if len(parts) > 4 else ''
                    nodes = parts[1:4]  # Drain, Gate, Source
                    components.append({
                        'type': first_char,
                        'id': parts[0],
                        'nodes': nodes,
                        'value': value,
                        'pins': ['drain', 'gate', 'source']
                    })
                    continue  # Skip the rest of the loop
                else:
                    value = parts[3]
                    nodes = parts[1:3]

                if len(parts) < 3:
                    raise ValueError(f"Invalid component definition: {line}")

                components.append({
                    'type': first_char,
                    'id': parts[0],
                    'nodes': nodes,
                    'value': value
                })
            elif first_char == 'X':
                # Handle subcircuits
                if len(parts) < 4:
                    raise ValueError(f"Invalid subcircuit definition: {line}")
                subcircuits.append({
                    'type': 'X',
                    'id': parts[0],
                    'nodes': parts[1:-1],  # Subcircuit nodes
                    'subckt': parts[-1]    # Subcircuit name
                })
            # Parse commands
            elif line.startswith('.'):
                commands.append({'command': parts[0][1:], 'params': parts[1:]})
            else:
                raise ValueError(f"Unknown element or command: {line}")
        except (IndexError, ValueError) as e:
            print(f"Error parsing line: {line}\n{e}")
            continue

    return components, commands, subcircuits

def netlist_to_xml(components, commands, subcircuits):
    # ... (Your existing netlist_to_xml function)
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

def pretty_print_xml(element):
    # ... (Your existing pretty_print_xml function)
    rough_string = ET.tostring(element, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")

def convert_netlist_to_xml_file(netlist, output_filename="spice_netlist.xml"):
    # ... (Your existing convert_netlist_to_xml_file function)
    components, commands, subcircuits = parse_spice_netlist(netlist)
    xml_root = netlist_to_xml(components, commands, subcircuits)
    pretty_xml = pretty_print_xml(xml_root)
    with open(output_filename, "w") as f:
        f.write(pretty_xml)
    print(pretty_xml)

def create_graph_from_xml(xml_root):
    # ... (Your existing create_graph_from_xml function)
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

        # Add component node
        G.add_node(cid, type=ctype, value=value)
        # Add connected nodes
        for node in nodes:
            if node == '0':
                G.add_node(node, type='ground')  # Treat ground as a special component
            else:
                G.add_node(node, type='node')    # Regular node
            # Add edge
            G.add_edge(cid, node)

    return G, components_element

def update_xml_positions(xml_root, pos):
    # ... (Your existing update_xml_positions function)
    components_elem = xml_root.find('components')
    for comp in components_elem:
        cid = comp.attrib['id']
        if cid in pos:
            x, y = pos[cid]
            comp.attrib['x'] = str(x)
            comp.attrib['y'] = str(y)

    # Add nodes to XML if not present
    nodes_elem = xml_root.find('nodes')
    if nodes_elem is None:
        nodes_elem = ET.SubElement(xml_root, 'nodes')
    for nid in pos:
        if any(comp.attrib['id'] == nid for comp in components_elem):
            continue  # Skip components
        node_elem = next((node for node in nodes_elem if node.attrib['id'] == nid), None)
        if node_elem is None:
            node_elem = ET.SubElement(nodes_elem, 'node', id=nid)
        x, y = pos[nid]
        node_elem.attrib['x'] = str(x)
        node_elem.attrib['y'] = str(y)

def draw_circuit(G, components_element, xml_root, xml_file, max_attempts=100, EnlargeSize=2.5, shrink_ratio=0.1, routing_method=1, auto=1):
    # ... (Your existing draw_circuit function)
    attempt = 0
    success = False
    all_wires = []  # Initialize all_wires
    pos = {}        # Initialize pos

    positions_in_xml = {}
    components_elem = xml_root.find('components')
    for comp in components_elem:
        cid = comp.attrib['id']
        if 'x' in comp.attrib and 'y' in comp.attrib:
            positions_in_xml[cid] = (float(comp.attrib['x']), float(comp.attrib['y']))
    nodes_elem = xml_root.find('nodes')
    if nodes_elem:
        for node in nodes_elem:
            nid = node.attrib['id']
            if 'x' in node.attrib and 'y' in node.attrib:
                positions_in_xml[nid] = (float(node.attrib['x']), float(node.attrib['y']))

    if auto == 1:
        # Automatic layout using tsmpy
        while attempt < max_attempts:
            attempt += 1
            print(f"Attempt {attempt}")

            # Generate orthogonal layout using tsmpy
            pos = generate_tsmpy_layout(G)

            if pos is None:
                print("Failed to generate layout with tsmpy.")
                continue

            d = schemdraw.Drawing()
            elements = {}
            pins = {}
            component_boxes = {}
            drawn_edges = set()
            all_wires = []

            # Draw components and nodes
            for node in G.nodes:
                node_info = G.nodes[node]
                x, y = pos[node]
                x, y = (EnlargeSize * x, EnlargeSize * -y)

                if np.isnan(x) or np.isnan(y) or np.isinf(x) or np.isinf(y):
                    print(f"Node {node} has invalid transformed position: ({x}, {y})")
                    continue  # Skip problematic nodes

                draw_component_or_node(d, elements, pins, node, node_info, x, y)

                # Get component bounding box
                if node in elements and hasattr(elements[node], 'get_bbox') and node_info.get('type') != 'node':
                    bbox = elements[node].get_bbox(transform=True)
                    x_min, y_min, x_max, y_max = bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax
                    # Check if bounding box is valid
                    if np.isnan([x_min, y_min, x_max, y_max]).any() or np.isinf([x_min, y_min, x_max, y_max]).any():
                        print(f"Element {node} has invalid bbox: {bbox}")
                        continue  # Skip problematic components

                    # Get shrink_ratio for the component type
                    comp_type = node_info.get('type')
                    comp_shrink_ratio = shrink_ratios.get(comp_type, shrink_ratio)  # Use type-specific shrink_ratio

                    # Calculate shrunken bounding box
                    width = x_max - x_min
                    height = y_max - y_min
                    x_offset = width * comp_shrink_ratio / 2
                    y_offset = height * comp_shrink_ratio / 2
                    shrinked_bbox = (
                        x_min + x_offset,
                        y_min + y_offset,
                        x_max - x_offset,
                        y_max - y_offset
                    )

                    component_boxes[node] = {
                        'bbox': bbox,
                        'shrinked_bbox': shrinked_bbox,
                        'polygon': Polygon([
                            (shrinked_bbox[0], shrinked_bbox[1]),
                            (shrinked_bbox[2], shrinked_bbox[1]),
                            (shrinked_bbox[2], shrinked_bbox[3]),
                            (shrinked_bbox[0], shrinked_bbox[3])
                        ]),
                        'type': node_info.get('type')  # Add component type
                    }
                else:
                    # For nodes without bounding boxes
                    pass

            # Build spatial index for components (used in A* algorithm)
            component_polygons = [comp_info['polygon'] for comp_info in component_boxes.values()]
            spatial_index = STRtree(component_polygons)

            # Draw connections but do not immediately render them
            draw_connections(
                d, G, components_element, pins, component_boxes, drawn_edges, routing_method, spatial_index, all_wires
            )

            # Check for overlapping wires
            overlapping = check_overlapping_wires(all_wires)

            if overlapping:
                print(f"Overlap detected in attempt {attempt}. Retrying...")
                continue  # Retry without rendering the circuit
            else:
                # Draw all wires onto the diagram
                for wire_info in all_wires:
                    line = wire_info['line']
                    color = wire_info['color']
                    d.add(elm.Line().at(line.coords[0]).to(line.coords[1]).color(color))
                print(f"Successful layout found after {attempt} attempts.")
                success = True
                # Update positions in XML
                update_xml_positions(xml_root, pos)
                # Save updated XML
                tree = ET.ElementTree(xml_root)
                tree.write(xml_file)
                break  # Successful layout found

        if not success:
            # Reached max attempts, draw the circuit with overlaps highlighted
            print(f"No collision-free layout found after {max_attempts} attempts. Showing the last attempt with overlaps.")
            if not all_wires:
                print("No wires to draw.")
                return
            # Draw all wires onto the diagram
            for wire_info in all_wires:
                line = wire_info['line']
                color = wire_info.get('color', 'black')
                d.add(elm.Line().at(line.coords[0]).to(line.coords[1]).color(color))
            # Update positions in XML
            update_xml_positions(xml_root, pos)
            # Save updated XML
            tree = ET.ElementTree(xml_root)
            tree.write(xml_file)
    else:
        # Use positions from XML
        if positions_in_xml:
            pos = positions_in_xml
            print("Using positions from XML.")
            d = schemdraw.Drawing()
            elements = {}
            pins = {}
            component_boxes = {}
            drawn_edges = set()
            all_wires = []

            # Draw components and nodes
            for node in G.nodes:
                node_info = G.nodes[node]
                if node in pos:
                    x, y = (EnlargeSize * pos[node][0], EnlargeSize * -pos[node][1])
                else:
                    print(f"No position found for node {node}")
                    continue

                if np.isnan(x) or np.isnan(y) or np.isinf(x) or np.isinf(y):
                    print(f"Node {node} has invalid transformed position: ({x}, {y})")
                    continue  # Skip problematic nodes

                draw_component_or_node(d, elements, pins, node, node_info, x, y)

                # Get component bounding box
                if node in elements and hasattr(elements[node], 'get_bbox') and node_info.get('type') != 'node':
                    bbox = elements[node].get_bbox(transform=True)
                    x_min, y_min, x_max, y_max = bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax
                    # Check if bounding box is valid
                    if np.isnan([x_min, y_min, x_max, y_max]).any() or np.isinf([x_min, y_min, x_max, y_max]).any():
                        print(f"Element {node} has invalid bbox: {bbox}")
                        continue  # Skip problematic components

                    # Get shrink_ratio for the component type
                    comp_type = node_info.get('type')
                    comp_shrink_ratio = shrink_ratios.get(comp_type, shrink_ratio)  # Use type-specific shrink_ratio

                    # Calculate shrunken bounding box
                    width = x_max - x_min
                    height = y_max - y_min
                    x_offset = width * comp_shrink_ratio / 2
                    y_offset = height * comp_shrink_ratio / 2
                    shrinked_bbox = (
                        x_min + x_offset,
                        y_min + y_offset,
                        x_max - x_offset,
                        y_max - y_offset
                    )

                    component_boxes[node] = {
                        'bbox': bbox,
                        'shrinked_bbox': shrinked_bbox,
                        'polygon': Polygon([
                            (shrinked_bbox[0], shrinked_bbox[1]),
                            (shrinked_bbox[2], shrinked_bbox[1]),
                            (shrinked_bbox[2], shrinked_bbox[3]),
                            (shrinked_bbox[0], shrinked_bbox[3])
                        ]),
                        'type': node_info.get('type')  # Add component type
                    }
                else:
                    # For nodes without bounding boxes
                    pass

            # Build spatial index for components (used in A* algorithm)
            component_polygons = [comp_info['polygon'] for comp_info in component_boxes.values()]
            spatial_index = STRtree(component_polygons)

            # Draw connections but do not immediately render them
            draw_connections(
                d, G, components_element, pins, component_boxes, drawn_edges, routing_method, spatial_index, all_wires
            )

            # Check for overlapping wires
            overlapping = check_overlapping_wires(all_wires)

            if overlapping:
                print(f"Overlap detected. Showing the circuit with overlaps.")
                # Draw all wires onto the diagram
                for wire_info in all_wires:
                    line = wire_info['line']
                    color = wire_info.get('color', 'black')
                    d.add(elm.Line().at(line.coords[0]).to(line.coords[1]).color(color))
            else:
                # Draw all wires onto the diagram
                for wire_info in all_wires:
                    line = wire_info['line']
                    color = wire_info['color']
                    d.add(elm.Line().at(line.coords[0]).to(line.coords[1]).color(color))
                print(f"Successfully drew the circuit.")
        else:
            print("No positions found in XML. Please run with auto=1 to generate positions.")
            return

    # Render the circuit diagram
    d.draw()

def generate_tsmpy_layout(G):
    """Generate orthogonal layout using tsmpy with GML input."""
    try:
        # Export NetworkX graph to GML file
        nx.write_gml(G, 'graph.gml')

        # 读取图数据
        A = nx.Graph(nx.read_gml('graph.gml'))

        # 初始化布局
        pos = {node: eval(node) for node in G}

        # 生成正交布局
        tsm = TSM(A, pos)

        # 返回布局位置
        return tsm.pos
    except Exception as e:
        print(f"Error generating tsmpy layout: {e}")
        return None

def check_overlapping_wires(all_wires):
    # ... (Your existing check_overlapping_wires function)
    overlapping = False
    num_wires = len(all_wires)
    for i in range(num_wires):
        line_i = all_wires[i]['line']
        for j in range(i + 1, num_wires):
            line_j = all_wires[j]['line']
            if line_i.equals(line_j) or line_i.contains(line_j) or line_j.contains(line_i):
                # Overlapping wires detected
                overlapping = True
                # Mark both wires as red
                all_wires[i]['color'] = 'red'
                all_wires[j]['color'] = 'red'
    return overlapping

def draw_connections(d, G, components_element, pins, component_boxes, drawn_edges, routing_method, spatial_index, all_wires):
    # ... (Your existing draw_connections function)
    for comp, neighbor in G.edges():
        edge_key = tuple(sorted([comp, neighbor]))
        if edge_key in drawn_edges:
            continue

        comp_info = G.nodes[comp]
        neighbor_info = G.nodes[neighbor]

        # Determine the pins to connect
        comp_pin = get_component_pin(comp, neighbor, comp_info, components_element, pins)
        neighbor_pin = get_component_pin(neighbor, comp, neighbor_info, components_element, pins)

        if comp_pin is None or neighbor_pin is None:
            continue  # Skip if pins cannot be determined

        # Get the coordinates of the connection points
        start_pos = get_pin_position(pins, comp, comp_pin)
        end_pos = get_pin_position(pins, neighbor, neighbor_pin)

        if start_pos is None or end_pos is None:
            continue  # Skip if coordinates are invalid

        # Choose routing method
        if routing_method == 1:
            # Current method
            selected_path, line_color = route_connection_current_method(
                start_pos, end_pos, component_boxes, comp, neighbor
            )
        elif routing_method == 2:
            # A* algorithm
            # You can implement route_connection_astar if needed
            print("A* routing method is not implemented.")
            continue
        else:
            print(f"Unknown routing method: {routing_method}")
            continue

        # Store the wires but do not render them yet
        for i in range(len(selected_path) - 1):
            segment_start = selected_path[i]
            segment_end = selected_path[i + 1]
            new_line = LineString([segment_start, segment_end])

            # Add the current wire to the wire list
            all_wires.append({
                'line': new_line,
                'color': 'black'  # Initial color is black
            })

        drawn_edges.add(edge_key)

def draw_component_or_node(d, elements, pins, node, node_info, x, y):
    # ... (Your existing draw_component_or_node function)
    # Positions are now provided, so we use x and y directly
    if node_info.get('type') == 'R':
        # Draw resistor
        label_text = f"{node}\n{node_info.get('value')}"
        elements[node] = d.add(elm.Resistor().at((x, y)).right().label(label_text))
        pins[node] = {
            'start': elements[node].absanchors['start'],
            'end': elements[node].absanchors['end']
        }
    elif node_info.get('type') == 'C':
        # Draw capacitor
        label_text = f"{node}\n{node_info.get('value')}"
        elements[node] = d.add(elm.Capacitor().at((x, y)).up().label(label_text))
        pins[node] = {
            'start': elements[node].absanchors['start'],
            'end': elements[node].absanchors['end']
        }
    elif node_info.get('type') == 'L':
        # Draw inductor
        label_text = f"{node}\n{node_info.get('value')}"
        elements[node] = d.add(elm.Inductor().at((x, y)).right().label(label_text))
        pins[node] = {
            'start': elements[node].absanchors['start'],
            'end': elements[node].absanchors['end']
        }
    elif node_info.get('type') == 'V':
        # Draw voltage source
        label_text = f"{node}\n{node_info.get('value')}"
        elements[node] = d.add(elm.SourceV().at((x, y)).up().label(label_text))
        # For upward voltage source, positive at 'end', negative at 'start'
        pins[node] = {
            'positive': elements[node].absanchors['end'],
            'negative': elements[node].absanchors['start']
        }
    elif node_info.get('type') == 'I':
        # Draw current source
        label_text = f"{node}\n{node_info.get('value')}"
        elements[node] = d.add(elm.SourceI().at((x, y)).up().label(label_text))
        # For upward current source, positive at 'start', negative at 'end'
        pins[node] = {
            'positive': elements[node].absanchors['start'],
            'negative': elements[node].absanchors['end']
        }
    elif node_info.get('type') == 'D':
        # Draw diode
        label_text = f"{node}\n{node_info.get('value')}"
        elements[node] = d.add(elm.Diode().at((x, y)).right().label(label_text))
        # Anode at 'start', cathode at 'end'
        pins[node] = {
            'anode': elements[node].absanchors['start'],
            'cathode': elements[node].absanchors['end']
        }
    elif node_info.get('type') == 'J':
        # Draw JFET
        label_text = f"{node}\n{node_info.get('value')}"
        elements[node] = d.add(elm.JFet().at((x, y)).right().label(label_text))
        # Use correct anchors
        pins[node] = {
            'drain': elements[node].absanchors['drain'],
            'gate': elements[node].absanchors['gate'],
            'source': elements[node].absanchors['source']
        }
    elif node_info.get('type') == 'ground':
        # Draw ground
        elements[node] = d.add(elm.Ground().at((x, y)))
        pins[node] = {
            'pin': (x, y)
        }
    else:
        # Draw regular node
        elements[node] = d.add(elm.Dot().at((x, y)))
        pins[node] = {
            'pin': (x, y)
        }
        # Add node label
        d.add(elm.Label().at((x, y)).label(node, ofst=0.2))

def route_connection_current_method(start_pos, end_pos, component_boxes, comp, neighbor):
    # ... (Your existing route_connection_current_method function)
    # Define initial two path options
    path1 = [start_pos, (end_pos[0], start_pos[1]), end_pos]  # Horizontal first, then vertical
    path2 = [start_pos, (start_pos[0], end_pos[1]), end_pos]  # Vertical first, then horizontal

    # All path options
    paths = [
        ('H-V', path1),
        ('V-H', path2),
    ]

    # Record path characteristics
    path_scores = []

    # Check if each path collides with components
    line_color = "black"
    selected_path = None
    for name, path in paths:
        crossing = False
        total_length = 0
        num_turns = len(path) - 2  # Number of turns is the number of intermediate points

        for i in range(len(path) - 1):
            segment_start = path[i]
            segment_end = path[i + 1]
            # Check if the segment collides with components
            if is_wire_crossing_components(
                segment_start, segment_end, component_boxes, comp, neighbor
            ):
                crossing = True
                break
            # Calculate total length
            segment_length = np.hypot(segment_end[0] - segment_start[0], segment_end[1] - segment_start[1])
            total_length += segment_length

        path_scores.append({
            'name': name,
            'path': path,
            'crossing': crossing,
            'total_length': total_length,
            'num_turns': num_turns
        })

        if not crossing and selected_path is None:
            # Choose the first non-crossing path
            selected_path = path

    if selected_path is None:
        # If all paths collide, choose the shortest path and mark it red
        non_selected_paths = [p for p in path_scores if p['crossing']]
        if non_selected_paths:
            best_path = min(non_selected_paths, key=lambda x: x['total_length'])
            selected_path = best_path['path']
            line_color = "red"
        else:
            # If no paths are available, default to path1 and mark it red
            selected_path = path1
            line_color = "red"

    # Print path scores for evaluation
    print("Path options for connection between", comp, "and", neighbor)
    for score in path_scores:
        print(f"Path {score['name']}: Crosses components: {score['crossing']}, "
              f"Total length: {score['total_length']:.2f}, Turns: {score['num_turns']}")

    return selected_path, line_color

def is_wire_crossing_components(start_pos, end_pos, component_boxes, comp, neighbor):
    # ... (Your existing is_wire_crossing_components function)
    line = LineString([start_pos, end_pos])
    line_start_point = Point(start_pos)
    line_end_point = Point(end_pos)
    for comp_id, comp_info in component_boxes.items():
        polygon = comp_info['polygon']
        if line.intersects(polygon):
            intersection = line.intersection(polygon)
            # If the intersection is only at the start or end point and connects related components, it's acceptable
            if (intersection.equals(line_start_point) or intersection.equals(line_end_point)) and (comp_id == comp or comp_id == neighbor):
                continue
            else:
                return True
    return False

def get_component_pin(comp, neighbor, comp_info, components_element, pins):
    # ... (Your existing get_component_pin function)
    if comp_info.get('type') in ['R', 'C', 'L']:
        # For non-directional components, find the pin corresponding to the neighbor node
        comp_pins = ['start', 'end']

        # Get coordinates of component pins
        comp_pin_coords = {pin: pins[comp][pin] for pin in comp_pins}

        # Get the two nodes connected to the component
        component = next(c for c in components_element if c.attrib['id'] == comp)
        node1 = component.find('node1').text
        node2 = component.find('node2').text
        nodes = [node1, node2]

        # Get coordinates of nodes
        node_coords = {}
        for node in nodes:
            node_coord = get_node_coordinate(pins, node)
            if node_coord:
                node_coords[node] = node_coord

        if node1 not in node_coords or node2 not in node_coords:
            return None  # Cannot determine node coordinates

        # Calculate total distance for two pin assignment schemes
        # Scheme 1: 'start' -> node1, 'end' -> node2
        distance1 = np.linalg.norm(np.array(comp_pin_coords['start']) - np.array(node_coords[node1])) + \
                    np.linalg.norm(np.array(comp_pin_coords['end']) - np.array(node_coords[node2]))
        # Scheme 2: 'start' -> node2, 'end' -> node1
        distance2 = np.linalg.norm(np.array(comp_pin_coords['start']) - np.array(node_coords[node2])) + \
                    np.linalg.norm(np.array(comp_pin_coords['end']) - np.array(node_coords[node1]))

        # Choose the scheme with shorter total distance
        if distance1 <= distance2:
            pin_node_pairs = [('start', node1), ('end', node2)]
        else:
            pin_node_pairs = [('start', node2), ('end', node1)]

        # Find the pin corresponding to the neighbor node
        for pin, node in pin_node_pairs:
            if node == neighbor:
                return pin

    elif comp_info.get('type') == 'V':
        # For voltage source
        component = next(c for c in components_element if c.attrib['id'] == comp)
        node1 = component.find('node1').text  # Positive terminal
        node2 = component.find('node2').text  # Negative terminal
        if neighbor == node1:
            return 'positive'
        elif neighbor == node2:
            return 'negative'

    elif comp_info.get('type') == 'I':
        # For current source
        component = next(c for c in components_element if c.attrib['id'] == comp)
        node1 = component.find('node1').text  # Current entering
        node2 = component.find('node2').text  # Current exiting
        if neighbor == node1:
            return 'positive'
        elif neighbor == node2:
            return 'negative'

    elif comp_info.get('type') == 'D':
        # For diode
        component = next(c for c in components_element if c.attrib['id'] == comp)
        node1 = component.find('node1').text  # Anode
        node2 = component.find('node2').text  # Cathode
        if neighbor == node1:
            return 'anode'
        elif neighbor == node2:
            return 'cathode'

    elif comp_info.get('type') == 'J':
        # For JFET
        component = next(c for c in components_element if c.attrib['id'] == comp)
        node_drain = component.find('node1').text
        node_gate = component.find('node2').text
        node_source = component.find('node3').text
        node_mapping = {
            node_drain: 'drain',
            node_gate: 'gate',
            node_source: 'source'
        }
        if neighbor in node_mapping:
            return node_mapping[neighbor]

    else:
        # For nodes and ground
        return 'pin'
    return None

def get_node_coordinate(pins, node):
    # ... (Your existing get_node_coordinate function)
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
    else:
        return None

def get_pin_position(pins, comp, pin_name):
    # ... (Your existing get_pin_position function)
    try:
        pos = pins[comp][pin_name]
        if isinstance(pos, schemdraw.util.Point):
            return (pos.x, pos.y)
        else:
            return pos
    except KeyError:
        print(f"Pin {pin_name} not found for component {comp}")
        return None

# Main program
if __name__ == "__main__":
    # Convert netlist to XML
    xml_file = "spice_netlist.xml"
    convert_netlist_to_xml_file(netlist, xml_file)

    # Parse XML
    tree = ET.parse(xml_file)
    xml_root = tree.getroot()

    # Create graph from XML
    G, components_element = create_graph_from_xml(xml_root)

    # Draw the circuit
    draw_circuit(
        G, components_element, xml_root, xml_file,
        max_attempts=max_attempts,
        EnlargeSize=EnlargeSize,
        shrink_ratio=shrink_ratio,
        routing_method=routing_method,
        auto=auto
    )
