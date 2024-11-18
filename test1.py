import networkx as nx
import schemdraw
import schemdraw.elements as elm
import xml.etree.ElementTree as ET
import numpy as np
from shapely.geometry import LineString, Polygon, Point
import xml.dom.minidom as minidom

# Define SPICE netlist
netlist = """
* AC Analysis Example
R1 N1 N2 1k        ; Resistor R1, 1kΩ
L1 N2 N3 10m       ; Inductor L1, 10mH
C1 N3 0 1u         ; Capacitor C1, 1μF
V1 N1 0 AC 1       ; Input AC voltage source, amplitude 1V
.AC LIN 100 1k 10k ; AC analysis from 1kHz to 10kHz, linear step of 100 points
.END
"""
EnlargeSize = 2.5

def parse_spice_netlist(netlist):
    """Parse the SPICE netlist into components, commands, and subcircuits."""
    components = []
    commands = []
    subcircuits = []
    for line in netlist.strip().split('\n'):
        # Remove inline comments starting with ';'
        if ';' in line:
            line = line.split(';')[0].strip()
        line = line.strip()
        # Skip comments (lines starting with '*')
        if line.startswith('*'):
            continue
        # Skip empty lines
        if not line.strip():
            continue

        try:
            parts = line.split()
            first_char = parts[0][0].upper()

            # Parse various components based on the first character
            if first_char in ['V', 'R', 'C', 'L', 'D', 'M', 'Q', 'I', 'J', 'K', 'S', 'T', 'E', 'F', 'G', 'H']:
                if first_char == 'V':
                    value = ' '.join(parts[3:])  # Voltage source
                    nodes = parts[1:3]
                elif first_char == 'I':
                    value = ' '.join(parts[3:])  # Current source
                    nodes = parts[1:3]
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
                    'subckt': parts[-1]  # Subcircuit name
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
    """Convert parsed netlist data into an XML structure."""
    root = ET.Element("spice_netlist")

    components_elem = ET.SubElement(root, "components")
    for comp in components:
        comp_elem = ET.SubElement(components_elem, "component", type=comp['type'], id=comp['id'])
        for i, node in enumerate(comp['nodes'], start=1):
            ET.SubElement(comp_elem, f"node{i}").text = node
        ET.SubElement(comp_elem, "value").text = comp['value']

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
    """Return a pretty-printed XML string for the Element."""
    rough_string = ET.tostring(element, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")

def convert_netlist_to_xml_file(netlist, output_filename="spice_netlist.xml"):
    """Convert netlist to XML and save it to a file."""
    components, commands, subcircuits = parse_spice_netlist(netlist)
    xml_root = netlist_to_xml(components, commands, subcircuits)
    pretty_xml = pretty_print_xml(xml_root)
    with open(output_filename, "w") as f:
        f.write(pretty_xml)
    print(pretty_xml)

def create_graph_from_xml(xml_file):
    """Create a graph from the XML data."""
    tree = ET.parse(xml_file)
    root = tree.getroot()
    components_element = root.find('components')
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
                G.add_node(node, type='node')  # Regular node
            # Add edge
            G.add_edge(cid, node)

    return G, components_element

def draw_circuit(G, components_element, max_attempts=100, EnlargeSize=2.5):
    """Attempt to draw the circuit diagram without wire crossings."""
    attempt = 0
    success = False

    while attempt < max_attempts:
        attempt += 1
        print(f"Attempt {attempt}")

        pos = nx.spring_layout(G, scale=EnlargeSize)

        # Reset drawing and related variables
        d = schemdraw.Drawing()
        elements = {}
        pins = {}
        component_boxes = {}  # Used to store component bounding boxes
        drawn_edges = set()
        any_red_lines = False  # Flag to check if any red lines were drawn

        # Draw components and nodes, and compute bounding boxes
        for node in G.nodes:
            node_info = G.nodes[node]
            node_pos = pos[node]
            x, y = (EnlargeSize * node_pos[0], EnlargeSize * -node_pos[1])

            if np.isnan(x) or np.isnan(y) or np.isinf(x) or np.isinf(y):
                print(f"Node {node} has invalid transformed position: ({x}, {y})")
                continue  # Skip problematic nodes

            draw_component_or_node(d, elements, pins, node, node_info, x, y)

            # Get component bounding box
            if node in elements and hasattr(elements[node], 'get_bbox') and node_info.get('type') != 'node':
                bbox = elements[node].get_bbox(transform=True)
                x_min, y_min, x_max, y_max = bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax
                # Check if bbox is valid
                if np.isnan([x_min, y_min, x_max, y_max]).any() or np.isinf([x_min, y_min, x_max, y_max]).any():
                    print(f"Element {node} has invalid bbox: {bbox}")
                    continue  # Skip problematic components

                # Compute shrunken bounding box
                shrink_ratio = 0.1  # Shrink ratio
                width = x_max - x_min
                height = y_max - y_min
                x_offset = width * shrink_ratio / 2
                y_offset = height * shrink_ratio / 2
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

        # Draw connections
        any_red_lines = draw_connections(
            d, G, components_element, pins, component_boxes, drawn_edges
        )

        if not any_red_lines:
            print(f"Successful layout found after {attempt} attempts.")
            success = True
            break  # Exit the loop if no red lines are present

    if not success:
        print(f"No collision-free layout found after {max_attempts} attempts. Showing the last attempt.")

    # Draw the circuit diagram
    d.draw()

def draw_component_or_node(d, elements, pins, node, node_info, x, y):
    """Draw a component or node on the schemdraw drawing."""
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
        # For upward voltage source, positive is at 'end', negative at 'start'
        pins[node] = {
            'positive': elements[node].absanchors['end'],
            'negative': elements[node].absanchors['start']
        }
    elif node_info.get('type') == 'ground':
        # Draw ground
        elements[node] = d.add(elm.Ground().at((x, y)))
        pins[node] = {
            'pin': (x, y)
        }
    else:
        # Draw node
        elements[node] = d.add(elm.Dot().at((x, y)))
        pins[node] = {
            'pin': (x, y)
        }
        # Add node label
        d.add(elm.Label().at((x, y)).label(node, ofst=0.2))

def is_wire_crossing_components(start_pos, end_pos, component_boxes, comp, neighbor):
    """Check if a wire between two points crosses any components."""
    line = LineString([start_pos, end_pos])
    line_start_point = Point(start_pos)
    line_end_point = Point(end_pos)
    for comp_id, comp_info in component_boxes.items():
        polygon = comp_info['polygon']
        if line.intersects(polygon):
            intersection = line.intersection(polygon)
            # If the intersection is only at the start or end point and the component is connected, it's not a collision
            if (intersection.equals(line_start_point) or intersection.equals(line_end_point)) and (comp_id == comp or comp_id == neighbor):
                continue
            else:
                return True
    return False

def draw_connections(d, G, components_element, pins, component_boxes, drawn_edges):
    """Draw connections between components, avoiding wire crossings."""
    any_red_lines = False  # Flag to check if any red lines were drawn
    for comp, neighbor in G.edges():
        edge_key = tuple(sorted([comp, neighbor]))
        if edge_key in drawn_edges:
            continue

        comp_info = G.nodes[comp]
        neighbor_info = G.nodes[neighbor]

        # Determine the connection pins
        comp_pin = get_component_pin(comp, neighbor, comp_info, components_element, pins)
        neighbor_pin = get_component_pin(neighbor, comp, neighbor_info, components_element, pins)

        if comp_pin is None or neighbor_pin is None:
            continue  # Skip if pins cannot be determined

        # Get the connection point coordinates
        start_pos = get_pin_position(pins, comp, comp_pin)
        end_pos = get_pin_position(pins, neighbor, neighbor_pin)

        if start_pos is None or end_pos is None:
            continue  # Skip if positions are invalid

        # Define path points for the connection, horizontal and vertical
        path1 = [start_pos, (end_pos[0], start_pos[1]), end_pos]  # Horizontal first, then vertical
        path2 = [start_pos, (start_pos[0], end_pos[1]), end_pos]  # Vertical first, then horizontal

        # Check which path does not cross components
        line_color = "black"
        selected_path = None
        for path in [path1, path2]:
            crossing = False
            for i in range(len(path) - 1):
                segment_start = path[i]
                segment_end = path[i + 1]
                # Check if it collides with components
                if is_wire_crossing_components(
                    segment_start, segment_end, component_boxes, comp, neighbor
                ):
                    crossing = True
                    break
            if not crossing:
                selected_path = path
                break
        if selected_path is None:
            # If both paths cross components, select path1, line color is set to red
            selected_path = path1
            line_color = "red"
            any_red_lines = True  # Set flag indicating red lines were drawn

        # Draw the connection
        for i in range(len(selected_path) - 1):
            d.add(elm.Line().at(selected_path[i]).to(selected_path[i + 1]).color(line_color))

        drawn_edges.add(edge_key)

    return any_red_lines

def get_component_pin(comp, neighbor, comp_info, components_element, pins):
    """Get the appropriate pin for the component based on the neighbor."""
    if comp_info.get('type') in ['R', 'C', 'L']:
        # For non-directional components, find the pin-node assignment with shortest total distance
        comp_pins = ['start', 'end']

        # Get the coordinates of the component's pins
        comp_pin_coords = {pin: pins[comp][pin] for pin in comp_pins}

        # Get the two nodes connected to the component
        component = next(c for c in components_element if c.attrib['id'] == comp)
        node1 = component.find('node1').text
        node2 = component.find('node2').text
        nodes = [node1, node2]

        # Get the coordinates of the nodes
        node_coords = {}
        for node in nodes:
            if node in pins and 'pin' in pins[node]:
                node_coords[node] = pins[node]['pin']
            elif node in pins and 'positive' in pins[node]:
                node_coords[node] = pins[node]['positive']
            elif node in pins and 'negative' in pins[node]:
                node_coords[node] = pins[node]['negative']
            else:
                continue

        if node1 not in node_coords or node2 not in node_coords:
            return None  # Cannot determine node coordinates

        # Calculate total distances for two possible pin-node assignments
        # Assignment 1: comp 'start' -> node1, 'end' -> node2
        distance1 = np.linalg.norm(np.array(comp_pin_coords['start']) - np.array(node_coords[node1])) + \
                    np.linalg.norm(np.array(comp_pin_coords['end']) - np.array(node_coords[node2]))
        # Assignment 2: comp 'start' -> node2, 'end' -> node1
        distance2 = np.linalg.norm(np.array(comp_pin_coords['start']) - np.array(node_coords[node2])) + \
                    np.linalg.norm(np.array(comp_pin_coords['end']) - np.array(node_coords[node1]))

        # Choose the assignment with the shorter total distance
        if distance1 <= distance2:
            pin_node_pairs = [('start', node1), ('end', node2)]
        else:
            pin_node_pairs = [('start', node2), ('end', node1)]

        # Find the pin corresponding to the neighbor
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
    else:
        # For nodes and ground
        return 'pin'
    return None

def get_pin_position(pins, comp, pin_name):
    """Get the position of a pin."""
    try:
        pos = pins[comp][pin_name]
        if isinstance(pos, schemdraw.util.Point):
            return (pos.x, pos.y)
        else:
            return pos
    except KeyError:
        print(f"Pin {pin_name} not found for component {comp}")
        return None

# Main execution
if __name__ == "__main__":
    # Convert netlist to XML
    convert_netlist_to_xml_file(netlist, "spice_netlist.xml")

    # Create graph from XML
    G, components_element = create_graph_from_xml("spice_netlist.xml")

    # Draw the circuit diagram
    draw_circuit(G, components_element, max_attempts=100, EnlargeSize=EnlargeSize)
