import networkx as nx
import schemdraw
import schemdraw.elements as elm
import xml.etree.ElementTree as ET
import numpy as np
from shapely.geometry import LineString, Polygon, Point
import xml.dom.minidom as minidom

from netlistAnalysisMethods import *
from routingDetection import *

# Define SPICE netlist
netlist = """
* 电流源驱动的二极管
I1 1 0 DC 1A         ; 恒定直流电流源，1A
D1 1 2 1N4148        ; 二极管1N4148，连接节点1和节点2
R1 2 0 10            ; 负载电阻10欧姆
.END

"""
enlargeSize = 3
max_attempts = 200

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
    elif node_info.get('type') == 'I':
        # Draw current source
        label_text = f"{node}\n{node_info.get('value')}"
        elements[node] = d.add(elm.SourceI().at((x, y)).up().label(label_text))
        # For upward current source, positive is at 'start', negative at 'end'
        pins[node] = {
            'positive': elements[node].absanchors['end'],
            'negative': elements[node].absanchors['start']
        }
    elif node_info.get('type') == 'D':
        # Draw diode
        label_text = f"{node}\n{node_info.get('value')}"
        elements[node] = d.add(elm.Diode().at((x, y)).right().label(label_text))
        # For diode, anode is at 'start', cathode at 'end'
        pins[node] = {
            'anode': elements[node].absanchors['start'],
            'cathode': elements[node].absanchors['end']
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
            elif node in pins and 'anode' in pins[node]:
                node_coords[node] = pins[node]['anode']
            elif node in pins and 'cathode' in pins[node]:
                node_coords[node] = pins[node]['cathode']
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

    elif comp_info.get('type') == 'I':
        # For current source
        component = next(c for c in components_element if c.attrib['id'] == comp)
        node1 = component.find('node1').text  # Positive terminal (current enters)
        node2 = component.find('node2').text  # Negative terminal (current exits)
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
    draw_circuit(G, components_element, max_attempts=max_attempts, EnlargeSize=enlargeSize)
