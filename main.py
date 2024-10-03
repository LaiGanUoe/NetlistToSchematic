import networkx as nx
import schemdraw
import numpy
import schemdraw.elements as elm
import xml.etree.ElementTree as ET
from netlistAnalysisMethods import *

# Define SPICE netlist
netlist = """
* RC Low-pass Filter Circuit
V1 in 0 DC 5
R1 in out 1k
C1 out 0 10uF
.tran 0 1m
.backanno
.end
"""

# Convert netlist to XML using function from scan.py
convert_netlist_to_xml_file(netlist, "spice_netlist.xml")

# Read and parse XML data
tree = ET.parse("spice_netlist.xml")
root = tree.getroot()

# Create directed graph
G = nx.DiGraph()

# Add components to graph
for component in root.find('components'):
    ctype = component.attrib['type']
    cid = component.attrib['id']
    node1 = component.find('node1').text
    node2 = component.find('node2').text

    # Add nodes and edges to graph
    G.add_node(cid, type=ctype)
    if ctype == 'V':
        G.add_edge(node1, cid)  # Positive terminal of V -> component
        G.add_edge(cid, node2)  # Component -> Negative terminal of V
    else:
        G.add_edge(node1, cid)  # Node1 -> Component
        G.add_edge(cid, node2)  # Component -> Node2

EnlargeSize = 2.5
grid_size = 1  # Define the grid size
pos = nx.spring_layout(G, scale=EnlargeSize)
d = schemdraw.Drawing()

def snap_to_grid(x, y, grid_size):
    """Snap the x, y coordinates to the nearest grid point."""
    return (round(x / grid_size) * grid_size, round(y / grid_size) * grid_size)

elements = {}
pins = {}
occupied_paths = set()  # Global path occupancy set

def is_path_occupied(start, end):
    """Check if the path is occupied."""
    return (start, end) in occupied_paths or (end, start) in occupied_paths

def mark_path_occupied(start, end):
    """Mark the path as occupied."""
    occupied_paths.add((start, end))
    occupied_paths.add((end, start))

def calculate_distance(point1, point2):
    """Calculate Euclidean distance between two points."""
    return ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2) ** 0.5

def get_best_pins(comp_pos, neighbor_pos, comp_pins, neighbor_pins):
    """Find the best pins to connect based on the shortest distance."""
    best_distance = float('inf')
    best_comp_pin, best_neighbor_pin = None, None

    for c_pin in comp_pins:
        for n_pin in neighbor_pins:
            if isinstance(comp_pos[c_pin], tuple) and isinstance(neighbor_pos[n_pin], tuple):
                distance = calculate_distance(comp_pos[c_pin], neighbor_pos[n_pin])
                if distance < best_distance:
                    best_distance = distance
                    best_comp_pin = c_pin
                    best_neighbor_pin = n_pin

    return best_comp_pin, best_neighbor_pin

def draw_segment(d, start_pos, end_pos):
    """Draw a segment from start_pos to end_pos, avoiding occupied paths by rerouting if necessary."""
    if start_pos != end_pos:
        if is_path_occupied(start_pos, end_pos):
            # If the path is occupied, try to reroute
            rerouted = False
            for offset in range(1, 4):  # Try different offsets for rerouting
                if not rerouted:
                    # Try right and left offsets for horizontal rerouting
                    for dx in [offset, -offset]:
                        mid_point1 = (start_pos[0] + dx, start_pos[1])
                        mid_point2 = (end_pos[0] + dx, end_pos[1])
                        if not is_path_occupied(start_pos, mid_point1) and not is_path_occupied(mid_point1, mid_point2) and not is_path_occupied(mid_point2, end_pos):
                            d += elm.Line().at(start_pos).to(mid_point1)
                            mark_path_occupied(start_pos, mid_point1)
                            d += elm.Line().at(mid_point1).to(mid_point2)
                            mark_path_occupied(mid_point1, mid_point2)
                            d += elm.Line().at(mid_point2).to(end_pos)
                            mark_path_occupied(mid_point2, end_pos)
                            rerouted = True
                            break
                    # Try up and down offsets for vertical rerouting
                    for dy in [offset, -offset]:
                        if rerouted:
                            break
                        mid_point1 = (start_pos[0], start_pos[1] + dy)
                        mid_point2 = (end_pos[0], end_pos[1] + dy)
                        if not is_path_occupied(start_pos, mid_point1) and not is_path_occupied(mid_point1, mid_point2) and not is_path_occupied(mid_point2, end_pos):
                            d += elm.Line().at(start_pos).to(mid_point1)
                            mark_path_occupied(start_pos, mid_point1)
                            d += elm.Line().at(mid_point1).to(mid_point2)
                            mark_path_occupied(mid_point1, mid_point2)
                            d += elm.Line().at(mid_point2).to(end_pos)
                            mark_path_occupied(mid_point2, end_pos)
                            rerouted = True
                            break
            if not rerouted:
                raise RuntimeError(f"Failed to find a rerouted path from {start_pos} to {end_pos}")
        else:
            d += elm.Line().at(start_pos).to(end_pos)
            mark_path_occupied(start_pos, end_pos)

for node in G.nodes:
    node_info = G.nodes[node]
    node_pos = pos[node]
    x, y = snap_to_grid(EnlargeSize * node_pos[0], EnlargeSize * -node_pos[1], grid_size)

    if node_info.get('type') == 'R':
        elements[node] = d.add(elm.Resistor().at((x, y)).right().label(node))
        pins[node] = {'start': (x, y), 'end': (x + 3, y)}  # assuming resistor is horizontal
    elif node_info.get('type') == 'C':
        elements[node] = d.add(elm.Capacitor().at((x, y)).up().label(node))
        pins[node] = {'start': (x, y), 'end': (x, y + 3)}  # assuming capacitor is vertical
    elif node_info.get('type') == 'V':
        elements[node] = d.add(elm.SourceV().at((x, y)).up().label(node))
        pins[node] = {'start': (x, y), 'end': (x, y + 3)}  # assuming positive terminal is up
    elif node == '0':
        elements[node] = d.add(elm.Ground().at((x, y)))
        pins[node] = {'start': (x, y), 'end': (x, y)}
    else:
        elements[node] = d.add(elm.Dot().at((x, y)).label(node))
        pins[node] = {'start': (x, y), 'end': (x, y)}

drawn_edges = set()

for comp in G.nodes:
    connected = list(G[comp])
    for neighbor in connected:
        if (comp, neighbor) not in drawn_edges and (neighbor, comp) not in drawn_edges:
            if G.nodes[comp].get('type') == 'V':
                # Voltage source: ensure correct polarity
                start_pos = pins[comp]['start']
                end_pos = pins[neighbor]['start'] if neighbor != '0' else pins[neighbor]['end']
                comp_pin, neighbor_pin = 'start', 'start'
            else:
                comp_pos = {'start': pins[comp]['start'], 'end': pins[comp]['end']}
                neighbor_pos = {'start': pins[neighbor]['start'], 'end': pins[neighbor]['end']}

                comp_pin, neighbor_pin = get_best_pins(comp_pos, neighbor_pos, ['start', 'end'], ['start', 'end'])

                start_pos = comp_pos[comp_pin]
                end_pos = neighbor_pos[neighbor_pin]

            primary_direction = 'horizontal' if start_pos[0] != end_pos[0] else 'vertical'
            if primary_direction == 'horizontal':
                mid_x, mid_y = end_pos[0], start_pos[1]
            else:
                mid_x, mid_y = start_pos[0], end_pos[1]

            # Check and mark path
            if is_path_occupied(start_pos, (mid_x, mid_y)):
                # If the primary direction is occupied, try the other direction
                if primary_direction == 'horizontal':
                    mid_x, mid_y = start_pos[0], end_pos[1]
                else:
                    mid_x, mid_y = end_pos[0], start_pos[1]

            # Draw horizontal segment
            draw_segment(d, start_pos, (mid_x, start_pos[1]))

            # Draw vertical segment
            draw_segment(d, (mid_x, start_pos[1]), (mid_x, mid_y))

            # Draw final segment to the end position
            draw_segment(d, (mid_x, mid_y), end_pos)

            pins[comp][comp_pin] = True
            pins[neighbor][neighbor_pin] = True
            drawn_edges.add((comp, neighbor))
            drawn_edges.add((neighbor, comp))

# Draw the circuit diagram
d.draw()

