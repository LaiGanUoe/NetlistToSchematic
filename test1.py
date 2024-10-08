import logging
import heapq
import networkx as nx
import schemdraw
import schemdraw.elements as elm
import xml.etree.ElementTree as ET
from netlistAnalysisMethods import *

# Configure logging
logging.basicConfig(filename='circuit_routing.log', level=logging.DEBUG,
                    format='%(asctime)s - %(levelname)s - %(message)s')


# A* algorithm heuristic function (Manhattan distance)
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


# A* algorithm implementation
def a_star(start, goal, occupied_areas):
    """Use A* algorithm to find the shortest path from start to goal"""
    try:
        frontier = []
        heapq.heappush(frontier, (0, start))

        came_from = {start: None}
        cost_so_far = {start: 0}

        while frontier:
            current_priority, current = heapq.heappop(frontier)

            if current == goal:
                break

            for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                neighbor = (current[0] + dx, current[1] + dy)

                if is_in_occupied_areas(neighbor, occupied_areas):
                    continue

                new_cost = cost_so_far[current] + 1
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(goal, neighbor)
                    heapq.heappush(frontier, (priority, neighbor))
                    came_from[neighbor] = current

        if goal not in came_from:
            logging.warning(f"Pathfinding failed from {start} to {goal}: No valid path found.")
            return None

        path = []
        current = goal
        while current is not None:
            path.append(current)
            current = came_from[current]
        path.reverse()
        return path
    except Exception as e:
        logging.error(f"Error during A* pathfinding from {start} to {goal}: {e}")
        return None


# Check if a point is in an occupied area
def is_in_occupied_areas(point, occupied_areas):
    for area in occupied_areas:
        if is_in_bounding_box(area, point):
            return True
    return False


# Mark an area as occupied
def mark_area_occupied(start, end):
    occupied_areas.append(get_bounding_box(start, end))


# Compute bounding box of a path (line segment)
def get_bounding_box(start, end):
    x_min = min(start[0], end[0])
    x_max = max(start[0], end[0])
    y_min = min(start[1], end[1])
    y_max = max(start[1], end[1])
    return (x_min, y_min, x_max, y_max)


# Detect overlap between two bounding boxes
def is_overlap(box1, box2):
    return not (box1[2] < box2[0] or box1[0] > box2[2] or
                box1[3] < box2[1] or box1[1] > box2[3])


# Draw the path and log success or failure
def draw_path(d, path, color):
    """Draw the path and set its color"""
    try:
        for i in range(len(path) - 1):
            d += elm.Line().at(path[i]).to(path[i + 1]).color(color)
        logging.info(f"Successfully drew path from {path[0]} to {path[-1]} with color {color}.")
    except Exception as e:
        logging.error(f"Error drawing path from {path[0]} to {path[-1]}: {e}")


# Handle exceptions in segment drawing and detect intersections
def draw_segment(d, start_pos, end_pos):
    """Handle pathfinding, log success/failure, and detect intersections."""
    try:
        if start_pos != end_pos:
            path = a_star(start_pos, end_pos, occupied_areas)
            if path:
                draw_path(d, path, 'green')
                for i in range(len(path) - 1):
                    mark_area_occupied(path[i], path[i + 1])
            else:
                logging.warning(f"Failed to route from {start_pos} to {end_pos}, marking in red.")
                draw_path(d, [start_pos, end_pos], 'red')
        else:
            draw_path(d, [start_pos, end_pos], 'green')
            mark_area_occupied(start_pos, end_pos)
    except Exception as e:
        logging.error(f"Exception in drawing segment from {start_pos} to {end_pos}: {e}")
        draw_path(d, [start_pos, end_pos], 'red')


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

# Convert netlist to XML using a function
convert_netlist_to_xml_file(netlist, "spice_netlist.xml")

# Read and parse XML data
tree = ET.parse("spice_netlist.xml")
root = tree.getroot()

# Create directed graph
G = nx.DiGraph()

for component in root.find('components'):
    try:
        ctype = component.attrib['type']
        cid = component.attrib['id']
        node1 = component.find('node1').text
        node2 = component.find('node2').text

        G.add_node(cid, type=ctype)
        if ctype == 'V':
            G.add_edge(node1, cid)
            G.add_edge(cid, node2)
        else:
            pos_node1 = pos[node1]
            pos_node2 = pos[node2]
            pos_cid = pos[cid]

            distance1 = calculate_distance(pos_node1, pos_cid) + calculate_distance(pos_cid, pos_node2)
            distance2 = calculate_distance(pos_node2, pos_cid) + calculate_distance(pos_cid, pos_node1)

            if distance1 <= distance2:
                G.add_edge(node1, cid)
                G.add_edge(cid, node2)
            else:
                G.add_edge(node2, cid)
                G.add_edge(cid, node1)
    except Exception as e:
        logging.error(f"Error processing component {component.attrib}: {e}")

# Layout and drawing setup
EnlargeSize = 3
pos = nx.spring_layout(G, scale=EnlargeSize)
d = schemdraw.Drawing()

elements = {}
pins = {}
occupied_areas = []  # Stores all occupied areas

# Add components and compute bounding boxes
for node in G.nodes:
    try:
        node_info = G.nodes[node]
        node_pos = pos[node]
        x, y = (EnlargeSize * node_pos[0], EnlargeSize * -node_pos[1])

        if node_info.get('type') == 'R':
            elements[node] = d.add(elm.Resistor().at((x, y)).right().label(node))
            pins[node] = {'start': (x, y), 'end': (x + 3, y)}
        elif node_info.get('type') == 'C':
            elements[node] = d.add(elm.Capacitor().at((x, y)).up().label(node))
            pins[node] = {'start': (x, y), 'end': (x, y + 3)}
        elif node_info.get('type') == 'V':
            elements[node] = d.add(elm.SourceV().at((x, y)).up().label(node))
            pins[node] = {'start': (x, y), 'end': (x, y + 3)}
        elif node == '0':
            elements[node] = d.add(elm.Ground().at((x, y)))
            pins[node] = {'start': (x, y), 'end': (x, y)}
        else:
            elements[node] = d.add(elm.Dot().at((x, y)).label(node))
            pins[node] = {'start': (x, y), 'end': (x, y)}
    except Exception as e:
        logging.error(f"Error adding component {node}: {e}")

# Process and detect connection paths
drawn_edges = set()

for comp in G.nodes:
    connected = list(G[comp])
    for neighbor in connected:
        if (comp, neighbor) not in drawn_edges and (neighbor, comp) not in drawn_edges:
            start_pos = pins[comp]['start']
            end_pos = pins[neighbor]['start'] if neighbor != '0' else pins[neighbor]['end']

            # Handle segment drawing with pathfinding and error handling
            draw_segment(d, start_pos, end_pos)

            drawn_edges.add((comp, neighbor))
            drawn_edges.add((neighbor, comp))

# Draw the circuit diagram
d.draw()
