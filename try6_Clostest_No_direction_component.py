import networkx as nx
from matplotlib.patches import PathPatch
from matplotlib.path import Path
import numpy as np
import schemdraw
import schemdraw.elements as elm
import xml.etree.ElementTree as ET
from shapely.geometry import LineString, Polygon, Point, MultiPoint
import xml.dom.minidom as minidom
import math
import copy
import sys
import matplotlib.pyplot as plt

# Sample Netlist (Ensure this is your actual netlist)
netlist = """
*common source jfet amplifier 
v1 1 0 dc 24 
v2 3 0 dc 15 
r1 1 2 10k 
r2 2 3 8.1k 
r3 2 0 4.7k 
.end 

"""

# Configuration Parameters
EnlargeSize = 8             # Enlarge the size of the positions of the components
max_attempts = 1000             # Maximum number of attempts to find a valid placement and routing
routing_method = 1           # Only Manhattan routing is used, so set to 1
auto = 0                     # Set to 0 for fixed positions, 1 for automatic layout
shrink_size = 0.03           # Shrink size for component bounding boxes
wire_safe_color = 'green'    # Safe wire color
wire_danger_color = 'red'    # Danger wire color
grid_size = 0.1              # Default Grid Interval: Fixed at 0.1 and only modify when default_scaling_ratios change.
grid_width = 0.8             # Grid line width
draw_grid_or_not = 0         # Set to 1 to draw grid, 0 to not draw grid
dot_radius = 0.06            # Anchor dot radius
max_intersections = 0       # Maximum number of allowed wire intersections


default_directions = {
    'GND': 'right',
    'R': 'right',
    'C': 'right',
    'L': 'right',
    'D': 'right',
    'V': 'up',
    'I': 'up',
    'S': 'right',
    'J': 'right',
    'Q': 'right',
    'M': 'right',
    'E': 'up',
    'H': 'up',
    'F': 'up',
    'G': 'up',
    'node': 'right',
    'ground': 'right'
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
    'M': 'none',
    'E': 'none',
    'H': 'none',
    'F': 'none',
    'G': 'none',
    'node': 'none',
    'ground': 'none'
}

# 缺省缩放比例
default_scaling_ratios = {
    'GND': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'R': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'C': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'L': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'D': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'V': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'I': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'S': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'J': {'vertical_scale': 6/5, 'horizontal_scale': 12/11},
    'Q': {'vertical_scale': 300/209, 'horizontal_scale': 600/451},
    'M': {'vertical_scale': 6/5, 'horizontal_scale': 90/82},
    'E': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'H': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'F': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'G': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'node': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'ground': {'vertical_scale': 1.0, 'horizontal_scale': 1.0}
}

# 元件映射
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
    'M': None,  # later replaced with MOSFET model
    'E': elm.SourceControlledV,
    'H': elm.SourceControlledV,
    'F': elm.SourceControlledI,
    'G': elm.SourceControlledI,
    'ground': elm.Ground,
    'node': elm.Dot(radius=dot_radius)
}


def parse_spice_netlist(netlist):
    print("[DEBUG] Starting to parse netlist...")
    components = []
    commands = []
    subcircuits = []
    for line_number, line in enumerate(netlist.strip().split('\n'), start=1):
        original_line = line
        if ';' in line:
            line = line.split(';')[0].strip()
        line = line.strip()
        if line.startswith('*'):
            continue
        if not line:
            continue

        try:
            parts = line.split()
            if not parts:
                continue

            first_char = parts[0][0].upper()

            if first_char in ['V', 'I', 'R', 'C', 'L', 'D', 'M', 'Q', 'J', 'S', 'E', 'F', 'G', 'H']:
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
                        'scale': 1.0
                    })
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
                        'scale': 1.0
                    })
                    continue
                elif first_char == 'M':
                    if len(parts) < 6:
                        raise ValueError(f"Invalid MOSFET definition: {line}")
                    cid = parts[0]
                    nodes = parts[1:5]
                    model = parts[5]
                    optional_params = parts[6:]
                    params_dict = {}
                    for param in optional_params:
                        if '=' in param:
                            key, val = param.split('=', 1)
                            params_dict[key.upper()] = val
                    components.append({
                        'type': first_char,
                        'id': cid,
                        'nodes': nodes,
                        'model': model,
                        'parameters': params_dict,
                        'pins': ['drain', 'gate', 'source', 'bulk'],
                        'scale': 1.0
                    })
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
                    'scale': 1.0
                })

            elif first_char == 'X':
                if len(parts) < 4:
                    raise ValueError(f"Invalid subcircuit definition: {line}")
                subcircuits.append({
                    'type': 'X',
                    'id': parts[0],
                    'nodes': parts[1:-1],
                    'subckt': parts[-1]
                })
            elif line.startswith('.'):
                commands.append({'command': parts[0][1:], 'params': parts[1:]})
            else:
                raise ValueError(f"Unknown element or command: {line}")
        except (IndexError, ValueError) as e:
            print(f"[ERROR] Parsing line {line_number}: {original_line}")
            print(f"        {e}")
            continue

    return components, commands, subcircuits


def netlist_to_xml(components, commands, subcircuits):
    root = ET.Element("spice_netlist")

    components_elem = ET.SubElement(root, "components")
    for comp in components:
        comp_type = comp['type']
        cid = comp['id']
        comp_elem = ET.SubElement(components_elem, "component", type=comp_type, id=cid)
        for i, node in enumerate(comp['nodes'], start=1):
            ET.SubElement(comp_elem, f"node{i}").text = node

        if 'model' in comp:
            ET.SubElement(comp_elem, "model").text = comp['model']
        if 'value' in comp:
            ET.SubElement(comp_elem, "value").text = comp['value']

        if 'parameters' in comp:
            params_elem = ET.SubElement(comp_elem, "parameters")
            for key, val in comp['parameters'].items():
                ET.SubElement(params_elem, key).text = val

        if 'pins' in comp:
            ET.SubElement(comp_elem, "pins").text = ' '.join(comp['pins'])

        comp_direction = default_directions.get(comp_type, 'right')
        comp_elem.set("direction", comp_direction)
        comp_flip = default_flips.get(comp_type, 'none')
        comp_elem.set("flip", comp_flip)

        scaling = default_scaling_ratios.get(comp_type, {'vertical_scale': 1.0, 'horizontal_scale': 1.0})
        comp_elem.set("vertical_scale", str(scaling.get('vertical_scale', 1.0)))
        comp_elem.set("horizontal_scale", str(scaling.get('horizontal_scale', 1.0)))

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
    rough_string = ET.tostring(element, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")


def convert_netlist_to_xml_file(netlist, output_filename="spice_netlist.xml"):
    components, commands, subcircuits = parse_spice_netlist(netlist)
    xml_root = netlist_to_xml(components, commands, subcircuits)
    pretty_xml = pretty_print_xml(xml_root)
    with open(output_filename, "w") as f:
        f.write(pretty_xml)


def create_graph_from_xml(xml_root):
    components_element = xml_root.find('components')
    if components_element is None or len(components_element) == 0:
        raise ValueError("No components found in XML.")

    G = nx.MultiGraph()

    for component in components_element:
        ctype = component.attrib['type']
        cid = component.attrib['id']

        value_elem = component.find('value')
        model_elem = component.find('model')

        if model_elem is not None and model_elem.text:
            model = model_elem.text.upper()
        else:
            model = ''

        if value_elem is not None and value_elem.text:
            value = value_elem.text
        else:
            value = ''

        parameters = {}
        params_elem = component.find('parameters')
        if params_elem is not None:
            for param in params_elem:
                parameters[param.tag] = param.text

        nodes = []
        i = 1
        while component.find(f'node{i}') is not None:
            node = component.find(f'node{i}').text
            if node:
                nodes.append(node)
            i += 1

        G.add_node(cid, type=ctype, value=value, model=model, parameters=parameters)

        for node in nodes:
            if node == '0':
                G.add_node(node, type='ground')
            else:
                G.add_node(node, type='node')
            G.add_edge(cid, node)

    return G, components_element


def align_to_grid(position, grid_size=1.0):
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
        d.add(elm.Line().at((x, -grid_extent)).to((x, grid_extent)).color('lightgray').linewidth(grid_width))
    for y in np.arange(-grid_extent, grid_extent + grid_size, grid_size):
        d.add(elm.Line().at((-grid_extent, y)).to((grid_extent, y)).color('lightgray').linewidth(grid_width))
    print(f"[DEBUG] Drawn grid with size {grid_size} and extent {grid_extent}.")



def update_xml_positions_and_directions(xml_root, pos, directions, flips=None, scaling_ratios=None):
    if flips is None:
        flips = {}
    if scaling_ratios is None:
        scaling_ratios = {}
    components_elem = xml_root.find('components')
    for comp in components_elem:
        cid = comp.attrib['id']
        if cid in pos:
            x, y = pos[cid]
            comp.set('x', str(x))
            comp.set('y', str(y))
        if cid in directions:
            comp.set("direction", directions[cid])
        if cid in flips:
            comp.set("flip", flips[cid])
        if cid in scaling_ratios:
            scaling = scaling_ratios[cid]
            comp.set("vertical_scale", str(scaling.get('vertical_scale', 1.0)))
            comp.set("horizontal_scale", str(scaling.get('horizontal_scale', 1.0)))

    nodes_elem = xml_root.find('nodes')
    if nodes_elem is None:
        nodes_elem = ET.SubElement(xml_root, 'nodes')
    for nid in pos:
        # 如果 components 里已经有同名id，就别重复
        if any(comp.attrib['id'] == nid for comp in components_elem):
            continue
        node_elem = next((node for node in nodes_elem if node.attrib.get('id') == nid), None)
        if node_elem is None:
            node_elem = ET.SubElement(nodes_elem, 'node', id=nid)
        x, y = pos[nid]
        node_elem.set('x', str(x))
        node_elem.set('y', str(y))


def add_anchor_dots(d, all_wires):
    n_dot = 0
    anchor_wire_count = {}
    for wire in all_wires:
        line = wire['line']
        start_pt = line.coords[0]
        end_pt   = line.coords[1]
        anchor_wire_count[start_pt] = anchor_wire_count.get(start_pt, 0) + 1
        anchor_wire_count[end_pt]   = anchor_wire_count.get(end_pt, 0) + 1

    added_dots = set()
    for anchor_pos, count in anchor_wire_count.items():
        if count >= 3 and anchor_pos not in added_dots:
            d.add(elm.Dot(radius=0.12).at(anchor_pos))
            added_dots.add(anchor_pos)
        if count >= 2:
            n_dot += 1
    return n_dot


def build_comp_node_mapping(components):
    comp_node_mapping = {}
    for comp in components:
        ctype = comp.attrib['type']
        cid   = comp.attrib['id']

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
            if len(nodes) < 3: continue
            node_collector = nodes[0]
            node_base      = nodes[1]
            node_emitter   = nodes[2]
            mapping = {}
            mapping.setdefault(node_collector, []).append('collector')
            mapping.setdefault(node_base,      []).append('base')
            mapping.setdefault(node_emitter,   []).append('emitter')
            comp_node_mapping[cid] = mapping

        elif ctype == 'D':
            if len(nodes) < 2: continue
            node_anode   = nodes[0]
            node_cathode = nodes[1]
            mapping = {}
            mapping.setdefault(node_anode,   []).append('anode')
            mapping.setdefault(node_cathode, []).append('cathode')
            comp_node_mapping[cid] = mapping

        elif ctype in ['R', 'C', 'L', 'S']:
            if len(nodes) < 2: continue
            node_start = nodes[0]
            node_end   = nodes[1]
            mapping = {}
            mapping.setdefault(node_start, []).append('start')
            mapping.setdefault(node_end,   []).append('end')
            comp_node_mapping[cid] = mapping

        elif ctype in ['V', 'I', 'E', 'H', 'F', 'G']:
            if len(nodes) < 2: continue
            node_positive = nodes[0]
            node_negative = nodes[1]
            mapping = {}
            mapping.setdefault(node_positive, []).append('positive')
            mapping.setdefault(node_negative, []).append('negative')
            comp_node_mapping[cid] = mapping

        elif ctype == 'J':
            if len(nodes) < 3: continue
            node_drain  = nodes[0]
            node_gate   = nodes[1]
            node_source = nodes[2]
            mapping = {}
            mapping.setdefault(node_drain,  []).append('drain')
            mapping.setdefault(node_gate,   []).append('gate')
            mapping.setdefault(node_source, []).append('source')
            comp_node_mapping[cid] = mapping

        elif ctype == 'M':
            if len(nodes) < 4: continue
            node_drain  = nodes[0]
            node_gate   = nodes[1]
            node_source = nodes[2]
            node_bulk   = nodes[3]
            mapping = {}
            mapping.setdefault(node_drain,  []).append('drain')
            mapping.setdefault(node_gate,   []).append('gate')
            mapping.setdefault(node_source, []).append('source')
            mapping.setdefault(node_bulk,   []).append('bulk')
            comp_node_mapping[cid] = mapping
    return comp_node_mapping


def get_component_pin(comp, neighbor, comp_info, components_element, pins, pin_mapping):
    """根据器件类型，为连线分配具体的引脚名字。"""
    ctype = comp_info.get('type')
    # BJT, Diode, MOSFET, JFET, R, C, L, S, V, I等用 pin_mapping
    if ctype in ['Q', 'D', 'M', 'J','R', 'C', 'L', 'S', 'V', 'I', 'E', 'H', 'F', 'G']:
        if comp in pin_mapping:
            mapping = pin_mapping.get(comp, {})
            if neighbor in mapping and mapping[neighbor]:
                pin = mapping[neighbor].pop(0)
                return pin
            else:
                return None
    # 如果都不匹配，默认为 'pin'
    return 'pin'


def get_pin_position(pins, node, pin_name, grid_size=1.0):
    """从 pins 里取出该 pin 的坐标，并对齐网格。"""
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
        return None


def is_wire_crossing_components(start_pos, end_pos, component_boxes, comp, neighbor):
    line = LineString([start_pos, end_pos])
    line_start_point = Point(start_pos)
    line_end_point   = Point(end_pos)
    for comp_id, comp_info in component_boxes.items():
        polygon = comp_info['polygon']
        if line.intersects(polygon):
            intersection = line.intersection(polygon)
            if (intersection.equals(line_start_point) or intersection.equals(line_end_point)) \
               and (comp_id == comp or comp_id == neighbor):
                continue
            else:
                return True
    return False


def route_connection_current_method(start_pos, end_pos, component_boxes, comp, neighbor, all_wires, grid_size=1.0):
    """使用曼哈顿路由方法连接起点和终点。"""
    start_pos = align_to_grid(start_pos, grid_size)
    end_pos   = align_to_grid(end_pos,   grid_size)

    mid_pos1 = align_to_grid((end_pos[0], start_pos[1]), grid_size)
    path1 = [start_pos, mid_pos1, end_pos]

    mid_pos2 = align_to_grid((start_pos[0], end_pos[1]), grid_size)
    path2 = [start_pos, mid_pos2, end_pos]

    def path_ok(path):
        for i in range(len(path) - 1):
            if is_wire_crossing_components(path[i], path[i + 1], component_boxes, comp, neighbor):
                return False
            seg = LineString([path[i], path[i + 1]])
            if check_overlapping_wires(all_wires):
                return False
            for w in all_wires:
                lw = w['line']
                if seg.overlaps(lw):
                    return False
        return True

    if path_ok(path1):
        return path1, wire_safe_color
    if path_ok(path2):
        return path2, wire_safe_color
    return path1, wire_danger_color


def check_overlapping_wires(all_wires, eps=1e-9):
    """
    Detect if any wires are overlapping or partially overlapping.
    Returns True if there is an overlap.
    """
    overlapping = False
    num_wires   = len(all_wires)

    for i in range(num_wires):
        line_i = all_wires[i]['line']
        for j in range(i + 1, num_wires):
            line_j = all_wires[j]['line']

            # 1) Exact equality check: lines are identical
            if line_i.equals(line_j):
                print(f"[DEBUG] Wires {i} and {j} completely overlap (equal).")
                overlapping = True
                all_wires[i]['color'] = wire_danger_color
                all_wires[j]['color'] = wire_danger_color
                continue

            # 2) Overlap check (shapely 'overlaps' means partial or full geometry overlap)
            if line_i.overlaps(line_j):
                print(f"[DEBUG] Wires {i} and {j} partially overlap.")
                overlapping = True
                all_wires[i]['color'] = wire_danger_color
                all_wires[j]['color'] = wire_danger_color
                continue

            # 3) Intersection check for partial overlap (line intersection can return a line)
            inter = line_i.intersection(line_j)
            if not inter.is_empty:
                # If intersection is a LineString/MultiLineString, we have partial overlap
                if inter.geom_type in ["LineString", "MultiLineString"]:
                    print(f"[DEBUG] Wires {i} and {j} partially overlap along a line.")
                    overlapping = True
                    all_wires[i]['color'] = wire_danger_color
                    all_wires[j]['color'] = wire_danger_color
                # If you want to treat small overlaps as the same, you can do:
                # if line_i.buffer(eps).overlaps(line_j.buffer(eps)):
                #    ...

    return overlapping



def count_line_intersections(all_wires):
    intersection_points = set()
    lines = [w['line'] for w in all_wires]
    num_lines = len(lines)

    for i in range(num_lines):
        for j in range(i+1, num_lines):
            line_i = lines[i]
            line_j = lines[j]
            inter  = line_i.intersection(line_j)
            if inter.is_empty:
                continue

            if inter.geom_type == "Point":
                intersection_points.add((round(inter.x, 6), round(inter.y, 6)))
            elif inter.geom_type == "MultiPoint":
                for p in inter.geoms:
                    intersection_points.add((round(p.x, 6), round(p.y, 6)))
            elif inter.geom_type in ["LineString", "MultiLineString"]:
                coords = inter.coords
                for p in coords:
                    intersection_points.add((round(p[0], 6), round(p[1], 6)))
    return len(intersection_points)


def set_element_direction(element, direction):
    if direction == 'up':
        element.up()
    elif direction == 'down':
        element.down()
    elif direction == 'left':
        element.left()
    else:
        element.right()


def rotate_ccw(x, y, cx, cy, theta=0):
    rad = math.radians(theta)
    x_new = (x - cx) * math.cos(rad) - (y - cy) * math.sin(rad) + cx
    y_new = (x - cx) * math.sin(rad) + (y - cy) * math.cos(rad) + cy
    return x_new, y_new


def flip_component(points, flip_point, mode="horizontal"):
    xf, yf = flip_point
    flipped_points = []
    for x, y in points:
        if mode == "horizontal":
            new_x = 2 * xf - x
            new_y = y
        elif mode == "vertical":
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
    # MOSFET 特殊处理
    if component_type == 'M':
        model = kwargs.get('model', '').upper()
        if model.startswith('P'):
            try:
                comp_class = elm.PFet
            except AttributeError:
                print("Error: schemdraw does not have 'PFet' element.")
                return
        else:
            try:
                comp_class = elm.NFet
            except AttributeError:
                print("Error: schemdraw does not have 'NFet' element.")
                return
    else:
        comp_class = component_map.get(component_type, None)
        if comp_class is None:
            raise ValueError(f"Unsupported component type: {component_type}")

    if component_type == 'M':
        bulk = kwargs.get('bulk', False)
        element = comp_class(bulk=bulk).at(pos)
    else:
        element = comp_class(**kwargs).at(pos)

    set_element_direction(element, direction)

    d = schemdraw.Drawing(show=False)
    element.scalex(horizontal_scale).scaley(vertical_scale)
    element = d.add(element)

    if 'start' in element.anchors:
        ref_anchor_name = 'start'
    elif 'drain' in element.anchors:
        if component_type == 'M':
            ref_anchor_name = 'source'
        else:
            ref_anchor_name = 'drain'
    elif 'base' in element.anchors:
        ref_anchor_name = 'base'
    else:
        ref_anchor_name = list(element.anchors.keys())[0]

    abs_ref   = element.absanchors[ref_anchor_name]
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

    if component_type in ['GND', 'ground']:
        ymax_global -= shrink_size
    elif component_type in ['R', 'C', 'L', 'D', 'V', 'I', 'E', 'H', 'F', 'G', 'S']:
        xmin_global += shrink_size
        xmax_global -= shrink_size
    elif component_type == 'J':
        xmax_global -= shrink_size
        ymin_global += shrink_size
        ymax_global -= shrink_size
        xmin_global -= shrink_size
    elif component_type == 'Q':
        xmin_global += shrink_size
        ymin_global += shrink_size
        ymax_global -= shrink_size
        xmax_global += shrink_size * 2
    elif component_type == 'M':
        xmin_global += shrink_size
        ymin_global += shrink_size
        ymax_global -= shrink_size
        xmax_global -= shrink_size

    anchors_global = {k: (v.x, v.y) for k, v in element.absanchors.items()}
    bbox = {
        'xmin': xmin_global,
        'ymin': ymin_global,
        'xmax': xmax_global,
        'ymax': ymax_global
    }

    vertices = [
        (bbox['xmin'], bbox['ymin']),
        (bbox['xmax'], bbox['ymin']),
        (bbox['xmax'], bbox['ymax']),
        (bbox['xmin'], bbox['ymax'])
    ]

    (x_center, y_center) = pos
    if direction == 'up':
        rotated_vertices = [rotate_ccw(x, y, x_center, y_center, 90) for x, y in vertices]
    elif direction == 'left':
        rotated_vertices = [rotate_ccw(x, y, x_center, y_center, 180) for x, y in vertices]
    elif direction == 'down':
        rotated_vertices = [rotate_ccw(x, y, x_center, y_center, 270) for x, y in vertices]
    else:
        rotated_vertices = vertices

    rotated_x = [v[0] for v in rotated_vertices]
    rotated_y = [v[1] for v in rotated_vertices]
    bbox = {
        'xmin': min(rotated_x),
        'ymin': min(rotated_y),
        'xmax': max(rotated_x),
        'ymax': max(rotated_y)
    }

    if component_type in ['J', 'M']:
        flipped_vertices = flip_component(rotated_vertices, (x_center, y_center), mode=flip)
    elif component_type == 'Q':
        collector = element.absanchors['collector']
        if direction in ['up', 'down']:
            if flip == 'horizontal':
                flipped_vertices = flip_component(rotated_vertices, (x_center, y_center), mode=flip)
            elif flip == 'vertical':
                flipped_vertices = flip_component(rotated_vertices, (collector.x, collector.y), mode=flip)
            else:
                flipped_vertices = rotated_vertices
        else:
            if flip == 'horizontal':
                flipped_vertices = flip_component(rotated_vertices, (collector.x, collector.y), mode=flip)
            elif flip == 'vertical':
                flipped_vertices = flip_component(rotated_vertices, (x_center, y_center), mode=flip)
            else:
                flipped_vertices = rotated_vertices
    else:
        flipped_vertices = rotated_vertices

    rotated_x = [v[0] for v in flipped_vertices]
    rotated_y = [v[1] for v in flipped_vertices]
    bbox = {
        'xmin': min(rotated_x),
        'ymin': min(rotated_y),
        'xmax': max(rotated_x),
        'ymax': max(rotated_y)
    }

    return bbox, element.anchors, anchors_global


def draw_component_or_node(d, elements, pins, node, node_info, x, y, direction, component_boxes, G,
                           flip='none', scaling_ratios=None):
    if scaling_ratios is None:
        scaling_ratios = {}
    ctype = node_info.get('type')
    if ctype is None:
        ctype = 'node'

    deg = G.degree(node)

    if ctype == 'node':
        if deg >= 3:
            elements[node] = d.add(elm.Dot(radius=0.12).at((x, y)))
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

    try:
        if ctype_for_bbox == 'M':
            model = node_info.get('model', '').upper()
            if not model:
                model = node_info.get('parameters', {}).get('MODEL', '').upper()
            if model.startswith('P'):
                try:
                    comp_class = elm.PFet
                except AttributeError:
                    print("Error: No PFet in schemdraw.")
                    return
            else:
                try:
                    comp_class = elm.NFet
                except AttributeError:
                    print("Error: No NFet in schemdraw.")
                    return
            bulk = 'bulk' in node_info.get('pins', [])
            params = node_info.get('parameters', {})
            params = {k: v for k, v in params.items() if k != 'MODEL'}
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
        if model.startswith('P'):
            try:
                element = elm.PFet(bulk=True, scale=1.0).at((x, y))
            except AttributeError:
                print("Error: schemdraw does not have 'PFet'.")
                return
        else:
            try:
                element = elm.NFet(bulk=True, scale=1.0).at((x, y))
            except AttributeError:
                print("Error: schemdraw does not have 'NFet'.")
                return
    else:
        comp_class = component_map.get(ctype_for_bbox, elm.Dot(radius=0.12))
        if comp_class is None:
            print(f"[ERROR] Unsupported component type: {ctype_for_bbox}")
            return
        element = comp_class(**node_info.get('parameters', {})).at((x, y))

    set_element_direction(element, direction)

    if direction in ['up', 'down']:
        if flip == 'horizontal':
            element.flip()
        elif flip == 'vertical':
            element.reverse()
    else:
        if flip == 'horizontal':
            element.reverse()
        elif flip == 'vertical':
            element.flip()

    element.label(label_text)
    element.scalex(horizontal_scale).scaley(vertical_scale)
    element = d.add(element)

    if ctype_for_bbox in ['V', 'I', 'E', 'H', 'F', 'G']:
        pins[node] = {
            'positive': (element.absanchors['end'].x,   element.absanchors['end'].y),
            'negative': (element.absanchors['start'].x, element.absanchors['start'].y)
        }
    elif ctype_for_bbox in ['R', 'C', 'L', 'S']:
        pins[node] = {
            'start': (element.absanchors['start'].x, element.absanchors['start'].y),
            'end':   (element.absanchors['end'].x,   element.absanchors['end'].y)
        }
    elif ctype_for_bbox == 'D':
        pins[node] = {
            'anode':   (element.absanchors['start'].x, element.absanchors['start'].y),
            'cathode': (element.absanchors['end'].x,   element.absanchors['end'].y)
        }
    elif ctype_for_bbox == 'J':
        pins[node] = {
            'drain':  (element.absanchors['drain'].x,  element.absanchors['drain'].y),
            'gate':   (element.absanchors['gate'].x,   element.absanchors['gate'].y),
            'source': (element.absanchors['source'].x, element.absanchors['source'].y)
        }
    elif ctype_for_bbox == 'Q':
        pins[node] = {
            'collector': (element.absanchors['collector'].x, element.absanchors['collector'].y),
            'base':      (element.absanchors['base'].x,      element.absanchors['base'].y),
            'emitter':   (element.absanchors['emitter'].x,   element.absanchors['emitter'].y)
        }
    elif ctype_for_bbox == 'M':
        pins[node] = {
            'drain':  (element.absanchors['drain'].x,  element.absanchors['drain'].y),
            'gate':   (element.absanchors['gate'].x,   element.absanchors['gate'].y),
            'source': (element.absanchors['source'].x, element.absanchors['source'].y),
            'bulk':   (element.absanchors['bulk'].x,   element.absanchors['bulk'].y)
        }
    elif ctype_for_bbox == 'ground':
        pins[node] = {'pin': (x, y)}
    elif ctype_for_bbox == 'node':
        pins[node] = {'pin': (x, y)}
    else:
        pins[node] = {'pin': (x, y)}

    x_min, y_min, x_max, y_max = bbox['xmin'], bbox['ymin'], bbox['xmax'], bbox['ymax']
    component_boxes[node] = {
        'bbox': bbox,
        'polygon': Polygon([(x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max)]),
        'type': ctype_for_bbox
    }


def draw_connections(d, G, components_element, pins, component_boxes, drawn_edges, routing_method,
                     all_wires, pin_mapping, pos, grid_size=1.0):

    """这里面做实际的线路连线。"""
    for comp, neighbor, key in G.edges(keys=True):
        edge_nodes = tuple(sorted([comp, neighbor]))
        edge_key = edge_nodes + (key,)
        if edge_key in drawn_edges:
            continue

        comp_info = G.nodes[comp]
        neighbor_info = G.nodes[neighbor]

        comp_pin = get_component_pin(comp, neighbor, comp_info, components_element, pins, pin_mapping)
        neighbor_pin = get_component_pin(neighbor, comp, neighbor_info, components_element, pins, pin_mapping)
        if comp_pin is None or neighbor_pin is None:
            continue

        start_pos = get_pin_position(pins, comp, comp_pin, grid_size=grid_size)
        end_pos   = get_pin_position(pins, neighbor, neighbor_pin, grid_size=grid_size)
        if start_pos is None or end_pos is None:
            continue

        selected_path, line_color = route_connection_current_method(
            start_pos, end_pos, component_boxes, comp, neighbor, all_wires, grid_size=grid_size
        )



        for i in range(len(selected_path) - 1):
            segment_start = selected_path[i]
            segment_end   = selected_path[i + 1]
            new_line      = LineString([segment_start, segment_end])
            all_wires.append({'line': new_line, 'color': line_color})

        drawn_edges.add(edge_key)

def draw_circuit(G, components_element, xml_root, xml_file, initial_comp_node_mapping,
                 max_attempts=100, EnlargeSize=2.5, routing_method=1, auto=1, grid_size=1.0):
    attempt = 0
    success = False

    last_pos = None
    last_all_wires = None

    positions_in_xml = {}
    directions_in_xml = {}
    flips_in_xml = {}
    scaling_ratios = {}

    components_elem = xml_root.find('components')
    if components_elem is not None:
        for comp in components_elem:
            cid = comp.attrib['id']
            dir_val  = comp.attrib.get('direction', 'right')
            flip_val = comp.attrib.get('flip', 'none')
            vertical_scale   = float(comp.attrib.get('vertical_scale', '1.0'))
            horizontal_scale = float(comp.attrib.get('horizontal_scale', '1.0'))
            scaling_ratios[cid] = {
                'vertical_scale': vertical_scale,
                'horizontal_scale': horizontal_scale
            }
            directions_in_xml[cid] = dir_val
            flips_in_xml[cid]      = flip_val
            if 'x' in comp.attrib and 'y' in comp.attrib:
                positions_in_xml[cid] = (float(comp.attrib['x']), float(comp.attrib['y']))

    nodes_elem = xml_root.find('nodes')
    if nodes_elem is not None:
        for node in nodes_elem:
            nid = node.attrib['id']
            if 'x' in node.attrib and 'y' in node.attrib:
                positions_in_xml[nid] = (float(node.attrib['x']), float(node.attrib['y']))

    if auto == 1:
        # -------------------------
        # 自动布局
        # -------------------------
        while attempt < max_attempts:
            attempt += 1
            print(f"Attempt {attempt}...")
            original_pos = nx.spring_layout(G)
            pos = {n: (EnlargeSize * original_pos[n][0],
                       EnlargeSize * (-original_pos[n][1])) for n in G.nodes}
            pos = {n: align_to_grid(pos[n], grid_size=grid_size) for n in pos}

            directions = {}
            flips = {}
            scales = {}
            for n in G.nodes:
                directions[n] = directions_in_xml.get(n, 'right')
                flips[n]      = flips_in_xml.get(n,   'none')
                scales[n]     = scaling_ratios.get(n,
                                  default_scaling_ratios.get(G.nodes[n]['type'],
                                  {'vertical_scale':1.0, 'horizontal_scale':1.0}))

            d = schemdraw.Drawing()
            if draw_grid_or_not == 1:
                draw_grid(d, grid_size=grid_size)

            elements = {}
            pins = {}
            component_boxes = {}
            drawn_edges = set()
            all_wires = []

            # 1) 绘制所有节点
            for node in G.nodes:
                node_info = G.nodes[node]
                x, y      = pos[node]
                direction = directions.get(node, 'right')
                flip_in_xml = flips.get(node, 'none')
                scale    = scales.get(node, {'vertical_scale':1.0, 'horizontal_scale':1.0})
                draw_component_or_node(d, elements, pins, node, node_info, x, y,
                                       direction, component_boxes, G,
                                       flip=flip_in_xml, scaling_ratios=scales)
                scales[node] = scale

            # 2) 针对无向器件，比较两种引脚分配距离并做调整
            for comp in G.nodes:
                ctype = G.nodes[comp].get('type','')
                # 只关心两引脚无向器件
                if ctype in ['R','C','L','S']:
                    comp_pins = pins.get(comp, {})
                    if 'start' not in comp_pins or 'end' not in comp_pins:
                        continue
                    neighbors = list(G[comp])  # 该器件连了哪些node
                    if len(neighbors) == 2:
                        node1, node2 = neighbors[0], neighbors[1]
                        # 若 node1,node2 也都在 pins 里，则各自都有一个 'pin' 或其它坐标
                        if node1 in pins and node2 in pins:
                            # 不同类型节点（node/device）可能存在多引脚，这里假设无向器件只连普通 node
                            # 若要更精细处理，你也可以检查 pins[node1] 的 key
                            pos1_key = list(pins[node1].keys())[0]
                            pos2_key = list(pins[node2].keys())[0]
                            node1_pos = np.array(pins[node1][pos1_key])
                            node2_pos = np.array(pins[node2][pos2_key])

                            start_pos = np.array(comp_pins['start'])
                            end_pos   = np.array(comp_pins['end'])

                            distance1 = np.linalg.norm(start_pos - node1_pos) + \
                                        np.linalg.norm(end_pos   - node2_pos)
                            distance2 = np.linalg.norm(start_pos - node2_pos) + \
                                        np.linalg.norm(end_pos   - node1_pos)

                            if distance2 < distance1:
                                # 交换
                                pins[comp]['start'], pins[comp]['end'] = pins[comp]['end'], pins[comp]['start']

            # 3) 连接线路
            pin_mapping_copy = copy.deepcopy(initial_comp_node_mapping)
            draw_connections(d, G, components_element, pins, component_boxes, drawn_edges,
                             routing_method, all_wires, pin_mapping_copy, pos, grid_size=grid_size)

            any_red_line = any(wire['color'] == wire_danger_color for wire in all_wires)
            overlapping  = check_overlapping_wires(all_wires)

            last_pos = pos
            last_all_wires = all_wires

            if any_red_line or overlapping:
                continue
            else:
                for wire_info in all_wires:
                    line  = wire_info['line']
                    color = wire_info['color']
                    d.add(elm.Line().at(line.coords[0]).to(line.coords[1]).color(color))
                n_dot = add_anchor_dots(d, all_wires)

                update_xml_positions_and_directions(xml_root, pos, directions, flips, scales)
                tree = ET.ElementTree(xml_root)
                tree.write(xml_file)
                # 可以统计交点
                n_inter = count_line_intersections(all_wires)
                print(f"horizontal and vertical line intersection number: {n_inter}")
                print(f"connection dots number: {n_dot}")
                n_real_inter = n_inter - n_dot
                print(f"real intersection number: {n_real_inter}")
                if n_real_inter > max_intersections:
                    continue
                # 调用绘图函数
                plot_networkx_graph(G, pos=pos)
                d.draw()
                success = True
                print(f"Attempt {attempt} succeeded.")
                break

        if not success:
            # 自动布局尝试所有方案都没成功，画最后一次
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
                    x, y      = last_pos[node]
                    direction = directions_in_xml.get(node, 'right')
                    flip_in_xml = flips_in_xml.get(node, 'none')
                    scale = scaling_ratios.get(node, {'vertical_scale':1.0, 'horizontal_scale':1.0})
                    draw_component_or_node(d, elements, pins, node, node_info, x, y,
                                           direction, component_boxes, G,
                                           flip=flip_in_xml, scaling_ratios=scaling_ratios)
                    scaling_ratios[node] = scale

                for wire_info in last_all_wires:
                    line  = wire_info['line']
                    color = wire_info['color']
                    d.add(elm.Line().at(line.coords[0]).to(line.coords[1]).color(color))
                n_dot = add_anchor_dots(d, last_all_wires)

                n_inter = count_line_intersections(all_wires)
                print(f"horizontal and vertical line intersection number: {n_inter}")
                print(f"connection dots number: {n_dot}")
                n_real_inter = n_inter - n_dot
                print(f"real intersection number: {n_real_inter}")
                d.draw()

    else:
        # -------------------------
        # 固定布局
        # -------------------------
        d = schemdraw.Drawing()
        if draw_grid_or_not == 1:
            draw_grid(d, grid_size=grid_size)
        elements = {}
        pins = {}
        component_boxes = {}
        drawn_edges = set()
        all_wires = []

        pos         = positions_in_xml
        directions  = directions_in_xml
        flips       = flips_in_xml

        # 1) 先把元件/节点绘制出来
        for node in G.nodes:
            node_info = G.nodes[node]
            if node not in pos:
                print(f"[WARNING] Node '{node}' 没有定义位置，跳过。")
                continue
            x, y       = pos[node]
            direction  = directions.get(node, 'right')
            flip_val   = flips.get(node, 'none')
            scale      = scaling_ratios.get(node, default_scaling_ratios.get(G.nodes[node]['type'],
                                  {'vertical_scale':1.0,'horizontal_scale':1.0}))
            draw_component_or_node(d, elements, pins, node, node_info, x, y,
                                   direction, component_boxes, G,
                                   flip=flip_val, scaling_ratios=scaling_ratios)
            scaling_ratios[node] = scale

        # 2) 做无向器件的“引脚-节点最近距离”分配
        for comp in G.nodes:
            ctype = G.nodes[comp].get('type','')
            if ctype in ['R','C','L','S']:
                comp_pins = pins.get(comp, {})
                if 'start' not in comp_pins or 'end' not in comp_pins:
                    continue
                neighbors = list(G[comp])
                if len(neighbors) == 2:
                    node1, node2 = neighbors[0], neighbors[1]
                    if node1 in pins and node2 in pins:
                        pos1_key = list(pins[node1].keys())[0]
                        pos2_key = list(pins[node2].keys())[0]
                        node1_pos = np.array(pins[node1][pos1_key])
                        node2_pos = np.array(pins[node2][pos2_key])

                        start_pos = np.array(comp_pins['start'])
                        end_pos   = np.array(comp_pins['end'])

                        distance1 = np.linalg.norm(start_pos - node1_pos) + \
                                    np.linalg.norm(end_pos   - node2_pos)
                        distance2 = np.linalg.norm(start_pos - node2_pos) + \
                                    np.linalg.norm(end_pos   - node1_pos)

                        if distance2 < distance1:
                            pins[comp]['start'], pins[comp]['end'] = pins[comp]['end'], pins[comp]['start']

        # 3) 画线路
        pin_mapping_copy = copy.deepcopy(initial_comp_node_mapping)
        draw_connections(d, G, components_element, pins, component_boxes, drawn_edges,
                         routing_method, all_wires, pin_mapping_copy, pos, grid_size=grid_size)

        any_red_line = any(wire['color'] == wire_danger_color for wire in all_wires)
        overlapping  = check_overlapping_wires(all_wires)

        if any_red_line or overlapping:
            print("[WARNING] 存在红色线条或重叠线条，部分连接可能有问题。")

        # 把线段画出来
        for wire_info in all_wires:
            line  = wire_info['line']
            color = wire_info['color']
            d.add(elm.Line().at(line.coords[0]).to(line.coords[1]).color(color))
        n_dot = add_anchor_dots(d, all_wires)

        update_xml_positions_and_directions(xml_root, pos, directions, flips, scaling_ratios)
        tree = ET.ElementTree(xml_root)
        tree.write(xml_file)

        # 如果想在固定布局模式下也查看交点数：
        n_inter = count_line_intersections(all_wires)
        print(f"horizontal and vertical line intersection number: {n_inter}")
        print(f"connection dots number: {n_dot}")
        n_real_inter = n_inter - n_dot
        print(f"real intersection number: {n_real_inter}")

        # 调用绘图函数
        plot_networkx_graph(G, pos=pos)
        d.draw()



def plot_networkx_graph(G, pos=None):
    """
    Draws the NetworkX graph with Bézier curves for multiple edges.
    :param G: NetworkX graph
    :param pos: Optional; dictionary of positions for nodes
    """
    if pos is None:
        pos = nx.spring_layout(G)  # Use spring layout if no positions are provided

    plt.figure(figsize=(10, 8))

    # Draw edges with Bézier curves for multi-edges
    for u, v, key, data in G.edges(keys=True, data=True):
        x_start, y_start = pos[u]
        x_end, y_end = pos[v]

        # Calculate curvature for multi-edges
        num_edges = len(G[u][v]) if G.has_edge(u, v) else 1
        edge_index = key - (num_edges - 1) / 2  # Offset index to center curves
        curvature = 0.2 * edge_index  # Adjust curvature factor if needed

        if curvature != 0:  # Bézier curve for multiple edges
            ctrl_x = (x_start + x_end) / 2 - curvature * (y_end - y_start)
            ctrl_y = (y_start + y_end) / 2 + curvature * (x_end - x_start)
            path = Path(
                [(x_start, y_start), (ctrl_x, ctrl_y), (x_end, y_end)],
                [Path.MOVETO, Path.CURVE3, Path.CURVE3],
            )
            patch = PathPatch(path, edgecolor="gray", facecolor="none", lw=1, alpha=0.7)  # Set facecolor to none
            plt.gca().add_patch(patch)
        else:  # Straight line for single edges
            plt.plot([x_start, x_end], [y_start, y_end], color="gray", lw=1, alpha=0.7)

    # Draw nodes
    nx.draw_networkx_nodes(G, pos, node_size=1500, node_color="lightblue")

    # Draw labels
    labels = {node: f"{node}\n{G.nodes[node].get('type', '')}" for node in G.nodes}
    nx.draw_networkx_labels(G, pos, labels=labels, font_size=8)

    # Display plot
    plt.title("NetworkX Graph Representation with Bézier Curves")
    plt.axis("off")
    plt.show()


def main():
    print("[DEBUG] Script started.")
    output_xml = "spice_netlist.xml"

    if auto == 1:
        convert_netlist_to_xml_file(netlist, output_xml)
    else:
        print("[DEBUG] Skipping netlist to XML conversion (auto=0). Make sure XML file exists or netlist is already converted.")

    try:
        tree = ET.parse(output_xml)
    except FileNotFoundError:
        print(f"[ERROR] '{output_xml}' not found. Make sure the netlist has been converted to XML.")
        sys.exit(1)
    except ET.ParseError as e:
        print(f"[ERROR] Error parsing XML: {e}")
        sys.exit(1)

    xml_root = tree.getroot()
    G, components_element = create_graph_from_xml(xml_root)

    # 建立器件-节点引脚映射
    initial_comp_node_mapping = build_comp_node_mapping(components_element)

    draw_circuit(
        G, components_element, xml_root, output_xml,
        initial_comp_node_mapping,
        max_attempts=max_attempts,
        EnlargeSize=EnlargeSize,
        routing_method=routing_method,
        auto=auto,
        grid_size=grid_size
    )
    print("[DEBUG] Script finished successfully.")


if __name__ == "__main__":
    main()
