import networkx as nx
import schemdraw
import schemdraw.elements as elm
import xml.etree.ElementTree as ET
import numpy as np
from shapely.geometry import LineString, Polygon, Point
from shapely.strtree import STRtree
import xml.dom.minidom as minidom
import math
from collections import defaultdict
import copy

netlist = """

*Q1 in in 0 N90 W=1u L=90n
*Q2 out in 0 N90 W=1u L=90n
Q1 d1 d1 0 N90 W=1u L=90n
Q2 d2 d1 0 N90 W=1u L=90n
Q3 in in d1 N90 W=1u L=90n
Q4 out in d2 N90 W=1u L=90n


Iin vdd in 10u

Vdd vdd D0 C 1.8
Vin out 0 DC 1.8

.dc Iin 0 10u 0.1u

.options savecurrents

.end

"""

EnlargeSize = 15
max_attempts = 300
routing_method = 1
auto = 0
shrink_size = 0.05
wire_safe_color = 'green'
wire_danger_color = 'red'

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
    'Q': 'right',
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
    'E': 'none',
    'H': 'none',
    'F': 'none',
    'G': 'none',
    'node': 'none',
    'ground': 'none'
}

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
    'E': elm.SourceControlledV,
    'H': elm.SourceControlledV,
    'F': elm.SourceControlledI,
    'G': elm.SourceControlledI,
    'ground': elm.Ground,
    'node': elm.Dot(radius=0.12)
}


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
                if first_char in ['V', 'I', 'S', 'E', 'F', 'G', 'H']:
                    value = ' '.join(parts[3:])
                    nodes = parts[1:3]
                elif first_char == 'D':
                    value = parts[3] if len(parts) > 3 else ''
                    nodes = parts[1:3]
                elif first_char == 'J':
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
                    if len(parts) < 4:
                        raise ValueError(f"Invalid BJT definition: {line}")
                    value = parts[4] if len(parts) > 4 else ''
                    nodes = parts[1:4]
                    components.append({
                        'type': first_char,
                        'id': parts[0],
                        'nodes': nodes,
                        'value': value,
                        'pins': ['collector', 'base', 'emitter']
                    })
                    continue
                else:
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
            print(f"Error parsing line: {line}\n{e}")
            continue

    return components, commands, subcircuits


def netlist_to_xml(components, commands, subcircuits):
    root = ET.Element("spice_netlist")

    components_elem = ET.SubElement(root, "components")
    for comp in components:
        comp_elem = ET.SubElement(components_elem, "component", type=comp['type'], id=comp['id'])
        #
        for i, node in enumerate(comp['nodes'], start=1):
            ET.SubElement(comp_elem, f"node{i}").text = node
        ET.SubElement(comp_elem, "value").text = comp['value']
        if 'pins' in comp:
            ET.SubElement(comp_elem, "pins").text = ' '.join(comp['pins'])
        print("type:", comp['type'])
        comp_direction = default_directions.get(comp['type'], 'right')
        comp_elem.set("direction", comp_direction)
        comp_flip = default_flips.get(comp['type'], 'none')
        comp_elem.set("flip", comp_flip)  # [NEW CODE] 默认无翻转

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
    print(pretty_xml)
    return xml_root


def create_graph_from_xml(xml_root):
    components_element = xml_root.find('components')
    if components_element is None or len(components_element) == 0:
        print("No components found in XML.")
        raise ValueError("No components found in XML.")

    G = nx.MultiGraph()

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


def update_xml_positions_and_directions(xml_root, pos, directions, flips=None):
    if flips is None:
        flips = {}
    components_elem = xml_root.find('components')
    for comp in components_elem:
        cid = comp.attrib['id']
        if cid in pos:
            x, y = pos[cid]
            comp.set('x', str(x))
            comp.set('y', str(y))
        if cid in directions:
            comp.set('direction', directions[cid])
        if cid in flips:  # [NEW CODE] 更新flip
            comp.set('flip', flips[cid])
    nodes_elem = xml_root.find('nodes')
    if nodes_elem is None:
        nodes_elem = ET.SubElement(xml_root, 'nodes')
    for nid in pos:
        if any(comp.attrib['id'] == nid for comp in components_elem):
            continue
        node_elem = next((node for node in nodes_elem if node.attrib.get('id') == nid), None)
        if node_elem is None:
            node_elem = ET.SubElement(nodes_elem, 'node', id=nid)
        x, y = pos[nid]
        node_elem.set('x', str(x))
        node_elem.set('y', str(y))


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


def build_comp_node_mapping(components):
    """
    根据解析后的组件列表，为每个组件构建节点到引脚的映射。
    """
    comp_node_mapping = {}
    for comp in components:
        ctype = comp.attrib['type']
        cid = comp.attrib['id']
        # 提取节点
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

        # 其他类型的组件可以按照需要继续添加

    return comp_node_mapping


def get_component_pin(comp, neighbor, comp_info, components_element, pins, pin_mapping):
    """
    根据组件和邻居节点，从 pin_mapping 中分配引脚。
    """
    ctype = comp_info.get('type')

    if comp in pin_mapping:
        mapping = pin_mapping.get(comp, {})
        if neighbor in mapping and mapping[neighbor]:
            pin = mapping[neighbor].pop(0)
            print(f"[DEBUG] Assigned pin '{pin}' for component '{comp}' to neighbor '{neighbor}'")
            return pin
        else:
            print(f"[DEBUG] No available pins for component '{comp}' connecting to '{neighbor}'")
            return None
    # 默认情况返回 'pin' 或者 None
    print(f"[DEBUG] Defaulting to 'pin' for component '{comp}' connecting to '{neighbor}'")
    return 'pin'


def get_pin_position(pins, comp, pin_name):
    try:
        pos = pins[comp][pin_name]
        if hasattr(pos, 'x') and hasattr(pos, 'y'):
            return pos.x, pos.y
        else:
            return pos
    except KeyError:
        print(f"Pin {pin_name} not found for component {comp}")
        return None


def get_node_coordinate(pins, node):
    # Try to find the coordinate corresponding to the node from pins
    pin_candidates = ['pin', 'positive', 'negative', 'anode', 'cathode', 'drain', 'gate', 'source', 'collector', 'base', 'emitter']
    if node in pins:
        for pc in pin_candidates:
            if pc in pins[node]:
                pos = pins[node][pc]
                if hasattr(pos, 'x') and hasattr(pos, 'y'):
                    return pos.x, pos.y
                return pos
    return None


def draw_circuit(G, components_element, xml_root, xml_file, initial_comp_node_mapping, max_attempts=100, EnlargeSize=2.5, routing_method=1, auto=1):
    attempt = 0
    success = False

    last_pos = None
    last_all_wires = None

    # 从xml读取已有的方向和位置
    positions_in_xml = {}
    directions_in_xml = {}
    flips_in_xml = {}  # [NEW CODE] 读取flip

    components_elem = xml_root.find('components')
    if components_elem is not None:
        for comp in components_elem:
            cid = comp.attrib['id']
            dir_val = comp.attrib.get('direction', 'right')
            flip_val = comp.attrib.get('flip', 'none')  # [NEW CODE] 读取flip属性
            directions_in_xml[cid] = dir_val
            flips_in_xml[cid] = flip_val  # [NEW CODE]
            if 'x' in comp.attrib and 'y' in comp.attrib:
                positions_in_xml[cid] = (float(comp.attrib['x']), float(comp.attrib['y']))

    nodes_elem = xml_root.find('nodes')
    if nodes_elem is not None:
        for node in nodes_elem:
            nid = node.attrib['id']
            if 'x' in node.attrib and 'y' in node.attrib:
                positions_in_xml[nid] = (float(node.attrib['x']), float(node.attrib['y']))

    if auto == 1:
        # 构建组件节点映射
        comp_node_mapping = build_comp_node_mapping([comp for comp in components_elem])

        while attempt < max_attempts:
            attempt += 1
            print(f"[DEBUG] Attempt {attempt} (auto=1, automatic layout)")

            # 使用comp_node_mapping的深拷贝
            pin_mapping_copy = copy.deepcopy(initial_comp_node_mapping)

            original_pos = nx.spring_layout(G)
            pos = {n: (EnlargeSize * original_pos[n][0], EnlargeSize * (-original_pos[n][1])) for n in G.nodes}

            directions = {}
            flips = {}  # [NEW CODE]
            for n in G.nodes:
                directions[n] = directions_in_xml.get(n, 'right')
                flips[n] = flips_in_xml.get(n, 'none')  # [NEW CODE]

            d = schemdraw.Drawing()
            elements = {}
            pins = {}
            component_boxes = {}
            drawn_edges = set()
            all_wires = []

            for node in G.nodes:
                node_info = G.nodes[node]
                x, y = pos[node]
                direction = directions.get(node, 'right')
                flip_in_xml = flips.get(node, 'none')  # [NEW CODE]
                print(f"[DEBUG] draw_component_or_node: node={node}, direction={direction}, flip={flip_in_xml}")
                draw_component_or_node(d, elements, pins, node, node_info, x, y, direction, component_boxes, G,
                                       flip=flip_in_xml)

            component_polygons = [comp_info['polygon'] for comp_info in component_boxes.values()]
            spatial_index = STRtree(component_polygons)

            draw_connections(d, G, components_element, pins, component_boxes, drawn_edges, routing_method,
                             spatial_index, all_wires, pin_mapping_copy)  # [MODIFIED] 传递pin_mapping_copy

            any_red_line = any(wire['color'] == wire_danger_color for wire in all_wires)
            overlapping = check_overlapping_wires(all_wires)

            last_pos = pos
            last_all_wires = all_wires

            if any_red_line or overlapping:
                print("[DEBUG] Overlap or red lines detected. Retrying...")
                continue
            else:
                for wire_info in all_wires:
                    line = wire_info['line']
                    color = wire_info['color']
                    d.add(elm.Line().at(line.coords[0]).to(line.coords[1]).color(color))
                add_anchor_dots(d, all_wires)
                print(f"[DEBUG] Successful layout after {attempt} attempts.")
                update_xml_positions_and_directions(xml_root, pos, directions, flips)  # [NEW CODE] 同时更新flip
                tree = ET.ElementTree(xml_root)
                tree.write(xml_file)
                d.draw()
                success = True
                break

        if not success:
            print("[DEBUG] No suitable layout found after all attempts. Showing last attempt with issues.")
            d = schemdraw.Drawing()
            if last_pos is not None and last_all_wires is not None:
                update_xml_positions_and_directions(xml_root, last_pos, directions_in_xml, flips_in_xml)
                tree = ET.ElementTree(xml_root)
                tree.write(xml_file)

                elements = {}
                pins = {}
                component_boxes = {}
                drawn_edges = set()
                for node in G.nodes:
                    node_info = G.nodes[node]
                    x, y = last_pos[node]
                    direction = directions_in_xml[node] if node in directions_in_xml else 'right'
                    flip_in_xml = flips_in_xml.get(node, 'none')  # [NEW CODE]
                    print(f"[DEBUG] Redraw last attempt: node={node}, direction={direction}, flip={flip_in_xml}")
                    draw_component_or_node(d, elements, pins, node, node_info, x, y, direction, component_boxes, G,
                                           flip=flip_in_xml)

                for wire_info in last_all_wires:
                    line = wire_info['line']
                    color = wire_info['color']
                    d.add(elm.Line().at(line.coords[0]).to(line.coords[1]).color(color))

                add_anchor_dots(d, last_all_wires)
                d.draw()
    else:
        # auto=0 使用固定位置和direction和flip
        if positions_in_xml:
            pos = positions_in_xml
            directions = directions_in_xml
            flips = flips_in_xml
            attempt = 0
            success = False
            last_all_wires = None
            while attempt < max_attempts:
                attempt += 1
                print(f"[DEBUG] Attempt {attempt} (auto=0, fixed positions)")

                d = schemdraw.Drawing()
                elements = {}
                pins = {}
                component_boxes = {}
                drawn_edges = set()
                all_wires = []

                for node in G.nodes:
                    node_info = G.nodes[node]
                    if node not in pos:
                        print(f"[DEBUG] No position for node {node}")
                        continue
                    x, y = pos[node]
                    direction = directions.get(node, 'right')
                    flip_in_xml = flips.get(node, 'none')  # [NEW CODE]
                    print(f"[DEBUG] draw_component_or_node (fixed): node={node}, direction={direction}, flip={flip_in_xml}")
                    draw_component_or_node(d, elements, pins, node, node_info, x, y, direction, component_boxes, G,
                                           flip=flip_in_xml)

                component_polygons = [comp_info['polygon'] for comp_info in component_boxes.values()]
                spatial_index = STRtree(component_polygons)

                draw_connections(d, G, components_element, pins, component_boxes, drawn_edges, routing_method,
                                 spatial_index, all_wires, {})  # [MODIFIED] 不传递pin_mapping_copy

                any_red_line = any(wire['color'] == wire_danger_color for wire in all_wires)
                overlapping = check_overlapping_wires(all_wires)

                last_all_wires = all_wires

                if any_red_line or overlapping:
                    print("[DEBUG] Overlap or red lines detected. Retrying...")
                    continue
                else:
                    for wire_info in all_wires:
                        line = wire_info['line']
                        color = wire_info.get('color', wire_safe_color)
                        d.add(elm.Line().at(line.coords[0]).to(line.coords[1]).color(color))
                    add_anchor_dots(d, all_wires)
                    print(f"[DEBUG] Successful routing after {attempt} attempts.")
                    tree = ET.ElementTree(xml_root)
                    tree.write(xml_file)
                    d.draw()
                    success = True
                    break

            if not success:
                print("[DEBUG] No suitable routing after max attempts with fixed positions. Showing last attempt.")
                d = schemdraw.Drawing()
                elements = {}
                pins = {}
                component_boxes = {}
                drawn_edges = set()
                for node in G.nodes:
                    node_info = G.nodes[node]
                    if node not in pos:
                        continue
                    x, y = pos[node]
                    direction = directions.get(node, 'right')
                    flip_in_xml = flips.get(node, 'none')  # [NEW CODE]
                    print(f"[DEBUG] Redraw last attempt (fixed): node={node}, direction={direction}, flip={flip_in_xml}")
                    draw_component_or_node(d, elements, pins, node, node_info, x, y, direction, component_boxes, G,
                                           flip=flip_in_xml)

                if last_all_wires is not None:
                    for wire_info in last_all_wires:
                        line = wire_info['line']
                        color = wire_info.get('color', wire_safe_color)
                        d.add(elm.Line().at(line.coords[0]).to(line.coords[1]).color(color))

                add_anchor_dots(d, last_all_wires or [])
                tree = ET.ElementTree(xml_root)
                tree.write(xml_file)
                d.draw()
        else:
            print("[DEBUG] No positions found in XML. Please run with auto=1 first.")


def set_element_direction(element, direction):
    print(f"[DEBUG] set_element_direction called with direction={direction}")
    if direction == 'up':
        print("[DEBUG] direction=up -> element.up()")
        element.up()
    elif direction == 'down':
        print("[DEBUG] direction=down -> element.down()")
        element.down()
    elif direction == 'left':
        print("[DEBUG] direction=left -> element.left()")
        element.left()
    else:
        print("[DEBUG] direction=right (default) -> element.right()")
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
                       **kwargs):
    comp_class = component_map.get(component_type, None)
    if comp_class is None:
        raise ValueError(f"Unsupported component type: {component_type}")

    d = schemdraw.Drawing(show=False)
    element = comp_class(**kwargs).at(pos)
    # 应用方向
    set_element_direction(element, direction)
    element = d.add(element)
    print("This is the parameters for this device:" + str(element.theta) + "----------------------------------------" + str(direction))

    # rotational point
    if 'start' in element.anchors:
        ref_anchor_name = 'start'
    elif 'drain' in element.anchors:
        ref_anchor_name = 'drain'
    elif 'base' in element.anchors:
        ref_anchor_name = 'base'
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

    if component_type in ['GND', 'ground']:
        ymax_global -= shrink_size
    elif component_type in ['R', 'C', 'L', 'D', 'V', 'I', 'E', 'H', 'F', 'G', 'S']:
        xmin_global += shrink_size
        xmax_global -= shrink_size
    elif component_type == 'J':
        xmax_global -= shrink_size
        ymin_global += shrink_size
        ymax_global -= shrink_size
        # prevent wire crossing the junction field-effect transistor.
        xmin_global -= shrink_size * 2
    elif component_type == 'Q':
        xmin_global += shrink_size
        ymin_global += shrink_size
        ymax_global -= shrink_size
        # prevent wire crossing the bipolar transistor.
        xmax_global += shrink_size * 2

    anchors_global = {k: (v.x, v.y) for k, v in element.absanchors.items()}
    bbox = {
        'xmin': xmin_global,
        'ymin': ymin_global,
        'xmax': xmax_global,
        'ymax': ymax_global
    }
    print("before bbox:", bbox)

    # Get all vertices of the rectangle
    vertices = [
        (bbox['xmin'], bbox['ymin']),
        (bbox['xmax'], bbox['ymin']),
        (bbox['xmax'], bbox['ymax']),
        (bbox['xmin'], bbox['ymax'])
    ]

    print("pos:", pos)
    (x_center, y_center) = pos
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
    print("after bbox:", bbox)

    # Get all vertices of the rectangle
    rotated_vertices = [
        (bbox['xmin'], bbox['ymin']),
        (bbox['xmax'], bbox['ymin']),
        (bbox['xmax'], bbox['ymax']),
        (bbox['xmin'], bbox['ymax'])
    ]

    print("rotated_vertices:", rotated_vertices)
    if component_type in ['J']:
        # because the direction is up or down, so the horizontal flip is the same as the vertical flip
        flipped_vertices = flip_component(rotated_vertices, (x_center, y_center), mode=flip)
        # Find new boundaries
        rotated_x = [v[0] for v in flipped_vertices]
        rotated_y = [v[1] for v in flipped_vertices]

    elif component_type in ['Q']:
        # because the direction is up or down, so the horizontal flip is the same as the vertical flip
        if direction == 'up' or direction == 'down':
            if flip == 'horizontal':
                flipped_vertices = flip_component(rotated_vertices, (x_center, y_center), mode=flip)
                print("flip horizontal")
            elif flip == 'vertical':
                collector = element.absanchors['collector']
                x_collector, y_collector = collector.x, collector.y
                flipped_vertices = flip_component(rotated_vertices, (x_collector, y_collector), mode=flip)
                print("flip vertical")
            else:
                flipped_vertices = rotated_vertices
            print("run there")
        else:
            if flip == 'horizontal':
                collector = element.absanchors['collector']
                x_collector, y_collector = collector.x, collector.y
                flipped_vertices = flip_component(rotated_vertices, (x_collector, y_collector), mode=flip)
            elif flip == 'vertical':
                flipped_vertices = flip_component(rotated_vertices, (x_center, y_center), mode=flip)
            else:
                flipped_vertices = rotated_vertices

            # Find new boundaries
            rotated_x = [v[0] for v in flipped_vertices]
            rotated_y = [v[1] for v in flipped_vertices]

    bbox = {
        'xmin': min(rotated_x),
        'ymin': min(rotated_y),
        'xmax': max(rotated_x),
        'ymax': max(rotated_y)
    }
    print("flipped bbox:", bbox)
    print("anchors_global:", anchors_global)
    return bbox, element.anchors, anchors_global


def draw_component_or_node(d, elements, pins, node, node_info, x, y, direction, component_boxes, G, flip='none'):
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

    print(f"[DEBUG] get_component_bbox: node={node}, ctype={ctype_for_bbox}, direction={direction}")
    bbox, anchors_local, anchors_global = get_component_bbox(ctype_for_bbox, pos=(x, y), direction=direction,
                                                             shrink_size=shrink_size, flip=flip)
    label_text = f"{node}\n{node_info.get('value','')}"

    comp_class = component_map.get(ctype_for_bbox, elm.Dot(radius=0.12))

    element = comp_class().at((x, y))
    # 应用方向
    set_element_direction(element, direction)
    # because the direction is up or down, so the horizontal flip is the same as the vertical flip
    print(direction)
    if direction == 'up' or direction == 'down':
        if flip == 'horizontal':
            element.flip()  # vertical flip
        elif flip == 'vertical':
            element.reverse()  # horizontal flip
        else:
            element = element
        print("run here-------------------------------------why")
    else:
        if flip == 'horizontal':
            element.reverse()  # horizontal flip
        elif flip == 'vertical':
            element.flip()  # vertical flip
        else:
            element = element

    print("flip:", flip)
    element.label(label_text)
    element = d.add(element)

    if ctype in ['V', 'I', 'E', 'H', 'F', 'G']:
        pins[node] = {'positive': anchors_global['end'], 'negative': anchors_global['start']}
    elif ctype in ['R', 'C', 'L', 'S']:
        pins[node] = {'start': anchors_global['start'], 'end': anchors_global['end']}
    elif ctype == 'D':
        pins[node] = {'anode': anchors_global['start'], 'cathode': anchors_global['end']}
    elif ctype == 'J':
        pins[node] = {
            'drain': anchors_global['drain'],
            'gate': anchors_global['gate'],
            'source': anchors_global['source']
        }
    elif ctype == 'Q':
        pins[node] = {
            'collector': anchors_global['collector'],
            'base': anchors_global['base'],
            'emitter': anchors_global['emitter']
        }
    elif ctype == 'ground':
        pins[node] = {'pin': (x, y)}
    elif ctype == 'node':
        pins[node] = {'pin': (x, y)}
    else:
        pins[node] = {'pin': (x, y)}

    x_min, y_min, x_max, y_max = bbox['xmin'], bbox['ymin'], bbox['xmax'], bbox['ymax']
    component_boxes[node] = {
        'bbox': bbox,
        'polygon': Polygon([(x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max)]),
        'type': ctype
    }


def check_overlapping_wires(all_wires):
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
                print(f"[DEBUG] Overlapping wires detected between wire {i} and wire {j}")
    return overlapping


def draw_connections(d, G, components_element, pins, component_boxes, drawn_edges, routing_method, spatial_index, all_wires, pin_mapping):
    for comp, neighbor, key in G.edges(keys=True):
        edge_nodes = tuple(sorted([comp, neighbor]))
        # 在 edge_key 中加入 key，从而区分多重边
        edge_key = edge_nodes + (key,)
        print("edge_key:", edge_key)
        if edge_key in drawn_edges:
            continue

        comp_info = G.nodes[comp]
        neighbor_info = G.nodes[neighbor]

        comp_pin = get_component_pin(comp, neighbor, comp_info, components_element, pins, pin_mapping)  # [MODIFIED] 传递pin_mapping
        print("comp_pin: " + str(comp_pin))
        neighbor_pin = get_component_pin(neighbor, comp, neighbor_info, components_element, pins, pin_mapping)  # [MODIFIED] 传递pin_mapping
        print("neighbor_pin" + str(neighbor_pin))
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
            all_wires.append({'line': new_line, 'color': line_color})

        drawn_edges.add(edge_key)


def route_connection_current_method(start_pos, end_pos, component_boxes, comp, neighbor, all_wires):
    path1 = [start_pos, (end_pos[0], start_pos[1]), end_pos]
    path2 = [start_pos, (start_pos[0], end_pos[1]), end_pos]

    def path_ok(path):
        for i in range(len(path) - 1):
            if is_wire_crossing_components(path[i], path[i + 1], component_boxes, comp, neighbor):
                return False
        for i in range(len(path) - 1):
            seg = LineString([path[i], path[i + 1]])
            for w in all_wires:
                lw = w['line']
                if seg.equals(lw) or seg.contains(lw) or lw.contains(seg):
                    return False
        return True

    if path_ok(path1):
        return path1, wire_safe_color
    if path_ok(path2):
        return path2, wire_safe_color

    return path1, wire_danger_color


def is_wire_crossing_components(start_pos, end_pos, component_boxes, comp, neighbor):
    line = LineString([start_pos, end_pos])
    line_start_point = Point(start_pos)
    line_end_point = Point(end_pos)
    for comp_id, comp_info in component_boxes.items():
        polygon = comp_info['polygon']
        if line.intersects(polygon):
            intersection = line.intersection(polygon)
            if (intersection.equals(line_start_point) or intersection.equals(line_end_point)) and (comp_id == comp or comp_id == neighbor):
                continue
            else:
                return True
    return False


if __name__ == "__main__":
    if auto == 1:
        xml_root = convert_netlist_to_xml_file(netlist, "spice_netlist.xml")

    tree = ET.parse("spice_netlist.xml")
    xml_root = tree.getroot()
    G, components_element = create_graph_from_xml(xml_root)

    # 构建组件节点映射
    initial_comp_node_mapping = build_comp_node_mapping([comp for comp in components_element])

    draw_circuit(
        G, components_element, xml_root, "spice_netlist.xml",
        initial_comp_node_mapping,  # 传递初始映射
        max_attempts=max_attempts,
        EnlargeSize=EnlargeSize,
        routing_method=routing_method,
        auto=auto
    )
