import networkx as nx
import schemdraw
import schemdraw.elements as elm
import xml.etree.ElementTree as ET
import numpy as np
from shapely.geometry import LineString, Polygon, Point, box
from shapely.strtree import STRtree
import shapely.ops
import xml.dom.minidom as minidom

# 定义 SPICE 网表，包含 JFET
netlist = """
* JFET共栅放大器
VDD 1 0 DC 12        ; 电源电压12V
Vi 2 0 AC 1          ; 输入信号1V AC
R1 1 3 5k            ; 漏极电阻5k欧姆
RS 4 0 1k            ; 源极电阻1k欧姆
J1 3 0 4 JFET_N      ; N型JFET，D=3, G=0, S=4
.END

"""

# 参数
EnlargeSize = 3
shrink_ratio = 0.08  # 默认 shrink_ratio
max_attempts = 30
routing_method = 1  # 1: 当前方法, 2: A* 算法

# Shrink ratios per component type
shrink_ratios = {
    'R': 0.08,    # 电阻器
    'C': 0.08,    # 电容器
    'L': 0.08,    # 电感器
    'V': 0.08,    # 电压源
    'I': 0.08,    # 电流源
    'D': 0.08,    # 二极管
    'J': 0.168,   # JFET
    'ground': 0.08,
    'node': 0.08
    # 如果有其他元件类型，可以继续添加
}

def parse_spice_netlist(netlist):
    """解析 SPICE 网表，提取元件、命令和子电路。"""
    components = []
    commands = []
    subcircuits = []
    for line in netlist.strip().split('\n'):
        # 移除以 ';' 开头的注释
        if ';' in line:
            line = line.split(';')[0].strip()
        line = line.strip()
        # 跳过以 '*' 开头的注释
        if line.startswith('*'):
            continue
        # 跳过空行
        if not line.strip():
            continue

        try:
            parts = line.split()
            first_char = parts[0][0].upper()

            # 根据第一个字符解析不同类型的元件
            if first_char in ['V', 'I', 'R', 'C', 'L', 'D', 'M', 'Q', 'J', 'K', 'S', 'T', 'E', 'F', 'G', 'H']:
                if first_char == 'V':
                    value = ' '.join(parts[3:])  # 电压源
                    nodes = parts[1:3]
                elif first_char == 'I':
                    value = ' '.join(parts[3:])  # 电流源
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
                    continue  # 跳过循环的其余部分
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
                # 处理子电路
                if len(parts) < 4:
                    raise ValueError(f"Invalid subcircuit definition: {line}")
                subcircuits.append({
                    'type': 'X',
                    'id': parts[0],
                    'nodes': parts[1:-1],  # 子电路节点
                    'subckt': parts[-1]  # 子电路名称
                })
            # 解析命令
            elif line.startswith('.'):
                commands.append({'command': parts[0][1:], 'params': parts[1:]})
            else:
                raise ValueError(f"Unknown element or command: {line}")
        except (IndexError, ValueError) as e:
            print(f"Error parsing line: {line}\n{e}")
            continue

    return components, commands, subcircuits

def netlist_to_xml(components, commands, subcircuits):
    """将解析后的网表数据转换为 XML 结构。"""
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
    """返回一个美化后的 XML 字符串。"""
    rough_string = ET.tostring(element, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")

def convert_netlist_to_xml_file(netlist, output_filename="spice_netlist.xml"):
    """将网表转换为 XML 并保存到文件。"""
    components, commands, subcircuits = parse_spice_netlist(netlist)
    xml_root = netlist_to_xml(components, commands, subcircuits)
    pretty_xml = pretty_print_xml(xml_root)
    with open(output_filename, "w") as f:
        f.write(pretty_xml)
    print(pretty_xml)

def create_graph_from_xml(xml_file):
    """从 XML 数据创建图。"""
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

        # 添加元件节点
        G.add_node(cid, type=ctype, value=value)
        # 添加连接的节点
        for node in nodes:
            if node == '0':
                G.add_node(node, type='ground')  # 将地视为特殊元件
            else:
                G.add_node(node, type='node')  # 普通节点
            # 添加边
            G.add_edge(cid, node)

    return G, components_element

def draw_circuit(G, components_element, max_attempts=100, EnlargeSize=2.5, shrink_ratio=0.1, routing_method=1):
    """尝试绘制电路图，避免与元器件碰撞。"""
    attempt = 0
    success = False

    while attempt < max_attempts:
        attempt += 1
        print(f"Attempt {attempt}")

        pos = nx.spring_layout(G, scale=EnlargeSize)

        # 重置绘图和相关变量
        d = schemdraw.Drawing()
        elements = {}
        pins = {}
        component_boxes = {}  # 用于存储元件的边界框
        drawn_edges = set()
        any_red_lines = False  # 标记是否绘制了红色连线

        # 绘制元件和节点，并计算边界框
        for node in G.nodes:
            node_info = G.nodes[node]
            node_pos = pos[node]
            x, y = (EnlargeSize * node_pos[0], EnlargeSize * -node_pos[1])

            if np.isnan(x) or np.isnan(y) or np.isinf(x) or np.isinf(y):
                print(f"Node {node} has invalid transformed position: ({x}, {y})")
                continue  # 跳过有问题的节点

            draw_component_or_node(d, elements, pins, node, node_info, x, y)

            # 获取元件的边界框
            if node in elements and hasattr(elements[node], 'get_bbox') and node_info.get('type') != 'node':
                bbox = elements[node].get_bbox(transform=True)
                x_min, y_min, x_max, y_max = bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax
                # 检查边界框是否有效
                if np.isnan([x_min, y_min, x_max, y_max]).any() or np.isinf([x_min, y_min, x_max, y_max]).any():
                    print(f"Element {node} has invalid bbox: {bbox}")
                    continue  # 跳过有问题的元件

                # 获取该元件类型的 shrink_ratio
                comp_type = node_info.get('type')
                comp_shrink_ratio = shrink_ratios.get(comp_type, shrink_ratio)  # 使用类型特定的 shrink_ratio

                # 计算缩小的边界框
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
                    'type': node_info.get('type')  # 添加元件类型
                }
            else:
                # 对于没有边界框的节点
                pass

        # 构建元件的空间索引（用于A*算法）
        component_polygons = [comp_info['polygon'] for comp_info in component_boxes.values()]
        spatial_index = STRtree(component_polygons)

        # 绘制连线
        any_red_lines = draw_connections(
            d, G, components_element, pins, component_boxes, drawn_edges, routing_method, spatial_index
        )

        if not any_red_lines:
            print(f"Successful layout found after {attempt} attempts.")
            success = True
            break  # 如果没有红色连线，则退出循环

    if not success:
        print(f"No collision-free layout found after {max_attempts} attempts. Showing the last attempt.")

    # 绘制电路图
    d.draw()

def draw_component_or_node(d, elements, pins, node, node_info, x, y):
    """在 schemdraw 绘图中绘制元件或节点。"""
    if node_info.get('type') == 'R':
        # 绘制电阻器
        label_text = f"{node}\n{node_info.get('value')}"
        elements[node] = d.add(elm.Resistor().at((x, y)).right().label(label_text))
        pins[node] = {
            'start': elements[node].absanchors['start'],
            'end': elements[node].absanchors['end']
        }
    elif node_info.get('type') == 'C':
        # 绘制电容器
        label_text = f"{node}\n{node_info.get('value')}"
        elements[node] = d.add(elm.Capacitor().at((x, y)).up().label(label_text))
        pins[node] = {
            'start': elements[node].absanchors['start'],
            'end': elements[node].absanchors['end']
        }
    elif node_info.get('type') == 'L':
        # 绘制电感器
        label_text = f"{node}\n{node_info.get('value')}"
        elements[node] = d.add(elm.Inductor().at((x, y)).right().label(label_text))
        pins[node] = {
            'start': elements[node].absanchors['start'],
            'end': elements[node].absanchors['end']
        }
    elif node_info.get('type') == 'V':
        # 绘制电压源
        label_text = f"{node}\n{node_info.get('value')}"
        elements[node] = d.add(elm.SourceV().at((x, y)).up().label(label_text))
        # 对于向上的电压源，正极在 'end'，负极在 'start'
        pins[node] = {
            'positive': elements[node].absanchors['end'],
            'negative': elements[node].absanchors['start']
        }
    elif node_info.get('type') == 'I':
        # 绘制电流源
        label_text = f"{node}\n{node_info.get('value')}"
        elements[node] = d.add(elm.SourceI().at((x, y)).up().label(label_text))
        # 对于向上的电流源，正极在 'start'，负极在 'end'
        pins[node] = {
            'positive': elements[node].absanchors['start'],
            'negative': elements[node].absanchors['end']
        }
    elif node_info.get('type') == 'D':
        # 绘制二极管
        label_text = f"{node}\n{node_info.get('value')}"
        elements[node] = d.add(elm.Diode().at((x, y)).right().label(label_text))
        # 对于二极管，阳极在 'start'，阴极在 'end'
        pins[node] = {
            'anode': elements[node].absanchors['start'],
            'cathode': elements[node].absanchors['end']
        }
    elif node_info.get('type') == 'J':
        # 绘制 JFET
        label_text = f"{node}\n{node_info.get('value')}"
        elements[node] = d.add(elm.JFet().at((x, y)).right().label(label_text))
        # 使用正确的锚点
        pins[node] = {
            'drain': elements[node].absanchors['drain'],
            'gate': elements[node].absanchors['gate'],
            'source': elements[node].absanchors['source']
        }
    elif node_info.get('type') == 'ground':
        # 绘制地
        elements[node] = d.add(elm.Ground().at((x, y)))
        pins[node] = {
            'pin': (x, y)
        }
    else:
        # 绘制普通节点
        elements[node] = d.add(elm.Dot().at((x, y)))
        pins[node] = {
            'pin': (x, y)
        }
        # 添加节点标签
        d.add(elm.Label().at((x, y)).label(node, ofst=0.2))
def route_connection_current_method(start_pos, end_pos, component_boxes, comp, neighbor):
    """使用当前方法路由连接，动态选择拐点位置，尽量减少拐点数量。"""
    # 定义初始的两个路径选项
    path1 = [start_pos, (end_pos[0], start_pos[1]), end_pos]  # 横向优先，然后纵向
    path2 = [start_pos, (start_pos[0], end_pos[1]), end_pos]  # 纵向优先，然后横向

    # 计算中点
    mid_x = (start_pos[0] + end_pos[0]) / 2
    mid_y = (start_pos[1] + end_pos[1]) / 2

    # 定义额外的路径选项，包含一个拐点
    path3 = [start_pos, (mid_x, start_pos[1]), (mid_x, end_pos[1]), end_pos]  # 横-纵-横
    path4 = [start_pos, (start_pos[0], mid_y), (end_pos[0], mid_y), end_pos]  # 纵-横-纵

    # 定义更多拐点的路径选项
    path5 = [start_pos, (mid_x, start_pos[1]), (mid_x, mid_y), (end_pos[0], mid_y), end_pos]  # 横-纵-横-纵
    path6 = [start_pos, (start_pos[0], mid_y), (mid_x, mid_y), (mid_x, end_pos[1]), end_pos]  # 纵-横-纵-横

    # 所有路径选项
    paths = [
        ('H-V', path1),
        ('V-H', path2),
        ('H-V-H', path3),
        ('V-H-V', path4),
        ('H-V-H-V', path5),
        ('V-H-V-H', path6)
    ]

    # 记录路径特征
    path_scores = []

    # 检查每条路径是否与元器件碰撞
    line_color = "black"
    selected_path = None
    for name, path in paths:
        crossing = False
        total_length = 0
        num_turns = len(path) - 2  # 拐点数量为中间点数量

        for i in range(len(path) - 1):
            segment_start = path[i]
            segment_end = path[i + 1]
            # 检查该段是否与元器件碰撞
            if is_wire_crossing_components(
                segment_start, segment_end, component_boxes, comp, neighbor
            ):
                crossing = True
                break
            # 计算总长度
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
            # 选择第一个无碰撞路径
            selected_path = path

    if selected_path is None:
        # 如果所有路径都与元器件碰撞，选择总长度最短的路径，并标记为红色
        non_selected_paths = [p for p in path_scores if p['crossing']]
        if non_selected_paths:
            best_path = min(non_selected_paths, key=lambda x: x['total_length'])
            selected_path = best_path['path']
            line_color = "red"
        else:
            # 如果没有路径可用，默认使用path1并标记为红色
            selected_path = path1
            line_color = "red"

    # 打印路径评分以供评估
    print("Path options for connection between", comp, "and", neighbor)
    for score in path_scores:
        print(f"Path {score['name']}: Crosses components: {score['crossing']}, "
              f"Total length: {score['total_length']:.2f}, Turns: {score['num_turns']}")

    return selected_path, line_color

def is_wire_crossing_components(start_pos, end_pos, component_boxes, comp, neighbor):
    """检查两点之间的连线是否与任何元器件碰撞。"""
    line = LineString([start_pos, end_pos])
    line_start_point = Point(start_pos)
    line_end_point = Point(end_pos)
    for comp_id, comp_info in component_boxes.items():
        polygon = comp_info['polygon']
        if line.intersects(polygon):
            intersection = line.intersection(polygon)
            # 如果交点仅在起点或终点，并且连接的是相关元件，则不算碰撞
            if (intersection.equals(line_start_point) or intersection.equals(line_end_point)) and (comp_id == comp or comp_id == neighbor):
                continue
            else:
                return True
    return False

def draw_connections(d, G, components_element, pins, component_boxes, drawn_edges, routing_method, spatial_index):
    """绘制元件之间的连线，避免与元器件碰撞。"""
    any_red_lines = False  # 标记是否绘制了红色连线
    for comp, neighbor in G.edges():
        edge_key = tuple(sorted([comp, neighbor]))
        if edge_key in drawn_edges:
            continue

        comp_info = G.nodes[comp]
        neighbor_info = G.nodes[neighbor]

        # 确定连接的引脚
        comp_pin = get_component_pin(comp, neighbor, comp_info, components_element, pins)
        neighbor_pin = get_component_pin(neighbor, comp, neighbor_info, components_element, pins)

        if comp_pin is None or neighbor_pin is None:
            continue  # 如果无法确定引脚，跳过

        # 获取连接点的坐标
        start_pos = get_pin_position(pins, comp, comp_pin)
        end_pos = get_pin_position(pins, neighbor, neighbor_pin)

        if start_pos is None or end_pos is None:
            continue  # 如果坐标无效，跳过

        # 选择路由方法
        if routing_method == 1:
            # 当前方法
            selected_path, line_color = route_connection_current_method(
                start_pos, end_pos, component_boxes, comp, neighbor
            )
        elif routing_method == 2:
            # A* 算法
            selected_path, line_color = route_connection_astar(
                start_pos, end_pos, component_boxes, spatial_index
            )
        else:
            print(f"Unknown routing method: {routing_method}")
            continue

        # 绘制连线
        for i in range(len(selected_path) - 1):
            d.add(elm.Line().at(selected_path[i]).to(selected_path[i + 1]).color(line_color))

        drawn_edges.add(edge_key)
        if line_color == "red":
            any_red_lines = True

    return any_red_lines

def get_component_pin(comp, neighbor, comp_info, components_element, pins):
    """根据邻居确定元件的相应引脚。"""
    if comp_info.get('type') in ['R', 'C', 'L']:
        # 对于无方向性的元件，找到与邻居节点对应的引脚
        comp_pins = ['start', 'end']

        # 获取元件引脚的坐标
        comp_pin_coords = {pin: pins[comp][pin] for pin in comp_pins}

        # 获取连接到元件的两个节点
        component = next(c for c in components_element if c.attrib['id'] == comp)
        node1 = component.find('node1').text
        node2 = component.find('node2').text
        nodes = [node1, node2]

        # 获取节点的坐标
        node_coords = {}
        for node in nodes:
            node_coord = get_node_coordinate(pins, node)
            if node_coord:
                node_coords[node] = node_coord

        if node1 not in node_coords or node2 not in node_coords:
            return None  # 无法确定节点坐标

        # 计算两种引脚分配方案的总距离
        # 方案1：'start' -> node1, 'end' -> node2
        distance1 = np.linalg.norm(np.array(comp_pin_coords['start']) - np.array(node_coords[node1])) + \
                    np.linalg.norm(np.array(comp_pin_coords['end']) - np.array(node_coords[node2]))
        # 方案2：'start' -> node2, 'end' -> node1
        distance2 = np.linalg.norm(np.array(comp_pin_coords['start']) - np.array(node_coords[node2])) + \
                    np.linalg.norm(np.array(comp_pin_coords['end']) - np.array(node_coords[node1]))

        # 选择距离较短的方案
        if distance1 <= distance2:
            pin_node_pairs = [('start', node1), ('end', node2)]
        else:
            pin_node_pairs = [('start', node2), ('end', node1)]

        # 找到对应邻居节点的引脚
        for pin, node in pin_node_pairs:
            if node == neighbor:
                return pin

    elif comp_info.get('type') == 'V':
        # 对于电压源
        component = next(c for c in components_element if c.attrib['id'] == comp)
        node1 = component.find('node1').text  # 正极
        node2 = component.find('node2').text  # 负极
        if neighbor == node1:
            return 'positive'
        elif neighbor == node2:
            return 'negative'

    elif comp_info.get('type') == 'I':
        # 对于电流源
        component = next(c for c in components_element if c.attrib['id'] == comp)
        node1 = component.find('node1').text  # 正极（电流进入）
        node2 = component.find('node2').text  # 负极（电流退出）
        if neighbor == node1:
            return 'positive'
        elif neighbor == node2:
            return 'negative'

    elif comp_info.get('type') == 'D':
        # 对于二极管
        component = next(c for c in components_element if c.attrib['id'] == comp)
        node1 = component.find('node1').text  # 阳极
        node2 = component.find('node2').text  # 阴极
        if neighbor == node1:
            return 'anode'
        elif neighbor == node2:
            return 'cathode'

    elif comp_info.get('type') == 'J':
        # 对于 JFET
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
        # 对于节点和地
        return 'pin'
    return None

def get_node_coordinate(pins, node):
    """获取节点的坐标。"""
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
    """获取引脚的位置。"""
    try:
        pos = pins[comp][pin_name]
        if isinstance(pos, schemdraw.util.Point):
            return (pos.x, pos.y)
        else:
            return pos
    except KeyError:
        print(f"Pin {pin_name} not found for component {comp}")
        return None

# 主程序
if __name__ == "__main__":
    # 将网表转换为 XML
    convert_netlist_to_xml_file(netlist, "spice_netlist.xml")

    # 从 XML 创建图
    G, components_element = create_graph_from_xml("spice_netlist.xml")

    # 绘制电路图
    draw_circuit(
        G, components_element,
        max_attempts=max_attempts,
        EnlargeSize=EnlargeSize,
        shrink_ratio=shrink_ratio,
        routing_method=routing_method
    )
