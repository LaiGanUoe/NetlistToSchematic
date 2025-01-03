import networkx as nx
import schemdraw
import schemdraw.elements as elm
import xml.etree.ElementTree as ET
import numpy as np
from shapely.geometry import LineString, Polygon, Point
import xml.dom.minidom as minidom
import math
from collections import defaultdict
import copy
import sys

# 示例 Netlist
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

# 配置参数
EnlargeSize = 15
max_attempts = 300
routing_method = 1  # 仅使用曼哈顿路由
auto = 1  # 设置为0使用固定布局，1为自动布局
shrink_size = 0.05
wire_safe_color = 'green'
wire_danger_color = 'red'
grid_size = 0.1  # 网格大小
draw_grid_or_not = 0  # 设置为1绘制网格，0不绘制

# 默认方向和翻转
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
    'Q': 'up',
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

# 默认缩放比例
default_scaling_ratios = {
    'GND': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'R': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'C': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'L': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'D': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'V': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'I': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'S': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'J': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'Q': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'M': {'vertical_scale': 0.8, 'horizontal_scale': 30/41},
    'E': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'H': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'F': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'G': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'node': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'ground': {'vertical_scale': 1.0, 'horizontal_scale': 1.0}
}

# 组件映射
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
    'M': None,  # 根据模型单独处理
    'E': elm.SourceControlledV,
    'H': elm.SourceControlledV,
    'F': elm.SourceControlledI,
    'G': elm.SourceControlledI,
    'ground': elm.Ground,
    'node': elm.Dot(radius=0.12)
}


def parse_spice_netlist(netlist):
    print("[DEBUG] 开始解析 Netlist...")
    components = []
    commands = []
    subcircuits = []
    for line_number, line in enumerate(netlist.strip().split('\n'), start=1):
        original_line = line
        # 去除注释
        if ';' in line:
            line = line.split(';')[0].strip()
        line = line.strip()
        if line.startswith('*'):
            print(f"[DEBUG] 跳过注释行 {line_number}: {original_line}")
            continue  # 跳过注释行
        if not line:
            print(f"[DEBUG] 跳过空行 {line_number}")
            continue  # 跳过空行

        try:
            parts = line.split()
            if not parts:
                print(f"[DEBUG] 第 {line_number} 行分割后为空。")
                continue  # 跳过分割后为空的行

            first_char = parts[0][0].upper()

            if first_char in ['V', 'I', 'R', 'C', 'L', 'D', 'M', 'Q', 'J', 'K', 'S', 'T', 'E', 'F', 'G', 'H']:
                if first_char in ['V', 'I', 'S', 'E', 'F', 'G', 'H']:
                    if len(parts) < 4:
                        raise ValueError(f"无效的电压/电流源定义: {line}")
                    value = ' '.join(parts[3:])
                    nodes = parts[1:3]
                elif first_char == 'D':
                    if len(parts) < 4:
                        raise ValueError(f"无效的二极管定义: {line}")
                    value = parts[3]
                    nodes = parts[1:3]
                elif first_char == 'J':
                    if len(parts) < 5:
                        raise ValueError(f"无效的 JFET 定义: {line}")
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
                    print(f"[DEBUG] 添加 JFET 组件: {parts[0]}")
                    continue
                elif first_char == 'Q':
                    if len(parts) < 5:
                        raise ValueError(f"无效的 BJT 定义: {line}")
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
                    print(f"[DEBUG] 添加 BJT 组件: {parts[0]}")
                    continue
                elif first_char == 'M':
                    if len(parts) < 6:
                        raise ValueError(f"无效的 MOSFET 定义: {line}")
                    id = parts[0]
                    nodes = parts[1:5]  # drain, gate, source, bulk
                    model = parts[5]
                    # 可选参数
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
                        'scale': 1.0
                    })
                    print(f"[DEBUG] 添加 MOSFET 组件: {id}")
                    continue
                else:
                    if len(parts) < 4:
                        raise ValueError(f"无效的组件定义: {line}")
                    value = ' '.join(parts[3:]) if len(parts) > 3 else ''
                    nodes = parts[1:3]

                components.append({
                    'type': first_char,
                    'id': parts[0],
                    'nodes': nodes,
                    'value': value,
                    'scale': 1.0
                })
                print(f"[DEBUG] 添加组件: {parts[0]} 类型: {first_char}")

            elif first_char == 'X':
                if len(parts) < 4:
                    raise ValueError(f"无效的子电路定义: {line}")
                subcircuits.append({
                    'type': 'X',
                    'id': parts[0],
                    'nodes': parts[1:-1],
                    'subckt': parts[-1]
                })
                print(f"[DEBUG] 添加子电路: {parts[0]}")
            elif line.startswith('.'):
                commands.append({'command': parts[0][1:], 'params': parts[1:]})
                print(f"[DEBUG] 添加命令: {parts[0]}")
            else:
                raise ValueError(f"未知的元件或命令: {line}")
        except (IndexError, ValueError) as e:
            print(f"[ERROR] 解析第 {line_number} 行: {original_line}")
            print(f"        {e}")
            continue

    print("[DEBUG] 完成 Netlist 解析。")
    return components, commands, subcircuits


def netlist_to_xml(components, commands, subcircuits):
    print("[DEBUG] 将 Netlist 转换为 XML...")
    root = ET.Element("spice_netlist")

    components_elem = ET.SubElement(root, "components")
    for comp in components:
        comp_type = comp['type']
        cid = comp['id']
        comp_elem = ET.SubElement(components_elem, "component", type=comp_type, id=cid)
        # 添加节点
        for i, node in enumerate(comp['nodes'], start=1):
            ET.SubElement(comp_elem, f"node{i}").text = node
        # 添加值或模型
        if 'model' in comp:
            ET.SubElement(comp_elem, "model").text = comp['model']
        if 'value' in comp:
            ET.SubElement(comp_elem, "value").text = comp['value']
        # 添加 MOSFET 的可选参数
        if 'parameters' in comp:
            params_elem = ET.SubElement(comp_elem, "parameters")
            for key, val in comp['parameters'].items():
                ET.SubElement(params_elem, key).text = val
        # 添加引脚
        if 'pins' in comp:
            ET.SubElement(comp_elem, "pins").text = ' '.join(comp['pins'])
        # 设置方向和翻转
        comp_direction = default_directions.get(comp_type, 'right')
        comp_elem.set("direction", comp_direction)
        comp_flip = default_flips.get(comp_type, 'none')
        comp_elem.set("flip", comp_flip)  # 默认不翻转
        # 添加垂直和水平缩放比例
        scaling = default_scaling_ratios.get(comp_type, {'vertical_scale': 1.0, 'horizontal_scale': 1.0})
        comp_elem.set("vertical_scale", str(scaling.get('vertical_scale', 1.0)))
        comp_elem.set("horizontal_scale", str(scaling.get('horizontal_scale', 1.0)))

        print(f"[DEBUG] 将组件 {cid} 转换为 XML。")

    subcircuits_elem = ET.SubElement(root, "subcircuits")
    for subckt in subcircuits:
        subckt_elem = ET.SubElement(subcircuits_elem, "subcircuit", id=subckt['id'], subckt=subckt['subckt'])
        for i, node in enumerate(subckt['nodes'], start=1):
            ET.SubElement(subckt_elem, f"node{i}").text = node
        print(f"[DEBUG] 将子电路 {subckt['id']} 转换为 XML。")

    commands_elem = ET.SubElement(root, "commands")
    for cmd in commands:
        cmd_elem = ET.SubElement(commands_elem, "command", name=cmd['command'])
        cmd_elem.text = ' '.join(cmd['params'])
        print(f"[DEBUG] 将命令 {cmd['command']} 转换为 XML。")

    print("[DEBUG] 完成 Netlist 转换为 XML。")
    return root


def pretty_print_xml(element):
    print("[DEBUG] 美化 XML 输出...")
    rough_string = ET.tostring(element, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")


def convert_netlist_to_xml_file(netlist, output_filename="spice_netlist.xml"):
    print("[DEBUG] 开始将 Netlist 转换为 XML 文件...")
    components, commands, subcircuits = parse_spice_netlist(netlist)
    xml_root = netlist_to_xml(components, commands, subcircuits)
    pretty_xml = pretty_print_xml(xml_root)
    with open(output_filename, "w") as f:
        f.write(pretty_xml)
    print(f"[DEBUG] Netlist 转换完成，保存至 {output_filename}。")


def create_graph_from_xml(xml_root):
    print("[DEBUG] 从 XML 创建图形结构...")
    components_element = xml_root.find('components')
    if components_element is None or len(components_element) == 0:
        print("[ERROR] XML 中未找到组件。")
        raise ValueError("XML 中未找到组件。")

    G = nx.MultiGraph()

    for component in components_element:
        ctype = component.attrib['type']
        cid = component.attrib['id']

        value_elem = component.find('value')
        model_elem = component.find('model')

        # 分离 'value' 和 'model'
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

        # 处理可选参数
        parameters = {}
        params_elem = component.find('parameters')
        if params_elem is not None:
            for param in params_elem:
                parameters[param.tag] = param.text

        nodes = []
        i = 1
        while component.find(f'node{i}') is not None:
            node = component.find(f'node{i}').text
            if node:  # 确保节点不为空
                nodes.append(node)
            i += 1

        if not nodes:
            print(f"[WARNING] 组件 {cid} 类型 {ctype} 未定义任何节点。")
            continue

        # 将组件作为图中的节点，包含类型、值、模型等属性
        G.add_node(cid, type=ctype, value=value, model=model, parameters=parameters)

        # 将组件与其节点连接
        for node in nodes:
            if node == '0':
                G.add_node(node, type='ground')
            else:
                G.add_node(node, type='node')
            G.add_edge(cid, node)
            print(f"[DEBUG] 连接组件 {cid} 到节点 {node}。")

    print("[DEBUG] 从 XML 创建图形结构完成。")
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
    print(f"[DEBUG] 绘制网格，大小 {grid_size}，范围 {grid_extent}。")


def update_xml_positions_and_directions(xml_root, pos, directions, flips=None, scaling_ratios=None):
    if flips is None:
        flips = {}
    if scaling_ratios is None:
        scaling_ratios = {}
    print("[DEBUG] 更新 XML 中的位置信息、方向、翻转和缩放比例...")
    components_elem = xml_root.find('components')
    for comp in components_elem:
        cid = comp.attrib['id']
        if cid in pos:
            x, y = pos[cid]
            comp.set('x', str(x))
            comp.set('y', str(y))
            print(f"[DEBUG] 设置组件 {cid} 的位置: x={x}, y={y}")
        if cid in directions:
            comp.set("direction", directions[cid])
            print(f"[DEBUG] 设置组件 {cid} 的方向: {directions[cid]}")
        if cid in flips:
            comp.set("flip", flips[cid])
            print(f"[DEBUG] 设置组件 {cid} 的翻转: {flips[cid]}")
        if cid in scaling_ratios:
            scaling = scaling_ratios[cid]
            comp.set("vertical_scale", str(scaling.get('vertical_scale', 1.0)))
            comp.set("horizontal_scale", str(scaling.get('horizontal_scale', 1.0)))
            print(f"[DEBUG] 设置组件 {cid} 的垂直缩放: {scaling.get('vertical_scale', 1.0)}")
            print(f"[DEBUG] 设置组件 {cid} 的水平缩放: {scaling.get('horizontal_scale', 1.0)}")
    nodes_elem = xml_root.find('nodes')
    if nodes_elem is None:
        nodes_elem = ET.SubElement(xml_root, 'nodes')
    for nid in pos:
        if any(comp.attrib['id'] == nid for comp in components_elem):
            continue
        node_elem = next((node for node in nodes_elem if node.attrib.get('id') == nid), None)
        if node_elem is None:
            node_elem = ET.SubElement(nodes_elem, 'node', id=nid)
            print(f"[DEBUG] 添加新节点元素 {nid}。")
        x, y = pos[nid]
        node_elem.set('x', str(x))
        node_elem.set('y', str(y))
        print(f"[DEBUG] 设置节点 {nid} 的位置: x={x}, y={y}")


def add_anchor_dots(d, all_wires):
    print("[DEBUG] 添加锚点以确保连接正确...")
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
            print(f"[DEBUG] 在位置 {anchor_pos} 添加锚点。")
            added_dots.add(anchor_pos)


def build_comp_node_mapping(components):
    """
    构建组件 ID 与其节点引脚关联的映射关系。
    """
    print("[DEBUG] 构建组件与节点引脚的映射关系...")
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
                print(f"[DEBUG] 组件 {cid} 类型 Q 的节点数量不足。")
                continue
            node_collector = nodes[0]
            node_base = nodes[1]
            node_emitter = nodes[2]
            mapping = {}
            mapping.setdefault(node_collector, []).append('collector')
            mapping.setdefault(node_base, []).append('base')
            mapping.setdefault(node_emitter, []).append('emitter')
            comp_node_mapping[cid] = mapping
            print(f"[DEBUG] 组件 {cid} 的映射: {mapping}")

        elif ctype == 'D':
            if len(nodes) < 2:
                print(f"[DEBUG] 组件 {cid} 类型 D 的节点数量不足。")
                continue
            node_anode = nodes[0]
            node_cathode = nodes[1]
            mapping = {}
            mapping.setdefault(node_anode, []).append('anode')
            mapping.setdefault(node_cathode, []).append('cathode')
            comp_node_mapping[cid] = mapping
            print(f"[DEBUG] 组件 {cid} 的映射: {mapping}")

        elif ctype in ['R', 'C', 'L', 'S']:
            if len(nodes) < 2:
                print(f"[DEBUG] 组件 {cid} 类型 {ctype} 的节点数量不足。")
                continue
            node_start = nodes[0]
            node_end = nodes[1]
            mapping = {}
            mapping.setdefault(node_start, []).append('start')
            mapping.setdefault(node_end, []).append('end')
            comp_node_mapping[cid] = mapping
            print(f"[DEBUG] 组件 {cid} 的映射: {mapping}")

        elif ctype in ['V', 'I', 'E', 'H', 'F', 'G']:
            if len(nodes) < 2:
                print(f"[DEBUG] 组件 {cid} 类型 {ctype} 的节点数量不足。")
                continue
            node_positive = nodes[0]
            node_negative = nodes[1]
            mapping = {}
            mapping.setdefault(node_positive, []).append('positive')
            mapping.setdefault(node_negative, []).append('negative')
            comp_node_mapping[cid] = mapping
            print(f"[DEBUG] 组件 {cid} 的映射: {mapping}")

        elif ctype == 'J':
            if len(nodes) < 3:
                print(f"[DEBUG] 组件 {cid} 类型 J 的节点数量不足。")
                continue
            node_drain = nodes[0]
            node_gate = nodes[1]
            node_source = nodes[2]
            mapping = {}
            mapping.setdefault(node_drain, []).append('drain')
            mapping.setdefault(node_gate, []).append('gate')
            mapping.setdefault(node_source, []).append('source')
            comp_node_mapping[cid] = mapping
            print(f"[DEBUG] 组件 {cid} 的映射: {mapping}")

        elif ctype == 'M':
            if len(nodes) < 4:
                print(f"[DEBUG] 组件 {cid} 类型 M 的节点数量不足。")
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
            print(f"[DEBUG] 组件 {cid} 的映射: {mapping}")

        # 根据需要添加更多组件类型

    print("[DEBUG] 组件与节点引脚的映射关系构建完成。")
    return comp_node_mapping


def get_component_pin(comp, neighbor, comp_info, components_element, pins, pin_mapping):
    """
    为组件分配特定的引脚以连接到邻居节点。
    """
    ctype = comp_info.get('type')

    if ctype in ['Q', 'D', 'M']:
        # 对于 BJT (Q)、二极管 (D) 和 MOSFET (M)，使用特定的引脚映射
        if comp in pin_mapping:
            mapping = pin_mapping.get(comp, {})
            if neighbor in mapping and mapping[neighbor]:
                pin = mapping[neighbor].pop(0)
                print(f"[DEBUG] 为组件 '{comp}' 分配引脚 '{pin}' 连接到邻居 '{neighbor}'")
                return pin
            else:
                print(f"[DEBUG] 组件 '{comp}' 没有可用的引脚连接到邻居 '{neighbor}'")
                return None
    elif ctype in ['R', 'C', 'L', 'S', 'V', 'I', 'E', 'H', 'F', 'G', 'J']:
        # 对于其他组件，使用通用的引脚映射
        if comp in pin_mapping:
            mapping = pin_mapping.get(comp, {})
            if neighbor in mapping and mapping[neighbor]:
                pin = mapping[neighbor].pop(0)
                print(f"[DEBUG] 为组件 '{comp}' 分配引脚 '{pin}' 连接到邻居 '{neighbor}'")
                return pin
            else:
                print(f"[DEBUG] 组件 '{comp}' 没有可用的引脚连接到邻居 '{neighbor}'")
                return None

    # 默认情况
    print(f"[DEBUG] 默认分配引脚 'pin' 给组件 '{comp}' 连接到邻居 '{neighbor}'")
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
        print(f"[ERROR] 找不到组件/节点 '{node}' 的引脚 '{pin_name}'。")
        return None


def route_connection_current_method(start_pos, end_pos, component_boxes, comp, neighbor, all_wires):
    """
    使用曼哈顿路由方法连接起点和终点。
    """
    # 定义可能的路径：先水平再垂直，或先垂直再水平
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
        print(f"[DEBUG] 路径1 可行: {path1}")
        return path1, wire_safe_color
    if path_ok(path2):
        print(f"[DEBUG] 路径2 可行: {path2}")
        return path2, wire_safe_color

    print(f"[DEBUG] 两条路径均不可行。选择路径1，使用危险颜色。")
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
                print(f"[DEBUG] 线从 {start_pos} 到 {end_pos} 与组件 {comp_id} 相交")
                return True
    return False


def check_overlapping_wires(all_wires):
    print("[DEBUG] 检查是否有重叠的连接线...")
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
                print(f"[DEBUG] 发现重叠的连接线: 线 {i} 和线 {j}。标记为危险颜色。")
    return overlapping


def set_element_direction(element, direction):
    print(f"[DEBUG] 设置方向: {direction}")
    if direction == 'up':
        print("[DEBUG] 应用 'up' 方向。")
        element.up()
    elif direction == 'down':
        print("[DEBUG] 应用 'down' 方向。")
        element.down()
    elif direction == 'left':
        print("[DEBUG] 应用 'left' 方向。")
        element.left()
    else:
        print("[DEBUG] 应用 'right' 方向（默认）。")
        element.right()


def rotate_ccw(x, y, cx, cy, theta=0):
    """
    绕点 (cx, cy) 逆时针旋转 (x, y) 角度 theta。

    :param x: 点的 x 坐标
    :param y: 点的 y 坐标
    :param cx: 旋转中心的 x 坐标
    :param cy: 旋转中心的 y 坐标
    :param theta: 旋转角度（度）
    :return: 旋转后的 (x, y)
    """
    rad = math.radians(theta)
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
        f"[DEBUG] 获取组件类型 '{component_type}' 的边界框，位置 {pos}，方向 '{direction}'，翻转 '{flip}'，垂直缩放 '{vertical_scale}'，水平缩放 '{horizontal_scale}'")
    # 处理 MOSFET
    if component_type == 'M':
        model = kwargs.get('model', '').upper()
        if model.startswith('P'):
            try:
                comp_class = elm.PFet
                print("[DEBUG] 检测到 PMOS 晶体管。")
            except AttributeError:
                print("Error: schemdraw 不支持 'PFet' 元素。请检查元素名称。")
                return
        else:
            try:
                comp_class = elm.NFet
                print("[DEBUG] 检测到 NMOS 晶体管。")
            except AttributeError:
                print("Error: schemdraw 不支持 'NFet' 元素。请检查元素名称。")
                return
    else:
        comp_class = component_map.get(component_type, None)
        if comp_class is None:
            raise ValueError(f"不支持的组件类型: {component_type}")
        print(f"[DEBUG] 使用 schemdraw 元素 '{comp_class.__name__}' 处理组件类型 '{component_type}'。")

    if component_type == 'M':
        # 提取额外参数
        bulk = kwargs.get('bulk', False)
        element = comp_class(bulk=bulk).at(pos)
    else:
        element = comp_class(**kwargs).at(pos)

    # 设置方向
    set_element_direction(element, direction)
    d = schemdraw.Drawing(show=False)
    # 应用缩放（在添加到绘图之前）
    element = element.scalex(horizontal_scale).scaley(vertical_scale)
    element = d.add(element)
    print(f"[DEBUG] 元件已添加到绘图，应用缩放: {element}")

    # 确定参考锚点
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
    xmin_global = xDiff + bbox_local.xmin
    xmax_global = xDiff + bbox_local.xmax
    ymin_global = yDiff + bbox_local.ymin
    ymax_global = yDiff + bbox_local.ymax

    # 根据组件类型调整边界框
    if component_type in ['GND', 'ground']:
        ymax_global -= shrink_size * vertical_scale
    elif component_type in ['R', 'C', 'L', 'D', 'V', 'I', 'E', 'H', 'F', 'G', 'S']:
        xmin_global += shrink_size * horizontal_scale
        xmax_global -= shrink_size * horizontal_scale
    elif component_type == 'J':
        xmax_global -= shrink_size * horizontal_scale
        ymin_global += shrink_size * vertical_scale
        ymax_global -= shrink_size * vertical_scale
        # 防止连接线穿过结点 FET
        xmin_global -= shrink_size * horizontal_scale * 2
    elif component_type == 'Q':
        xmin_global += shrink_size * horizontal_scale
        ymin_global += shrink_size * vertical_scale
        ymax_global -= shrink_size * vertical_scale
        # 防止连接线穿过双极晶体管
        xmax_global += shrink_size * horizontal_scale * 2
    elif component_type == 'M':
        xmin_global += shrink_size * horizontal_scale * 100
        ymin_global += shrink_size * vertical_scale * 100
        ymax_global -= shrink_size * vertical_scale * 100
        xmax_global -= shrink_size * horizontal_scale * 100 # 按照用户配置调整

    anchors_global = {k: (v.x, v.y) for k, v in element.absanchors.items()}
    bbox = {
        'xmin': xmin_global,
        'ymin': ymin_global,
        'xmax': xmax_global,
        'ymax': ymax_global
    }
    print(f"[DEBUG] 缩放后的边界框: {bbox}")

    # 获取边界框的所有顶点
    vertices = [
        (bbox['xmin'], bbox['ymin']),
        (bbox['xmax'], bbox['ymin']),
        (bbox['xmax'], bbox['ymax']),
        (bbox['xmin'], bbox['ymax'])
    ]

    print(f"[DEBUG] 原始顶点: {vertices}")
    (x_center, y_center) = pos
    print(f"[DEBUG] 旋转中心: ({x_center}, {y_center})")
    print(f"[DEBUG] 参考锚点: {abs_ref}")

    # 旋转每个顶点
    if direction == 'up':
        rotated_vertices = [rotate_ccw(x, y, x_center, y_center, 90) for x, y in vertices]
    elif direction == 'left':
        rotated_vertices = [rotate_ccw(x, y, x_center, y_center, 180) for x, y in vertices]
    elif direction == 'down':
        rotated_vertices = [rotate_ccw(x, y, x_center, y_center, 270) for x, y in vertices]
    else:
        rotated_vertices = vertices
    # 计算旋转后的边界
    rotated_x = [v[0] for v in rotated_vertices]
    rotated_y = [v[1] for v in rotated_vertices]

    bbox = {
        'xmin': min(rotated_x),
        'ymin': min(rotated_y),
        'xmax': max(rotated_x),
        'ymax': max(rotated_y)
    }
    print(f"[DEBUG] 旋转后的边界框: {bbox}")

    # 获取旋转后的顶点
    rotated_vertices = [
        (bbox['xmin'], bbox['ymin']),
        (bbox['xmax'], bbox['ymin']),
        (bbox['xmax'], bbox['ymax']),
        (bbox['xmin'], bbox['ymax'])
    ]

    print(f"[DEBUG] 旋转后的顶点: {rotated_vertices}")

    # 翻转组件
    if component_type in ['J', 'M']:
        # 方向为上或下时，水平翻转等同于垂直翻转
        flipped_vertices = flip_component(rotated_vertices, (x_center, y_center), mode=flip)
    elif component_type == 'Q':
        # 方向为上或下时，水平翻转等同于垂直翻转
        if direction in ['up', 'down']:
            if flip == 'horizontal':
                flipped_vertices = flip_component(rotated_vertices, (x_center, y_center), mode=flip)
                print("应用水平翻转")
            elif flip == 'vertical':
                collector = element.absanchors.get('collector', Point(x_center, y_center))
                print(f"[DEBUG] 参考点: ({collector[0]}, {collector[1]})")
                flipped_vertices = flip_component(rotated_vertices, collector, mode=flip)
                print("应用垂直翻转")
            else:
                flipped_vertices = rotated_vertices
            print("完成翻转处理")
        else:
            if flip == 'horizontal':
                collector = element.absanchors.get('collector', Point(x_center, y_center))
                print(f"[DEBUG] 参考点: ({collector[0]}, {collector[1]})")
                flipped_vertices = flip_component(rotated_vertices, collector, mode=flip)
                print("应用水平翻转")
            elif flip == 'vertical':
                flipped_vertices = flip_component(rotated_vertices, (x_center, y_center), mode=flip)
                print("应用垂直翻转")
            else:
                flipped_vertices = rotated_vertices
    else:
        # 其他组件不需要翻转
        flipped_vertices = rotated_vertices

    # 计算翻转后的边界框
    rotated_x = [v[0] for v in flipped_vertices]
    rotated_y = [v[1] for v in flipped_vertices]

    bbox = {
        'xmin': min(rotated_x),
        'ymin': min(rotated_y),
        'xmax': max(rotated_x),
        'ymax': max(rotated_y)
    }
    print(f"[DEBUG] 翻转后的边界框: {bbox}")
    print(f"[DEBUG] 全局锚点: {anchors_global}")

    return bbox, element.anchors, anchors_global


def draw_component_or_node(d, elements, pins, node, node_info, x, y, direction, component_boxes, G, flip='none',
                           scaling_ratios=None):
    if scaling_ratios is None:
        scaling_ratios = {}
    print(f"[DEBUG] 绘制组件/节点: {node} 类型: {node_info.get('type')}")
    ctype = node_info.get('type')
    if ctype is None:
        ctype = 'node'

    deg = G.degree(node)

    if ctype == 'node':
        if deg >= 3:
            elements[node] = d.add(elm.Dot(radius=0.12).at((x, y)))
            d.add(elm.Label().at((x, y)).label(node, ofst=0.2))
            pins[node] = {'pin': (x, y)}
            print(f"[DEBUG] 绘制节点 '{node}' 作为连接点 (度={deg})。")
        else:
            d.add(elm.Label().at((x, y)).label(node, ofst=0.2))
            pins[node] = {'pin': (x, y)}
            print(f"[DEBUG] 绘制节点 '{node}' 作为标签 (度={deg})。")
        return

    if ctype == 'ground':
        ctype_for_bbox = 'GND'
    else:
        ctype_for_bbox = ctype

    print(f"[DEBUG] 用于边界框的组件类型: {ctype_for_bbox}")
    try:
        if ctype_for_bbox == 'M':
            # 获取模型以确定是 PMOS 还是 NMOS
            model = node_info.get('model', '').upper()
            if not model:
                # 尝试从参数中获取模型
                model = node_info.get('parameters', {}).get('MODEL', '').upper()
            if model.startswith('P'):
                comp_class = elm.PFet
                print("[DEBUG] 检测到 PMOS 晶体管。")
            else:
                comp_class = elm.NFet
                print("[DEBUG] 检测到 NMOS 晶体管。")
            # 提取 bulk 参数
            bulk = 'bulk' in node_info.get('pins', [])
            # 获取额外参数
            params = node_info.get('parameters', {})
            # 移除 'MODEL' 以避免传递给元素
            params = {k: v for k, v in params.items() if k != 'MODEL'}
            # 获取垂直和水平缩放比例
            scale_info = scaling_ratios.get(node, default_scaling_ratios.get(ctype_for_bbox, {'vertical_scale': 1.0,
                                                                                              'horizontal_scale': 1.0}))
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
            # 获取垂直和水平缩放比例
            scale_info = scaling_ratios.get(node, default_scaling_ratios.get(ctype_for_bbox, {'vertical_scale': 1.0,
                                                                                              'horizontal_scale': 1.0}))
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
        # 实例化正确的 MOSFET 类型
        if model.startswith('P'):
            try:
                element = elm.PFet(bulk=True).at((x, y))
            except AttributeError:
                print("Error: schemdraw 不支持 'PFet' 元素。请检查元素名称。")
                return
        else:
            try:
                element = elm.NFet(bulk=True).at((x, y))
            except AttributeError:
                print("Error: schemdraw 不支持 'NFet' 元素。请检查元素名称。")
                return
    else:
        comp_class = component_map.get(ctype_for_bbox, elm.Dot(radius=0.12))
        if comp_class is None:
            print(f"[ERROR] 不支持的组件类型: {ctype_for_bbox}")
            return
        # 应用水平缩放和垂直缩放
        element = comp_class(**node_info.get('parameters', {})).at((x, y))

    # 设置方向
    set_element_direction(element, direction)

    # 应用翻转（非 MOSFET）
    if ctype_for_bbox != 'M':
        if direction in ['up', 'down']:
            if flip == 'horizontal':
                element.flip()
                print(f"[DEBUG] 应用水平翻转到组件 '{node}'。")
            elif flip == 'vertical':
                element.reverse()
                print(f"[DEBUG] 应用垂直翻转到组件 '{node}'。")
        else:
            if flip == 'horizontal':
                element.reverse()
                print(f"[DEBUG] 应用水平翻转到组件 '{node}'。")
            elif flip == 'vertical':
                element.flip()
                print(f"[DEBUG] 应用垂直翻转到组件 '{node}'。")

    # 添加标签
    element.label(label_text)
    element.scalex(horizontal_scale).scaley(vertical_scale)
    # 将元件添加到绘图（缩放已在前面应用）
    element = d.add(element)
    print(f"[DEBUG] 绘制组件 '{node}'，标签 '{label_text}'。")

    # 根据组件类型分配引脚
    if ctype_for_bbox in ['V', 'I', 'E', 'H', 'F', 'G']:
        pins[node] = {
            'positive': (element.absanchors['end'].x, element.absanchors['end'].y),
            'negative': (element.absanchors['start'].x, element.absanchors['start'].y)
        }
        print(f"[DEBUG] 为组件 '{node}' 分配 'positive' 和 'negative' 引脚。")
    elif ctype_for_bbox in ['R', 'C', 'L', 'S']:
        pins[node] = {
            'start': (element.absanchors['start'].x, element.absanchors['start'].y),
            'end': (element.absanchors['end'].x, element.absanchors['end'].y)
        }
        print(f"[DEBUG] 为组件 '{node}' 分配 'start' 和 'end' 引脚。")
    elif ctype_for_bbox == 'D':
        pins[node] = {
            'anode': (element.absanchors['start'].x, element.absanchors['start'].y),
            'cathode': (element.absanchors['end'].x, element.absanchors['end'].y)
        }
        print(f"[DEBUG] 为组件 '{node}' 分配 'anode' 和 'cathode' 引脚。")
    elif ctype_for_bbox == 'J':
        pins[node] = {
            'drain': (element.absanchors['drain'].x, element.absanchors['drain'].y),
            'gate': (element.absanchors['gate'].x, element.absanchors['gate'].y),
            'source': (element.absanchors['source'].x, element.absanchors['source'].y)
        }
        print(f"[DEBUG] 为组件 '{node}' 分配 'drain'、'gate' 和 'source' 引脚。")
    elif ctype_for_bbox == 'Q':
        pins[node] = {
            'collector': (element.absanchors['collector'].x, element.absanchors['collector'].y),
            'base': (element.absanchors['base'].x, element.absanchors['base'].y),
            'emitter': (element.absanchors['emitter'].x, element.absanchors['emitter'].y)
        }
        print(f"[DEBUG] 为组件 '{node}' 分配 'collector'、'base' 和 'emitter' 引脚。")
    elif ctype_for_bbox == 'M':
        pins[node] = {
            'drain': (element.absanchors['drain'].x, element.absanchors['drain'].y),
            'gate': (element.absanchors['gate'].x, element.absanchors['gate'].y),
            'source': (element.absanchors['source'].x, element.absanchors['source'].y),
            'bulk': (element.absanchors['bulk'].x, element.absanchors['bulk'].y)
        }
        print(f"[DEBUG] 为组件 '{node}' 分配 'drain'、'gate'、'source' 和 'bulk' 引脚。")
    elif ctype_for_bbox == 'ground':
        pins[node] = {'pin': (x, y)}
        print(f"[DEBUG] 为地节点 '{node}' 分配单个 'pin'。")
    elif ctype_for_bbox == 'node':
        pins[node] = {'pin': (x, y)}
        print(f"[DEBUG] 为节点 '{node}' 分配单个 'pin'。")
    else:
        pins[node] = {'pin': (x, y)}
        print(f"[DEBUG] 为组件/节点 '{node}' 分配默认的 'pin'。")

    x_min, y_min, x_max, y_max = bbox['xmin'], bbox['ymin'], bbox['xmax'], bbox['ymax']
    component_boxes[node] = {
        'bbox': bbox,
        'polygon': Polygon([(x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max)]),
        'type': ctype_for_bbox
    }
    print(f"[DEBUG] 组件 '{node}' 的边界框: {bbox}")


def draw_connections(d, G, components_element, pins, component_boxes, drawn_edges, routing_method, all_wires,
                     pin_mapping, grid_size=1.0):
    print("[DEBUG] 开始绘制连接线...")
    for comp, neighbor, key in G.edges(keys=True):
        edge_nodes = tuple(sorted([comp, neighbor]))
        # 包含 key 以区分多条边
        edge_key = edge_nodes + (key,)
        print(f"[DEBUG] 处理边缘键: {edge_key}")
        if edge_key in drawn_edges:
            print(f"[DEBUG] 边缘 {edge_key} 已绘制，跳过。")
            continue

        comp_info = G.nodes[comp]
        neighbor_info = G.nodes[neighbor]

        comp_pin = get_component_pin(comp, neighbor, comp_info, components_element, pins, pin_mapping)
        print(f"[DEBUG] 组件 '{comp}' 的引脚: {comp_pin}")
        neighbor_pin = get_component_pin(neighbor, comp, neighbor_info, components_element, pins, pin_mapping)
        print(f"[DEBUG] 组件 '{neighbor}' 的引脚: {neighbor_pin}")
        if comp_pin is None or neighbor_pin is None:
            print(f"[WARNING] 跳过边缘 {edge_key}，因为引脚分配失败。")
            continue

        start_pos = get_pin_position(pins, comp, comp_pin, grid_size=grid_size)
        end_pos = get_pin_position(pins, neighbor, neighbor_pin, grid_size=grid_size)

        print(
            f"[DEBUG] 连接 '{comp}' 的引脚 '{comp_pin}' 在 {start_pos} 到 '{neighbor}' 的引脚 '{neighbor_pin}' 在 {end_pos}")
        if start_pos is None or end_pos is None:
            print(f"[WARNING] 跳过边缘 {edge_key}，因为位置信息缺失。")
            continue

        selected_path, line_color = route_connection_current_method(
            start_pos, end_pos, component_boxes, comp, neighbor, all_wires
        )

        for i in range(len(selected_path) - 1):
            segment_start = selected_path[i]
            segment_end = selected_path[i + 1]
            # 确保连接线对齐到网格
            segment_start = align_to_grid(segment_start, grid_size=grid_size)
            segment_end = align_to_grid(segment_end, grid_size=grid_size)
            new_line = LineString([segment_start, segment_end])
            all_wires.append({'line': new_line, 'color': line_color})
            print(f"[DEBUG] 添加连接线段从 {segment_start} 到 {segment_end}，颜色 '{line_color}'")

        drawn_edges.add(edge_key)
        print(f"[DEBUG] 完成绘制边缘 {edge_key}")

    print("[DEBUG] 完成所有连接线的绘制。")


def draw_circuit(G, components_element, xml_root, xml_file, initial_comp_node_mapping, max_attempts=100,
                 EnlargeSize=2.5, routing_method=1, auto=1, grid_size=1.0):
    print("[DEBUG] 开始绘制电路...")
    attempt = 0
    success = False

    last_pos = None
    last_all_wires = None

    # 从 XML 中读取现有的方向、翻转、位置和缩放比例
    positions_in_xml = {}
    directions_in_xml = {}
    flips_in_xml = {}  # 读取翻转属性
    scaling_ratios = {}  # 读取缩放比例

    components_elem = xml_root.find('components')
    if components_elem is not None:
        for comp in components_elem:
            cid = comp.attrib['id']
            dir_val = comp.attrib.get('direction', 'right')
            flip_val = comp.attrib.get('flip', 'none')  # 读取翻转属性
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
                f"[DEBUG] 组件 '{cid}': 方向='{dir_val}', 翻转='{flip_val}', 垂直缩放='{vertical_scale}', 水平缩放='{horizontal_scale}', 位置={positions_in_xml.get(cid, '未设置')}")

    nodes_elem = xml_root.find('nodes')
    if nodes_elem is not None:
        for node in nodes_elem:
            nid = node.attrib['id']
            if 'x' in node.attrib and 'y' in node.attrib:
                positions_in_xml[nid] = (float(node.attrib['x']), float(node.attrib['y']))
                print(f"[DEBUG] 节点 '{nid}': 位置=({node.attrib['x']}, {node.attrib['y']})")

    if auto == 1:
        # 自动布局
        while attempt < max_attempts:
            attempt += 1
            print(f"[DEBUG] 尝试 {attempt} (自动布局)")

            original_pos = nx.spring_layout(G)
            # 放大布局
            pos = {n: (EnlargeSize * original_pos[n][0], EnlargeSize * (-original_pos[n][1])) for n in G.nodes}

            # 对齐到网格
            pos = {n: align_to_grid(pos[n], grid_size=grid_size) for n in pos}

            directions = {}
            flips = {}
            scales = {}
            for n in G.nodes:
                directions[n] = directions_in_xml.get(n, 'right')
                flips[n] = flips_in_xml.get(n, 'none')
                scales[n] = scaling_ratios.get(n, default_scaling_ratios.get(G.nodes[n]['type'], {'vertical_scale': 1.0,
                                                                                                  'horizontal_scale': 1.0}))

            d = schemdraw.Drawing()
            if draw_grid_or_not == 1:
                draw_grid(d, grid_size=grid_size)  # 添加网格线

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
                scale = scales.get(node, {'vertical_scale': 1.0, 'horizontal_scale': 1.0})  # 获取缩放比例
                print(
                    f"[DEBUG] 绘制组件/节点 '{node}' 在 ({x}, {y})，方向 '{direction}'，翻转 '{flip_in_xml}'，缩放 {scale}")
                draw_component_or_node(d, elements, pins, node, node_info, x, y, direction, component_boxes, G,
                                       flip=flip_in_xml, scaling_ratios=scales)  # 传递缩放比例
                # 更新缩放比例
                scaling_ratios[node] = scale

            # 使用组件-节点映射的深拷贝
            pin_mapping_copy = copy.deepcopy(initial_comp_node_mapping)

            draw_connections(d, G, components_element, pins, component_boxes, drawn_edges, routing_method,
                             all_wires, pin_mapping_copy, grid_size=grid_size)

            any_red_line = any(wire['color'] == wire_danger_color for wire in all_wires)
            overlapping = check_overlapping_wires(all_wires)

            last_pos = pos
            last_all_wires = all_wires

            if any_red_line or overlapping:
                print("[DEBUG] 检测到重叠或危险颜色线。重试...")
                continue
            else:
                print(f"[DEBUG] 布局成功，尝试次数 {attempt}。绘制连接线...")
                for wire_info in all_wires:
                    line = wire_info['line']
                    color = wire_info['color']
                    d.add(elm.Line().at(line.coords[0]).to(line.coords[1]).color(color))
                    print(f"[DEBUG] 绘制连接线从 {line.coords[0]} 到 {line.coords[1]}，颜色 '{color}'")
                add_anchor_dots(d, all_wires)
                print(f"[DEBUG] 布局成功，尝试次数 {attempt}。")
                update_xml_positions_and_directions(xml_root, pos, directions, flips, scaling_ratios)  # 传递缩放比例
                tree = ET.ElementTree(xml_root)
                tree.write(xml_file)
                d.draw()
                success = True
                break

        if not success:
            print("[DEBUG] 所有尝试均未找到合适的布局。显示最后一次尝试的结果（可能有问题）。")
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
                    scale = scaling_ratios.get(node, {'vertical_scale': 1.0, 'horizontal_scale': 1.0})  # 获取缩放比例
                    print(
                        f"[DEBUG] 重新绘制组件/节点 '{node}' 在 ({x}, {y})，方向 '{direction}'，翻转 '{flip_in_xml}'，缩放 {scale}")
                    draw_component_or_node(d, elements, pins, node, node_info, x, y, direction, component_boxes, G,
                                           flip=flip_in_xml, scaling_ratios=scaling_ratios)  # 传递缩放比例
                    # 更新缩放比例
                    scaling_ratios[node] = scale

                for wire_info in last_all_wires:
                    line = wire_info['line']
                    color = wire_info['color']
                    d.add(elm.Line().at(line.coords[0]).to(line.coords[1]).color(color))
                    print(f"[DEBUG] 绘制连接线从 {line.coords[0]} 到 {line.coords[1]}，颜色 '{color}'")
                add_anchor_dots(d, last_all_wires)
                d.draw()

    else:
        # 固定布局
        print("[DEBUG] 使用固定布局绘制电路 (auto=0)...")
        d = schemdraw.Drawing()
        if draw_grid_or_not == 1:
            draw_grid(d, grid_size=grid_size)  # 添加网格线
        elements = {}
        pins = {}
        component_boxes = {}
        drawn_edges = set()
        all_wires = []

        # 使用 XML 中的位置信息
        pos = positions_in_xml
        directions = directions_in_xml
        flips = flips_in_xml

        for node in G.nodes:
            node_info = G.nodes[node]
            if node not in pos:
                print(f"[WARNING] 节点 '{node}' 未定义位置，跳过。")
                continue
            x, y = pos[node]
            direction = directions.get(node, 'right')
            flip_val = flips.get(node, 'none')
            scale = scaling_ratios.get(node, default_scaling_ratios.get(G.nodes[node]['type'], {'vertical_scale': 1.0,
                                                                                                'horizontal_scale': 1.0}))
            print(f"[DEBUG] 绘制组件/节点 '{node}' 在 ({x}, {y})，方向 '{direction}'，翻转 '{flip_val}'，缩放 {scale}")
            draw_component_or_node(d, elements, pins, node, node_info, x, y, direction, component_boxes, G,
                                   flip=flip_val, scaling_ratios=scaling_ratios)  # 传递缩放比例
            # 更新缩放比例
            scaling_ratios[node] = scale

        # 使用组件-节点映射的深拷贝
        pin_mapping_copy = copy.deepcopy(initial_comp_node_mapping)

        draw_connections(d, G, components_element, pins, component_boxes, drawn_edges, routing_method,
                         all_wires, pin_mapping_copy, grid_size=grid_size)

        # 检查连接线颜色和重叠
        any_red_line = any(wire['color'] == wire_danger_color for wire in all_wires)
        overlapping = check_overlapping_wires(all_wires)

        if any_red_line or overlapping:
            print("[WARNING] 存在危险颜色线条或重叠线条，部分连接可能有问题。")

        print(f"[DEBUG] 绘制连接线...")
        for wire_info in all_wires:
            line = wire_info['line']
            color = wire_info['color']
            d.add(elm.Line().at(line.coords[0]).to(line.coords[1]).color(color))
            print(f"[DEBUG] 绘制连接线从 {line.coords[0]} 到 {line.coords[1]}，颜色 '{color}'")
        add_anchor_dots(d, all_wires)

        # 更新 XML 中的位置信息和方向
        update_xml_positions_and_directions(xml_root, pos, directions, flips, scaling_ratios)
        tree = ET.ElementTree(xml_root)
        tree.write(xml_file)
        d.draw()

    print("[DEBUG] 电路绘制完成。")


def main():
    print("[DEBUG] 脚本启动。")
    # 输出 XML 文件名
    output_xml = "spice_netlist.xml"

    if auto == 1:
        print("[DEBUG] 将 Netlist 转换为 XML 文件...")
        convert_netlist_to_xml_file(netlist, output_xml)
    else:
        print("[DEBUG] 跳过 Netlist 转换为 XML (auto=0)。确保 XML 文件存在并包含组件位置。")

    try:
        print("[DEBUG] 解析 XML 文件...")
        tree = ET.parse(output_xml)
    except FileNotFoundError:
        print(f"[ERROR] 未找到 '{output_xml}' 文件。确保 Netlist 已转换为 XML。")
        sys.exit(1)
    except ET.ParseError as e:
        print(f"[ERROR] 解析 XML 时出错: {e}")
        sys.exit(1)

    xml_root = tree.getroot()
    G, components_element = create_graph_from_xml(xml_root)

    # 构建组件与节点引脚的映射关系
    initial_comp_node_mapping = build_comp_node_mapping(components_element)

    if auto == 1:
        print("[DEBUG] 开始自动布局...")
    else:
        print("[DEBUG] 开始固定布局...")

    # 绘制电路
    draw_circuit(
        G, components_element, xml_root, output_xml,
        initial_comp_node_mapping,
        max_attempts=max_attempts,
        EnlargeSize=EnlargeSize,
        routing_method=routing_method,
        auto=auto,
        grid_size=grid_size
    )

    print("[DEBUG] 脚本执行完成。")


if __name__ == "__main__":
    main()
