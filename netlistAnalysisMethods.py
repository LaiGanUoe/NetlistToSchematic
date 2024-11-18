import xml.etree.ElementTree as ET
import xml.dom.minidom as minidom



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
