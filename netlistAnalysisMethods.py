import xml.etree.ElementTree as ET
import xml.dom.minidom as minidom

# Function to parse the SPICE netlist
def parse_spice_netlist(netlist):
    components = []
    commands = []
    subcircuits = []
    for line in netlist.strip().split('\n'):
        # Skip comments and empty lines
        if line.startswith('*') or not line.strip():
            continue
        # Parse components and subcircuits
        parts = line.split()
        first_char = parts[0][0]
        if first_char in ['V', 'R', 'C']:
            if first_char == 'V':
                value = ' '.join(parts[3:])  # Combine remaining parts for voltage source
            else:
                value = parts[-1]
            components.append({
                'type': first_char,
                'id': parts[0],
                'nodes': parts[1:3],
                'value': value
            })
        elif first_char == 'X':
            # Handle subcircuits, assuming 'X' is followed by the instance name, nodes, and the subcircuit name
            subcircuits.append({
                'type': 'X',  # Subcircuit identifier
                'id': parts[0],
                'nodes': parts[1:-1],
                'subckt': parts[-1]
            })
        # Parse commands
        elif line.startswith('.'):
            commands.append({'command': parts[0][1:], 'params': parts[1:]})

    return components, commands, subcircuits

# Convert parsed netlist to XML
def netlist_to_xml(components, commands, subcircuits):
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
