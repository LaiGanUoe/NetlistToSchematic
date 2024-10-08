import xml.etree.ElementTree as ET
import xml.dom.minidom as minidom

# Function to parse the SPICE netlist
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
        try:
            parts = line.split()
            first_char = parts[0][0]

            # Parse various components based on the number of nodes
            if first_char in ['V', 'R', 'C', 'L', 'D', 'M', 'Q', 'I', 'J', 'K', 'S', 'T', 'E', 'F', 'G', 'H']:
                if first_char == 'V':
                    value = ' '.join(parts[3:])  # Voltage source
                    nodes = parts[1:3]
                elif first_char == 'I':
                    value = ' '.join(parts[3:])  # Current source
                    nodes = parts[1:3]
                elif first_char == 'M':
                    value = ' '.join(parts[4:])  # MOSFET with model and dimensions
                    nodes = parts[1:4]  # MOSFET has 3 nodes (d, g, s)
                elif first_char == 'Q':
                    value = ' '.join(parts[4:])  # Bipolar transistor with model and area
                    nodes = parts[1:4]  # Bipolar Transistor has 3 nodes (c, b, e)
                elif first_char == 'J':
                    value = ' '.join(parts[4:])  # Junction FET
                    nodes = parts[1:4]  # JFET has 3 nodes (d, g, s)
                elif first_char == 'T':
                    value = ' '.join(parts[5:])  # Transmission line
                    nodes = parts[1:5]  # Transmission Line has 4 nodes (A+, A-, B+, B-)
                elif first_char == 'D':
                    value = parts[3]  # Diode with model
                    nodes = parts[1:3]
                elif first_char == 'K':
                    value = parts[3]  # Inductor coupling
                    nodes = parts[1:3]
                elif first_char == 'S':
                    value = ' '.join(parts[5:])  # Voltage-controlled switch
                    nodes = parts[1:5]  # Switch can have 4 nodes (control and output nodes)
                elif first_char == 'E':
                    # Voltage Controlled Voltage Source, 4 nodes (+node, -node, +control, -control)
                    nodes = parts[1:5]
                    value = ' '.join(parts[5:])  # Gain or POLY form
                elif first_char in ['F', 'G', 'H']:
                    value = ' '.join(parts[4:])  # Controlled sources
                    nodes = parts[1:3]
                else:
                    value = parts[-1]
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


def convert_netlist_to_xml_file(netlist):
    components, commands, subcircuits = parse_spice_netlist(netlist)
    xml_root = netlist_to_xml(components, commands, subcircuits)
    pretty_xml = pretty_print_xml(xml_root)
    print(pretty_xml)
