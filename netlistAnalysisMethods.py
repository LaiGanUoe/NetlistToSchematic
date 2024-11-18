import networkx as nx
import schemdraw
import schemdraw.elements as elm
import xml.etree.ElementTree as ET
import numpy as np
from shapely.geometry import LineString, Polygon, Point
import xml.dom.minidom as minidom


def parse_spice_netlist(netlist):
    """Parse the SPICE netlist into components, commands, and subcircuits."""
    components = []
    commands = []
    subcircuits = []
    for line in netlist.strip().split('\n'):
        # Remove inline comments starting with ';'
        if ';' in line:
            line = line.split(';')[0].strip()
        line = line.strip()
        # Skip comments (lines starting with '*')
        if line.startswith('*'):
            continue
        # Skip empty lines
        if not line.strip():
            continue

        try:
            parts = line.split()
            first_char = parts[0][0].upper()

            # Parse various components based on the first character
            if first_char in ['V', 'R', 'C', 'L', 'D', 'M', 'Q', 'I', 'J', 'K', 'S', 'T', 'E', 'F', 'G', 'H']:
                if first_char == 'V':
                    value = ' '.join(parts[3:])  # Voltage source
                    nodes = parts[1:3]
                elif first_char == 'I':
                    value = ' '.join(parts[3:])  # Current source
                    nodes = parts[1:3]
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

def netlist_to_xml(components, commands, subcircuits):
    """Convert parsed netlist data into an XML structure."""
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
    """Return a pretty-printed XML string for the Element."""
    rough_string = ET.tostring(element, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")

def convert_netlist_to_xml_file(netlist, output_filename="spice_netlist.xml"):
    """Convert netlist to XML and save it to a file."""
    components, commands, subcircuits = parse_spice_netlist(netlist)
    xml_root = netlist_to_xml(components, commands, subcircuits)
    pretty_xml = pretty_print_xml(xml_root)
    with open(output_filename, "w") as f:
        f.write(pretty_xml)
    print(pretty_xml)
