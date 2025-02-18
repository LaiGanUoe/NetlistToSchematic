import networkx as nx
import schemdraw
import schemdraw.elements as elm
import xml.etree.ElementTree as ET
import numpy as np
from shapely.geometry import LineString, Polygon, Point
import xml.dom.minidom as minidom
import math
import copy
import sys

########################################################################
# 1) 配置区
########################################################################

# 这里是你实际要转换的 SPICE Netlist
netlist = """
* Netlist for emitter follower circuit
V1 Vin 0 SIN(0 1 1k)
V2 5V 0 DC 5
R1 5V B 4k
Q3 B 0 0 NPN
Q1 Vout B 0 NPN
Q2 0 Vout B PNP
.end
"""

# 是否自动布局 (1: auto, 0: fixed)
auto = 1

# 给地节点拆分阈值: 若 node 0 超过多少条连线，就多生成 0_1, 0_2,...
max_conn = 3

# 其他配置
EnlargeSize = 15
max_attempts = 3000
routing_method = 1
shrink_size = 0.03
wire_safe_color = 'green'
wire_danger_color = 'red'
grid_size = 0.1
grid_width = 0.8
draw_grid_or_not = 0
dot_radius = 0.06

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

default_scaling_ratios = {
    'GND': {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'R':   {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'C':   {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'L':   {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'D':   {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'V':   {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'I':   {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'S':   {'vertical_scale': 1.0, 'horizontal_scale': 1.0},
    'J':   {'vertical_scale': 6/5,  'horizontal_scale': 12/11},
    'Q':   {'vertical_scale': 300/209, 'horizontal_scale': 600/451},
    'M':   {'vertical_scale': 6/5,  'horizontal_scale': 90/82},
    'E':   {'vertical_scale': 1.0,  'horizontal_scale': 1.0},
    'H':   {'vertical_scale': 1.0,  'horizontal_scale': 1.0},
    'F':   {'vertical_scale': 1.0,  'horizontal_scale': 1.0},
    'G':   {'vertical_scale': 1.0,  'horizontal_scale': 1.0},
    'node':{'vertical_scale': 1.0,  'horizontal_scale': 1.0},
    'ground':{'vertical_scale': 1.0,'horizontal_scale': 1.0}
}

component_map = {
    'GND': elm.Ground,
    'R':   elm.Resistor,
    'C':   elm.Capacitor,
    'L':   elm.Inductor2,
    'D':   elm.Diode,
    'V':   elm.SourceV,
    'I':   elm.SourceI,
    'S':   elm.Switch,
    'J':   elm.JFet,
    'Q':   elm.Bjt,
    'M':   None,  # 由 model 决定 PFET or NFET
    'E':   elm.SourceControlledV,
    'H':   elm.SourceControlledV,
    'F':   elm.SourceControlledI,
    'G':   elm.SourceControlledI,
    'ground': elm.Ground,
    'node':   elm.Dot(radius=dot_radius)
}

########################################################################
# 2) 函数定义
########################################################################

def parse_spice_netlist(netlist):
    print("[DEBUG] parse_spice_netlist() start")
    components = []
    commands = []
    subcircuits = []
    for line_number, line in enumerate(netlist.strip().split('\n'), start=1):
        original_line = line
        if ';' in line:
            line = line.split(';')[0].strip()
        line = line.strip()
        if line.startswith('*'):
            print(f"[DEBUG] Skip comment line {line_number}: {original_line}")
            continue
        if not line:
            continue

        parts = line.split()
        if not parts:
            continue

        first_char = parts[0][0].upper()
        if first_char in ['V','I','R','C','L','D','M','Q','J','K','S','T','E','F','G','H']:
            if first_char in ['V','I','S','E','F','G','H']:
                if len(parts) < 4:
                    print(f"[ERROR] Invalid voltage/current source: {line}")
                    continue
                value = ' '.join(parts[3:])
                nodes = parts[1:3]
            elif first_char == 'D':
                if len(parts) < 4:
                    print(f"[ERROR] Invalid diode definition: {line}")
                    continue
                value = parts[3]
                nodes = parts[1:3]
            elif first_char == 'J':
                if len(parts) < 5:
                    print(f"[ERROR] Invalid JFET definition: {line}")
                    continue
                value = parts[4] if len(parts) > 4 else ''
                nodes = parts[1:4]
                components.append({
                    'type': first_char,
                    'id': parts[0],
                    'nodes': nodes,
                    'value': value,
                    'pins': ['drain','gate','source'],
                    'scale': 1.0
                })
                print(f"[DEBUG] Added JFET: {parts[0]} => {nodes}")
                continue
            elif first_char == 'Q':
                if len(parts) < 5:
                    print(f"[ERROR] Invalid BJT definition: {line}")
                    continue
                value = parts[4] if len(parts) > 4 else ''
                nodes = parts[1:4]
                components.append({
                    'type': first_char,
                    'id': parts[0],
                    'nodes': nodes,
                    'value': value,
                    'pins': ['collector','base','emitter'],
                    'scale': 1.0
                })
                print(f"[DEBUG] Added BJT: {parts[0]} => {nodes}")
                continue
            elif first_char == 'M':
                if len(parts) < 6:
                    print(f"[ERROR] Invalid MOSFET definition: {line}")
                    continue
                cid = parts[0]
                nodes = parts[1:5]
                model = parts[5]
                optional_params = parts[6:]
                params_dict = {}
                for param in optional_params:
                    if '=' in param:
                        key,val = param.split('=',1)
                        params_dict[key.upper()] = val
                components.append({
                    'type': first_char,
                    'id': cid,
                    'nodes': nodes,
                    'model': model,
                    'parameters': params_dict,
                    'pins': ['drain','gate','source','bulk'],
                    'scale': 1.0
                })
                print(f"[DEBUG] Added MOSFET: {cid} => {nodes}")
                continue
            else:
                if len(parts) < 4:
                    print(f"[ERROR] Invalid component: {line}")
                    continue
                value = ' '.join(parts[3:]) if len(parts)>3 else ''
                nodes = parts[1:3]
            components.append({
                'type': first_char,
                'id': parts[0],
                'nodes': nodes,
                'value': value,
                'scale': 1.0
            })
            print(f"[DEBUG] Added component: {parts[0]} => {nodes}")

        elif first_char == 'X':
            if len(parts) < 4:
                print(f"[ERROR] Invalid subcircuit: {line}")
                continue
            subcircuits.append({
                'type':'X',
                'id': parts[0],
                'nodes': parts[1:-1],
                'subckt': parts[-1]
            })
            print(f"[DEBUG] Added subckt: {parts[0]}")
        elif line.startswith('.'):
            commands.append({'command': parts[0][1:], 'params': parts[1:]})
            print(f"[DEBUG] Added command: {parts[0]}")
        else:
            print(f"[ERROR] Unknown line: {line}")

    print("[DEBUG] parse_spice_netlist() end")
    return components, commands, subcircuits


def netlist_to_xml(components, commands, subcircuits):
    print("[DEBUG] netlist_to_xml() start")
    root = ET.Element("spice_netlist")
    comps_elem = ET.SubElement(root, "components")
    for comp in components:
        comp_type = comp['type']
        cid = comp['id']
        comp_elem = ET.SubElement(comps_elem, "component", type=comp_type, id=cid)
        for i,nd in enumerate(comp['nodes'], start=1):
            ET.SubElement(comp_elem, f"node{i}").text = nd
        if 'model' in comp:
            ET.SubElement(comp_elem, "model").text = comp['model']
        if 'value' in comp:
            ET.SubElement(comp_elem, "value").text = comp['value']
        if 'parameters' in comp:
            params_elem = ET.SubElement(comp_elem, "parameters")
            for k,v in comp['parameters'].items():
                ET.SubElement(params_elem, k).text = v
        if 'pins' in comp:
            ET.SubElement(comp_elem, "pins").text = ' '.join(comp['pins'])
        # direction & flip
        comp_elem.set("direction", default_directions.get(comp_type,'right'))
        comp_elem.set("flip",      default_flips.get(comp_type,'none'))
        # scale
        sc = default_scaling_ratios.get(comp_type, {'vertical_scale':1.0,'horizontal_scale':1.0})
        comp_elem.set("vertical_scale", str(sc.get('vertical_scale',1.0)))
        comp_elem.set("horizontal_scale", str(sc.get('horizontal_scale',1.0)))
    subs_elem = ET.SubElement(root, "subcircuits")
    for sc in subcircuits:
        s = ET.SubElement(subs_elem, "subcircuit", id=sc['id'], subckt=sc['subckt'])
        for i,n in enumerate(sc['nodes'],start=1):
            ET.SubElement(s, f"node{i}").text = n
    cmds_elem = ET.SubElement(root,"commands")
    for c in commands:
        c_elem = ET.SubElement(cmds_elem,"command", name=c['command'])
        c_elem.text = ' '.join(c['params'])
    print("[DEBUG] netlist_to_xml() end")
    return root


def pretty_print_xml(element):
    rough_string = ET.tostring(element,'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")


def convert_netlist_to_xml_file(netlist, output_filename="spice_netlist.xml"):
    print(f"[DEBUG] convert_netlist_to_xml_file() => {output_filename}")
    comps,cmds,subs = parse_spice_netlist(netlist)
    root = netlist_to_xml(comps,cmds,subs)
    px = pretty_print_xml(root)
    with open(output_filename,"w") as f:
        f.write(px)
    print("[DEBUG] netlist->xml done.")


def reorganize_ground_nodes(G, max_conn=3):
    """
    拆分地节点 node='0', 若连线超出 max_conn, 生成 0_1, 0_2...。
    """
    print("[DEBUG] reorganize_ground_nodes() start, max_conn=", max_conn)
    if '0' not in G.nodes:
        print("[DEBUG] No node '0' in graph, skip.")
        return
    ground_edges = list(G.edges('0'))
    total_edges = len(ground_edges)
    print(f"[DEBUG] node '0' has total {total_edges} edges => {ground_edges}")

    if total_edges <= max_conn:
        print("[DEBUG] no need to split, end.\n")
        return

    # remove all edges from 0
    for e in ground_edges:
        G.remove_edge(e[0], e[1])
    # chunk
    chunked = [ ground_edges[i:i+max_conn] for i in range(0,total_edges,max_conn) ]
    print(f"[DEBUG] chunked => {chunked}")

    # first chunk => keep in '0'
    first_grp = chunked[0]
    for e in first_grp:
        G.add_edge('0', e[1])
    print(f"[DEBUG] reattached first group to '0': {first_grp}")

    # subsequent => create 0_1, 0_2...
    for idx, grp in enumerate(chunked[1:], start=1):
        if not grp:
            print(f"[DEBUG] chunk idx={idx} is empty, skip.")
            continue
        new_gnd = f'0_{idx}'
        G.add_node(new_gnd, type='ground')
        for e in grp:
            G.add_edge(new_gnd, e[1])
        print(f"[DEBUG] reattached group {idx} edges => {new_gnd}: {grp}")

    # debug check
    print(f"[DEBUG] after reorganize => edges(0): {list(G.edges('0'))}")
    for i in range(1,len(chunked)):
        ng = f'0_{i}'
        if ng in G.nodes:
            print(f"[DEBUG] edges({ng}): {list(G.edges(ng))}")

    print("[DEBUG] reorganize_ground_nodes() end.\n")


def create_graph_from_xml(xml_root):
    print("[DEBUG] create_graph_from_xml() start")
    comps_elem = xml_root.find('components')
    if comps_elem is None or len(comps_elem)==0:
        raise ValueError("[ERROR] no components in XML")

    G = nx.MultiGraph()
    for comp in comps_elem:
        ctype = comp.attrib['type']
        cid   = comp.attrib['id']
        val_e = comp.find('value')
        model_e = comp.find('model')
        if model_e is not None and model_e.text:
            model = model_e.text.upper()
        else:
            model = ''
        if val_e is not None and val_e.text:
            value = val_e.text
        else:
            value = ''
        params = {}
        par_e = comp.find('parameters')
        if par_e is not None:
            for p in par_e:
                params[p.tag] = p.text
        nodes = []
        i=1
        while comp.find(f'node{i}') is not None:
            nd_text = comp.find(f'node{i}').text
            if nd_text:
                nodes.append(nd_text)
            i+=1

        G.add_node(cid, type=ctype, value=value, model=model, parameters=params)
        for nd in nodes:
            if nd=='0':
                G.add_node('0', type='ground')
                G.add_edge(cid,'0')
            else:
                G.add_node(nd, type='node')
                G.add_edge(cid, nd)

    print("[DEBUG] graph creation done, now reorganize ground nodes.")
    reorganize_ground_nodes(G, max_conn=max_conn)

    print("[DEBUG] create_graph_from_xml() end")
    return G, comps_elem


def align_to_grid(position, grid_size=1.0):
    x,y = position
    return (round(x/grid_size)*grid_size, round(y/grid_size)*grid_size)


def draw_grid(d, grid_size=1.0, grid_extent=20):
    print("[DEBUG] draw_grid() => draw lines ±", grid_extent)
    for x in np.arange(-grid_extent, grid_extent+grid_size, grid_size):
        d.add(elm.Line().at((x,-grid_extent)).to((x,grid_extent)).color('lightgray').linewidth(grid_width))
    for y in np.arange(-grid_extent, grid_extent+grid_size, grid_size):
        d.add(elm.Line().at((-grid_extent,y)).to((grid_extent,y)).color('lightgray').linewidth(grid_width))


def update_xml_positions_and_directions(xml_root, pos, directions, flips=None, scaling_ratios=None):
    if flips is None: flips={}
    if scaling_ratios is None: scaling_ratios={}
    comps_elem = xml_root.find('components')
    for comp in comps_elem:
        cid=comp.attrib['id']
        if cid in pos:
            x,y=pos[cid]
            comp.set('x', str(x))
            comp.set('y', str(y))
        if cid in directions:
            comp.set("direction", directions[cid])
        if cid in flips:
            comp.set("flip", flips[cid])
        if cid in scaling_ratios:
            sc=scaling_ratios[cid]
            comp.set("vertical_scale",   str(sc.get('vertical_scale',1.0)))
            comp.set("horizontal_scale", str(sc.get('horizontal_scale',1.0)))

    nodes_elem = xml_root.find('nodes')
    if nodes_elem is None:
        nodes_elem=ET.SubElement(xml_root,"nodes")
    for nid in pos:
        # skip if there's same id in components
        if any(c.attrib['id']==nid for c in comps_elem):
            continue
        nd_e = next((n for n in nodes_elem if n.attrib.get('id')==nid), None)
        if nd_e is None:
            nd_e = ET.SubElement(nodes_elem,"node",id=nid)
        x,y=pos[nid]
        nd_e.set('x', str(x))
        nd_e.set('y', str(y))


def add_anchor_dots(d, all_wires):
    print("[DEBUG] add_anchor_dots() start")
    anchor_wire_count = {}
    for w in all_wires:
        line = w['line']
        s_pt = line.coords[0]
        e_pt = line.coords[1]
        anchor_wire_count[s_pt] = anchor_wire_count.get(s_pt,0)+1
        anchor_wire_count[e_pt] = anchor_wire_count.get(e_pt,0)+1

    added=set()
    for pt,cnt in anchor_wire_count.items():
        if cnt>=3 and pt not in added:
            d.add(elm.Dot(radius=0.12).at(pt))
            added.add(pt)
            print(f"[DEBUG] anchor dot at {pt}")


def build_comp_node_mapping(components):
    print("[DEBUG] build_comp_node_mapping() start")
    comp_node_mapping={}
    for c in components:
        ctype=c.attrib['type']
        cid=c.attrib['id']
        nodes=[]
        i=1
        while True:
            n=c.find(f'node{i}')
            if n is not None:
                nodes.append(n.text)
                i+=1
            else:
                break
        # build pin mapping
        if ctype=='Q':
            if len(nodes)<3:
                continue
            mapping={}
            mapping.setdefault(nodes[0],[]).append('collector')
            mapping.setdefault(nodes[1],[]).append('base')
            mapping.setdefault(nodes[2],[]).append('emitter')
            comp_node_mapping[cid]=mapping
        elif ctype=='D':
            if len(nodes)<2:
                continue
            mapping={}
            mapping.setdefault(nodes[0],[]).append('anode')
            mapping.setdefault(nodes[1],[]).append('cathode')
            comp_node_mapping[cid]=mapping
        elif ctype in ['R','C','L','S']:
            if len(nodes)<2:
                continue
            mapping={}
            mapping.setdefault(nodes[0],[]).append('start')
            mapping.setdefault(nodes[1],[]).append('end')
            comp_node_mapping[cid]=mapping
        elif ctype in ['V','I','E','H','F','G']:
            if len(nodes)<2:
                continue
            mapping={}
            mapping.setdefault(nodes[0],[]).append('positive')
            mapping.setdefault(nodes[1],[]).append('negative')
            comp_node_mapping[cid]=mapping
        elif ctype=='J':
            if len(nodes)<3:
                continue
            mapping={}
            mapping.setdefault(nodes[0],[]).append('drain')
            mapping.setdefault(nodes[1],[]).append('gate')
            mapping.setdefault(nodes[2],[]).append('source')
            comp_node_mapping[cid]=mapping
        elif ctype=='M':
            if len(nodes)<4:
                continue
            mapping={}
            mapping.setdefault(nodes[0],[]).append('drain')
            mapping.setdefault(nodes[1],[]).append('gate')
            mapping.setdefault(nodes[2],[]).append('source')
            mapping.setdefault(nodes[3],[]).append('bulk')
            comp_node_mapping[cid]=mapping
    print("[DEBUG] build_comp_node_mapping() end =>", comp_node_mapping)
    return comp_node_mapping


def get_component_pin(comp, neighbor, comp_info, comps_elem, pins, pin_mapping):
    ctype = comp_info.get('type')
    if ctype in ['Q','D','M','J']:
        if comp in pin_mapping:
            mapping = pin_mapping[comp]
            if neighbor in mapping and mapping[neighbor]:
                pin = mapping[neighbor].pop(0)
                print(f"[DEBUG] get_component_pin: {comp}->{neighbor} => {pin}")
                return pin
            else:
                print(f"[DEBUG] no pin left? {comp}->{neighbor}")
                return None
    elif ctype in ['R','C','L','S','V','I','E','H','F','G']:
        if comp in pin_mapping:
            mapping = pin_mapping[comp]
            if neighbor in mapping and mapping[neighbor]:
                pin = mapping[neighbor].pop(0)
                print(f"[DEBUG] get_component_pin: {comp}->{neighbor} => {pin}")
                return pin
            else:
                print(f"[DEBUG] no pin left? {comp}->{neighbor}")
                return None
    print(f"[DEBUG] get_component_pin default 'pin': {comp}->{neighbor}")
    return 'pin'


def get_pin_position(pins, node, pin_name, grid_size=1.0):
    try:
        pos = pins[node][pin_name]
        aligned = align_to_grid(pos, grid_size)
        return aligned
    except KeyError:
        print(f"[WARNING] get_pin_position: node '{node}' no pin '{pin_name}'")
        return None


def is_wire_crossing_components(start_pos, end_pos, comp_boxes, comp, neighbor):
    line=LineString([start_pos,end_pos])
    sp=Point(start_pos)
    ep=Point(end_pos)
    for cid, cinfo in comp_boxes.items():
        poly = cinfo['polygon']
        if line.intersects(poly):
            inter = line.intersection(poly)
            if (inter.equals(sp) or inter.equals(ep)) and (cid==comp or cid==neighbor):
                continue
            else:
                print(f"[DEBUG] is_wire_crossing => wire {start_pos}->{end_pos} crosses {cid}")
                return True
    return False


def route_connection_current_method(start_pos, end_pos, comp_boxes, comp, neighbor, all_wires, grid_size=1.0):
    start_pos = align_to_grid(start_pos, grid_size)
    end_pos   = align_to_grid(end_pos,   grid_size)
    mid_pos1 = align_to_grid((end_pos[0],start_pos[1]),grid_size)
    mid_pos2 = align_to_grid((start_pos[0],end_pos[1]),grid_size)
    path1=[start_pos, mid_pos1, end_pos]
    path2=[start_pos, mid_pos2, end_pos]

    def path_ok(path):
        for i in range(len(path)-1):
            if is_wire_crossing_components(path[i],path[i+1],comp_boxes,comp,neighbor):
                return False
            seg=LineString([path[i],path[i+1]])
            for w in all_wires:
                lw=w['line']
                if seg.equals(lw) or seg.contains(lw) or lw.contains(seg):
                    return False
        return True

    if path_ok(path1):
        return path1, wire_safe_color
    if path_ok(path2):
        return path2, wire_safe_color
    return path1, wire_danger_color


def check_overlapping_wires(all_wires):
    print("[DEBUG] check_overlapping_wires() start")
    overlapping=False
    n=len(all_wires)
    for i in range(n):
        li=all_wires[i]['line']
        for j in range(i+1,n):
            lj=all_wires[j]['line']
            if li.equals(lj) or li.contains(lj) or lj.contains(li):
                overlapping=True
                all_wires[i]['color']=wire_danger_color
                all_wires[j]['color']=wire_danger_color
                print(f"[DEBUG] overlapping wire {i} & {j}")
    return overlapping


def count_line_intersections(all_wires):
    intersection_points=set()
    lines=[w['line'] for w in all_wires]
    n=len(lines)
    for i in range(n):
        for j in range(i+1,n):
            l_i=lines[i]
            l_j=lines[j]
            inter=l_i.intersection(l_j)
            if inter.is_empty:
                continue
            if inter.geom_type=="Point":
                intersection_points.add((round(inter.x,6), round(inter.y,6)))
            elif inter.geom_type=="MultiPoint":
                for p in inter.geoms:
                    intersection_points.add((round(p.x,6), round(p.y,6)))
            elif inter.geom_type in ["LineString","MultiLineString"]:
                for p in inter.coords:
                    intersection_points.add((round(p[0],6), round(p[1],6)))
    return len(intersection_points)


def set_element_direction(element, direction):
    if direction=='up':
        element.up()
    elif direction=='down':
        element.down()
    elif direction=='left':
        element.left()
    else:
        element.right()


def rotate_ccw(x,y,cx,cy,theta=0):
    rad=math.radians(theta)
    xnew=(x-cx)*math.cos(rad)-(y-cy)*math.sin(rad)+cx
    ynew=(x-cx)*math.sin(rad)+(y-cy)*math.cos(rad)+cy
    return xnew,ynew


def flip_component(points, flip_point, mode="horizontal"):
    xf,yf=flip_point
    flipped=[]
    for x,y in points:
        if mode=="horizontal":
            nx=2*xf-x
            ny=y
        elif mode=="vertical":
            nx=x
            ny=2*yf-y
        else:
            nx=x
            ny=y
        flipped.append((nx,ny))
    return flipped


def get_component_bbox(component_type, pos=(1,2), direction='right', shrink_size=0.1, flip='none',
                       vertical_scale=1.0, horizontal_scale=1.0, **kwargs):
    # handle MOSFET
    if component_type=='M':
        model=kwargs.get('model','').upper()
        if model.startswith('P'):
            try:
                comp_class=elm.PFet
            except AttributeError:
                print("[ERROR] no PFet in schemdraw.")
                return
        else:
            try:
                comp_class=elm.NFet
            except AttributeError:
                print("[ERROR] no NFet in schemdraw.")
                return
    else:
        comp_class=component_map.get(component_type,None)
        if comp_class is None:
            raise ValueError(f"Unsupported type: {component_type}")

    if component_type=='M':
        bulk=kwargs.get('bulk',False)
        element=comp_class(bulk=bulk).at(pos)
    else:
        element=comp_class(**kwargs).at(pos)

    set_element_direction(element,direction)
    d=schemdraw.Drawing(show=False)
    element.scalex(horizontal_scale).scaley(vertical_scale)
    element=d.add(element)

    # pick anchor
    if 'start' in element.anchors:
        ref_name='start'
    elif 'drain' in element.anchors:
        if component_type=='M':
            ref_name='source'
        else:
            ref_name='drain'
    elif 'base' in element.anchors:
        ref_name='base'
    else:
        ref_name=list(element.anchors.keys())[0]

    abs_ref=element.absanchors[ref_name]
    local_ref=element.anchors[ref_name]
    if isinstance(local_ref,(tuple,list)):
        xDiff=abs_ref.x-local_ref[0]
        yDiff=abs_ref.y-local_ref[1]
    else:
        xDiff=abs_ref.x-local_ref.x
        yDiff=abs_ref.y-local_ref.y

    bbox_local=element.get_bbox()
    xmin_global=xDiff+bbox_local.xmin*horizontal_scale
    xmax_global=xDiff+bbox_local.xmax*horizontal_scale
    ymin_global=yDiff+bbox_local.ymin*vertical_scale
    ymax_global=yDiff+bbox_local.ymax*vertical_scale

    if component_type in ['GND','ground']:
        ymax_global-=shrink_size
    elif component_type in ['R','C','L','D','V','I','E','H','F','G','S']:
        xmin_global+=shrink_size
        xmax_global-=shrink_size
    elif component_type=='J':
        xmax_global-=shrink_size
        ymin_global+=shrink_size
        ymax_global-=shrink_size
        xmin_global-=shrink_size
    elif component_type=='Q':
        xmin_global+=shrink_size
        ymin_global+=shrink_size
        ymax_global-=shrink_size
        xmax_global+=shrink_size*2
    elif component_type=='M':
        xmin_global+=shrink_size
        ymin_global+=shrink_size
        ymax_global-=shrink_size
        xmax_global-=shrink_size

    # rotate
    vertices=[
        (xmin_global,ymin_global),
        (xmax_global,ymin_global),
        (xmax_global,ymax_global),
        (xmin_global,ymax_global)
    ]
    (cx,cy)=pos
    if direction=='up':
        rot_v=[rotate_ccw(x,y,cx,cy,90) for x,y in vertices]
    elif direction=='left':
        rot_v=[rotate_ccw(x,y,cx,cy,180) for x,y in vertices]
    elif direction=='down':
        rot_v=[rotate_ccw(x,y,cx,cy,270) for x,y in vertices]
    else:
        rot_v=vertices

    rx=[v[0] for v in rot_v]
    ry=[v[1] for v in rot_v]
    bb={
        'xmin':min(rx),'ymin':min(ry),
        'xmax':max(rx),'ymax':max(ry)
    }

    # flip
    if component_type in ['J','M']:
        flipped=flip_component(rot_v,(cx,cy),mode=flip)
    elif component_type=='Q':
        if 'collector' in element.absanchors:
            collector=element.absanchors['collector']
            if direction in ['up','down']:
                if flip=='horizontal':
                    flipped=flip_component(rot_v,(cx,cy),'horizontal')
                elif flip=='vertical':
                    flipped=flip_component(rot_v,(collector.x,collector.y),'vertical')
                else:
                    flipped=rot_v
            else:
                if flip=='horizontal':
                    flipped=flip_component(rot_v,(collector.x,collector.y),'horizontal')
                elif flip=='vertical':
                    flipped=flip_component(rot_v,(cx,cy),'vertical')
                else:
                    flipped=rot_v
        else:
            flipped=rot_v
    else:
        flipped=rot_v

    rxf=[v[0] for v in flipped]
    ryf=[v[1] for v in flipped]
    bb={
        'xmin':min(rxf),'ymin':min(ryf),
        'xmax':max(rxf),'ymax':max(ryf)
    }

    return bb, element.anchors, {k:(v.x,v.y) for k,v in element.absanchors.items()}


def draw_component_or_node(d, elements, pins, node, node_info, x, y, direction, comp_boxes, G,
                           flip='none', scaling_ratios=None):
    if scaling_ratios is None: scaling_ratios={}
    ctype=node_info.get('type','node')
    deg=G.degree(node)

    # just a node
    if ctype=='node':
        if deg>=3:
            elements[node]=d.add(elm.Dot(radius=0.12).at((x,y)))
            d.add(elm.Label().at((x,y)).label(node, ofst=0.2))
            pins[node]={'pin':(x,y)}
        else:
            d.add(elm.Label().at((x,y)).label(node, ofst=0.2))
            pins[node]={'pin':(x,y)}
        return

    if ctype=='ground':
        ctype_for_bbox='GND'
    else:
        ctype_for_bbox=ctype

    scale_info=scaling_ratios.get(node,default_scaling_ratios.get(ctype_for_bbox,{'vertical_scale':1.0,'horizontal_scale':1.0}))
    vs=scale_info.get('vertical_scale',1.0)
    hs=scale_info.get('horizontal_scale',1.0)

    try:
        bbox, anchors_loc, anchors_glob = get_component_bbox(ctype_for_bbox,
            pos=(x,y), direction=direction,
            shrink_size=shrink_size, flip=flip,
            vertical_scale=vs, horizontal_scale=hs,
            **node_info.get('parameters',{})
        )
    except ValueError as e:
        print(f"[ERROR] {e}")
        return

    lbl = f"{node}\n{node_info.get('value','')}"
    # instantiate element
    if ctype_for_bbox=='M':
        model=node_info.get('model','').upper()
        if model.startswith('P'):
            try:
                element=elm.PFet(bulk=True,scale=1.0).at((x,y))
            except:
                print("[ERROR] no PFet in schemdraw.")
                return
        else:
            try:
                element=elm.NFet(bulk=True,scale=1.0).at((x,y))
            except:
                print("[ERROR] no NFet in schemdraw.")
                return
    else:
        comp_class=component_map.get(ctype_for_bbox,elm.Dot(radius=0.12))
        if comp_class is None:
            print(f"[ERROR] unknown ctype: {ctype_for_bbox}")
            return
        element=comp_class(**node_info.get('parameters',{})).at((x,y))

    set_element_direction(element,direction)

    # flip
    if direction in ['up','down']:
        if flip=='horizontal':
            element.flip()
        elif flip=='vertical':
            element.reverse()
    else:
        if flip=='horizontal':
            element.reverse()
        elif flip=='vertical':
            element.flip()

    element.label(lbl)
    element.scalex(hs).scaley(vs)
    element=d.add(element)

    # assign pins
    if ctype_for_bbox in ['V','I','E','H','F','G']:
        pins[node]={'positive':(element.absanchors['end'].x,element.absanchors['end'].y),
                    'negative':(element.absanchors['start'].x,element.absanchors['start'].y)}
    elif ctype_for_bbox in ['R','C','L','S']:
        pins[node]={'start':(element.absanchors['start'].x,element.absanchors['start'].y),
                    'end':(element.absanchors['end'].x,element.absanchors['end'].y)}
    elif ctype_for_bbox=='D':
        pins[node]={'anode':(element.absanchors['start'].x,element.absanchors['start'].y),
                    'cathode':(element.absanchors['end'].x,element.absanchors['end'].y)}
    elif ctype_for_bbox=='J':
        pins[node]={'drain':(element.absanchors['drain'].x,element.absanchors['drain'].y),
                    'gate': (element.absanchors['gate'].x,element.absanchors['gate'].y),
                    'source':(element.absanchors['source'].x,element.absanchors['source'].y)}
    elif ctype_for_bbox=='Q':
        pins[node]={'collector':(element.absanchors['collector'].x,element.absanchors['collector'].y),
                    'base':(element.absanchors['base'].x,element.absanchors['base'].y),
                    'emitter':(element.absanchors['emitter'].x,element.absanchors['emitter'].y)}
    elif ctype_for_bbox=='M':
        pins[node]={'drain':(element.absanchors['drain'].x,element.absanchors['drain'].y),
                    'gate': (element.absanchors['gate'].x,element.absanchors['gate'].y),
                    'source':(element.absanchors['source'].x,element.absanchors['source'].y),
                    'bulk':(element.absanchors['bulk'].x,element.absanchors['bulk'].y)}
    elif ctype_for_bbox=='ground':
        pins[node]={'pin':(x,y)}
    elif ctype_for_bbox=='node':
        pins[node]={'pin':(x,y)}
    else:
        pins[node]={'pin':(x,y)}

    x_min,y_min,x_max,y_max=bbox['xmin'],bbox['ymin'],bbox['xmax'],bbox['ymax']
    comp_boxes[node]={
        'bbox':bbox,
        'polygon':Polygon([(x_min,y_min),(x_max,y_min),(x_max,y_max),(x_min,y_max)]),
        'type':ctype_for_bbox
    }


def draw_connections(d, G, comps_elem, pins, comp_boxes, drawn_edges, routing_method,
                     all_wires, pin_mapping, pos, grid_size=1.0):
    print("[DEBUG] draw_connections() start")
    for comp,neighbor,key in G.edges(keys=True):
        edge_nodes=tuple(sorted([comp,neighbor]))
        edge_key=edge_nodes+(key,)
        if edge_key in drawn_edges:
            continue

        comp_info=G.nodes[comp]
        neigh_info=G.nodes[neighbor]
        cpin=get_component_pin(comp,neighbor,comp_info,comps_elem,pins,pin_mapping)
        npin=get_component_pin(neighbor,comp,neigh_info,comps_elem,pins,pin_mapping)
        if cpin is None or npin is None:
            print(f"[DEBUG] skip edge {edge_key}, cpin or npin = None.")
            continue
        start_pos=get_pin_position(pins,comp,cpin,grid_size=grid_size)
        end_pos  =get_pin_position(pins,neighbor,npin,grid_size=grid_size)
        print(f"[DEBUG] connecting {comp}.{cpin} => {neighbor}.{npin}, {start_pos} => {end_pos}")
        if start_pos is None or end_pos is None:
            continue

        route_path,clr=route_connection_current_method(start_pos,end_pos,comp_boxes,comp,neighbor,all_wires,grid_size=grid_size)
        for i in range(len(route_path)-1):
            seg_start=route_path[i]
            seg_end=route_path[i+1]
            new_line=LineString([seg_start,seg_end])
            all_wires.append({'line':new_line,'color':clr})
            print(f"[DEBUG] add wire seg => {seg_start}->{seg_end}, color={clr}")

        drawn_edges.add(edge_key)
    print("[DEBUG] draw_connections() end")


def draw_circuit(G, comps_elem, xml_root, xml_file, initial_comp_node_mapping,
                 max_attempts=100,EnlargeSize=2.5,routing_method=1,auto=1,grid_size=1.0):
    print("[DEBUG] draw_circuit() start, auto=",auto)
    attempt=0
    success=False
    last_pos=None
    last_wires=None

    positions_in_xml={}
    directions_in_xml={}
    flips_in_xml={}
    scaling_ratios={}

    # read existing info
    for comp in comps_elem:
        cid=comp.attrib['id']
        dir_val=comp.attrib.get('direction','right')
        flip_val=comp.attrib.get('flip','none')
        vs=float(comp.attrib.get('vertical_scale','1.0'))
        hs=float(comp.attrib.get('horizontal_scale','1.0'))
        scaling_ratios[cid]={'vertical_scale':vs,'horizontal_scale':hs}
        directions_in_xml[cid]=dir_val
        flips_in_xml[cid]=flip_val
        if 'x' in comp.attrib and 'y' in comp.attrib:
            positions_in_xml[cid]=(float(comp.attrib['x']),float(comp.attrib['y']))

    nodes_elem=xml_root.find('nodes')
    if nodes_elem is not None:
        for n in nodes_elem:
            nid=n.attrib['id']
            if 'x' in n.attrib and 'y' in n.attrib:
                positions_in_xml[nid]=(float(n.attrib['x']),float(n.attrib['y']))

    if auto==1:
        while attempt<max_attempts:
            attempt+=1
            print(f"[DEBUG] attempt {attempt}")
            original_pos=nx.spring_layout(G)
            pos={ n:(EnlargeSize*original_pos[n][0],
                      EnlargeSize*(-original_pos[n][1]))
                  for n in G.nodes}

            pos={ n: align_to_grid(pos[n],grid_size) for n in pos}

            directions={}
            flips={}
            scales={}
            for n in G.nodes:
                directions[n]=directions_in_xml.get(n,'right')
                flips[n]=flips_in_xml.get(n,'none')
                sc=scaling_ratios.get(n, default_scaling_ratios.get(G.nodes[n]['type'],{'vertical_scale':1.0,'horizontal_scale':1.0}))
                scales[n]=sc

            d=schemdraw.Drawing()
            if draw_grid_or_not==1:
                draw_grid(d, grid_size=grid_size)

            elements={}
            pins={}
            comp_boxes={}
            drawn_edges=set()
            all_wires=[]

            # draw
            for node in G.nodes:
                node_info=G.nodes[node]
                x,y=pos[node]
                draw_component_or_node(d,elements,pins,node,node_info,x,y,
                                       directions.get(node,'right'),
                                       comp_boxes,G,
                                       flip=flips.get(node,'none'),
                                       scaling_ratios=scales)

            # 先做无向器件引脚自动调换
            for comp in G.nodes:
                cty=G.nodes[comp].get('type','')
                if cty in ['R','C','L','S']:
                    c_pins=pins.get(comp,{})
                    if 'start' not in c_pins or 'end' not in c_pins:
                        continue
                    neighs=list(G[comp])
                    if len(neighs)==2:
                        n1,n2=neighs[0],neighs[1]
                        if n1 in pins and n2 in pins:
                            k1=list(pins[n1].keys())[0]
                            k2=list(pins[n2].keys())[0]
                            n1pos=np.array(pins[n1][k1])
                            n2pos=np.array(pins[n2][k2])
                            st=np.array(c_pins['start'])
                            ed=np.array(c_pins['end'])
                            d1=np.linalg.norm(st-n1pos)+np.linalg.norm(ed-n2pos)
                            d2=np.linalg.norm(st-n2pos)+np.linalg.norm(ed-n1pos)
                            if d2<d1:
                                pins[comp]['start'],pins[comp]['end']=pins[comp]['end'],pins[comp]['start']

            pin_mapping_copy=copy.deepcopy(initial_comp_node_mapping)
            draw_connections(d,G,comps_elem,pins,comp_boxes,drawn_edges,
                             routing_method,all_wires,pin_mapping_copy,pos,grid_size=grid_size)

            any_red = any(w['color']==wire_danger_color for w in all_wires)
            overlapping= check_overlapping_wires(all_wires)

            last_pos=pos
            last_wires=all_wires

            if any_red or overlapping:
                print("[DEBUG] detected red or overlapping, retry.")
                continue
            else:
                # draw wires
                for w in all_wires:
                    line=w['line']
                    color=w['color']
                    d.add(elm.Line().at(line.coords[0]).to(line.coords[1]).color(color))
                add_anchor_dots(d,all_wires)

                update_xml_positions_and_directions(xml_root,pos,directions,flips,scales)
                ET.ElementTree(xml_root).write(xml_file)
                d.draw()
                success=True
                n_inter = count_line_intersections(all_wires)
                print("[DEBUG] layout success, line intersections=", n_inter)
                break

        if not success:
            print("[DEBUG] all attempts fail, show last")
            d=schemdraw.Drawing()
            if last_pos is not None and last_wires is not None:
                update_xml_positions_and_directions(xml_root,last_pos,directions_in_xml,flips_in_xml,scaling_ratios)
                ET.ElementTree(xml_root).write(xml_file)
                elements={}
                pins={}
                comp_boxes={}
                drawn_edges=set()
                for node in G.nodes:
                    node_info=G.nodes[node]
                    if node not in last_pos:
                        continue
                    x,y=last_pos[node]
                    draw_component_or_node(d,elements,pins,node,node_info,x,y,
                                           directions_in_xml.get(node,'right'),comp_boxes,G,
                                           flip=flips_in_xml.get(node,'none'),scaling_ratios=scaling_ratios)
                for w in last_wires:
                    d.add(elm.Line().at(w['line'].coords[0]).to(w['line'].coords[1]).color(w['color']))
                add_anchor_dots(d,last_wires)
                d.draw()
    else:
        # fixed layout
        print("[DEBUG] fixed layout")
        d=schemdraw.Drawing()
        if draw_grid_or_not==1:
            draw_grid(d,grid_size=grid_size)
        elements={}
        pins={}
        comp_boxes={}
        drawn_edges=set()
        all_wires=[]

        pos=positions_in_xml
        directions=directions_in_xml
        flips=flips_in_xml

        for node in G.nodes:
            node_info=G.nodes[node]
            if node not in pos:
                print("[WARNING] node no pos =>",node)
                continue
            x,y=pos[node]
            draw_component_or_node(d,elements,pins,node,node_info,x,y,
                                   directions.get(node,'right'),comp_boxes,G,
                                   flip=flips.get(node,'none'),scaling_ratios=scaling_ratios)
        # same 'R','C','L','S' logic
        for comp in G.nodes:
            cty=G.nodes[comp].get('type','')
            if cty in ['R','C','L','S']:
                c_pins=pins.get(comp,{})
                if 'start' not in c_pins or 'end' not in c_pins:
                    continue
                neighs=list(G[comp])
                if len(neighs)==2:
                    n1,n2=neighs
                    if n1 in pins and n2 in pins:
                        k1=list(pins[n1].keys())[0]
                        k2=list(pins[n2].keys())[0]
                        n1pos=np.array(pins[n1][k1])
                        n2pos=np.array(pins[n2][k2])
                        st=np.array(c_pins['start'])
                        ed=np.array(c_pins['end'])
                        d1=np.linalg.norm(st-n1pos)+np.linalg.norm(ed-n2pos)
                        d2=np.linalg.norm(st-n2pos)+np.linalg.norm(ed-n1pos)
                        if d2<d1:
                            pins[comp]['start'],pins[comp]['end']=pins[comp]['end'],pins[comp]['start']

        pin_mapping_copy=copy.deepcopy(initial_comp_node_mapping)
        draw_connections(d,G,comps_elem,pins,comp_boxes,drawn_edges,
                         routing_method,all_wires,pin_mapping_copy,pos,grid_size=grid_size)
        any_red= any(w['color']==wire_danger_color for w in all_wires)
        overlapping= check_overlapping_wires(all_wires)
        if any_red or overlapping:
            print("[WARNING] red or overlapping wire found.")
        for w in all_wires:
            d.add(elm.Line().at(w['line'].coords[0]).to(w['line'].coords[1]).color(w['color']))
        add_anchor_dots(d,all_wires)

        update_xml_positions_and_directions(xml_root,pos,directions,flips,scaling_ratios)
        ET.ElementTree(xml_root).write(xml_file)
        d.draw()
        n_inter=count_line_intersections(all_wires)
        print("[DEBUG] intersection number:",n_inter)

    print("[DEBUG] draw_circuit() end")


def main():
    print("[DEBUG] Script started.")
    outxml="spice_netlist.xml"

    if auto==1:
        print("[DEBUG] converting netlist->xml =>", outxml)
        convert_netlist_to_xml_file(netlist,outxml)
    else:
        print("[DEBUG] skip netlist->xml, using existing xml:", outxml)

    try:
        tree=ET.parse(outxml)
    except FileNotFoundError:
        print("[ERROR]", outxml,"not found.")
        sys.exit(1)
    except ET.ParseError as e:
        print("[ERROR] parse error =>",e)
        sys.exit(1)

    xml_root=tree.getroot()
    G, comps_elem = create_graph_from_xml(xml_root)

    init_mapping=build_comp_node_mapping(comps_elem)

    draw_circuit(G, comps_elem, xml_root, outxml, init_mapping,
                 max_attempts=max_attempts,EnlargeSize=EnlargeSize,
                 routing_method=routing_method,auto=auto,grid_size=grid_size)

    print("[DEBUG] Script finished.")


if __name__=="__main__":
    main()
