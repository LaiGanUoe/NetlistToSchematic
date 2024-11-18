from shapely.geometry import LineString, Polygon, Point

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