from shapely.geometry import Polygon

# 创建两个多边形
polygon1 = Polygon([(0, 0), (1, 0), (1, 1), (0, 1)])
polygon2 = Polygon([(2, 2), (3, 2), (3, 3), (2, 3)])

# 计算最小距离
min_clearance = polygon1.distance(polygon2)

print(min_clearance)