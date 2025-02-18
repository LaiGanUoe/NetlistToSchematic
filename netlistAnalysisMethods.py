# from shapely.geometry import LineString
#
# # 假设 `all_wires` 是一个包含多根导线的列表，每根导线是一个字典，字典中包含 'line' 键
# all_wires = [
#     {'line': LineString([(0, 0), (1, 1)])},  # 示例1
#     {'line': LineString([(2, 2), (3, 3)])},  # 示例2
#     # 更多的导线...
# ]
#
# # 最小导线间距
# min_wire_spacing = 0.5
#
# # 创建导线缓冲区
# existing_buffers = [wire['line'].buffer(min_wire_spacing / 2, cap_style=2, join_style=2) for wire in all_wires]
#
# # 输出缓冲区结果
# for buffer in existing_buffers:
#     print(buffer)
# from shapely.geometry import LineString, Polygon, Point
# # Define geometric objects
# wire = LineString([(0, 0), (2, 2)])
# component = Polygon([(1, 1), (3, 1), (3, 3), (1, 3)])
# connection_point = Point(1.5, 1.5)
# # Collision detection
# if wire.intersects(component):
#     overlap = wire.intersection(component)
#     print("Collision detected:", overlap)
# # Point containment
# if connection_point.within(component):
#     print("Connection point is valid.")

import matplotlib.pyplot as plt
from shapely.geometry import LineString

# 示例线段和颜色
all_wires = [
    {'line': LineString([(0, 0), (2, 2)]), 'color': 'green'},
    {'line': LineString([(1, 1), (3, 3)]), 'color': 'green'},
    {'line': LineString([(2, 2), (4, 4)]), 'color': 'green'},
    {'line': LineString([(0, 2), (2, 0)]), 'color': 'green'},
]

# 定义检测重叠的函数
def check_overlapping_wires(all_wires):
    overlapping = False
    num_wires = len(all_wires)
    for i in range(num_wires):
        line_i = all_wires[i]['line']
        for j in range(i + 1, num_wires):
            line_j = all_wires[j]['line']
            if line_i.equals(line_j) or line_i.overlaps(line_j):
                overlapping = True
                all_wires[i]['color'] = 'red'
                all_wires[j]['color'] = 'red'
    return overlapping

# 检测重叠
check_overlapping_wires(all_wires)

# 绘图
plt.figure(figsize=(8, 6))
for wire in all_wires:
    x, y = wire['line'].xy
    plt.plot(x, y, linestyle='--', color=wire['color'], linewidth=2, label=f"{wire['color']} line")

plt.title("Wire Overlaps Visualization")
plt.xlabel("X-coordinate")
plt.ylabel("Y-coordinate")
plt.axhline(0, color='black', linewidth=0.5, linestyle='-')
plt.axvline(0, color='black', linewidth=0.5, linestyle='-')
plt.grid(color='lightgray', linestyle='--', linewidth=0.5)
plt.legend(loc='upper left')
plt.show()
