from shapely.geometry import LineString

# 假设 `all_wires` 是一个包含多根导线的列表，每根导线是一个字典，字典中包含 'line' 键
all_wires = [
    {'line': LineString([(0, 0), (1, 1)])},  # 示例1
    {'line': LineString([(2, 2), (3, 3)])},  # 示例2
    # 更多的导线...
]

# 最小导线间距
min_wire_spacing = 0.5

# 创建导线缓冲区
existing_buffers = [wire['line'].buffer(min_wire_spacing / 2, cap_style=2, join_style=2) for wire in all_wires]

# 输出缓冲区结果
for buffer in existing_buffers:
    print(buffer)
