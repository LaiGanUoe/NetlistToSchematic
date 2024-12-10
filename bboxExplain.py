import schemdraw
import schemdraw.elements as elm

# # ground component
# # Create a real drawing instance for ground component
# with schemdraw.Drawing() as d:
#     # set the shrink size for wire connection
#     shrink_size_oneNode = 0.1
#
#     print("schemdraw version:", schemdraw.__version__)
#     # adding a component
#     direction = 90
#     element = d.add(elm.Resistor(theta=direction).at((1, 2)))  # default direction is right
#
#     # Get global position of anchors for absolute positioning
#     absStart = element.absanchors['start']
#
#     # Get local position of anchors for relative positioning and wait for the user to convert to the global position
#     start = element.anchors['start']
#
#     # Calculate the difference between the global and local positions
#     xDiff = absStart.x - start[0]
#     yDiff = absStart.y - start[1]
#
#     # Calculate the global position of the bounding box---for only right direction
#     ymin_global = yDiff + element.get_bbox().ymin
#     # slightly shrink the ymax_global due to the ground symbol connection port is at the top side of the symbol
#     ymax_global = yDiff + element.get_bbox().ymax - shrink_size_oneNode
#     xmin_global = xDiff + element.get_bbox().xmin
#     xmax_global = xDiff + element.get_bbox().xmax
#
#     # print the global position of the bounding box
#     print(f"Global position BBox: xmin={xmin_global}, ymin={ymin_global}, xmax={xmax_global}, ymax={ymax_global}")
#
#
#     # 打印锚点信息（局部坐标）
#     print("Anchors (Local position):")
#     print(element.anchors)
#
#     # 打印全局锚点位置
#     print("\nAbsanchors (Global position):")
#     print(element.absanchors)
#
# #
# # Resistor has a default direction of right.        Resistor()
# # Capacitor has a default direction of right.       Capacitor()
# # Inductor has a default direction of right.        Inductor2(loops=3)
# # Voltage source has a default direction of right.  SourceV()
# # Current source has a default direction of right.  SourceI()
# # Diode has a default direction of right.           Diode()
# # Switch(action = "open") has a default direction of right. way of changing the action Switch(action = "open")
#
# # Create a real drawing instance for two terminal components
# with schemdraw.Drawing() as d:
#     # set the shrink size for wire connection
#     shrink_size_twoNode = 0.1
#
#     # adding a component
#     element = d.add(elm.Switch().at((1, 2)).right())  # default direction is right
#
#     # Get global position of anchors for absolute positioning
#     absStart = element.absanchors['start']
#
#     # Get local position of anchors for relative positioning and wait for the user to convert to the global position
#     start = element.anchors['start']
#
#     # Calculate the difference between the global and local positions
#     xDiff = absStart.x - start.x
#     yDiff = absStart.y - start.y
#
#     # Calculate the global position of the bounding box---for only right direction
#     ymin_global = yDiff + element.get_bbox().ymin
#     ymax_global = yDiff + element.get_bbox().ymax
#     xmin_global = xDiff + element.get_bbox().xmin + shrink_size_twoNode
#     xmax_global = xDiff + element.get_bbox().xmax - shrink_size_twoNode
#
#
#     # 查看 Resistor 的 anchors 和 absanchors
#     print("Anchors (Local position):")
#     print(element.anchors)  # 局部坐标系下的锚点位置
#
#     print("\nAbsanchors (Global position):")
#     print(element.absanchors)  # 全局坐标系下的锚点位置
#
#     print("\nBBox boundary (local position):")
#     print(element.get_bbox())  # 全局坐标系下的锚点位置
#
#     # 显示绘图
#     d.save('drawing.svg')
#
#     print(f"Global position BBox: xmin={xmin_global}, ymin={ymin_global}, xmax={xmax_global}, ymax={ymax_global}")
#
# jfet component
# Create a real drawing instance for three terminal components
with schemdraw.Drawing() as d:
    # adding a jfet component
    element = d.add(elm.SourceControlledV(circle = True).at((1, 2)).left())  # JFet，方向为右

    # # Resistor has a default direction of right.        Resistor()
    # # Capacitor has a default direction of right.       Capacitor()
    # # Inductor has a default direction of right.        Inductor2(loops=3)
    # # Voltage source has a default direction of right.  SourceV()
    # # Current source has a default direction of right.  SourceI()
    # # Diode has a default direction of right.           Diode()
    # # Switch(action = "open") has a default direction of right. way of changing the action Switch(action = "open")

    # # 打印锚点信息
    # print("Anchors (Local position):")
    # print(element.anchors)  # 局部坐标系下的锚点位置
    #
    # print("\nAbsanchors (Global position):")
    # print(element.absanchors)  # 全局坐标系下的锚点位置
    #
    # print("\nBBox boundary (local position):")
    # print(element.get_bbox())  # 局部边界框

    port = 'start'

    # Get global position of anchors for absolute positioning
    absSource = element.absanchors[port]

    # Get local position of anchors for relative positioning and wait for the user to convert to the global position
    source = element.anchors[port]
    print("source:", source)
    print("absSource:", absSource)
    # Calculate the difference between the global and local positions
    xDiff = absSource.x - source[0]
    yDiff = absSource.y - source[1]

    # Calculate the global position of the bounding box---for only right direction
    ymin_global = yDiff + element.get_bbox().ymin
    ymax_global = yDiff + element.get_bbox().ymax
    xmin_global = xDiff + element.get_bbox().xmin
    xmax_global = xDiff + element.get_bbox().xmax

    print(f"\nGlobal BBox: xmin={xmin_global}, ymin={ymin_global}, xmax={xmax_global}, ymax={ymax_global}")



