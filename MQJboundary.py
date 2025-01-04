import schemdraw
import schemdraw.elements as elm

# with schemdraw.Drawing() as d:
#     # adding a jfet component
#     element =elm.JFet(bulk=True).at((1, 2)).right()
#     element.scalex(12/11)
#     element.scaley(6/5)
#     element = d.add(element)  # JFet，方向为右
#
#     # Get global position of anchors for absolute positioning
#     absSource = element.absanchors['drain']
#
#     # Get local position of anchors for relative positioning and wait for the user to convert to the global position
#     source = element.anchors['drain']
#     print("source:", source)
#     print("absSource:", absSource)
#     # Calculate the difference between the global and local positions
#     xDiff = absSource.x - source[0]
#     yDiff = absSource.y - source[1]
#
#     # Calculate the global position of the bounding box---for only right direction
#     ymin_global = yDiff + element.get_bbox().ymin
#     ymax_global = yDiff + element.get_bbox().ymax
#     xmin_global = xDiff + element.get_bbox().xmin
#     xmax_global = xDiff + element.get_bbox().xmax
#     print(f"\nBBox: xmin={element.get_bbox().ymin}, ymin={element.get_bbox().ymin}, xmax={element.get_bbox().xmax}, ymax={element.get_bbox().ymax}")
#
#     print('abs_ref:' +str(element.absanchors['drain']))
#     print('abs_ref:' +str(element.absanchors['gate']))
#     print('abs_ref:' +str(element.absanchors['source']))
#
#     # print('abs_ref:' +str(element.absanchors['source']))

with schemdraw.Drawing() as d:
    # adding a jfet component
    element = elm.PFet(bulk=True).at((1, 2)).up()
    element.scalex(90/82)
    element.scaley(6/5)
    element = d.add(element)  # JFet，方向为右

    # Get global position of anchors for absolute positioning
    absSource = element.absanchors['drain']

    # Get local position of anchors for relative positioning and wait for the user to convert to the global position
    source = element.anchors['drain']
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
    print(
        f"\nBBox: xmin={element.get_bbox().ymin}, ymin={element.get_bbox().ymin}, xmax={element.get_bbox().xmax}, ymax={element.get_bbox().ymax}")

    print('abs_ref:' + str(element.absanchors['drain']))
    print('abs_ref:' + str(element.absanchors['gate']))
    print('abs_ref:' + str(element.absanchors['source']))
    print('abs_ref:' +str(element.absanchors['bulk']))


# with schemdraw.Drawing() as d:
#     # adding a jfet component
#     element = elm.Bjt(bulk=True).at((1, 2)).right()
#     element.scalex(600/451)
#     element.scaley(300/209)
#     element = d.add(element)  # JFet，方向为右
#
#     # Get global position of anchors for absolute positioning
#     absSource = element.absanchors['base']
#
#     # Get local position of anchors for relative positioning and wait for the user to convert to the global position
#     source = element.anchors['base']
#     print("source:", source)
#     print("absSource:", absSource)
#     # Calculate the difference between the global and local positions
#     xDiff = absSource.x - source[0]
#     yDiff = absSource.y - source[1]
#
#     # Calculate the global position of the bounding box---for only right direction
#     ymin_global = yDiff + element.get_bbox().ymin
#     ymax_global = yDiff + element.get_bbox().ymax
#     xmin_global = xDiff + element.get_bbox().xmin
#     xmax_global = xDiff + element.get_bbox().xmax
#     print(
#         f"\nBBox: xmin={element.get_bbox().ymin}, ymin={element.get_bbox().ymin}, xmax={element.get_bbox().xmax}, ymax={element.get_bbox().ymax}")
#
#     print('abs_ref:' + str(element.absanchors['base']))
#     print('abs_ref:' + str(element.absanchors['collector']))
#     print('abs_ref:' + str(element.absanchors['emitter']))
#
