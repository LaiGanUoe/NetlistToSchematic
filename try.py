from shapely.geometry import LineString, Polygon, Point
import matplotlib.pyplot as plt


def find_turn_points_shapely(start, goal, obstacle):
    # 定义路径和障碍物
    path = LineString([start, goal])
    obstacle_polygon = Polygon([
        (obstacle[0], obstacle[1]),  # 左下角
        (obstacle[0], obstacle[3]),  # 左上角
        (obstacle[2], obstacle[3]),  # 右上角
        (obstacle[2], obstacle[1]),  # 右下角
    ])

    # 检测路径是否与障碍物相交
    if path.intersects(obstacle_polygon):
        # 获取障碍物的边界
        x_min, y_min, x_max, y_max = obstacle

        # 根据起点位置动态生成拐点
        turn_points = []
        if start[0] <= x_min:  # 起点在障碍物左侧
            turn_points.append((x_min - 1, start[1]))
            turn_points.append((x_min - 1, goal[1]))
        elif start[0] >= x_max:  # 起点在障碍物右侧
            turn_points.append((x_max + 1, start[1]))
            turn_points.append((x_max + 1, goal[1]))
        elif start[1] <= y_min:  # 起点在障碍物下方
            turn_points.append((start[0], y_min - 1))
            turn_points.append((goal[0], y_min - 1))
        elif start[1] >= y_max:  # 起点在障碍物上方
            turn_points.append((start[0], y_max + 1))
            turn_points.append((goal[0], y_max + 1))

        return turn_points
    else:
        # 无需绕障碍物
        return []


def plot_path_with_shapely(start, goal, obstacle, turn_points):
    plt.figure(figsize=(6, 6))
    # 绘制起点和终点
    plt.plot(*start, 'go', label="Start")
    plt.plot(*goal, 'ro', label="Goal")

    # 绘制障碍物
    x_min, y_min, x_max, y_max = obstacle
    plt.plot([x_min, x_max, x_max, x_min, x_min],
             [y_min, y_min, y_max, y_max, y_min], 'k-', label="Obstacle")

    # 绘制路径
    if turn_points:
        path_points = [start] + turn_points + [goal]
        x_coords, y_coords = zip(*path_points)
        plt.plot(x_coords, y_coords, 'b--', label="Path")
    else:
        plt.plot([start[0], goal[0]], [start[1], goal[1]], 'b--', label="Path")

    plt.legend()
    plt.grid()
    plt.show()


# 示例调用
start = (2, 2)
goal = (8, 8)
obstacle = (4, 4, 6, 6)
turn_points = find_turn_points_shapely(start, goal, obstacle)
plot_path_with_shapely(start, goal, obstacle, turn_points)
