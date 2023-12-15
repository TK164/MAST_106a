"""

A* grid planning

author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)

See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)

"""

import math
import numpy as np
from ccma import CCMA

import matplotlib.pyplot as plt

show_animation = True


class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr):
        """
        Initialize grid map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,
                                                                     open_set[
                                                                         o]))
            current = open_set[c_id]

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(
                                                 0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        """
        calc grid position

        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        if min(ox) > 0:
            self.min_x = math.ceil(min(ox))
        else:
            self.min_x = math.floor(min(ox))
        if min(oy) > 0:
            self.min_y = math.ceil(min(oy))
        else:
            self.min_y = math.floor(min(oy))
        if max(ox) > 0:
            self.max_x = math.ceil(max(ox))
        else:
            self.max_x = math.floor(max(ox))
        if max(oy) > 0:
            self.max_y = math.ceil(max(oy))
        else:
            self.max_y = math.floor(max(oy))

        # self.min_x = round(min(ox))
        # self.min_y = round(min(oy))
        # self.max_x = round(max(ox))
        # self.max_y = round(max(oy))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion

def plan_trajectory(robot_x, robot_y, cup_x, cup_y, goal_x, goal_y, smooth):

    grid_size = 0.05  # [m]
    CUP_OFFSET = int(0.28 / grid_size) # 11 in = 0.28 m
    GOAL_OFFSET = int(0.23 / grid_size) # 9 in = 0.23 m
    # robot_radius = 0.17  # [m]
    robot_radius = 0.1

    ws = 0.1 # wall spacing
    # outer walls
    ox, oy = [], []
    for i in np.arange(-ws, 2.44 + ws + ws, ws):
        ox.append(i)
        oy.append(-ws)
    for i in np.arange(-ws, 2.44 + ws + ws, ws):
        ox.append(i)
        oy.append(1.83 + ws)
    for i in np.arange(0, 1.83 + ws, ws):
        ox.append(2.44 + ws)
        oy.append(i)
    for i in np.arange(0, 1.83 + ws, ws):
        ox.append(-ws)
        oy.append(i)
    # bottom inside walls
    for i in np.arange(0, 0.61, ws):
        ox.append(1.22)
        oy.append(i)
    # top inside walls
    for i in np.arange(1.22 + ws, 1.83, ws):
        ox.append(1.22)
        oy.append(i)

    # PATH FROM CUP TO GOAL
    # start and goal position
    sx = cup_x  # [m]
    sy = cup_y  # [m]
    gx = goal_x  # [m]
    gy = goal_y  # [m]

    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
    rx2, ry2 = a_star.planning(sx, sy, gx, gy)

    # a star gives reversed coords so we flip arrays
    rx2 = rx2[::-1]
    ry2 = ry2[::-1]

    # Incorporate offset into waypoints from cup to goal
    tempx = rx2
    tempy = ry2
    for i in range(CUP_OFFSET):
        rx2 = [tempx[0] + tempx[0] - tempx[1 + i]] + rx2
        ry2 = [tempy[0] + tempy[0] - tempy[1 + i]] + ry2

    # Smooth path out
    if smooth:
        ccma = CCMA(w_ma=10, w_cc=3)
        points_smoothed2 = ccma.filter(np.array(list(zip(rx2, ry2))))
        rx2 = [point[0] for point in points_smoothed2]
        ry2 = [point[1] for point in points_smoothed2]

    # Cut path short to account for robot-to-goal distance
    rx2 = rx2[:-GOAL_OFFSET]
    ry2 = ry2[:-GOAL_OFFSET]
    
    theta2 = []
    for i in range(len(rx2)-1):
        # angle is based off this waypoint to next waypoint
        theta2.append(math.atan2(ry2[i+1]-ry2[i], rx2[i+1]-rx2[i]))
    # final point angle is the angle from final point to goal
    theta2.append(math.atan2(goal_y-ry2[-1], goal_x-rx2[-1]))

    waypoints_to_goal = list(zip(rx2, ry2, theta2))

    # PATH FROM ROBOT TO CUP
    # start and goal position
    sx = robot_x  # [m]
    sy = robot_y  # [m]
    gx = rx2[0]  # [m]
    gy = ry2[0]  # [m]

    # add cup as obstacle to make sure we don't run into it
    ox.append(cup_x)
    oy.append(cup_y)

    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
    rx1, ry1 = a_star.planning(sx, sy, gx, gy)

    # a star gives reversed coords so we flip arrays
    rx1 = rx1[::-1]
    ry1 = ry1[::-1]

    # Smooth path out
    if smooth:
        ccma = CCMA(w_ma=10, w_cc=3)
        points_smoothed1 = ccma.filter(np.array(list(zip(rx1, ry1))))
        rx1 = [point[0] for point in points_smoothed1] + [rx1[-1]]
        ry1 = [point[1] for point in points_smoothed1] + [ry1[-1]]

    theta1 = []
    for i in range(len(rx1)-1):
        # angle is based off this waypoint to next waypoint
        theta1.append(math.atan2(ry1[i+1]-ry1[i], rx1[i+1]-rx1[i]))
    # final point angle is based off prev waypoint to this waypoint
    theta1.append(theta1[-1])

    waypoints_to_cup = list(zip(rx1, ry1, theta1))

    # for waypoint in waypoints_to_cup:
    #     print(waypoint)
    # for waypoint in waypoints_to_goal:
    #     print(waypoint)

    return waypoints_to_cup, waypoints_to_goal



def plan_home_trajectory(robot_x, robot_y, smooth):

    grid_size = 0.05  # [m]
    robot_radius = 0.17  # [m]
    
    ws = 0.1 # wall spacing
    # outer walls
    ox, oy = [], []
    for i in np.arange(-2, 2, ws):
        ox.append(i)
        oy.append(-2)
    for i in np.arange(-2, 2, ws):
        ox.append(i)
        oy.append(2)
    for i in np.arange(-2, 2, ws):
        ox.append(-2)
        oy.append(i)
    for i in np.arange(-2, 2, ws):
        ox.append(2)
        oy.append(i)
    # bottom inside walls

    # PATH FROM CUP TO GOAL
    # start and goal position
    sx = robot_x  # [m]
    sy = robot_y  # [m]
    gx = 0  # [m]
    gy = 0  # [m]

    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
    rx2, ry2 = a_star.planning(sx, sy, gx, gy)
    rx2 = rx2[::-1]
    ry2 = ry2[::-1]

    # Smooth path out
    if smooth:
        ccma = CCMA(w_ma=10, w_cc=3)
        points_smoothed2 = ccma.filter(np.array(list(zip(rx2, ry2))))
        rx2 = [point[0] for point in points_smoothed2]
        ry2 = [point[1] for point in points_smoothed2]
    
    theta2 = []
    for i in range(len(rx2)-1):
        # angle is based off this waypoint to next waypoint
        theta2.append(math.atan2(ry2[i+1]-ry2[i], rx2[i+1]-rx2[i]))
    # final point angle is the angle from second to last point to last point
    theta2.append(theta2[-1])

    waypoints_to_home = list(zip(rx2, ry2, theta2))
    print(waypoints_to_home)

    return waypoints_to_home


def goal_trajectory(robot_x, robot_y, goal_x, goal_y, smooth, mymap):

    grid_size = 0.1  # [m]
    CUP_OFFSET = int(0.28 / grid_size) # 11 in = 0.28 m
    GOAL_OFFSET = int(0.23 / grid_size) # 9 in = 0.23 m
    # robot_radius = 0.17  # [m]
    robot_radius = 0.175
    
    # outer wall
    
    oy = [mymap[i][0] for i in range(len(mymap))]
    ox = [mymap[i][1] for i in range(len(mymap))]
    
    
    # PATH FROM CUP TO GOAL
    # start and goal position
    sx = robot_x  # [m]
    sy = robot_y  # [m]
    gx = goal_x  # [m]
    gy = goal_y  # [m]

    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
    rx2 = []
    ry2 = []
    while len(rx2) < 1 and len(ry2) < 1:
        rx2, ry2 = a_star.planning(sx, sy, gx, gy)
        if len(rx2) < 1:
            gx = (gx-sx)*.95 + sx
            gy = (gy-sy)*.95 + sy

    # rx2, ry2 = a_star.planning(sx, sy, gx, gy)


    # a star gives reversed coords tso we flip arrays
    rx2 = rx2[::-1]
    ry2 = ry2[::-1]

    # Incorporate offset into waypoints from cup to goal
    #tempx = rx2
    #tempy = ry2
    #for i in range(CUP_OFFSET):
        #rx2 = [tempx[0] + tempx[0] - tempx[1 + i]] + rx2
        #ry2 = [tempy[0] + tempy[0] - tempy[1 + i]] + ry2

    # Smooth path out
    if smooth and len(rx2) >= 3:
        ccma = CCMA(w_ma=10, w_cc=3)
        points_smoothed2 = ccma.filter(np.array(list(zip(rx2, ry2))))
        rx2 = [point[0] for point in points_smoothed2]
        ry2 = [point[1] for point in points_smoothed2]

    # Cut path short to account for robot-to-goal distance
    #rx2 = rx2[:-GOAL_OFFSET]
    #ry2 = ry2[:-GOAL_OFFSET]
    
    theta2 = []
    for i in range(len(rx2)-1):
        # angle is based off this waypoint to next waypoint
        theta2.append(math.atan2(ry2[i+1]-ry2[i], rx2[i+1]-rx2[i]))
    # final point angle is the angle from final point to goal
    theta2.append(math.atan2(goal_y-ry2[-1], goal_x-rx2[-1]))

    waypoints_to_goal = list(zip(rx2, ry2, theta2))

    

    return waypoints_to_goal

def main():
    print(__file__ + " start!!")
    

    ROBOT_X = -0.5
    ROBOT_Y = 0.5
    CUP_X = 0.5
    CUP_Y = 0.5
    GOAL_X = .5 #2.44 - 0.25
    GOAL_Y = -.5 #0.0 + 0.25

    #waypoints_to_cup, waypoints_to_goal = plan_trajectory(ROBOT_X, ROBOT_Y, CUP_X, CUP_Y, GOAL_X, GOAL_Y, True)
    #rx_s = [point[0] for point in waypoints_to_cup] + [point[0] for point in waypoints_to_goal]
    #ry_s = [point[1] for point in waypoints_to_cup] + [point[1] for point in waypoints_to_goal]
    
    waypoints_to_cup, waypoints_to_goal = test_plan_trajectory(ROBOT_X, ROBOT_Y, CUP_X, CUP_Y, GOAL_X, GOAL_Y, True)
    rx_s = [point[0] for point in waypoints_to_cup] + [point[0] for point in waypoints_to_goal]
    ry_s = [point[1] for point in waypoints_to_cup] + [point[1] for point in waypoints_to_goal]

    if show_animation:  # pragma: no cover
        plt.plot(rx_s, ry_s, ".r")
        # plt.plot(rx, ry, ".b")
        plt.pause(0.001)
        plt.show()

    # waypoints_to_home = plan_home_trajectory(-1, -1, 0)
    # rx_s = [point[0] for point in waypoints_to_home]
    # ry_s = [point[1] for point in waypoints_to_home]
    
    # if show_animation:  # pragma: no cover
    #     plt.plot(rx_s, ry_s, ".r")
    #     # plt.plot(rx, ry, ".b")
    #     plt.pause(0.001)
    #     plt.show()


if __name__ == '__main__':
    main()
