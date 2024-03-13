"""
Free Space (Parking Lot) A Star Route Planner

Reference:
PythonRobotics A* grid planning (author: Atsushi Sakai(@Atsushi_twi) / Nikos Kanargias (nkana@tee.gr))
https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/AStar/a_star.py
"""

import math

import matplotlib.pyplot as plt
from parking_lot import ParkingLot


# TODO: (1) Implement Node Class
class Node:
    def __init__(self):
        pass


class AStarRoutePlanner:
    def __init__(self, parking_lot):
        self.parking_lot: ParkingLot = parking_lot

        # Motion Model: dx, dy, cost
        # TODO: (2) Implement Motions
        self.motions = []

        self.goal_node: Node = None

    def search_route(self, start_point, goal_point, show_process=True):
        # TODO: (3) Implement Search Route

        print("Cannot find Route")
        return [], []

    def calculate_heuristic_cost(self, node):
        # TODO: (4) Implement Heuristic Cost
        cost = 0.0
        return cost

    def process_route(self, closed_set):
        # TODO: (5) Implement Process Route
        rx = []
        ry = []
        return rx, ry

    @staticmethod
    def plot_process(current_node, closed_set):
        # show graph
        plt.plot(current_node.x, current_node.y, "xc")
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            "key_release_event",
            lambda event: [exit(0) if event.key == "escape" else None],
        )
        if len(closed_set.keys()) % 10 == 0:
            plt.pause(0.001)


def main():
    parking_lot = ParkingLot()
    obstacle_x = [obstacle[0] for obstacle in parking_lot.obstacles]
    obstacle_y = [obstacle[1] for obstacle in parking_lot.obstacles]
    plt.plot(obstacle_x, obstacle_y, ".k")

    # start and goal point
    start_point = [75, 54]
    goal_point = [5, 54]
    print(f"Start A Star Route Planner (start {start_point}, end {goal_point})")

    plt.plot(start_point[0], start_point[1], "og")
    plt.plot(goal_point[0], goal_point[1], "xb")
    plt.title("A Star Route Planner")
    plt.grid(True)
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.axis("equal")

    a_star = AStarRoutePlanner(parking_lot)
    rx, ry = a_star.search_route(start_point, goal_point)

    plt.plot(rx, ry, "-r")
    plt.pause(0.001)
    plt.show()


if __name__ == "__main__":
    main()
