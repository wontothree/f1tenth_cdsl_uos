"""
Road Network (OpenDRIVE) A Star Route Planner

Reference:
PythonRobotics A* grid planning (author: Atsushi Sakai(@Atsushi_twi) / Nikos Kanargias (nkana@tee.gr))
https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/AStar/a_star.py
"""

import math
import os
from pathlib import Path

import imap.global_var as global_var
from imap.lib.convertor import Opendrive2Apollo
from imap.lib.opendrive.road import Road


class Node:
    def __init__(
            self, road: Road, is_same_direction: bool, cost: float, parent_node_index: str
    ):
        self.road: Road = road
        # True: 기존의 진행 방향과 동일 / False: 기존의 진행 방향과 반대
        self.is_same_direction: bool = is_same_direction
        self.cost: float = cost
        self.parent_node_index: str = parent_node_index


class AStarRoutePlanner:
    def __init__(self, map_file):
        self.opendrive2apollo = Opendrive2Apollo(map_file)
        self.opendrive_map = self.opendrive2apollo.xodr_map

        self.goal_node = None

    def search_route(self, start_road_index, goal_road_index):
        start_node = Node(self.opendrive_map.roads[start_road_index], True, 0.0, "")
        self.goal_node = Node(self.opendrive_map.roads[goal_road_index], True, 0.0, "")

        open_set, closed_set = dict(), dict()
        open_set[start_road_index] = start_node

        while len(open_set) > 0:
            current_node_index = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calculate_heuristic_cost(open_set[o]),
            )
            current_node = open_set[current_node_index]

            if current_node.road == self.goal_node.road:
                print("Find goal")
                self.goal_node.parent_node_index = current_node.parent_node_index
                self.goal_node.cost = current_node.cost
                return self.process_route(current_node, closed_set)

            # Remove the item from the open set
            del open_set[current_node_index]

            # Add it to the closed set
            closed_set[current_node_index] = current_node

            next_nodes = self.find_next_nodes(current_node)
            for next_node in next_nodes:
                next_node_index = next_node.road.road_id

                if next_node_index in closed_set:
                    continue

                if next_node_index not in open_set:
                    open_set[next_node_index] = next_node  # discovered a new node
                else:
                    if open_set[next_node_index].cost > next_node.cost:
                        # This path is the best until now. record it
                        open_set[next_node_index] = next_node

        print("Cannot find Route")
        return None

    def find_next_nodes(self, current_node: Node):
        connected_roads = []
        if current_node.is_same_direction:
            if current_node.road.link.successor.element_type == "junction":
                # 현재 Road Filter
                connected_roads = [
                    road[0]
                    for road in current_node.road.link.successor_junction.connected_roads
                    if road[0].road_id != current_node.road.road_id
                ]

            elif current_node.road.link.successor.element_type == "road":
                connected_roads = [current_node.road.link.successor_road]
        else:
            if current_node.road.link.predecessor.element_type == "junction":
                # 현재 Road Filter
                connected_roads = [
                    road[0]
                    for road in current_node.road.link.predecessor_junction.connected_roads
                    if road[0].road_id != current_node.road.road_id
                ]

            elif current_node.road.link.predecessor.element_type == "road":
                connected_roads = [current_node.road.link.predecessor_road]

        return [
            self.convert_road_to_node(road, current_node) for road in connected_roads
        ]

    @staticmethod
    def convert_road_to_node(next_road, current_node):
        cost = current_node.cost + current_node.road.length
        if next_road.link.predecessor.element_type == "junction":
            is_same_direction = any(
                [
                    road[0].road_id == current_node.road.road_id
                    for road in next_road.link.predecessor_junction.connected_roads
                ]
            )
        else:
            is_same_direction = (
                    next_road.link.predecessor.element_id == current_node.road.road_id
            )
        return Node(next_road, is_same_direction, cost, current_node.road.road_id)

    def calculate_heuristic_cost(self, node):
        node_point = node.road.plan_view.geometrys[0]
        goal_point = self.goal_node.road.plan_view.geometrys[0]
        distance = math.sqrt(
            (node_point.x - goal_point.x) ** 2 + (node_point.y - goal_point.y) ** 2
        )

        cost = distance
        return cost

    def process_route(self, current, closed_set):
        route = [self.goal_node.road]
        parent_node = current.parent_node_index
        while parent_node != "":
            n = closed_set[parent_node]
            route.append(n.road)
            parent_node = n.parent_node_index

        self.show_open_drive_map(route)

        return route

    def show_open_drive_map(self, route):
        self.opendrive2apollo.set_parameters(only_driving=False)
        self.opendrive2apollo.convert(route)


def main():
    # 1. Init Global Variable
    global_var._init()
    global_var.set_element_vaule("sampling_length", 1.0)
    global_var.set_element_vaule("debug_mode", False)
    global_var.set_element_vaule("enable_z_axis", False)
    global_var.set_element_vaule("need_save_figure", False)

    # 2. Set Map Path
    project_path = Path(os.path.abspath(__file__)).parent
    map_path = project_path / "town.xodr"
    if not map_path.is_file():
        print("File not exist! '{}'".format(map_path))
        return

    start_road = "6"
    goal_road = "21"
    print(f"Start A Star Route Planner (start {start_road}, end {goal_road})")

    route_planner = AStarRoutePlanner(str(map_path))
    route_planner.search_route(start_road, goal_road)


if __name__ == "__main__":
    main()
