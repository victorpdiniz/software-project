import math
import heapq
from utils.Point import Point
from utils.Geometry import Geometry


M_TO_MM: float = 1000.0


class Node:
    def __init__(self, position: Point, parent: Point=None):
        self.position = position
        self.parent = parent
        self.g = 0  # Cumulated cost
        self.h = 0  # Heuristics cost
        self.f = 0  # Total cost (g + h)

    def __lt__(self, other):
        return self.f < other.f


def closest_obstacle_distance(position: Point, obstacles: dict) -> float:
    """Returns the distance from the closest obstacle to the position."""
    min_distance_to_obstacle = 0.20
    closest_distance = float("inf")
    
    for _, obstacle in obstacles.items():
        distance_to_obstacle = position.dist_to(obstacle)
        closest_distance = min(distance_to_obstacle, closest_distance)
        
    return closest_distance if closest_distance <= min_distance_to_obstacle else 0.0


def heuristic(position: Point, target: Point, obstacles: dict) -> float:
    """Calculates the heuristics cost considering the distance to target and to the closest obstacle."""
    distance_to_target = position.dist_to(target)
    distance_to_obstacle = closest_obstacle_distance(position, obstacles)
    
    return 20 * distance_to_target + 800 * distance_to_obstacle


def is_valid(position: Point) -> bool:
    """Checks if the position is inside field's limits."""
    return (-2.82 <= position.x <= 2.82 and -1.82 <= position.y <= 1.82)


def trace_path(node: Node) -> list:
    """Collects the waypoints of the path from robot to target."""
    path = []
    
    while node is not None:
        path.append(node.position)
        node = node.parent
    print(f"a_star: number of waypoints={path.__len__()}.")
    
    return path[::-1]


def a_star(robot: Point, target: Point, obstacles: dict) -> list:
    """Returns the waypoints of the path from robot to target."""
    print(f"a_star: target location (x, y)={target*M_TO_MM}.")
    possible_nodes = []
    visited_nodes = {}

    start_node = Node(robot)
    heapq.heappush(possible_nodes, start_node)

    iteration_count = 0

    # While there is a path to follow.
    while possible_nodes:
        iteration_count += 1
        current_node = heapq.heappop(possible_nodes)

        # Verifies if the path has been reached.
        if current_node.position.dist_to(target) < 0.1:
            print(f"a_star: path found.")
            print(f"a_star: path iterations={iteration_count}.")
            return trace_path(current_node)

        # Current node value.
        visited_nodes[current_node.position] = current_node.g

        # Generates neighbors.
        step_size = 0.05
        directions = [
            (step_size, 0), (-step_size, 0),                    # Left and right neighbors
            (0, step_size), (0, -step_size),                    # Upward and downward neighbors
            (step_size, step_size), (-step_size, -step_size),   # Right and upward, left and downward neighbors
            (step_size, -step_size), (-step_size, step_size)    # Right and downward, left and upward neighbors
        ]
        
        # Finds the minimum cost neighbor.
        for dx, dy in directions:
            neighbor_position = Point(
                current_node.position.x + dx,
                current_node.position.y + dy
            )
            
            # Checks if the neighbor is valid.
            if not is_valid(neighbor_position):
              continue

            neighbor_g = current_node.g + step_size
            neighbor_key = neighbor_position

            # Checks if the neighbor has already been added and has the minimum cost.
            if neighbor_key in visited_nodes and neighbor_g >= visited_nodes[neighbor_key]:
                continue
            
            # Updates the minimum cost for this neighbor.
            neighbor_node = Node(neighbor_position, current_node)
            neighbor_node.g = neighbor_g
            neighbor_node.h = heuristic(neighbor_position, target, obstacles)
            neighbor_node.f = neighbor_node.g + neighbor_node.h

            # Pushes the neighbor to the visited nodes list.
            heapq.heappush(possible_nodes, neighbor_node)
            visited_nodes[neighbor_key] = neighbor_g

    print("a_star: path not found.")
    return []