import math
import heapq
from utils.Point import Point
from utils.Geometry import Geometry
from rsoccer_gym.Entities import Robot


class Node:
    def __init__(self, position: Point, parent: Point=None):
        self.position = position
        self.parent = parent
        self.g = 0  # Cumulated cost
        self.h = 0  # Heuristics cost
        self.f = 0  # Total cost (g + h)

    def __lt__(self, other):
        return self.f < other.f

def closest_obstacle_distance(position: Point, obstacles: dict, hulls: dict) -> float:
    """Returns the distance from the closest obstacle to the position."""
    min_distance_to_obstacle = 0.20
    close_obstacles_distances = []
    
    for _, obstacle in obstacles.items():
        distance_to_obstacle = position.dist_to(obstacle)
        if distance_to_obstacle <= min_distance_to_obstacle:
            close_obstacles_distances.append(distance_to_obstacle)

    sum = 0
    for distance in close_obstacles_distances:
        sum += distance
     
    return sum

def closest_hull_distance(position: Point, hulls: dict) -> float:
    """Returns the distance from the closest hull to the position."""
    sum = 0
    
    for hull, min_distance_to_hull in hulls.items():
        distance_to_hull = position.dist_to(hull)
    
        if distance_to_hull <= min_distance_to_hull:
            sum += distance_to_hull
     
    return sum

def heuristic(position: Point, target: Point, obstacles: dict, hulls: dict) -> float:
    """Calculates the heuristics cost considering the distance to target, to the closest hull, to the closest obstacle."""
    distance_to_target = position.dist_to(target)
    distance_to_obstacle = closest_obstacle_distance(position, obstacles, hulls)
    distance_to_hull = closest_hull_distance(position, hulls)
    
    return 6 * distance_to_target + distance_to_hull + 40 * distance_to_obstacle


def is_valid(position: Point) -> bool:
    """Checks if the position is inside field's limits."""
    return (-3 <= position.x <= 3 and -2 <= position.y <= 2)


def trace_path(node: Node) -> list:
    """Collects the waypoints of the path from robot to target."""
    path = []
    
    while node is not None:
        path.append(node.position)
        node = node.parent
    print(f"a_star: path waypoints={path.__len__()}.")
    
    return path[::-1]

def create_hull(obstacles: dict) -> dict:
    """Adds opponents-like points to the dict to avoid bugs."""
    hulls = {}
    
    for i, obstacle_1 in obstacles.items():
        location_1 = Point(obstacle_1.x, obstacle_1.y)
        
        for j, obstacle_2 in obstacles.items():
            location_2 = Point(obstacle_2.x, obstacle_2.y)
            center = Geometry.medium_point(location_1, location_2)
        
            if location_1.dist_to(location_2) <= 0.40 and obstacle_1 != obstacle_2 and center not in hulls.keys():
                print(f"hull_creation: new hull center with location (x,y)={center}.")
                hulls[center] = (location_1.dist_to(location_2) / 2) + 0.2
    
    return hulls

def a_star(robot: Point, target: Point, obstacles: dict) -> list:
    """Returns the waypoints of the path from robot to target."""
    print(f"a_star: searching for path...")
    possible_nodes = []
    visited_nodes = {}
    hulls = create_hull(obstacles)
    
    start_node = Node(robot)
    heapq.heappush(possible_nodes, start_node)

    iteration_count = 0
    max_iterations = 10000
    
    # While there is a path to follow.
    while possible_nodes and iteration_count < max_iterations:
        iteration_count += 1
        current_node = heapq.heappop(possible_nodes)
        
        # Verifies if the path has been reached.
        if current_node.position.dist_to(target) < 0.1 or iteration_count == max_iterations - 1:
            print(f"a_star: path found, did {iteration_count} iterations.")
            return trace_path(current_node)

        # Current node value.
        visited_nodes[current_node.position] = current_node.g

        # Generates neighbors.
        step_size = 0.05
        
        x_normalized = step_size * abs(current_node.position.x - target.x) / current_node.position.dist_to(target)
        y_normalized = step_size * abs(current_node.position.y - target.y) / current_node.position.dist_to(target)
        
        directions = [
            (step_size, 0), (-step_size, 0),                    # Left and right neighbors
            (0, step_size), (0, -step_size),                    # Upward and downward neighbors
            (x_normalized, y_normalized), (-x_normalized, -y_normalized),   # Right and upward, left and downward neighbors
            (x_normalized, -y_normalized), (-x_normalized, y_normalized)    # Right and downward, left and upward neighbors
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
            neighbor_node.h = heuristic(neighbor_position, target, obstacles, hulls)
            neighbor_node.f = neighbor_node.g + neighbor_node.h

            # Pushes the neighbor to the visited nodes list.
            heapq.heappush(possible_nodes, neighbor_node)
            visited_nodes[neighbor_key] = neighbor_g

    print("a_star: path not found.")
    return []