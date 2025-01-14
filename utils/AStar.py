import math
import heapq
from utils.Point import Point
from utils.Geometry import Geometry

class Node:
    def __init__(self, position: Point, parent=None):
        self.position = position
        self.parent = parent
        self.g = 0  # Custo acumulado
        self.h = 0  # Heurística
        self.f = 0  # Custo total (g + h)

    def __lt__(self, other):
        return self.f < other.f


def is_close(position: Point, obstacles: dict) -> float:
    """Calcula a distância até o obstáculo mais próximo, retornando 0 se não há obstáculos próximos."""
    min_dist = 0.18
    closest_distance = float("inf")
    
    for _, obstacle in obstacles.items():
        dist = position.dist_to(obstacle)
        if dist < closest_distance:
            closest_distance = dist
    return closest_distance if closest_distance < min_dist else 0.0


def is_destination(position: Point, target: Point) -> bool:
    """Verifica se o nó atual é o destino."""
    return Geometry.dist_to(position, target) < 0.1


def heuristic(position: Point, target: Point, obstacles: dict) -> float:
    """Heurística considerando a distância até o destino e obstáculos."""
    distance_to_target = Geometry.dist_to(position, target)
    distance_to_obstacle = is_close(position, obstacles)
    return 1 * distance_to_target + 100 * distance_to_obstacle


def is_valid(position: Point) -> bool:
    """Verifica se a posição está dentro dos limites e longe de obstáculos."""
    if not (-2.82 <= position.x <= 2.82 and -1.82 <= position.y <= 1.82):
        return False
    return True


def trace_path(node: Node) -> list:
    """Reconstroi o caminho do destino até a origem."""
    path = []
    while node is not None:
        path.append(node.position)
        node = node.parent
    print(f"a_star: waypoints={path.__len__()}.")
    return path[::-1]


def a_star_search(robot: Point, target: Point, obstacles: dict) -> list:
    print(f"a_star: target location (x, y)={target}.")
    """Busca A* adaptada para um mapa contínuo, priorizando caminhos mais próximos do destino."""
    open_list = []
    closed_set = {}

    start_node = Node(robot)
    heapq.heappush(open_list, start_node)

    max_iterations = 100000 # Limite de iterações para evitar loops infinitos
    iteration_count = 0

    while open_list and iteration_count < max_iterations:
        iteration_count += 1
        current_node = heapq.heappop(open_list)

        # Verifica se o destino foi alcançado
        if is_destination(current_node.position, target):
            print(f"a_star: path_iterations={iteration_count}.")
            return trace_path(current_node)

        closed_set[(current_node.position.x, current_node.position.y)] = current_node.g

        # Gerar vizinhos
        step_size = 0.05
        directions = [
            (step_size, 0), (-step_size, 0),
            (0, step_size), (0, -step_size),
            (step_size, step_size), (-step_size, -step_size),
            (step_size, -step_size), (-step_size, step_size)
        ]

        for dx, dy in directions:
            neighbor_position = Point(
                current_node.position.x + dx,
                current_node.position.y + dy
            )

            if not is_valid(neighbor_position):
                continue

            neighbor_g = current_node.g + step_size
            neighbor_key = (neighbor_position.x, neighbor_position.y)

            # Verificar se o nó já foi processado com menor custo
            if neighbor_key in closed_set and neighbor_g >= closed_set[neighbor_key]:
                continue

            neighbor_node = Node(neighbor_position, current_node)
            neighbor_node.g = neighbor_g
            neighbor_node.h = heuristic(neighbor_position, target, obstacles)
            neighbor_node.f = neighbor_node.g + neighbor_node.h

            heapq.heappush(open_list, neighbor_node)
            closed_set[neighbor_key] = neighbor_g  # Atualiza com o menor custo encontrado

    print("a_star: path not found or max_iterations reached.")
    return []