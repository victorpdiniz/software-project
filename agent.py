from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.Geometry import Geometry
from utils.Point import Point
from utils.AStar import a_star_search

class ExampleAgent(BaseAgent):
    def __init__(self, id=0, yellow=False):
        super().__init__(id, yellow)
        self.path = []
        self.current_target = None
        self.path_index = 0

    def decision(self):
        if len(self.targets) == 0:
            return
    
        # target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, self.targets[0])
        # self.set_vel(target_velocity)
        # self.set_angle_vel(target_angle_velocity)

        position = Point(self.robot.x, self.robot.y)
        next_target = self.targets[0]

        # Recalcular o caminho somente se o alvo mudar ou se o caminho estiver vazio
        if self.current_target != next_target:
            print(f"agent: current target (x,y)={self.current_target}")
            print(f"agent: next_target (x,y)={next_target}")
            if self.current_target != next_target:
                print("agent: new target generated.")
            else:
                print("agent: path is empty.")
            self.path = a_star_search(position, next_target, self.opponents)
            print("agent: path completed.")
            self.current_target = next_target
            self.path_index = 0  # Reinicia o índice do caminho

        # Seguir o próximo ponto no caminho
        if self.path_index < len(self.path):
            next_waypoint = self.path[self.path_index]

            # Verifica se está próximo do próximo ponto
            if position.dist_to(next_waypoint) < 0.05:
                self.path_index += 1  # Avança para o próximo ponto

            # Verifica se o próximo ponto é o alvo
            if next_target.dist_to(next_waypoint) <= 0.720:
                is_target = True
            else:
                is_target = False
            
            # Define as velocidades para o próximo ponto
            if self.path_index < len(self.path):  # Garante que ainda há pontos no caminho
                target_velocity, target_angle_velocity = Navigation.goToPoint(
                    self.robot, self.path[self.path_index], is_target
                )
                self.set_vel(target_velocity)
                self.set_angle_vel(target_angle_velocity)

        return

    def post_decision(self):
        pass
