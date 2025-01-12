from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.Geometry import Geometry
from utils.Point import Point

class ExampleAgent(BaseAgent):
    def __init__(self, id=0, yellow=False):
        super().__init__(id, yellow)

    def decision(self):
        if len(self.targets) == 0:
            return

        robot_point = Point(self.robot.x, self.robot.y)
        dist_min_to_opponent = 0.2
        
        for i in range(1, 21):
            opponent_point = Point(self.opponents[i].x, self.opponents[i].y)
            if Geometry.dist_to(robot_point, opponent_point) < dist_min_to_opponent:
                print(f"Robot collided with opponent {i}.")         

        target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, self.targets[0])
        self.set_vel(target_velocity)
        self.set_angle_vel(target_angle_velocity)

        return

    def post_decision(self):
        pass
