from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.Geometry import Geometry
from utils.Point import Point
from utils.AStar import a_star

class ExampleAgent(BaseAgent):
    def __init__(self, id=0, yellow=False):
        super().__init__(id, yellow)
        self.path = []              # Path from robot to target
        self.current_target = None  # Target position
        self.path_index = 0         # Path index
        self.difficulty = 1         # Selects algorithm level of implementation 

    def decision(self):
        """Take robot's decision to navigation or obstacle avoidance."""
        if len(self.targets) == 0:
            return

        # Level 0
        if self.difficulty == 0:
            target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, self.targets[0])
            self.set_vel(target_velocity)
            self.set_angle_vel(target_angle_velocity)
        
        # Level 1
        elif self.difficulty == 1:
            
            position = Point(self.robot.x, self.robot.y)
            next_target = self.targets[0]
                
            # Calculates a new path if a new target is generated.
            if self.current_target != next_target:
                print("-" * 100)
                print("agent: new target generated.")
                
                self.path = a_star(position, next_target, self.opponents)
                print("agent: search for path completed.")
                
                # Restarts index and target position.
                self.current_target = next_target
                self.path_index = 0
                
            # Calculates a new path if target was not reached.
            if position != self.targets[0] and len(self.path) == self.path_index + 1:
                print("agent: target was not reached, recalculating route.")

                self.path = a_star(position, next_target, self.opponents)
                print("agent: search for new path completed.")

                # Restarts index and target position.
                self.current_target = next_target
                self.path_index = 0

            # Follows the next waypoint in path.
            if self.path_index < len(self.path):
                next_waypoint = self.path[self.path_index]

                # Checks if the robot is close to the next waypoint.
                if position.dist_to(next_waypoint) < 0.05:
                    self.path_index += 1 

                # Defines velocity and angle velocity.
                target_velocity, target_angle_velocity = Navigation.goToPoint(
                    self.robot, self.path[self.path_index]
                )
                self.set_vel(target_velocity)
                self.set_angle_vel(target_angle_velocity)

        return

    def post_decision(self):
        pass
