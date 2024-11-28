#!/usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander
from std_msgs.msg import Int32
from typing import List
from geometry_msgs.msg import Pose, PoseStamped
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface

class ControlRobot:
    def __init__(self) -> None: 
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group_name = "robot"  
        self.move_group = MoveGroupCommander(self.group_name)
        self.add_floor()

    def get_motor_angles(self) -> List[float]: 
        return self.move_group.get_current_state().joint_state.position

    def move_motors(self, joint_goal: List[float], wait: bool=True) -> bool:
        return self.move_group.go(joint_goal, wait=wait)

    def get_pose(self) -> Pose:
        return self.move_group.get_current_pose().pose  

    def move_to_pose(self, pose_goal: Pose, wait: bool=True) -> bool:  
        self.move_group.set_pose_target(pose_goal)
        return self.move_group.go(wait=wait)

    def add_to_planning_scene(self, pose_box: Pose, name: str, size: tuple = (0.1, 0.1, 0.1)) -> None:
        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose = pose_box  
        self.scene.add_box(name, box_pose, size=size)
        
    def move_trajectory (self, poses: List[Pose], wait: bool=True) -> bool:
        (plan, fraction) = self.move_group.compute_cartesian_path(poses, 0.01, 0.0)
        
        if fraction != 1.0:
            return False
        
        return self.move_group.execute(plan, wait=wait)

    def add_floor(self) -> None:
        pose_floor = Pose()
        pose_floor.position.z = -0.025
        self.add_to_planning_scene(pose_floor, "floor", (2, 2, 0.05)) 

def control_callback(msg, control_robot):
    action_code = msg.data
    if action_code == 1:
        # Añadir obstáculos a la escena de planificación
        control_robot.add_floor()
        print("Se ha añadido el suelo a la escena de planificación.")
    elif action_code == 2:
        # Mover el robot a una pose
        print("Introduzca la pose objetivo (x y z qx qy qz qw):")
        input_str = input()
        values = [float(x) for x in input_str.strip().split()]
        if len(values) != 7:
            print("Entrada inválida. Se esperaban 7 valores.")
            return
        pose_goal = Pose()
        pose_goal.position.x = values[0]
        pose_goal.position.y = values[1]
        pose_goal.position.z = values[2]
        pose_goal.orientation.x = values[3]
        pose_goal.orientation.y = values[4]
        pose_goal.orientation.z = values[5]
        pose_goal.orientation.w = values[6]
        success = control_robot.move_to_pose(pose_goal)
        if success:
            print("El robot se ha movido a la pose objetivo.")
        else:
            print("No se pudo mover a la pose objetivo.")
    elif action_code == 3:
        # Mover el robot a una configuración
        print("Introduzca los ángulos de las articulaciones (en radianes) separados por espacios:")
        input_str = input()
        joint_goal = [float(x) for x in input_str.strip().split()]
        success = control_robot.move_motors(joint_goal)
        if success:
            print("El robot se ha movido a la configuración objetivo.")
        else:
            print("No se pudo mover a la configuración objetivo.")
    elif action_code == 4:
        # Mover el extremo del robot por una trayectoria dada
        print("Introduzca el número de puntos de la trayectoria:")
        num_waypoints = int(input())
        waypoints = []
        for i in range(num_waypoints):
            print(f"Introduzca el punto {i+1} (x y z):")
            input_str = input()
            values = [float(x) for x in input_str.strip().split()]
            if len(values) != 3:
                print("Entrada inválida. Se esperaban 3 valores.")
                return
            pose = Pose()
            pose.position.x = values[0]
            pose.position.y = values[1]
            pose.position.z = values[2]
            # Mantener la orientación actual
            current_pose = control_robot.get_pose()
            pose.orientation = current_pose.orientation
            waypoints.append(pose)
        success = control_robot.move_trajectory(waypoints)
        if success:
            print("El robot se ha movido a lo largo de la trayectoria.")
        else:
            print("No se pudo completar la trayectoria.")
    else:
        print("Código de acción inválido recibido.")

def listener():
    rospy.init_node('control_robot_node', anonymous=True)
    control_robot = ControlRobot()
    rospy.Subscriber('/consignas', Int32, control_callback, callback_args=control_robot)
    rospy.spin()

if __name__ == '__main__':
    listener()