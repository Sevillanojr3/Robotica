#!/usr/bin/python3

import sys
import copy
import rospy
from moveit_commander import MoveGroupCommander, RobotCommander, roscpp_initialize, PlanningSceneInterface
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cosh, sqrt, cos, sin
from std_msgs.msg import String, Float32, Int8
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
import yaml
from time import sleep

class ControlRobot:
    def __init__(self, group_name:str = 'robot') -> None:
        roscpp_initialize(sys.argv)
        rospy.init_node("control_robot", anonymous=True)
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group_name = group_name
        self.move_group = MoveGroupCommander(self.group_name)
        self.add_floor()

        self.publisher = rospy.Publisher('control_movimiento', Int8, queue_size=10)

    def get_joint_angles(self) -> list:
        return self.move_group.get_current_joint_values()
    
    def get_pose(self) -> Pose:
        return self.move_group.get_current_pose().pose
    
    def set_joint_angles(self, joint_goal: list, wait: bool = True) -> bool:
        return self.move_group.go(joint_goal, wait=wait)
    
    def set_pose(self, pose_goal: Pose, wait: bool = True) -> bool:
        self.move_group.set_pose_target(pose_goal)
        return self.move_group.go(wait=wait)
    
    def set_joint_trajectory(self, trajectory:list = []) -> bool:
        for i in range(len(trajectory)):
            state = self.set_joint_angles(trajectory[i])
            rospy.loginfo(f'Punto {i} alcanzado \n')
            if not state: print('Trayectoria Fallida'); return False
        rospy.loginfo('Trayectoria Finalizada')
        return state
    
    def set_pose_trajectory(self, trajectory:list = []) -> bool:
        for i in range(len(trajectory)):
            state = self.set_pose(trajectory[i])
            rospy.loginfo(f'Punto {i} alcanzado \n')
            if not state: print('Trayectoria Fallida'); return False
        rospy.loginfo('Trayectoria Finalizada')
        return state
    
    def set_carthesian_path(self, waypoints:list = [], eef_step:Float32 = 0.01, avoid_collisions:bool = True ,wait:bool = True) -> bool:
        if eef_step == 0.0:
            eef_step = 0.01
            print('eef_step modificado a valor 0.01 por requisitos de funcionamiento')
            
        waypoints.insert(0, self.get_pose())

        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, eef_step = eef_step, avoid_collisions= avoid_collisions)
        

        if fraction != 1.0:
            rospy.logwarn('Trayectoria Inalcanzable')
            rospy.loginfo(f'Porcentaje de la trayectoria alcanzable: {fraction*100:.2f}%')
            return False
        else: 
            rospy.loginfo('Ejecutando Trayectoria')
            message = Int8(1)
            self.publisher.publish(message)
            #sleep(1)
            result = self.move_group.execute(plan, wait=wait)
            message = Int8(0)
            self.publisher.publish(message)
            return result
        
    def set_box_obstacle(self, box_name:String, box_pose:Pose, size:tuple = (.1, .1, .1)) -> None:
        box_pose_stamped = PoseStamped()
        box_pose_stamped.header.frame_id = "base_link"
        box_pose_stamped.pose = box_pose
        self.scene.add_box(box_name, box_pose_stamped, size=size)

    def add_floor(self) -> None:
        pose_suelo = Pose()
        pose_suelo.position.z -= .03
        self.set_box_obstacle('floor', pose_suelo, (2,2,.05))

    def create_pose(self, pos_list:list, ori_list) -> Pose:
        if len(pos_list) != 3 or len(ori_list) != 4: return False
        pose = Pose()
        pose.position.x = pos_list[0]
        pose.position.y = pos_list[1]
        pose.position.z = pos_list[2]
        pose.orientation.x = ori_list[0]
        pose.orientation.y = ori_list[1]
        pose.orientation.z = ori_list[2]
        pose.orientation.w = ori_list[3]

        return pose
    
    def save_in_yaml(self, doc_name:str, key_name:str, data:list) -> None:
        diccionario_configuraciones = {key_name:data}
        with open(doc_name, '+a') as f:
            yaml.dump(diccionario_configuraciones, f)
    
    def get_point_from_yaml(self, doc_name:str, key_name:str) -> list:
        with open(doc_name, '+r') as f:
            configuraciones =  yaml.load(f, yaml.Loader)

        return configuraciones[key_name]

    def get_trayectory_from_yaml(self, doc_name:str) -> list:
        with open(doc_name, '+r') as f:
            configuraciones =  yaml.load(f, yaml.Loader)

        trayectoria = []
        for nombre_punto in list(configuraciones.keys()):
            trayectoria.append(configuraciones[nombre_punto])

        return trayectoria
    
    def get_dict_trayectory_from_yaml(self, doc_name:str) -> dict:
        with open(doc_name, '+r') as f:
            configuraciones =  yaml.load(f, yaml.Loader)

        return configuraciones


if __name__ == '__main__':

    control = ControlRobot('robot')

    print("------------------------------------------")
    print("¿Que quieres hacer?")
    print("   1. Guardar trayectorias")
    print("   2. Ejecturar trayectorias")
    print("------------------------------------------")
    accion = input("Acción: ")
    
    # --- CREAR FICHERO DE TRAYECTORIAS ---
    if accion == "1":
        trayectoria = int(input("Num. Trayectoria: "))
        punto = 0
        pose = []
        joint_state = 0
        while True:
            i = input("Accion: ")
            if i == "n": # Guardar Trayectoria
                control.save_in_yaml(f"JointStates_{trayectoria}.yaml", f"Fin", control.get_joint_angles())
                trayectoria +=1
                punto = 0
                joint_state = 0
                pose = []

            if i == "t": # Guardar Pose
                if punto == 0:
                    control.save_in_yaml(f"JointStates_{trayectoria}.yaml", f"Inicio", control.get_joint_angles())

                pose = control.get_pose()
                punto += 1
                control.save_in_yaml(f"Trayectoria_{trayectoria}.yaml", f"Punto_{punto}", pose)

            if i == 'j':
                joint_state += 1
                control.save_in_yaml(f"JointStates_{trayectoria}.yaml", f"Punto_{joint_state}", control.get_joint_angles())

            if i == "q": # Salir
                exit()


    # --- EJECUCIÓN DE TRAYECTORIAS ---
    if accion == "2":
        while True:
            print("\n\n\n****************************************")
            print(" ¿Qué trayectoria quieres hacer?")
            print("      1. Semicircular")
            print("      2. Rectángulo")
            print("      3. Línea Recta")
            print(" PD: Para salir cualquier otro número")
            print("****************************************")

            

            trayectoria = int(input(" Trayectoria: "))
            if trayectoria == 1:
                nombre_trayectoria = "Semicirculo"
            elif trayectoria == 2:
                nombre_trayectoria = "Rectangulo"
            elif trayectoria == 3:
                nombre_trayectoria = "LineaRecta"
            else:
                exit()


            poses_trayectory = control.get_trayectory_from_yaml(f"Trayectoria_{nombre_trayectoria}.yaml")
            joint_trayectory = control.get_dict_trayectory_from_yaml(f"JointStates_{nombre_trayectoria}.yaml")

            control.set_joint_angles(joint_trayectory["Inicio"])
            control.set_carthesian_path(poses_trayectory)
            control.set_joint_angles(joint_trayectory["Fin"])



    # trayectoria=[]
    # punto = control.get_pose()
    # print('Punto inicial de la trayectoria')
    # print(punto)
    # trayectoria.append(copy.deepcopy(punto))
    # punto.position.x -= 0.01
    # trayectoria.append(copy.deepcopy(punto))
    # punto.position.y -= 0.1
    # trayectoria.append(copy.deepcopy(punto))
    # print('Punto final de la trayectoria')
    # print(punto)
    # #control.set_pose_trajectory(trayectoria_pose)
    # #control.set_joint_trajectory(trayectoria_joint)
    # if not control.set_carthesian_path(trayectoria, eef_step=0.01):
    #     pass
    # else:
    #     punto = control.get_pose()
    #     print('Posición Alcanzada')
    #     print(punto)
    

# Funcion que cree un array de Poses
# Terminar funcion de trayectoria cartesiana
# Funcion que cree un JointTarget
# ,,.


    # if accion == '3':
    #     poses_trayectory = control.get_trayectory_from_yaml(f"Trayectoria_Circular.yaml")
    #     joint_trayectory = control.get_dict_trayectory_from_yaml(f"JointStates_Circular.yaml")

    #     #control.set_joint_angles(joint_trayectory["Inicio"])
    #     control.set_joint_angles(joint_trayectory["Inicio"])
    #     control.set_carthesian_path(poses_trayectory)
    #     control.set_joint_angles(joint_trayectory["Fin"])

    #     # joint_trayectory = control.get_dict_trayectory_from_yaml(f"JointStates_Circular.yaml")
    #     # punto = 0
    #     # control.set_joint_angles(joint_trayectory["Inicio"])

    #     # pose_act = control.get_pose()
    #     # pose_act.position.y -= .3
    #     # # (x-x_0)^2 + (y-y_0)^2 = r^2

    #     # waypoints = []
    #     # pose_inicial = control.get_pose()
    #     # x_0 = pose_inicial.position.x
    #     # y_0 = pose_inicial.position.y
    #     # modulo = .25
    #     # aux = copy.deepcopy(pose_inicial)

    #     # control.save_in_yaml(f"Trayectoria_Circular.yaml", f"Punto_{punto}", pose_inicial)

    #     # for i in range(100):
    #     #     fase = pi*i/100
    #     #     x = modulo*cos(fase)
    #     #     y = modulo*sin(fase)
    #     #     aux.position.x = x_0 + x - modulo
    #     #     aux.position.y = y_0 + y
    #     #     control.save_in_yaml(f"Trayectoria_Circular.yaml", f"Punto_{punto+i+1}", aux)
            
    #     #     waypoints.append(copy.deepcopy(aux))

    #     # control.set_carthesian_path(waypoints)

    #     quit()

#     from(bucket: "Grupo_7")
#   |> range(start: v.timeRangeStart, stop: v.timeRangeStop)
#   |> filter(fn: (r) => r["_measurement"] == "Trayectory_1732876046")
#   |> filter(fn: (r) => r["_field"] == "position")