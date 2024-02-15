import socket
import rospy 
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
import numpy as np

#Experimental function for converting quaternion rotation vector to euler angles (rx,ry,rz)
# [w,rx,ry,rz]
def quaternion_to_rotation_vector(quaternion):
    quaternion /= np.linalg.norm(quaternion)
    scalar = quaternion[0]
    vector = quaternion[1:]
    theta = 2 * np.arccos(scalar)
    if np.sin(theta / 2) == 0:
        rotation_vector = np.zeros(3)
    else:
        rotation_vector = vector * (theta / np.sin(theta / 2))
    return rotation_vector

#Function to initialize the controller node and subscribe to rostopic /joint_states
def joint_state_Subscriber():
    rospy.init_node('joint_state_subscriber', anonymous=True)
    joint_state_topic = '/joint_states' 
    rospy.Subscriber(joint_state_topic, JointState, joint_state_callback)

#Function to subscribe to the rostopic /tf (Tool position)
def tf_state_Subscriber():
    #rospy.init_node('tf_state_subscriber', anonymous=True)
    tf_state_topic = '/tf' 
    rospy.Subscriber(tf_state_topic, TFMessage, pose_state_callback)

#Process joint states format to match teach pendant notation
def joint_state_callback(msg):
    global joint_positions
    joint_positions = msg.position
    joint_positions = list(joint_positions)
    base = joint_positions[2]
    elbow = joint_positions[0]
    joint_positions[0],joint_positions[2] = base,elbow

#Process tool position format to match teach pendant notation
def pose_state_callback(msg):
    global tf_position
    tf_position_processed = msg.transforms
    tf_position_processed = [item.strip() for item in str(tf_position_processed[0]).split('\n ')]
    index_trans = tf_position_processed.index('translation:')
    index_rot =tf_position_processed.index('rotation:')
    translation_vector = tf_position_processed[index_trans+1:index_trans+4]
    rotation_vector = tf_position_processed[index_rot+1:index_rot+5]
    translation_vector = list(map(float,[item.split()[1] for item in translation_vector]))
    translation_vector[2] = translation_vector[2] - 0.400001208
    rotation_vector = list(map(float,[item.split()[1] for item in rotation_vector]))
    rotation_vector[0],rotation_vector[3] = rotation_vector[3],rotation_vector[0]
    new_rot_vec = quaternion_to_rotation_vector(rotation_vector)
    new_rot_vec = [item*-1 for item in new_rot_vec]
    new_rot_vec[0],new_rot_vec[1],new_rot_vec[2] = new_rot_vec[2],new_rot_vec[0],new_rot_vec[1]
    translation_vector.extend(new_rot_vec)
    tf_position = translation_vector

#Definition of gripper object 
class Gripper:
    #Gripper properties (ip, port and socket connection)
    def __init__(self,ip,port):
        self.ip = ip
        self.port = port
        self.socket = None
    #Connection to gripper socket
    def connect(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.ip,self.port))
    #Function to send commands to gripper
    def send_command(self,command):
        byte_array = command.encode('utf-8')
        self.socket.sendall(byte_array + b'\n')
    #Funciton to move gripper to a given distance (0-255)(Open-Closed)
    def move_to(self,distance):
        command = f"SET POS {str(distance)}"
        self.send_command(command)

#Definition of robot object
class Robot:
    #Robot properties (ip, port, socket connection and moving state)
    def __init__(self,ip,port): 
        self.ip = ip
        self.port = port
        self.socket = None
        self.is_moving = False
    #Connection to robot socket
    def connect(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.ip,self.port))
    #Connection to robot socket
    def send_command(self,command):
        byte_array = command.encode('utf-8')
        self.socket.sendall(byte_array + b'\n')
    #Funciton to move robot to a given tool pose 
    def move_to_pose(self,position,motion_type ='j',acc=0.5,velocity=1.5):
        if not self.is_moving:
            command = f"move{motion_type}(p[{', '.join(map(str, position))}], a={str(acc)}, v={str(velocity)})"
            self.send_command(command)
    #Funciton to move robot to a given pose in joint angles notation
    def move_to_joints(self,position,acc=0.5,velocity=1.5):
        if not self.is_moving:
            command = f"movej([{', '.join(map(str, position))}], a={str(acc)}, v={str(velocity)})"
            self.send_command(command)
    #Function to change moving state once goal pose has been reached
    def next_movement(self,current,goal,tolerance=0.01):
        for (ca,ga) in zip(current,goal):
            if not ga - tolerance <= ca <= ga + tolerance:
                self.is_moving = True
                return self.is_moving
        self.is_moving = False
        return self.is_moving
    #Function to indicate next position when robot has complete one movement
    def execute_routine(self,routine,movement = 'joints',type = 'j'):
        for pos in routine:
            if movement == 'joints':
                self.move_to_joints(pos)
            elif movement == 'tf':
                self.move_to_pose(pos,type)
            while self.next_movement(joint_positions,pos):
                continue
    #Robot sad animation
    def perder(self):
        pos1 = [-0.145,0.235,0.409,2.006,2.263,-0.181]
        pos2 = [0.003,0.249,0.459,1,627,1.940,0.441]
        pos3 = [-0.310,0.239,0.473,2.322,2.458,-1.11]
        pos4 = [-0.145,0.235,0.409,2.006,2.263,-0.181]
        routine = [homep,pos1,pos2,pos3,pos4,homep]
        self.execute_routine(routine,movement='tf')
    #Robot happy animation
    def ganar(self):
        pos1 = [3.114,-0.7978,-1.91,-0.4119,-0.005934,1.508]
        pos2 = [3.114,-0.7973,-1.91,0.04398,-0.005934,1.508]
        pos3 = [3.114,-0.7973,-1.91,-0.8166,-0.005934,1.508]
        routine = [homej,pos1,pos2,pos3,homej]
        self.execute_routine(routine)
#Home positions
homej = [4.75,-1.442,1.03,-1.152,-1.563,4.634]
home_ur = [0,-1.57,0,-1.57,0,0]
homep = [-0.8752,0.52295,0.24076,2.235,2.210,-0.016]
#Main program
def main():
    robot.ganar()

if __name__ == "__main__":
    joint_positions = None
    tf_position = None
    robot = Robot(ip="0.0.0.0", port=30002)
    robot.connect()
    joint_state_Subscriber()
    tf_state_Subscriber()
    try:
        while not rospy.is_shutdown():
            if joint_positions is not None and tf_position is not None:
                main()
                break
            else:
                print('Waiting for joint positions...')
            rospy.sleep(0.1)
    except rospy.ROSInterruptException:
        pass
