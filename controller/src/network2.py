import time
import math
import torch
import rospy
import ctypes
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Pose,Twist,Quaternion
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from mujoco_sim.msg import motor_data

def quat_to_euler(q):
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                           1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(2.0 * (q.w * q.y - q.z * q.x))
    return roll, pitch

class leg():
    def __init__(self):
        self.endp = np.zeros(3, dtype=np.double)
        self.endv = np.zeros(3, dtype=np.double)
        self.endp_tar = np.zeros(3, dtype=np.double)
        self.endf_tar = np.zeros(3, dtype=np.double)
        # self.kp = 0.25
        # self.kd = 0.005
        self.kp = 0.0
        self.kd = 0.0

class NetworkInterface():
    def __init__(self):
        " load network "
        path = "./test3.pt"
        self.policy = torch.load(path).to(device="cpu")
        self.policy.eval()
        self.obs = torch.zeros(14)

        " Init Param "
        self.bodyImu = Imu()
        self.bodydv = Vector3Stamped()
        self.bodyMotor = [motor_data(), motor_data(), motor_data()]
        self.bodyActor = [motor_data(), motor_data(), motor_data()]
        self.pauseflag = String("1")
        for i in range(0,3):
            self.bodyActor[i].id = i + 1
            self.bodyActor[i].pos_tar = 0.0
            self.bodyActor[i].vel_tar = 0.0
            self.bodyActor[i].tor_tar= 0.0
            # self.bodyActor[i].kp = 5.0
            # self.bodyActor[i].kd = 0.2
            self.bodyActor[i].kp = 1500
            self.bodyActor[i].kd = 15
        self.loop_rate = rospy.Rate(500)
        self.count = 0

        " Init Ros "
        self.pubCmds = rospy.Publisher('/cybergear_cmds', motor_data, queue_size=3)
        rospy.Subscriber("/imu/data", Imu, self.imu_data_callback, queue_size=1)
        rospy.Subscriber("/imu/dv", Vector3Stamped, self.imu_dv_callback, queue_size=1)
        rospy.Subscriber("/cybergear_msgs", motor_data, self.cybergear_msgs_callback, queue_size=3)
        rospy.Subscriber('/pause', String, self.pause_callback, queue_size=10)

    def imu_data_callback(self, msg):
        self.bodyImu = msg

    def imu_dv_callback(self, msg):
        self.bodydv = msg

    def cybergear_msgs_callback(self, msg):
        id = msg.id
        self.bodyMotor[id - 1] = msg

    def pause_callback(self, msg):
        self.pauseflag = msg

    " from endp_tar to pos_tar and tor_tar "
    def pub_cmds(self):
        for i in range(0,3):
            self.pubCmds.publish(self.bodyActor[i])

    " from action to endp_tar "
    def compute_endp(self, action):
        self.bodyActor[0].pos_tar = action[0] * 0.075
        self.bodyActor[1].pos_tar = action[1] * 0.075
        self.bodyActor[2].pos_tar = action[2] * 0.075

    " compute observation "
    def compute_observation(self):
        self.obs[0] = torch.tensor(self.bodydv.vector.x)
        self.obs[1] = torch.tensor(self.bodydv.vector.y)
        self.obs[2] = torch.tensor(self.bodydv.vector.z)
        
        self.obs[3] = torch.tensor(self.bodyImu.angular_velocity.x)
        self.obs[4] = torch.tensor(self.bodyImu.angular_velocity.y)
        self.obs[5] = torch.tensor(self.bodyImu.angular_velocity.z)

        roll, pitch = quat_to_euler(self.bodyImu.orientation)
        self.obs[6] = torch.tensor(roll)
        self.obs[7] = torch.tensor(pitch)

        self.obs[8]  = torch.tensor(0.0)
        self.obs[9]  = torch.tensor(0.0)
        self.obs[10] = torch.tensor(0.0)

        self.obs[11] = torch.tensor(self.bodyMotor[0].pos)
        self.obs[12] = torch.tensor(self.bodyMotor[1].pos)
        self.obs[13] = torch.tensor(self.bodyMotor[2].pos)
    
    " loop "
    def loop(self):
        while not rospy.is_shutdown():
            if self.pauseflag.data != "1":
                # print(self.bodyMotor)

                start_time = time.time()
                 
                self.compute_observation()
                action = self.policy(self.obs.detach()).detach().numpy()
                self.compute_endp(action)
                self.pub_cmds()

                end_time = time.time()

                print("exec_time:", end_time - start_time)
                print(self.obs, action)

            self.loop_rate.sleep()

def main():
    rospy.init_node('network2', anonymous=True)

    nw = NetworkInterface()
    print("Init Finish")
    nw.loop()

if __name__ == "__main__":
    main()