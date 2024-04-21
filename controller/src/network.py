import time
import torch
import rospy
import ctypes
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Pose,Twist,Quaternion
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from mujoco_sim.msg import motor_data
        
def quat_rotate_inverse(q, v):
    q_w = q.w
    q_vec = np.array([q.x, q.y, q.z])
    v_vec = np.array([v.x, v.y, v.z])
    a = v_vec * (2.0 * q_w ** 2 - 1.0)
    b = np.cross(q_vec, v_vec) * q_w * 2.0
    c = q_vec * np.dot(q_vec, v_vec) * 2.0
    return a - b + c

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

        self.bias_z = 212

class NetworkInterface():
    def __init__(self):
        " load network "
        path = "./RLSLIP.pt"
        self.policy = torch.load(path).to(device="cpu")
        self.policy.eval()
        self.obs = torch.zeros(21)

        " load cpp Delta lib "
        self.lib = ctypes.cdll.LoadLibrary("./Delta.so")
        self.delta = self.lib.delta_new(
            ctypes.c_double(62.5),ctypes.c_double(40.0),
            ctypes.c_double(110.0),ctypes.c_double(250.0))
        
        " Init leg "
        self.leg = leg()
        for i in range(0,3):
            self.lib.set_motor_pos(self.delta,ctypes.c_double(0.0),i)
        self.lib.cal_jacob(self.delta)
        for i in range(0,3):
            self.lib.get_endp.restype = ctypes.c_double
            self.leg.endp[i] = self.lib.get_endp(self.delta,i)
            self.leg.endp_tar[i] = self.leg.endp[i]
            self.lib.set_endp_tar(
                self.delta,ctypes.c_double(self.leg.endp_tar[i]),i)
        self.lib.inverse_kinematics(self.delta)
        # self.lib.get_motor_pos_tar.restype = ctypes.c_double
        # ans = self.lib.get_motor_pos_tar(self.delta,0)

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
            self.bodyActor[i].kp = 5.0
            self.bodyActor[i].kd = 0.2
        self.loop_rate = rospy.Rate(50)
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
        " calculate pos_tar "
        for i in range(0,3):
            self.lib.set_endp_tar(
                self.delta,ctypes.c_double(self.leg.endp_tar[i]),i)
        self.lib.inverse_kinematics(self.delta)
        for i in range(0,3):
            self.lib.get_motor_pos_tar.restype = ctypes.c_double
            self.bodyActor[i].pos_tar = \
                self.lib.get_motor_pos_tar(self.delta,i)
        
        " calculate endf_tar and tor_tar "
        for i in range(0,3):
            self.leg.endf_tar[i] = self.leg.kp *\
                (self.leg.endp_tar[i] - self.leg.endp[i]) +\
                self.leg.kd *\
                (0.0 - self.leg.endv[i])
            self.lib.set_endf_tar(
                self.delta,ctypes.c_double(self.leg.endf_tar[i]),i)
        self.lib.statics(self.delta)
        for i in range(0,3):
            self.lib.get_motor_tor_tar.restype = ctypes.c_double
            self.bodyActor[i].tor_tar = \
                self.lib.get_motor_tor_tar(self.delta,i)

        " publish "
        for i in range(0,3):
            self.pubCmds.publish(self.bodyActor[i])

    " from action to endp_tar "
    def compute_endp(self, action):
        self.leg.endp_tar[0] = action[0] * 1.0
        self.leg.endp_tar[1] = action[1] * 1.0
        self.leg.endp_tar[2] = - action[2] * 1.0 + self.leg.bias_z

        # self.count = self.count + 1
        # theta = self.count / 50 * np.pi
        # self.leg.endp_tar[0] = 40 * np.cos(theta)
        # self.leg.endp_tar[1] = 40 * np.sin(theta)

    " compute observation "
    def compute_observation(self):
        " calculate jacob and endp and endv "
        for i in range(0,3):
            self.lib.set_motor_pos(
                self.delta,ctypes.c_double(self.bodyMotor[i].pos),i)
            self.lib.set_motor_vel(
                self.delta,ctypes.c_double(self.bodyMotor[i].vel),i)
        self.lib.cal_jacob(self.delta)
        self.lib.forward_vf(self.delta)
        for i in range(0,3):
            self.lib.get_endp.restype = ctypes.c_double
            self.leg.endp[i] = self.lib.get_endp(self.delta,i)
            self.lib.get_endv.restype = ctypes.c_double
            self.leg.endv[i] = self.lib.get_endv(self.delta,i)

        " calculate observation "
        base_lin_vel = quat_rotate_inverse(
            self.bodyImu.orientation,self.bodydv.vector)
        self.obs[0] = torch.tensor(base_lin_vel[0] * 2.0)
        self.obs[1] = torch.tensor(base_lin_vel[1] * 2.0)
        self.obs[2] = torch.tensor(base_lin_vel[2] * 2.0)

        base_ang_vel = quat_rotate_inverse(
            self.bodyImu.orientation,self.bodyImu.angular_velocity)
        self.obs[3] = torch.tensor(base_ang_vel[0] * 0.25)
        self.obs[4] = torch.tensor(base_ang_vel[1] * 0.25)
        self.obs[5] = torch.tensor(base_ang_vel[2] * 0.25)
        
        projected_gravity = quat_rotate_inverse(
            self.bodyImu.orientation,self.bodyImu.linear_acceleration)
        self.obs[6] = torch.tensor(projected_gravity[0])
        self.obs[7] = torch.tensor(projected_gravity[1])
        self.obs[8] = torch.tensor(projected_gravity[2])

        self.obs[9]  = torch.tensor(0.0)
        self.obs[10] = torch.tensor(0.0)
        self.obs[11] = torch.tensor(0.0)

        self.obs[12] = torch.tensor(self.leg.endp[0] * 0.001)
        self.obs[13] = torch.tensor(self.leg.endp[1] * 0.001)
        self.obs[14] = torch.tensor(
            (self.leg.endp[2] - self.leg.bias_z) * 0.001)
        
        self.obs[15] = torch.tensor(self.leg.endv[0] * 0.001 * 0.05)
        self.obs[16] = torch.tensor(self.leg.endv[1] * 0.001 * 0.05)
        self.obs[17] = torch.tensor(self.leg.endv[2] * 0.001 * 0.05)

        self.obs[18] = torch.tensor(self.leg.endp_tar[0])
        self.obs[19] = torch.tensor(self.leg.endp_tar[1])
        self.obs[20] = torch.tensor(
            (- self.leg.endp_tar[2] + self.leg.bias_z))
    
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
    rospy.init_node('network', anonymous=True)

    nw = NetworkInterface()
    print("Init Finish")
    nw.loop()

if __name__ == "__main__":
    main()