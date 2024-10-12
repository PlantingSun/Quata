import mujoco as mj
import numpy as np
from mujoco_base import MuJoCoBase
from mujoco.glfw import glfw
import rospy
import rospkg
from std_msgs.msg import String
from geometry_msgs.msg import Pose,Twist,Quaternion
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from mujoco_sim.msg import motor_data
import threading
import torch
import math
import time

def quat_to_euler(q):
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                           1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(2.0 * (q.w * q.y - q.z * q.x))
    return roll, pitch

class QuataSim(MuJoCoBase):
	motor_cmd = [motor_data() for _ in range(4)]	#store motor_cmd

	def __init__(self, xml_path):
		super().__init__(xml_path)
		self.simend = 1000.0

		totalMass = sum(self.model.body_mass)
		print('Total mass: ', totalMass)

		# * show the model
		mj.mj_step(self.model, self.data)
		# enable contact force visualization
		self.opt.flags[mj.mjtVisFlag.mjVIS_CONTACTFORCE] = True
		# get framebuffer viewport
		viewport_width, viewport_height = glfw.get_framebuffer_size(
			self.window)
		viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
		# Update scene and render
		mj.mjv_updateScene(self.model, self.data, self.opt, None, self.cam,
							mj.mjtCatBit.mjCAT_ALL.value, self.scene)
		mj.mjr_render(viewport, self.scene, self.context)

		#init
		self.bodyImu = Imu()
		self.bodydv = Vector3Stamped()
		self.bodyMotor = [motor_data(), motor_data(), motor_data()]
		for i in range(1,4):
			self.motor_cmd[i].pos_tar = 0.0
			self.motor_cmd[i].vel_tar = 0.0
			self.motor_cmd[i].tor_tar = 0.0
			self.motor_cmd[i].kp = 1500
			self.motor_cmd[i].kd = 15
                  
		" load network "       
		path = "./test3.pt"
		self.policy = torch.load(path).to(device="cpu")
		self.policy.eval()
		self.obs = torch.zeros(14)
		print('Load Network Finish')

	def get_sensor_data_and_publish(self):
		'''Get Imu data and Motor data, 
		then publish them to ros topic /bodyImu and /bodyMotor
		'''
		# 0~9 belongs to imu data: 0~3:quat, 4-6:gyro, 7-9:acc
		# 10~18 belongs to actuator data: 10~12:joint pos, 13~15:joint vel, 16~18:joint torque
		# quaternion format is: [w,x,y,z] -> (cos(t/2), sin(t/2)*x,sin(t/2)*y,sin*z)
		# publish Imu data
		self.bodyImu.orientation.w = self.data.sensor('BodyQuat').data[0].copy()
		self.bodyImu.orientation.x = self.data.sensor('BodyQuat').data[1].copy()
		self.bodyImu.orientation.y = self.data.sensor('BodyQuat').data[2].copy()
		self.bodyImu.orientation.z = self.data.sensor('BodyQuat').data[3].copy()
		self.bodyImu.angular_velocity.x = self.data.sensor('BodyGyro').data[0].copy()
		self.bodyImu.angular_velocity.y = self.data.sensor('BodyGyro').data[1].copy()
		self.bodyImu.angular_velocity.z = self.data.sensor('BodyGyro').data[2].copy()
		self.bodyImu.linear_acceleration.x = self.data.sensor('BodyAcc').data[0].copy()
		self.bodyImu.linear_acceleration.y = self.data.sensor('BodyAcc').data[1].copy()
		self.bodyImu.linear_acceleration.z = self.data.sensor('BodyAcc').data[2].copy()
		# publish dv data
		self.bodydv.vector.x = self.data.sensor('BodyVel').data[0].copy()
		self.bodydv.vector.y = self.data.sensor('BodyVel').data[1].copy()
		self.bodydv.vector.z = self.data.sensor('BodyVel').data[2].copy()
		# publish Motor data       ID :0~2 -> A B C
		# bodyMotor = [motor_data(), motor_data(), motor_data()]
		self.bodyMotor[0].id = 1
		self.bodyMotor[0].pos = self.data.sensor('JointXPos').data.copy()
		self.bodyMotor[0].vel = self.data.sensor('JointXVel').data.copy()
		self.bodyMotor[0].tor = self.motor_cmd[1].tor_tar
		self.bodyMotor[1].id = 2
		self.bodyMotor[1].pos = self.data.sensor('JointYPos').data.copy()
		self.bodyMotor[1].vel = self.data.sensor('JointYVel').data.copy()
		self.bodyMotor[1].tor = self.motor_cmd[2].tor_tar
		self.bodyMotor[2].id = 3
		self.bodyMotor[2].pos = self.data.sensor('JointZPos').data.copy()
		self.bodyMotor[2].vel = self.data.sensor('JointZVel').data.copy()
		self.bodyMotor[2].tor = self.motor_cmd[3].tor_tar	

	def apply_force(self):
		for i in range(1, 4):
			act = self.motor_cmd[i]
			P = act.kp * (act.pos_tar - self.bodyMotor[i - 1].pos)
			D = act.kd * (act.vel_tar - self.bodyMotor[i - 1].vel)
			self.data.ctrl[i - 1] = act.tor_tar + P + D
	
	def reset(self):
		# Set camera configuration
		self.cam.azimuth = 60
		self.cam.elevation = -15
		self.cam.distance = 1.5
		self.cam.lookat = np.array([0.0, 0.0, 0.5])
		#init motor_cmd 
		for i in range(1,4):
			self.motor_cmd[i].pos_tar = 0.0
			self.motor_cmd[i].vel_tar = 0.0
			self.motor_cmd[i].tor_tar = 0.0
			self.motor_cmd[i].kp = 1500
			self.motor_cmd[i].kd = 15
 
	def compute_endp(self, action):
		self.motor_cmd[1].pos_tar = action[0] * 0.075
		self.motor_cmd[2].pos_tar = action[1] * 0.075
		self.motor_cmd[3].pos_tar = action[2] * 0.075
	
	def compute_obervation(self):
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

	def simulate(self):
		while not glfw.window_should_close(self.window):
			simstart = self.data.time

			while (self.data.time - simstart <= 1.0/24.0 and not self.pause_flag):
				# get current absolute time 
				now = glfw.get_time()
				# Publish joint positions and velocities
				self.get_sensor_data_and_publish()

				# compute endp
				self.compute_obervation()
				action = self.policy(self.obs.detach()).detach().numpy()
				self.compute_endp(action)

				#apply force to motor
				self.apply_force()
				# Step simulation environment
				mj.mj_step(self.model, self.data)
				# a bug that mujoco runs slowly
				# self.apply_force()
				# mj.mj_step(self.model, self.data)
				# sleep until 1ms don't use rospy.Rate
				while (glfw.get_time() - now) < 0.001:
					pass
			
			if self.data.time >= self.simend:
				break

			# get framebuffer viewport
			viewport_width, viewport_height = glfw.get_framebuffer_size(
				self.window)
			viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
			# Update scene and render
			mj.mjv_updateScene(self.model, self.data, self.opt, None, self.cam,
								mj.mjtCatBit.mjCAT_ALL.value, self.scene)
			mj.mjr_render(viewport, self.scene, self.context)
			# swap OpenGL buffers (blocking call due to v-sync)
			glfw.swap_buffers(self.window)
			# process pending GUI events, call GLFW callbacks
			glfw.poll_events()

		glfw.terminate()

def main():
	# ros init
	rospy.init_node('RLSLIP_sim', anonymous=True)

	# get xml path
	rospack = rospkg.RosPack()
	rospack.list()
	quata_desc_path = rospack.get_path('quata_description')
	xml_path = quata_desc_path + "/RLSLIP_description/mjcf/RLSLIP.xml"
	sim = QuataSim(xml_path)
	sim.reset()
	sim.simulate()

if __name__ == "__main__":
    main()
