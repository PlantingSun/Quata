from ossaudiodev import control_labels
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


class QuataSim(MuJoCoBase):
	motor_cmd = [motor_data() for _ in range(4)]	#store motor_cmd

	def __init__(self, xml_path):
		super().__init__(xml_path)
		self.simend = 1000.0

		totalMass = sum(self.model.body_mass)
		print('Total mass: ', totalMass)

		# * Set subscriber and publisher
		self.pubImu = rospy.Publisher('/imu/data', Imu, queue_size=1)
		self.pubdv = rospy.Publisher('/imu/dv', Vector3Stamped, queue_size=1)
		self.pubMotor = rospy.Publisher('/cybergear_msgs', motor_data, queue_size=6)
		rospy.Subscriber("/cybergear_cmds", motor_data, self.run_motor_callback, queue_size=10)
		self.pubPause = rospy.Publisher('/pause', String, queue_size=1)
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

	
	def run_motor_callback(self, msg):
		'''subscribe motor command data and apply it to the model
			id[0]->qpos[1], id[1]->qpos[7], id[2]->qpos[12]
     	'''
		# print("Python heard")
		id = msg.id
		self.motor_cmd[id] = msg

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
		self.pubImu.publish(self.bodyImu)

		# publish dv data
		self.bodydv.vector.x = self.data.sensor('BodyVel').data[0].copy()
		self.bodydv.vector.y = self.data.sensor('BodyVel').data[1].copy()
		self.bodydv.vector.z = self.data.sensor('BodyVel').data[2].copy()
		self.pubdv.publish(self.bodydv)

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
		self.pubMotor.publish(self.bodyMotor[0])
		self.pubMotor.publish(self.bodyMotor[1])
		self.pubMotor.publish(self.bodyMotor[2])

		# publish Pause data
		self.pubPause.publish('0')		

	def apply_force(self):
		for i in range(1, 4):
			msg = self.motor_cmd[i]
			self.data.ctrl[i - 1] = msg.tor_tar +\
			msg.kp*(msg.pos_tar - self.bodyMotor[i - 1].pos) +\
			msg.kd*(msg.vel_tar - self.bodyMotor[i - 1].vel)
	
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
 
	def simulate(self):
		while not glfw.window_should_close(self.window):
			simstart = self.data.time

			if self.pause_flag:
				self.pubPause.publish('1')

			while (self.data.time - simstart <= 1.0/24.0 and not self.pause_flag):
				# get current absolute time 
				now = glfw.get_time()

				# Publish joint positions and velocities
				self.get_sensor_data_and_publish()

				#apply force to motor
				self.apply_force()

				# Step simulation environment
				mj.mj_step(self.model, self.data)

				# a bug that mujoco runs slowly
				self.apply_force()
				mj.mj_step(self.model, self.data)
				# print("ground force:", self.data.sensor('touchSensor').data.copy())
		
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
