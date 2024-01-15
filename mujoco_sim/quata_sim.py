import mujoco as mj
import numpy as np
from mujoco_base import MuJoCoBase
from mujoco.glfw import glfw
import rospy
import rospkg
from std_msgs.msg import Float32MultiArray,Bool
from geometry_msgs.msg import Pose,Twist,Quaternion
from sensor_msgs.msg import Imu
from mujoco_sim.msg import motor_data
import threading


class QuataSim(MuJoCoBase):
	def __init__(self, xml_path):
		super().__init__(xml_path)
		self.simend = 1000.0
		# print('Total number of DoFs in the model:', self.model.nv)
		# print('Generalized positions:', self.data.qpos)  
		# print('Generalized velocities:', self.data.qvel)
		# print('Actuator forces:', self.data.qfrc_actuator)
		# print('Actoator controls:', self.data.ctrl)
		# mj.set_mjcb_control(self.controller)
		# Set initial joint positions
		self.data.qpos[-10:] = np.array([ 0.0, 0.0, -0.52, 1.04, -0.52, 
		                                 0.0, 0.0, 0.52, -1.04, 0.52])
		# self.data.qpos[1] = np.array(-1)
		# self.data.qpos[7] = np.array(-1)
		# self.data.qpos[12] = np.array(-1)

		totalMass = sum(self.model.body_mass)
		print('Total mass: ', totalMass)

		# * Set subscriber and publisher
		self.pubJoints = rospy.Publisher('/jointsPosVel', Float32MultiArray, queue_size=10)
		self.pubPose = rospy.Publisher('/bodyPose', Pose, queue_size=10)
		self.pubTwist = rospy.Publisher('/bodyTwist', Twist, queue_size=10)

		self.pubImu = rospy.Publisher('/bodyImu', Imu, queue_size=10)
		self.pubMotor = rospy.Publisher('/bodyMotor', motor_data, queue_size=10)

		# subscribe joints torque and position
		rospy.Subscriber("/jointsTorque", Float32MultiArray, self.controlCallback) 
		rospy.Subscriber("/jointsCmd", motor_data, self.publishMotorCmd) 
		# rospy.Subscriber("/jointsTorque", Float32MultiArray, self.controlCallback) 
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

	# def create_overlay(model, data):
	# 	bottomLeft = mj.mjtGridPos.mjGRID_BOTTOMLEFT
	# 	add_overlay(bottomLeft,"Restart","Backspace",)
	# 	add_overlay(bottomLeft,"Start simulation","space",)

	def publishMotorCmd(self, data):
		
	def controlCallback(self, data):
		d = list(data.data[:])
		self.data.ctrl[:] = d
  
	def reset(self):
		# Set camera configuration
		self.cam.azimuth = 89.608063
		self.cam.elevation = -11.588379
		self.cam.distance = 5.0
		self.cam.lookat = np.array([0.0, 0.0, 1.5])
	
	# def controller(self, model, data):
  	#   	self.data.ctrl[0] = 100
  	#   	pass


	def get_sensor_data_and_publish(self):
		'''Get Imu data and Motor data, 
		then publish them to ros topic /bodyImu and /bodyMotor
		'''

		# 0~9 belongs to imu data: 0~3:quat, 4-6:gyro, 7-9:acc
		# 10~18 belongs to actuator data: 10~12:joint pos, 13~15:joint vel, 16~18:joint torque
		# quaternion format is: [w,x,y,z] -> (cos(t/2), sin(t/2)*x,sin(t/2)*y,sin*z)

		# Todo: use motor_msg to tranform motor msg
		# publish Imu data
		bodyImu = Imu()
		bodyImu.orientation.w = self.data.sensor('BodyQuat').data[0].copy()
		bodyImu.orientation.x = self.data.sensor('BodyQuat').data[1].copy()
		bodyImu.orientation.y = self.data.sensor('BodyQuat').data[2].copy()
		bodyImu.orientation.z = self.data.sensor('BodyQuat').data[3].copy()
		bodyImu.angular_velocity.x = self.data.sensor('BodyGyro').data[0].copy()
		bodyImu.angular_velocity.y = self.data.sensor('BodyGyro').data[1].copy()
		bodyImu.angular_velocity.z = self.data.sensor('BodyGyro').data[2].copy()
		bodyImu.linear_acceleration.x = self.data.sensor('BodyAcc').data[0].copy()
		bodyImu.linear_acceleration.y = self.data.sensor('BodyAcc').data[1].copy()
		bodyImu.linear_acceleration.z = self.data.sensor('BodyAcc').data[2].copy()
		self.pubImu.publish(bodyImu)

		# publish Motor data       ID :0~2 -> A B C
		bodyMotor = [motor_data(), motor_data(), motor_data()]
		bodyMotor[0].id = 0
		bodyMotor[0].pos = self.data.sensor('JointAPos').data.copy()
		bodyMotor[0].vel = self.data.sensor('JointAVel').data.copy()
		bodyMotor[0].tor = self.data.sensor('JointATor').data.copy()
		bodyMotor[1].id = 1
		bodyMotor[1].pos = self.data.sensor('JointBPos').data.copy()
		bodyMotor[1].vel = self.data.sensor('JointBVel').data.copy()
		bodyMotor[1].tor = self.data.sensor('JointBTor').data.copy()
		bodyMotor[2].id = 2
		bodyMotor[2].pos = self.data.sensor('JointCPos').data.copy()
		bodyMotor[2].vel = self.data.sensor('JointCVel').data.copy()
		bodyMotor[2].tor = self.data.sensor('JointCTor').data.copy()
		self.pubMotor.publish(bodyMotor[0])
		self.pubMotor.publish(bodyMotor[1])
		self.pubMotor.publish(bodyMotor[2])

		# * Publish body twist
		# bodyTwist = Twist()
		# vel = self.data.sensor('BodyVel').data.copy()
		
		# # * get body velocity in world frame
		# vel = self.data.qvel[:3].copy()
		# angVel = self.data.sensor('BodyGyro').data.copy()
		# bodyTwist.linear.x = vel[0]
		# bodyTwist.linear.y = vel[1]
		# bodyTwist.linear.z = vel[2]
		# bodyTwist.angular.x = angVel[0]
		# bodyTwist.angular.y = angVel[1]
		# bodyTwist.angular.z = angVel[2]
		# self.pubTwist.publish(bodyTwist)
		


	# These three functions are used to test ROS data
	def info_callback(self, msg):
		print(msg)

	#init a ros subscriber to get sensor data
	def get_sensor_data(self):
		# rospy.init_node('data_subscriber', anonymous=True)
		rospy.Subscriber("/bodyImu", Imu, self.info_callback)
		rospy.Subscriber("/bodyMotor", motor_data, self.info_callback)
		rospy.spin()

	def test_ros_publish(self):
		talk_theread = threading.Thread(target=self.get_sensor_data_and_publish)
		listen_theread = threading.Thread(target=self.get_sensor_data)
		talk_theread.start()
		listen_theread.start()


	def simulate(self):
		print("-----------------------")

		# self.test_ros_publish()
		# input()
		while not glfw.window_should_close(self.window):
			simstart = self.data.time
			print("simstart:",simstart)
			while (self.data.time - simstart <= 1.0/60.0 and not self.pause_flag):
				# get current absolute time 
				now = glfw.get_time()			
				# Step simulation environment
				mj.mj_step(self.model, self.data)
		
				# * Publish joint positions and velocities
				self.get_sensor_data_and_publish()
				

				# sleep untile 2ms don't use rospy.Rate
				while (glfw.get_time() - now) < 0.00099:
					pass
			
			print("curent time:",self.data.time)
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
	rospy.init_node('pai_sim', anonymous=True)

	# get xml path
	rospack = rospkg.RosPack()
	rospack.list()
	hector_desc_path = rospack.get_path('quata_description')
	xml_path = hector_desc_path + "/mjcf/quata.xml"
	sim = QuataSim(xml_path)
	sim.reset()
	sim.simulate()

if __name__ == "__main__":
    main()
