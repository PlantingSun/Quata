from ossaudiodev import control_labels
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
	motor_cmd = [motor_data() for _ in range(4)]	#store motor_cmd
	map_id_to_motor = [0, "A", "B", "C"]

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
		# self.data.qpos[-10:] = np.array([ 0.0, 0.0, -0.52, 1.04, -0.52, 
		# 									0.0, 0.0, 0.52, -1.04, 0.52])
		# self.data.qpos[1] = np.array(-1)
		# self.data.qpos[7] = np.array(-1)
		# self.data.qpos[12] = np.array(-1)

		totalMass = sum(self.model.body_mass)
		print('Total mass: ', totalMass)

		# * Set subscriber and publisher
		# self.pubJoints = rospy.Publisher('/jointsPosVel', Float32MultiArray, queue_size=10)
		# self.pubPose = rospy.Publisher('/bodyPose', Pose, queue_size=10)
		# self.pubTwist = rospy.Publisher('/bodyTwist', Twist, queue_size=10)

		self.pubImu = rospy.Publisher('/bodyImu', Imu, queue_size=10)
		# self.pubMotor = rospy.Publisher('/bodyMotor', motor_data, queue_size=10)
		self.pubMotor = rospy.Publisher('/cybergear_msgs', motor_data, queue_size=5)


		# subscribe joints torque and position
		# rospy.Subscriber("/jointsTorque", Float32MultiArray, self.controlCallback) 
		rospy.Subscriber("/cybergear_cmds", motor_data, self.run_motor_callback, queue_size=10)
		# rospy.Subscriber("/jointsTorque", Float32MultiArray, self.controlCallback) 
		# rospy.Subscriber("/jointCmd", motor_data, self.run_motor_callback)
		# rospy.Subscriber("/jointsTorque", Float32MultiArray, self.controlCallback) 
		listen_theread = threading.Thread(target=self.start_subscribe)
		listen_theread.start()
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

		#init motor_cmd 
		for i in range(1,4):
			self.motor_cmd[i].pos_tar = 1.0
			self.motor_cmd[i].vel_tar = 0.0
			self.motor_cmd[i].tor_tar = 0.0
			self.motor_cmd[i].kp = 0
			self.motor_cmd[i].kd = 0

 
	def controlCallback(self, data):
		d = list(data.data[:])
		self.data.ctrl[:] = d
	
	def run_motor_callback(self, msg):
		'''subscribe motor command data and apply it to the model
			id[0]->qpos[1], id[1]->qpos[7], id[2]->qpos[12]
     	'''
		# print("Python heard")
		id = msg.id
		self.motor_cmd[id] = msg
	
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
		bodyMotor[0].id = 1
		bodyMotor[0].pos = self.data.sensor('JointAPos').data.copy()
		bodyMotor[0].vel = self.data.sensor('JointAVel').data.copy()
		bodyMotor[0].tor = self.data.sensor('JointATor').data.copy()
		bodyMotor[1].id = 2
		bodyMotor[1].pos = self.data.sensor('JointBPos').data.copy()
		bodyMotor[1].vel = self.data.sensor('JointBVel').data.copy()
		bodyMotor[1].tor = self.data.sensor('JointBTor').data.copy()
		bodyMotor[2].id = 3
		bodyMotor[2].pos = self.data.sensor('JointCPos').data.copy()
		bodyMotor[2].vel = self.data.sensor('JointCVel').data.copy()
		bodyMotor[2].tor = self.data.sensor('JointCTor').data.copy()
		self.pubMotor.publish(bodyMotor[0])
		self.pubMotor.publish(bodyMotor[1])
		self.pubMotor.publish(bodyMotor[2])

		# if self.data.ncon > 0:
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

	# def create_overlay(model, data):
	# 	bottomLeft = mj.mjtGridPos.mjGRID_BOTTOMLEFT
	# 	add_overlay(bottomLeft,"Restart","Backspace",)
	# 	add_overlay(bottomLeft,"Start simulation","space",)

	# def publishMotorCmd(self, data):
  
	def start_subscribe(self):
		rospy.Subscriber("/cybergear_cmds", motor_data, self.run_motor_callback)
		rospy.spin()
	
	def controller_test(self, data):
		f = 5
		data.qfrc_applied[1] = f
		data.qfrc_applied[7] = f
		data.qfrc_applied[12] = f

	def apply_force(self):
		for i in range(1, 4):
			msg = self.motor_cmd[i]
			kp = msg.kp
			kd = msg.kd
			motor_id = self.data.joint('JointupperLeg'+self.map_id_to_motor[i]).id
			self.data.qfrc_applied[motor_id] = msg.tor_tar +\
			kp*(msg.pos_tar - self.data.qpos[motor_id]) +\
			kd*(msg.vel_tar - self.data.qvel[motor_id])
			# print("motor_id:", motor_id, "force",self.data.qfrc_applied[motor_id])

	
	
	def reset(self):
		# Set camera configuration
		self.cam.azimuth = 89.608063
		self.cam.elevation = -11.588379
		self.cam.distance = 5.0
		self.cam.lookat = np.array([0.0, 0.0, 1.5])
		#init motor_cmd 
		for i in range(1,4):
			self.motor_cmd[i].pos_tar = 0
			self.motor_cmd[i].vel_tar = 0.0
			self.motor_cmd[i].tor_tar = 0.0
			self.motor_cmd[i].kp = 5
			self.motor_cmd[i].kd = 0.05
		
	
	def controller(self,data):
		self.get_max_geight(self.data)
        #velocity control PD
		tar_pos = 0
		cur_pos = data.qpos[1]
		kp_pos = 0.2
		
		tar_vel = 0
		cur_vel = data.qvel[1]
		kp_vel = 0.2

		pos_tar = kp_vel*(cur_vel - tar_vel) + kp_pos*(cur_pos - tar_pos)
		if pos_tar < -1.2:
			pos_tar = -1.2
		if pos_tar > 1.5:
			pos_tar = 1.5
		self.motor_cmd[1].pos_tar = pos_tar

		#height control	(could use energy)
		if self.top_height == -1:
			return
		tar_height = 2
		kp_height = 20
		tar_kp = kp_height*(tar_height - self.top_height) + 13.5
		# print("tar_kp:", tar_kp)
		if tar_kp < 10:
			tar_kp = 10
		if tar_kp > 100:
			tar_kp = 100
		for i in range(1,4):
			self.motor_cmd[i].kp = tar_kp

	
	_increse = 0
	top_height = -1
	last = 0
	def get_max_geight(self,data):
		# print("position of base body:",data.xpos[1])
		#use current max_height renew max_height only when current_height is become larger
		if data.xpos[1][2] > self.last:
			self._increse = 1
		else:
			if self._increse == 1:
				self.top_height = data.xpos[1][2]
				self._increse = 0
				print("max height:", self.top_height)
				print("kp:", self.motor_cmd[1].kp)

		self.last = data.xpos[1][2]
		
 
 
	def simulate(self):
		print("close the following lines to delete intiate pos and vel")
		self.data.qpos[1] = -2	#init x position
		self.data.qvel[1] = -1	#init x velocity

		
		
		while not glfw.window_should_close(self.window):
			simstart = self.data.time

			while (self.data.time - simstart <= 1.0/60.0 and not self.pause_flag):
				# get current absolute time 
				now = glfw.get_time()		
				
#*********************open this to run controller***********************
				self.controller(self.data)


				#apply force to motor
				self.apply_force()

				# print("ground force:", self.data.sensor('touchSensor').data.copy())
				# Step simulation environment
				mj.mj_step(self.model, self.data)
		
				# * Publish joint positions and velocities
				self.get_sensor_data_and_publish()
					

				# sleep untile 2ms don't use rospy.Rate
				while (glfw.get_time() - now) < 0.00099:
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
