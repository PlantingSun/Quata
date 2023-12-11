import mujoco as mj
import numpy as np
from mujoco_base import MuJoCoBase
from mujoco.glfw import glfw
import rospy
import rospkg
from std_msgs.msg import Float32MultiArray,Bool
from geometry_msgs.msg import Pose,Twist


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

    totalMass = sum(self.model.body_mass)
    print('total mass: ', totalMass)
    # * Set subscriber and publisher
    self.pubJoints = rospy.Publisher('/jointsPosVel', Float32MultiArray, queue_size=10)
    self.pubPose = rospy.Publisher('/bodyPose', Pose, queue_size=10)
    self.pubTwist = rospy.Publisher('/bodyTwist', Twist, queue_size=10)
    rospy.Subscriber("/jointsTorque", Float32MultiArray, self.controlCallback) 
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
  #   self.data.ctrl[0] = 100
  #   pass

  def simulate(self):
    while not glfw.window_should_close(self.window):
      simstart = self.data.time

      while (self.data.time - simstart <= 1.0/60.0 and not self.pause_flag):
        # get current absolute time 
        now = glfw.get_time()
        # Step simulation environment
        mj.mj_step(self.model, self.data)
        # * Publish joint positions and velocities
        jointsPosVel = Float32MultiArray()
        # get last 10 element of qpos and qvel
        qp = self.data.qpos[-10:].copy()
        qv = self.data.qvel[-10:].copy()
        jointsPosVel.data = np.concatenate((qp,qv))

        self.pubJoints.publish(jointsPosVel)
        # # * Publish body pose
        # bodyPose = Pose()
        # pos = self.data.sensor('BodyPos').data.copy()
        # ori = self.data.sensor('BodyQuat').data.copy()
        # # pos = self.data.qpos[:3].copy()
        # # ori = self.data.qpos[3:7].copy()
        # bodyPose.position.x = pos[0]
        # bodyPose.position.y = pos[1]
        # bodyPose.position.z = pos[2]
        # bodyPose.orientation.x = ori[1]
        # bodyPose.orientation.y = ori[2]
        # bodyPose.orientation.z = ori[3]
        # bodyPose.orientation.w = ori[0]
        # self.pubPose.publish(bodyPose)
        # # * Publish body twist
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
