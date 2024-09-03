from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class RLSLIPCfg( LeggedRobotCfg ):
    class env( LeggedRobotCfg.env):
        num_envs = 4096
        num_observations = 14
        num_actions = 3

    class terrain( LeggedRobotCfg.terrain ):
        mesh_type = 'plane'
        measure_heights = False

    class commands( LeggedRobotCfg.commands ):
        curriculum = False
        max_curriculum = 1.
        # num_commands = 4 # default: lin_vel_x, lin_vel_y, ang_vel_yaw, heading (in heading mode ang_vel_yaw is recomputed from heading error)
        num_commands = 4
        resampling_time = 10. # time before command are changed[s]
        heading_command = False # if true: compute ang vel command from heading error
        class ranges:
            lin_vel_x = [-0.25, 0.25] # min max [m/s]
            lin_vel_y = [-0.25, 0.25]   # min max [m/s]
            ang_vel_yaw = [-0.0, 0.0]    # min max [rad/s]
            heading = [-3.14, 3.14]

    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.297] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            'xdof_Joint': 0.,
            'ydof_Joint': 0.,
            'zdof_Joint': 0.,
        }

    class control( LeggedRobotCfg.control ):
        control_type = 'P'
        # PD Drive parameters:
        stiffness = {   'xdof': 25.0, 'ydof': 25.0, 'zdof': 25.0}  # [N*m/m]
        damping = {   'xdof':0.5, 'ydof': 0.5, 'zdof': 0.5}  # [N*m/m/s]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 1.0
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4
        
    class asset( LeggedRobotCfg.asset ):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/RLSLIP/urdf/RLSLIP.urdf'
        name = "RLSLIP"
        foot_name = 'zdof'
        terminate_after_contacts_on = ["base_link"]
        flip_visual_attachments = False
        self_collisions = 1 # 1 to disable, 0 to enable...bitwise filter
  
    class rewards( LeggedRobotCfg.rewards ):
        soft_dof_pos_limit = 0.95
        soft_dof_vel_limit = 0.9
        soft_torque_limit = 0.8
        max_contact_force = 300.
        only_positive_rewards = False
        class scales( LeggedRobotCfg.rewards.scales ):
            termination = -200.
            tracking_ang_vel = 0.0
            torques = -5.e-7
            dof_acc = -2.e-7
            feet_air_time = 0.
            dof_pos_limits = -1.
            dof_vel = -0.0
            ang_vel_xy = -0.1
            orientation = -0.6
            feet_contact_forces = -0.
            # encourage to jump
            lin_vel_z = 0.25
            base_height = 0.25
        
    # class noise:
    #     add_noise = True
    #     noise_level = 1.0 # scales other values
    #     class noise_scales:
    #         dof_pos = 0.01
    #         dof_vel = 1.5
    #         lin_vel = 0.1
    #         ang_vel = 0.2
    #         gravity = 0.05
    #         height_measurements = 0.1

class RLSLIPCfgPPO( LeggedRobotCfgPPO ):
    class policy( LeggedRobotCfgPPO.policy ):
        actor_hidden_dims = [256, 256, 128, 128]
        critic_hidden_dims = [256, 256, 128, 128]
        activation = 'elu' # can be elu, relu, selu, crelu, lrelu, tanh, sigmoid

    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        experiment_name = 'RLSLIP'
        max_iterations = 1000

  