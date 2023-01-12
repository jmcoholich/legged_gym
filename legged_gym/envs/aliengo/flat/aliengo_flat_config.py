

from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class AliengoFlatCfg(LeggedRobotCfg):
    class init_state(LeggedRobotCfg.init_state):
        pos = [0.0, 0.0, 0.48]  # x,y,z [m]
        # pos = [0.0, 0.0, 1.48]  # x,y,z [m]
        # pos = [0.0, 0.0, 10.0]  # x,y,z [m]
        # rot = [0.707, 0, 0, 0.707]
        # default_joint_angles = {  # = target angles [rad] when action = 0.0
        #     'FL_hip_joint': 0.1,   # [rad]
        #     'RL_hip_joint': 0.1,   # [rad]
        #     'FR_hip_joint': -0.1 ,  # [rad]
        #     'RR_hip_joint': -0.1,   # [rad]

        #     'FL_thigh_joint': 0.8,     # [rad]
        #     'RL_thigh_joint': 1.,   # [rad]
        #     'FR_thigh_joint': 0.8,     # [rad]
        #     'RR_thigh_joint': 1.,   # [rad]

        #     'FL_calf_joint': -1.5,   # [rad]
        #     'RL_calf_joint': -1.5,    # [rad]
        #     'FR_calf_joint': -1.5,  # [rad]
        #     'RR_calf_joint': -1.5,    # [rad]
        # }
        hip = 0.037199
        thigh = 0.660252
        knee = -1.200187
        # self.reset_joint_angles = torch.tensor(
        #     [hip, thigh, knee,
        #      -hip, thigh, knee,
        #      hip, thigh, knee,
        #      -hip, thigh, knee],
        #     device=self.device)
        default_joint_angles = {  # = target angles [rad] when action = 0.0
            'FL_hip_joint': hip,   # [rad]
            'RL_hip_joint': hip,   # [rad]
            'FR_hip_joint': -hip,  # [rad]
            'RR_hip_joint': -hip,   # [rad]

            'FL_thigh_joint': thigh,     # [rad]
            'RL_thigh_joint': thigh,   # [rad]
            'FR_thigh_joint': thigh,     # [rad]
            'RR_thigh_joint': thigh,   # [rad]

            'FL_calf_joint': knee,   # [rad]
            'RL_calf_joint': knee,    # [rad]
            'FR_calf_joint': knee,  # [rad]
            'RR_calf_joint': knee,    # [rad]
        }

    class env(LeggedRobotCfg.env):
        env_spacing = 0.75  # not used with heightfields/trimeshes 
        num_observations = 48

    class terrain(LeggedRobotCfg.terrain):
        # mesh_type = "plane" # "heightfield" # none, plane, heightfield or trimesh
        mesh_type = "trimesh" # "heightfield" # none, plane, heightfield or trimesh
        terrain_kwargs = {'type': 'random_uniform_terrain', 'terrain_kwargs': {'min_height': -0.05, 'max_height': 0.05, 'step': 0.005, 'downsampled_scale':0.2}} # Dict of arguments for selected terrain
        selected = True # select a unique terrain type and pass all arguments
        num_rows= 1 # number of terrain rows (levels)
        num_cols = 1 # number of terrain cols (types)
        horizontal_scale = 0.1 # [m]
        vertical_scale = 0.005 # [m]
        curriculum = False
        terrain_length = 32
        terrain_width = 32
        measure_heights = False

    # viewer camera:
    class viewer(LeggedRobotCfg.viewer):
        ref_env = 0
        pos = [0, 3, 1.5]  # [m]
        lookat = [0, 0, 0.25]  # [m]

    class control(LeggedRobotCfg.control):
        # PD Drive parameters:
        control_type = 'P'
        stiffness = {'joint': 100.}  # [N*m/rad]
        damping = {'joint': 0.5}     # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.15
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4

    class asset(LeggedRobotCfg.asset):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/aliengo/urdf/aliengo.urdf'
        # flip_visual_attachments = False

        name = "Aliengo"
        foot_name = "foot"
        penalize_contacts_on = ["thigh", "calf"]
        terminate_after_contacts_on = ["base", "trunk", "thigh", "calf"]
        # terminate_after_contacts_on = ["base", "trunk"]
        self_collisions = 0  # 1 to disable, 0 to enable...bitwise filter
        # fix_base_link = True
        # disable_gravity = True
    

    class rewards(LeggedRobotCfg.rewards):
        soft_dof_pos_limit = 0.9
        base_height_target = 0.35
        max_contact_force = 500 
        # only_positive_rewards = False

        class scales(LeggedRobotCfg.rewards.scales):
            torques = -0.000002
            dof_pos_limits = -10.0
            feet_air_time = 0.5
            # orientation = -5.0

class AliengoFlatCfgPPO(LeggedRobotCfgPPO):
    class algorithm(LeggedRobotCfgPPO.algorithm):
        entropy_coef = 0.01

    class runner(LeggedRobotCfgPPO.runner):
        run_name = ''
        experiment_name = 'flat_aliengo'
        max_iterations = 15000

