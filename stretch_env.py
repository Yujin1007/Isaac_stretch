
import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets import ArticulationCfg
from omni.isaac.lab.utils.assets import ISAACLAB_NUCLEUS_DIR

STRETCH_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="/home/portal/Isaac/stretch_modified.usda", # modified path
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=100.0,
            # enable_gyroscopic_forces=True,
            # kinematic_enabled=True,
            # disable_gravity=True,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
        ),
        collision_props=sim_utils.CollisionPropertiesCfg(
            collision_enabled=True,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(1.0, 0.0, 0.0), \
            joint_pos={"joint_right_wheel": 0.0, "joint_left_wheel": 0.0, "joint_lift": 0.0, \
                        "joint_arm_l3": 0.0, "joint_arm_l2": 0.0, "joint_arm_l1": 0.0, "joint_arm_l0": 0.0, \
                        "joint_wrist_yaw": 0.3, "joint_wrist_pitch": 0.0, "joint_wrist_roll": 0.0, \
                        "joint_gripper_finger_right": 0.0, "joint_gripper_finger_left": 0.0}
        
    ),
    actuators={
        "lift_actuator": ImplicitActuatorCfg(
            joint_names_expr=["joint_lift"],
            effort_limit=100.0,
            velocity_limit=1.0,
            stiffness=0.0,
            damping=0.0,
            armature=0.0,
            friction=0.0,
        ),
        "arm_actuator0": ImplicitActuatorCfg(
            joint_names_expr=["joint_arm_l0"],
            effort_limit=56.0,
            velocity_limit=1.0,
            stiffness=0.0,
            damping=0.0,
            armature=0.0,
            friction=0.0,
        ),"arm_actuator1": ImplicitActuatorCfg(
            joint_names_expr=["joint_arm_l1"],
            effort_limit=56.0,
            velocity_limit=1.0,
            stiffness=0.0,
            damping=0.0,
            armature=0.0,
            friction=0.0,
        ),"arm_actuator2": ImplicitActuatorCfg(
            joint_names_expr=["joint_arm_l2"],
            effort_limit=56.0,
            velocity_limit=1.0,
            stiffness=0.0,
            damping=0.0,
            armature=0.0,
            friction=0.0,
        ),"arm_actuator3": ImplicitActuatorCfg(
            joint_names_expr=["joint_arm_l3"],
            effort_limit=56.0,
            velocity_limit=1.0,
            stiffness=0.0,
            damping=0.0,
            armature=0.0,
            friction=0.0,
        ),
        "wrist_yaw_actuator": ImplicitActuatorCfg(
            joint_names_expr=["joint_wrist_yaw"],
            effort_limit=10.0,
            velocity_limit=1.0,
            stiffness=0.0,
            damping=0.0,
            armature=0.0,
            friction=0.0,
        ),
        "wrist_pitch_actuator": ImplicitActuatorCfg(
            joint_names_expr=["joint_wrist_pitch"],
            effort_limit=10.0,
            velocity_limit=1.0,
            stiffness=0.0,
            damping=0.0,
            armature=0.0,
            friction=0.0,
        ),
        "wrist_roll_actuator": ImplicitActuatorCfg(
            joint_names_expr=["joint_wrist_roll"],
            effort_limit=10.0,
            velocity_limit=1.0,
            stiffness=0.0,
            damping=0.0,
            armature=0.0,
            friction=0.0,
        ),
        "gripper_right_actuator": ImplicitActuatorCfg(
            joint_names_expr=["joint_gripper_finger_right"],
            effort_limit=10.0,
            velocity_limit=1.0,
            stiffness=0.0,
            damping=0.0,
            armature=0.0,
            friction=0.0,
        ),
        "gripper_left_actuator": ImplicitActuatorCfg(
            joint_names_expr=["joint_gripper_finger_left"],
            effort_limit=10.0,
            velocity_limit=1.0,
            stiffness=0.0,
            damping=0.0,
            armature=0.0,
            friction=0.0,
        ),
        "head_pan_actuator": ImplicitActuatorCfg(
            joint_names_expr=["joint_head_pan"],
            # damping=21.75,
            # friction=10.48,
            stiffness=0.0,
            damping=0.0,
            armature=0.0,
            friction=0.0,
            effort_limit=10.0,
            velocity_limit=1.0,
        ),"head_tilt_actuator": ImplicitActuatorCfg(
            joint_names_expr=["joint_head_tilt"],
            # damping=21.75,
            # friction=10.48,
            stiffness=0.0,
            damping=0.0,
            armature=0.0,
            friction=0.0,
            effort_limit=10.0,
            velocity_limit=1.0,
        ),
        "wheel_right_actuator": ImplicitActuatorCfg(
            joint_names_expr=["joint_right_wheel"],
            damping=21.75,
            friction=100.48,
            # damping=0.0,
            # friction=0.0,
            stiffness=0.0,
            armature=0.0,
            effort_limit=100.0,
            velocity_limit=1.0,
        ),
        "wheel_left_actuator": ImplicitActuatorCfg(
            joint_names_expr=["joint_left_wheel"],
            damping=21.75,
            friction=100.48,
            # damping=0.0,
            # friction=0.0,
            stiffness=0.0,
            armature=0.0,
            effort_limit=100.0,
            velocity_limit=1.0,
        ),
    },
)