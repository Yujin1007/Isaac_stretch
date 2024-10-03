# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause


import argparse

from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Tutorial on using the interactive scene interface.")
parser.add_argument("--num_envs", type=int, default=2, help="Number of environments to spawn.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import ArticulationCfg, AssetBaseCfg
from omni.isaac.lab.scene import InteractiveScene, InteractiveSceneCfg
from omni.isaac.lab.sim import SimulationContext
from omni.isaac.lab.utils import configclass

##
# Pre-defined configs
##
from omni.isaac.lab_assets import CARTPOLE_CFG  # isort:skip
from omni.isaac.lab_assets import FRANKA_PANDA_CFG
from stretch_env import STRETCH_CFG


@configclass
class CartpoleSceneCfg(InteractiveSceneCfg):
    """Configuration for a cart-pole scene."""

    # ground plane
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )

    # articulation
    # cartpole: ArticulationCfg = CARTPOLE_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    stretch: ArticulationCfg = STRETCH_CFG.replace(prim_path="{ENV_REGEX_NS}/Stretch")
    # franka: ArticulationCfg = FRANKA_PANDA_CFG.replace(prim_path="{ENV_REGEX_NS}/Franka")

def PD_controller(q, qd, count):
    # ID:  [2]  names: ['joint_lift']
    # ID:  [8]  names: ['joint_arm_l0']
    # ID:  [7]  names: ['joint_arm_l1']
    # ID:  [6]  names: ['joint_arm_l2']
    # ID:  [5]  names: ['joint_arm_l3']
    # ID:  [9]  names: ['joint_wrist_yaw']
    # ID:  [10]  names: ['joint_wrist_pitch']
    # ID:  [11]  names: ['joint_wrist_roll']
    # ID:  [13]  names: ['joint_gripper_finger_right']
    # ID:  [12]  names: ['joint_gripper_finger_left']
    # ID:  [0]  names: ['joint_head_pan']
    # ID:  [4]  names: ['joint_head_tilt']
    # ID:  [3]  names: ['joint_right_wheel']
    # ID:  [1]  names: ['joint_left_wheel']
    kp = torch.ones_like(q)
    kp[0][5:9] = 100
    kp[0][2] = 200
    kd = torch.ones_like(qd) * 0.1
    kd[0][5:9] = 50
    kd[0][2] = 100
    
    q_des = q.clone()
    qd_des = torch.zeros_like(qd)

    q_des[0][5:9] = 0.0
    q_des[0][2] = 0.3
    
    # efforts = kp * (q_des - q) + kd * (qd_des - qd)
    efforts = torch.mul(kp, (q_des - q)) + torch.mul(kd,(qd_des - qd))
    # Robot base should move forward. 
    efforts[0][1] = min(30, 0.1*(count))
    efforts[0][3] = min(30, 0.1*(count))

    print("effort lift:", efforts[0][2], "  efsfort arm", efforts[0][5:9])
    return efforts

def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    """Runs the simulation loop."""
    # Extract scene entities
    # note: we only do this here for readability.
    # robot = scene["cartpole"]
    stretch = scene["stretch"]
    # franka = scene["franka"]
    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    count = 0
    # Simulation loop
    while simulation_app.is_running():
        # Reset
        if count % 500 == 0:
            # reset counter
            count = 0
            
            stretch_state = stretch.data.default_root_state.clone()
            stretch_state[:, :3] += scene.env_origins
            stretch.write_root_state_to_sim(stretch_state)
            # set joint positions with some noise
            joint_pos, joint_vel = stretch.data.default_joint_pos.clone(), stretch.data.default_joint_vel.clone()
            # joint_pos += torch.rand_like(joint_pos) * 0.1
            stretch.write_joint_state_to_sim(joint_pos, joint_vel)

            # clear internal buffers
            scene.reset()
            print("[INFO]: Resetting robot state...")
        

        # target_position =stretch.data.default_root_state.clone()
        
        # print("q_arm:", torch.round(stretch.data.joint_pos[0][5:9], decimals=2))
        # print("q_lift:", torch.round(stretch.data.joint_pos[0][2], decimals=2))
        efforts = PD_controller(stretch.data.joint_pos, stretch.data.joint_vel, count)
        
        stretch.set_joint_effort_target(efforts)
        # print("wheel force", efforts[0][1])
        # target = torch.zeros_like(stretch.data.joint_pos)
        # target[0][2] = 0.6 / 500 * (count % 500) 
        # stretch.set_joint_position_target(target=target)
        # print("q lift ", stretch.data.joint_pos[0][2])
        
        scene.write_data_to_sim()
        # Perform step
        sim.step()
        # Increment counter
        count += 1
        # Update buffers
        scene.update(sim_dt)


def main():
    """Main function."""
    # Load kit helper
    sim_cfg = sim_utils.SimulationCfg(device="cpu")
    sim = SimulationContext(sim_cfg)
    # Set main camera
    sim.set_camera_view([4.5, 0.0, 1.0], [0.0, 0.0, 1.0])
    # Design scene, How many environment are you gonna use? 
    scene_cfg = CartpoleSceneCfg(num_envs=args_cli.num_envs, env_spacing=2.0)
    # Interactive Scene combines multiple environment in one scene
    scene = InteractiveScene(scene_cfg)
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")
    # Run the simulator
    run_simulator(sim, scene)


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()