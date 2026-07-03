"""M1 verification: load the converted Aloha USD and exercise every actuated joint.

Sweeps each arm + lift joint through a small sinusoid and prints joint names so we
can confirm Isaac picked up the full articulation. Run with the GUI to watch:

    source ~/dev/isaac/env_isaaclab/bin/activate
    python isaac/m1_verify.py
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="M1 verify converted aloha USD")
parser.add_argument("--steps", type=int, default=0, help="Run N steps then exit (0 = run until window closed).")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.actuators import ImplicitActuatorCfg  # noqa: E402
from isaaclab.assets import Articulation, ArticulationCfg  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402


REPO_ROOT = Path(__file__).resolve().parent.parent
USD_PATH = REPO_ROOT / "isaac" / "assets" / "aloha.usd"


ALOHA_CFG = ArticulationCfg(
    prim_path="/World/Aloha",
    spawn=sim_utils.UsdFileCfg(
        usd_path=str(USD_PATH),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=12,
            solver_velocity_iteration_count=1,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.05)),
    actuators={
        "arms_and_lift": ImplicitActuatorCfg(
            joint_names_expr=["(left|right)_joint[1-6]", "vertical_.*"],
            stiffness=200.0,
            damping=20.0,
        ),
        "wheels": ImplicitActuatorCfg(
            joint_names_expr=["wheel.*"],
            stiffness=0.0,
            damping=2.0,
        ),
    },
)


def design_scene():
    sim_utils.GroundPlaneCfg().func("/World/Ground", sim_utils.GroundPlaneCfg())
    sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)).func(
        "/World/Light", sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )
    robot = Articulation(cfg=ALOHA_CFG)
    return robot


def main():
    sim = SimulationContext(sim_utils.SimulationCfg(device=args_cli.device, dt=1.0 / 120.0))
    sim.set_camera_view(eye=[1.8, 1.8, 1.2], target=[0.0, 0.0, 0.4])

    robot = design_scene()
    sim.reset()

    print("\n=== Articulation summary ===")
    print(f"  num_joints: {robot.num_joints}")
    for i, name in enumerate(robot.joint_names):
        print(f"    [{i:2d}] {name}")
    print("=============================\n")

    default_pos = robot.data.default_joint_pos.clone()
    arm_lift_idx = [
        i
        for i, n in enumerate(robot.joint_names)
        if n.startswith(("left_joint", "right_joint")) or n.startswith("vertical")
    ]

    t = 0.0
    dt = sim.get_physics_dt()
    step = 0

    while simulation_app.is_running():
        if args_cli.steps and step >= args_cli.steps:
            break
        step += 1
        targets = default_pos.clone()
        amp = 0.3
        for i in arm_lift_idx:
            targets[:, i] = default_pos[:, i] + amp * math.sin(t + 0.2 * i)
        robot.set_joint_position_target(targets)
        robot.write_data_to_sim()
        sim.step()
        robot.update(dt)
        t += dt


if __name__ == "__main__":
    main()
    simulation_app.close()
