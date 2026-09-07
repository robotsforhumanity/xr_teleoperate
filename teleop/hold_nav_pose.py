#!/usr/bin/env python3
"""Hold G1 arms + BrainCo hands in a fixed pose so you can test locomotion.

Default is motion mode (rt/arm_sdk): the walking policy keeps the legs,
this script only overlays arms/hands. The robot must already be in AI /
regular loco mode (not debug).

Usage:
    python hold_nav_pose.py
    python hold_nav_pose.py --config config/nav_pose.yaml
    python hold_nav_pose.py --print-current
    python hold_nav_pose.py --save-current config/nav_pose.yaml

Edit teleop/config/nav_pose.yaml to change the default posture.
Ctrl+C releases arm_sdk weight and stops hand commands.
"""

import argparse
import os
import sys
import time

import numpy as np
import yaml

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_

from teleop.robot_control.robot_arm import G1_29_ArmController, G1_29_JointIndex
from teleop.robot_control.robot_hand_brainco import (
    Brainco_Left_Hand_JointIndex,
    Brainco_Right_Hand_JointIndex,
    _fill_hand_cmds,
    brainco_Num_Motors,
    kTopicbraincoLeftCommand,
    kTopicbraincoLeftState,
    kTopicbraincoRightCommand,
    kTopicbraincoRightState,
)

import logging_mp

logging_mp.basicConfig(level=logging_mp.INFO)
logger = logging_mp.getLogger(__name__)

ARM_NAMES = [
    "shoulder_pitch", "shoulder_roll", "shoulder_yaw",
    "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw",
]
HAND_NAMES = ["thumb", "thumb_aux", "index", "middle", "ring", "pinky"]


def load_pose(path):
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}
    left_arm = np.array(cfg["left_arm"], dtype=float)
    right_arm = np.array(cfg["right_arm"], dtype=float)
    left_hand = np.clip(np.array(cfg["left_hand"], dtype=float), 0.0, 1.0)
    right_hand = np.clip(np.array(cfg["right_hand"], dtype=float), 0.0, 1.0)
    if left_arm.size != 7 or right_arm.size != 7:
        raise ValueError("left_arm / right_arm must have 7 values")
    if left_hand.size != 6 or right_hand.size != 6:
        raise ValueError("left_hand / right_hand must have 6 values")
    move_time = float(cfg.get("move_time", 3.0))
    return np.concatenate([left_arm, right_arm]), left_hand, right_hand, move_time


def dump_pose(arm_q, left_hand, right_hand, move_time=3.0):
    return {
        "move_time": move_time,
        "left_arm": [round(float(v), 4) for v in arm_q[:7]],
        "right_arm": [round(float(v), 4) for v in arm_q[7:]],
        "left_hand": [round(float(v), 4) for v in left_hand],
        "right_hand": [round(float(v), 4) for v in right_hand],
    }


def log_vec(label, names, values):
    pretty = ", ".join(f"{n}={v:.3f}" for n, v in zip(names, values))
    logger.info("%s: %s", label, pretty)


class BraincoHold:
    def __init__(self):
        self.left_pub = ChannelPublisher(kTopicbraincoLeftCommand, MotorCmds_)
        self.left_pub.Init()
        self.right_pub = ChannelPublisher(kTopicbraincoRightCommand, MotorCmds_)
        self.right_pub.Init()
        self.left_sub = ChannelSubscriber(kTopicbraincoLeftState, MotorStates_)
        self.left_sub.Init()
        self.right_sub = ChannelSubscriber(kTopicbraincoRightState, MotorStates_)
        self.right_sub.Init()

        self.left_msg = MotorCmds_()
        self.left_msg.cmds = [unitree_go_msg_dds__MotorCmd_() for _ in range(brainco_Num_Motors)]
        self.right_msg = MotorCmds_()
        self.right_msg.cmds = [unitree_go_msg_dds__MotorCmd_() for _ in range(brainco_Num_Motors)]

    def read_state(self, timeout=2.0):
        left = np.zeros(brainco_Num_Motors)
        right = np.zeros(brainco_Num_Motors)
        got_l = got_r = False
        t0 = time.time()
        while time.time() - t0 < timeout and not (got_l and got_r):
            left_msg = self.left_sub.Read()
            right_msg = self.right_sub.Read()
            if left_msg is not None:
                for idx, jid in enumerate(Brainco_Left_Hand_JointIndex):
                    left[idx] = left_msg.states[jid].q
                got_l = True
            if right_msg is not None:
                for idx, jid in enumerate(Brainco_Right_Hand_JointIndex):
                    right[idx] = right_msg.states[jid].q
                got_r = True
            time.sleep(0.02)
        if not (got_l and got_r):
            logger.warning(
                "No llegó estado DDS de BrainCo. ¿Está corriendo brainco_hand_server?"
            )
        return left, right

    def send(self, left_q, right_q):
        _fill_hand_cmds(self.left_msg, Brainco_Left_Hand_JointIndex, left_q)
        _fill_hand_cmds(self.right_msg, Brainco_Right_Hand_JointIndex, right_q)
        self.left_pub.Write(self.left_msg)
        self.right_pub.Write(self.right_msg)


def interpolate(start, goal, t, duration):
    if duration <= 0:
        return goal
    a = min(1.0, max(0.0, t / duration))
    a = 0.5 - 0.5 * np.cos(np.pi * a)
    return (1.0 - a) * start + a * goal


def release_arm_sdk(arm_ctrl):
    if not arm_ctrl.motion_mode:
        return
    logger.info("Soltando overlay de brazos (arm_sdk weight → 0)...")
    for weight in np.linspace(1.0, 0.0, num=51):
        arm_ctrl.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = float(weight)
        time.sleep(0.02)


def main():
    parser = argparse.ArgumentParser(description="Hold G1 arms + BrainCo hands for navigation tests")
    parser.add_argument(
        "--config",
        default=os.path.join(current_dir, "config", "nav_pose.yaml"),
        help="YAML with left/right arm and hand targets",
    )
    parser.add_argument("--network-interface", default=None)
    parser.add_argument("--sim", action="store_true")
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Use debug/lowcmd instead of arm_sdk. Do not use this while walking.",
    )
    parser.add_argument("--print-current", action="store_true", help="Print current q and exit")
    parser.add_argument("--save-current", default=None, help="Write current q to this YAML and exit")
    parser.add_argument("--hz", type=float, default=50.0)
    args = parser.parse_args()

    ChannelFactoryInitialize(1 if args.sim else 0, networkInterface=args.network_interface)

    motion_mode = not args.debug
    arm_ctrl = G1_29_ArmController(motion_mode=motion_mode, simulation_mode=args.sim)
    hands = BraincoHold()

    current_arm = arm_ctrl.get_current_dual_arm_q()
    current_left_hand, current_right_hand = hands.read_state()
    log_vec("brazo izq actual", ARM_NAMES, current_arm[:7])
    log_vec("brazo der actual", ARM_NAMES, current_arm[7:])
    log_vec("mano izq actual", HAND_NAMES, current_left_hand)
    log_vec("mano der actual", HAND_NAMES, current_right_hand)

    if args.print_current or args.save_current:
        pose = dump_pose(current_arm, current_left_hand, current_right_hand)
        if args.save_current:
            os.makedirs(os.path.dirname(os.path.abspath(args.save_current)) or ".", exist_ok=True)
            with open(args.save_current, "w", encoding="utf-8") as f:
                yaml.safe_dump(pose, f, sort_keys=False, allow_unicode=True)
            logger.info("Postura actual guardada en %s", args.save_current)
        else:
            print(yaml.safe_dump(pose, sort_keys=False, allow_unicode=True))
        return

    target_arm, target_left_hand, target_right_hand, move_time = load_pose(args.config)
    log_vec("brazo izq objetivo", ARM_NAMES, target_arm[:7])
    log_vec("brazo der objetivo", ARM_NAMES, target_arm[7:])
    log_vec("mano izq objetivo", HAND_NAMES, target_left_hand)
    log_vec("mano der objetivo", HAND_NAMES, target_right_hand)
    logger.info(
        "Moviendo en %.1fs y manteniendo. Modo: %s. Ctrl+C para salir.",
        move_time,
        "motion/arm_sdk (navegación)" if motion_mode else "debug/lowcmd",
    )

    tau = np.zeros(14)
    dt = 1.0 / args.hz
    t0 = time.time()
    holding = False
    try:
        while True:
            now = time.time() - t0
            arm_q = interpolate(current_arm, target_arm, now, move_time)
            left_q = interpolate(current_left_hand, target_left_hand, now, move_time)
            right_q = interpolate(current_right_hand, target_right_hand, now, move_time)
            arm_ctrl.ctrl_dual_arm(arm_q, tau)
            hands.send(left_q, right_q)
            if not holding and now >= move_time:
                holding = True
                logger.info("Postura alcanzada. Ya podés navegar; este proceso tiene que seguir corriendo.")
            time.sleep(dt)
    except KeyboardInterrupt:
        logger.info("Saliendo...")
    finally:
        release_arm_sdk(arm_ctrl)
        logger.info("Listo.")


if __name__ == "__main__":
    main()
