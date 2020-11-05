#!/usr/bin/env python

# Copyright 1996-2020 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""universal_robot_ros controller."""

import argparse
import rospy

from controller import Robot
from joint_state_publisher import JointStatePublisher
from trajectory_follower import TrajectoryFollower
from gripper_command import GripperCommander
from rosgraph_msgs.msg import Clock


parser = argparse.ArgumentParser()
parser.add_argument('--node-name', dest='nodeName', default='double_panda', help='Specifies the name of the node.')
arguments, unknown = parser.parse_known_args()

rospy.init_node(arguments.nodeName, disable_signals=True)

jointPrefix = rospy.get_param('prefix', '')
if jointPrefix:
    print('Setting prefix to %s' % jointPrefix)

robot = Robot()
nodeName = arguments.nodeName + '/' if arguments.nodeName != 'ur_driver' else ''
jointStatePublisher = JointStatePublisher(robot, jointPrefix, nodeName)
gripperCommander1 = GripperCommander(robot, jointStatePublisher, jointPrefix, 'panda_1')
gripperCommander2 = GripperCommander(robot, jointStatePublisher, jointPrefix, 'panda_2')
trajectoryFollower1 = TrajectoryFollower(robot, jointStatePublisher, jointPrefix, 'panda_1')
trajectoryFollower2 = TrajectoryFollower(robot, jointStatePublisher, jointPrefix, 'panda_2')
trajectoryFollower1.start()
trajectoryFollower2.start()
gripperCommander1.start()
gripperCommander2.start()
init_pos = {
    "panda_1_joint1": 0.000,
    "panda_1_joint2": -0.785,
    "panda_1_joint3": 0.0,
    "panda_1_joint4": -2.356,
    "panda_1_joint5": 0.0,
    "panda_1_joint6": 1.57,
    "panda_1_joint7": 0.4,
    "panda_2_joint1": 0.000,
    "panda_2_joint2": -0.785,
    "panda_2_joint3": 0.0,
    "panda_2_joint4": -2.356,
    "panda_2_joint5": 0.0,
    "panda_2_joint6": 1.57,
    "panda_2_joint7": 0.4
            }
for jt in init_pos:
    robot.getMotor(jt).setPosition(init_pos[jt])

# we want to use simulation time for ROS
clockPublisher = rospy.Publisher('clock', Clock, queue_size=1)
if not rospy.get_param('use_sim_time', False):
    rospy.logwarn('use_sim_time is not set!')

timestep = int(robot.getBasicTimeStep())

while robot.step(timestep) != -1 and not rospy.is_shutdown():
    jointStatePublisher.publish()
    trajectoryFollower1.update()
    trajectoryFollower2.update()
    gripperCommander1.update()
    gripperCommander2.update()
    # pulish simulation clock
    msg = Clock()
    time = robot.getTime()
    msg.clock.secs = int(time)
    # round prevents precision issues that can cause problems with ROS timers
    msg.clock.nsecs = round(1000 * (time - msg.clock.secs)) * 1.0e+6
    clockPublisher.publish(msg)
