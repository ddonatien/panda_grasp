#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import threading


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class DoublePandaControl(object):
  """DoublePandaControl"""
  def __init__(self):
    super(DoublePandaControl, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('robot_control', anonymous=True)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    arm_1_group = "panda_1_arm"
    arm_2_group = "panda_2_arm"
    hand_1_group = "hand_1"
    hand_2_group = "hand_2"
    move_group_1 = moveit_commander.MoveGroupCommander(arm_1_group)
    move_group_2 = moveit_commander.MoveGroupCommander(arm_2_group)
    hand_1 = moveit_commander.MoveGroupCommander(hand_1_group)
    hand_2 = moveit_commander.MoveGroupCommander(hand_2_group)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame_1 = move_group_1.get_planning_frame()
    planning_frame_2 = move_group_2.get_planning_frame()

    eef1_link = move_group_1.get_end_effector_link()
    eef2_link = move_group_2.get_end_effector_link()

    group_names = robot.get_group_names()

    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group_1 = move_group_1
    self.move_group_2 = move_group_2
    self.hand_1 = hand_1
    self.hand_2 = hand_2
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame_1 = planning_frame_1
    self.planning_frame_2 = planning_frame_2
    self.eef1_link = eef1_link
    self.eef2_link = eef2_link
    self.group_names = group_names


  def open_hand_1(self, wait=True):
    joint_goal = self.hand_1.get_current_joint_values()
    joint_goal[0] = 0.04
    print("Opening hand 1")

    self.hand_1.go(joint_goal, wait=wait)

    self.hand_1.stop()

    current_joints = self.hand_1.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def close_hand_1(self, wait=True):
    joint_goal = self.hand_1.get_current_joint_values()
    joint_goal[0] = 0.00
    print("Closing hand 1")

    self.hand_1.go(joint_goal, wait=wait)

    if (not wait ):
      return

    self.hand_1.stop()

    current_joints = self.hand_1.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def open_hand_2(self, wait=True):
    joint_goal = self.hand_2.get_current_joint_values()
    joint_goal[0] = 0.04
    print("Opening hand 2")

    self.hand_2.go(joint_goal, wait=wait)

    self.hand_2.stop()

    current_joints = self.hand_2.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def close_hand_2(self, wait=True):
    joint_goal = self.hand_2.get_current_joint_values()
    joint_goal[0] = 0.00
    print("Closing hand 2")

    self.hand_2.go(joint_goal, wait=wait)

    if (not wait ):
      return
    
    self.hand_2.stop()

    current_joints = self.hand_2.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_joint_state_1(self, wait=True):
    joint_goal = self.move_group_1.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/4
    joint_goal[2] = 0
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = pi/3
    joint_goal[6] = 0

    self.move_group_1.go(joint_goal, wait=wait)

    self.move_group_1.stop()

    current_joints = self.move_group_1.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_joint_state_2(self, wait=True):
    joint_goal = self.move_group_2.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/4
    joint_goal[2] = 0
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = pi/3
    joint_goal[6] = 0

    self.move_group_2.go(joint_goal, wait=wait)

    self.move_group_2.stop()

    current_joints = self.move_group_2.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_pose_goal_1(self, pos, ori):
    pose_goal = geometry_msgs.msg.Pose()
    q = quaternion_from_euler(ori[0], ori[1], ori[2])
    pose_goal.orientation = geometry_msgs.msg.Quaternion(*q)
    pose_goal.position.x = pos[0]
    pose_goal.position.y = pos[1]
    pose_goal.position.z = pos[2]

    self.move_group_1.set_goal_orientation_tolerance(0.0001)
    self.move_group_1.set_goal_position_tolerance(0.0001)

    self.move_group_1.set_planning_time(10)

    self.move_group_1.set_pose_target(pose_goal)

    plan = self.move_group_1.go(wait=True)
    self.move_group_1.stop()
    self.move_group_1.clear_pose_targets()

    current_pose = self.move_group_1.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)



  def go_to_pose_goal_2(self, pos, ori):
    pose_goal = geometry_msgs.msg.Pose()
    q = quaternion_from_euler(ori[0], ori[1], ori[2])
    pose_goal.orientation = geometry_msgs.msg.Quaternion(*q)
    pose_goal.position.x = pos[0]
    pose_goal.position.y = pos[1]
    pose_goal.position.z = pos[2]

    self.move_group_2.set_goal_orientation_tolerance(0.0001)
    self.move_group_2.set_goal_position_tolerance(0.0001)

    self.move_group_2.set_planning_time(10)

    self.move_group_2.set_pose_target(pose_goal)

    plan = self.move_group_2.go(wait=True)
    self.move_group_2.stop()
    self.move_group_2.clear_pose_targets()

    current_pose = self.move_group_2.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def plan_cartesian_path_1(self, scale=1):
    waypoints = []

    wpose = self.move_group_1.get_current_pose().pose
    wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = self.move_group_1.compute_cartesian_path(
                                         waypoints,   # waypoints to follow
                                         0.01,        # eef_step
                                         0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction


  def plan_cartesian_path_2(self, scale=1):
    waypoints = []

    wpose = self.move_group_2.get_current_pose().pose
    wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = self.move_group_2.compute_cartesian_path(
                                         waypoints,   # waypoints to follow
                                         0.01,        # eef_step
                                         0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction



  def display_trajectory(self, plan):
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)


  def execute_plan_1(self, plan):
    self.move_group_1.execute(plan, wait=True)


  def execute_plan_2(self, plan):
    self.move_group_2.execute(plan, wait=True)


def close_move_1(robot):

    robot.close_hand_1()
    robot.go_to_pose_goal_2([-0.6, 0.1, 1.5], [pi/2, 0, 0])

def close_move_2(robot):

    robot.close_hand_2()
    robot.go_to_pose_goal_1([-0.6, -0.1, 1.41], [-pi/2, 0, 0])

def main():
  try:
    robot = DoublePandaControl()

    robot.go_to_joint_state_1()
    robot.go_to_joint_state_2()
    robot.open_hand_1()
    robot.open_hand_2()

    robot.go_to_pose_goal_2([0.6, 0.5, 1.7], [pi, 0, 0])
    robot.go_to_pose_goal_2([0.6, 0.5, 1.53], [pi, 0, 0])
    robot.close_hand_2()
    robot.go_to_pose_goal_2([-0.13, 0, 1.15], [pi, 0, 0])
    robot.open_hand_2()

    robot.go_to_pose_goal_2([-0.6, 0.3, 1.3], [pi/2, 0, 0])
    robot.go_to_pose_goal_1([-0.6, -0.3, 1.2], [-pi/2, 0, 0])

    robot.go_to_pose_goal_2([-0.6, 0.1, 1.3], [pi/2, 0, 0])
    robot.go_to_pose_goal_1([-0.6, -0.1, 1.21], [-pi/2, 0, 0])

    t1 = threading.Thread(target=close_move_1, args=(robot,))
    t2 = threading.Thread(target=close_move_2, args=(robot,))
    t1.start()
    t2.start()

    print("============ Python tutorial demo complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()