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
    both_arms_group = "panda_both_arms"
    hand_1_group = "hand_1"
    hand_2_group = "hand_2"
    move_group_1 = moveit_commander.MoveGroupCommander(arm_1_group)
    move_group_2 = moveit_commander.MoveGroupCommander(arm_2_group)
    move_group_both = moveit_commander.MoveGroupCommander(both_arms_group)
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
    self.move_group_both = move_group_both
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
    joint_goal[0] = 0.0399

    self.hand_1.set_goal_joint_tolerance(0.001)
    self.hand_1.go(joint_goal, wait=wait)

    self.hand_1.stop()

    current_joints = self.hand_1.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def close_hand_1(self, wait=True):
    joint_goal = self.hand_1.get_current_joint_values()
    joint_goal[0] = 0.0001

    self.hand_1.set_goal_joint_tolerance(0.001)
    self.hand_1.go(joint_goal, wait=wait)

    if (not wait ):
      return

    self.hand_1.stop()

    current_joints = self.hand_1.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def open_hand_2(self, wait=True):
    joint_goal = self.hand_2.get_current_joint_values()
    joint_goal[0] = 0.0399

    self.hand_2.set_goal_joint_tolerance(0.001)
    self.hand_2.go(joint_goal, wait=wait)

    self.hand_2.stop()

    current_joints = self.hand_2.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def close_hand_2(self, wait=True):
    joint_goal = self.hand_2.get_current_joint_values()
    joint_goal[0] = 0.0001

    self.hand_2.set_goal_joint_tolerance(0.001)
    self.hand_2.go(joint_goal, wait=wait)

    if (not wait ):
      return
    
    self.hand_2.stop()

    current_joints = self.hand_2.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_joint_state_1(self, wait=True, pos=[0, -pi/4, 0, -pi/2, 0, pi/3, 0]):
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

  def go_to_joint_state_2(self, wait=True, pos=[0, -pi/4, 0, -pi/2, 0, pi/3, 0]):
    joint_goal = self.move_group_2.get_current_joint_values()
    joint_goal[0] = pos[0]
    joint_goal[1] = pos[1]
    joint_goal[2] = pos[2]
    joint_goal[3] = pos[3]
    joint_goal[4] = pos[4]
    joint_goal[5] = pos[5]
    joint_goal[6] = pos[6]

    self.move_group_2.go(joint_goal, wait=wait)

    self.move_group_2.stop()

    current_joints = self.move_group_2.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_joint_state_both(self, wait=True, pos1=[0, -pi/4, 0, -pi/2, 0, pi/3, 0], pos2=[0, -pi/4, 0, -pi/2, 0, pi/3, 0]):
    joint_goal = self.move_group_both.get_current_joint_values()
    joint_goal[0]  = pos1[0]
    joint_goal[1]  = pos1[1]
    joint_goal[2]  = pos1[2]
    joint_goal[3]  = pos1[3]
    joint_goal[4]  = pos1[4]
    joint_goal[5]  = pos1[5]
    joint_goal[6]  = pos1[6]
    joint_goal[7]  = pos2[0]
    joint_goal[8]  = pos2[1]
    joint_goal[9]  = pos2[2]
    joint_goal[10] = pos2[3]
    joint_goal[11] = pos2[4]
    joint_goal[12] = pos2[5]
    joint_goal[13] = pos2[6]

    self.move_group_both.go(joint_goal, wait=wait)

    self.move_group_both.stop()

    current_joints = self.move_group_both.get_current_joint_values()
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

    self.move_group_2.set_pose_target(pose_goal)

    plan = self.move_group_2.go(wait=True)
    self.move_group_2.stop()
    self.move_group_2.clear_pose_targets()

    current_pose = self.move_group_2.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def go_to_pose_goal_both(self, pos1, pos2, ori1, ori2):
    pose_goal1 = geometry_msgs.msg.Pose()
    q = quaternion_from_euler(ori1[0], ori1[1], ori1[2])
    pose_goal1.orientation = geometry_msgs.msg.Quaternion(*q)
    pose_goal1.position.x = pos1[0]
    pose_goal1.position.y = pos1[1]
    pose_goal1.position.z = pos1[2]

    pose_goal2 = geometry_msgs.msg.Pose()
    q = quaternion_from_euler(ori2[0], ori2[1], ori2[2])
    pose_goal2.orientation = geometry_msgs.msg.Quaternion(*q)
    pose_goal2.position.x = pos2[0]
    pose_goal2.position.y = pos2[1]
    pose_goal2.position.z = pos2[2]

    self.move_group_both.set_goal_orientation_tolerance(0.0001)
    self.move_group_both.set_goal_position_tolerance(0.0001)

    self.move_group_both.set_pose_target(pose_goal1, end_effector_link=self.eef1_link)
    self.move_group_both.set_pose_target(pose_goal2, end_effector_link=self.eef2_link)

    plan = self.move_group_both.go(wait=True)
    self.move_group_both.stop()
    self.move_group_both.clear_pose_targets()

    current_pose1 = self.move_group_both.get_current_pose(self.eef1_link).pose
    current_pose2 = self.move_group_both.get_current_pose(self.eef2_link).pose
    return all_close(pose_goal1, current_pose1, 0.01), all_close(pose_goal2, current_pose2, 0.01)


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


  def plan_cartesian_path_both(self, scale=1):
    waypoints = []

    wpose = self.move_group_both.get_current_pose(self.eef1_link).pose
    wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = self.move_group_both.compute_cartesian_path(
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


  def execute_plan_both(self, plan):
    self.move_group_both.execute(plan, wait=True)


def close_move_1(robot):

    robot.close_hand_1()
    robot.go_to_pose_goal_2([-0.6, 0.1, 1.5], [pi/2, pi/4, 0])

def close_move_2(robot):

    robot.close_hand_2()
    robot.go_to_pose_goal_1([-0.6, -0.1, 1.42], [-pi/2, -pi/4, 0])

def main():
  try:
    robot = DoublePandaControl()

    # Init
    robot.go_to_joint_state_both()
    robot.open_hand_1()
    robot.open_hand_2()

    # One armed pick and place
    robot.go_to_pose_goal_2([0.6, 0.5, 1.7], [pi, 0, -pi/4])
    robot.go_to_pose_goal_2([0.6, 0.5, 1.53], [pi, 0, -pi/4])
    robot.close_hand_2()
    robot.go_to_pose_goal_2([0.4, 0.7, 1.6], [pi, 0, -pi/4])
    robot.go_to_pose_goal_2([-0.13, 1.0, 1.15], [pi, 0, 0])
    robot.open_hand_2()

    # Two armed pick and move
    robot.go_to_pose_goal_both([-0.6, -0.3, 1.22], [-0.6, 0.3, 1.3], [-pi/2, -pi/4, 0], [pi/2, pi/4, 0])

    robot.go_to_pose_goal_both([-0.6, -0.1, 1.22], [-0.6, 0.1, 1.3], [-pi/2, -pi/4, 0],  [pi/2, pi/4, 0])
    robot.close_hand_2()
    robot.close_hand_1()

    robot.go_to_pose_goal_both([-0.6, -0.1, 1.42], [-0.6, 0.1, 1.5], [-pi/2, -pi/4, 0],  [pi/2, pi/4, 0])
    robot.go_to_pose_goal_both([-0.4, -0.1, 1.42], [-0.4, 0.1, 1.5], [-pi/2, -pi/4, 0],  [pi/2, pi/4, 0])
    robot.open_hand_2()
    robot.open_hand_1()

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()