#!/usr/bin/env python
import sys
import random
import rospy
import moveit_commander
import geometry_msgs.msg
from std_srvs.srv import Trigger
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest
from moveit_msgs.msg import DisplayTrajectory, RobotState
from visualization_msgs.msg import Marker
from copy import deepcopy

def generate_random_number():
    # Generate a random number within the intervals (-0.5, -0.2) and (0.2, 0.5)
    random_number = random.uniform(-0.5, -0.2) if random.randint(0,1) == 0 else random.uniform(0.2, 0.5)
    return random_number

def generate_random_point_in_reach():
    x = random.uniform(0.2, 0.5)
    y = generate_random_number()
    z = random.uniform(0.1, 0.5)
    return [x, y, z]

def generate_cube(scene, name, position, size=0.05):
    pose = geometry_msgs.msg.PoseStamped()
    pose.header.frame_id = "base_link"
    pose.pose.position.x = position[0]
    pose.pose.position.y = position[1]
    pose.pose.position.z = position[2]
    scene.add_box(name, pose, size=(size, size, size))

def cartesian_trajectory(group, home_position, cube_position, orientation, vel_f, acc_f):
    waypoints = []
    w_pose_target = geometry_msgs.msg.Pose()
    w_pose_target.orientation.w = orientation[0]
    w_pose_target.orientation.x = orientation[1]
    w_pose_target.orientation.y = orientation[2]
    w_pose_target.orientation.z = orientation[3]
    w_pose_target.position.x = cube_position[0]
    w_pose_target.position.y = cube_position[1]
    w_pose_target.position.z = cube_position[2]
    waypoints.append(deepcopy(w_pose_target))
    waypoints.append(deepcopy(waypoints[0]))
    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0, avoid_collisions = False)
    if 1-fraction < 0.01:
        rospy.loginfo('Path computed successfully.')
    else:
        rospy.loginfo('Path planning failed')
    rospy.loginfo('Fraction: %f' % fraction)
    plan = group.retime_trajectory(group.get_current_state(), plan, vel_f, acc_f)
    return plan

def publishTrajectoryLine(plan, fk_service, joint_names, link_name, publisher):
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.type = marker.LINE_STRIP
    marker.action = marker.ADD
    marker.scale.x = 0.01
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.id = 0
    for point in plan.joint_trajectory.points:
        robot_state = RobotState()
        robot_state.joint_state.name = joint_names
        robot_state.joint_state.position = point.positions
        fk_request = GetPositionFKRequest()
        fk_request.fk_link_names = [link_name]
        fk_request.robot_state = robot_state
        response = fk_service(fk_request)
        if len(response.pose_stamped) > 0:
            marker.points.append(response.pose_stamped[0].pose.position)
    publisher.publish(marker)

def main():
    # Initialize ROS node
    rospy.init_node('robot_init', anonymous=True)

    # Create publisher for visualization markers
    trajectory_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    # Wait for 1 second
    rospy.sleep(1)

    # Set visibility parameter
    visible_object = True
    rospy.set_param('object_visible', visible_object)

    # Create MoveGroupCommander and PlanningSceneInterface
    group = moveit_commander.MoveGroupCommander('manipulator')
    scene = moveit_commander.PlanningSceneInterface()

    # Create RobotCommander and wait for reset_robot service
    robot = moveit_commander.RobotCommander()
    rospy.wait_for_service('/reset_robot')

    # Call reset_robot service and wait for 0.5 seconds
    reset_robot  = rospy.ServiceProxy('/reset_robot', Trigger)
    reset_robot.call()
    rospy.sleep(0.5)

    # Get initial pose and set parameters
    w_pose_initial = rospy.wait_for_message('/current_tcp_pose', geometry_msgs.msg.PoseStamped, timeout=None)
    home_position = [w_pose_initial.pose.position.x, w_pose_initial.pose.position.y, w_pose_initial.pose.position.z]
    orientation = [w_pose_initial.pose.orientation.w, w_pose_initial.pose.orientation.x, w_pose_initial.pose.orientation.y, w_pose_initial.pose.orientation.z]
    temp_position = home_position
    rospy.set_param('object_pos', home_position)

    # Set velocity and acceleration scaling factors
    vel_scaling_f = 1.0
    acc_scaling_f = 1.0
    group.set_max_velocity_scaling_factor(vel_scaling_f)
    group.set_max_acceleration_scaling_factor(acc_scaling_f)

    # Set planner ID and create service proxy
    group.set_planner_id('OMPL')
    fk_service = rospy.ServiceProxy('compute_fk', GetPositionFK)

    while not rospy.is_shutdown():
        # Remove previous cube from planning scene
        scene.remove_world_object("target_cube")

        # Generate random position for the cube and add it to the scene
        cube_pos = generate_random_point_in_reach()
        generate_cube(scene, "target_cube", cube_pos)

        # Generate Cartesian trajectory from current position to the cube position
        plan = cartesian_trajectory(group, temp_position, cube_pos, orientation, vel_scaling_f, acc_scaling_f)

        if plan:
            # Log the number of intermediate points in the trajectory
            rospy.loginfo('Intermediate points on the robot trajectory: %d' % len(plan.joint_trajectory.points))

            # Publish the trajectory line as visualization markers
            publishTrajectoryLine(plan, fk_service, group.get_active_joints(), group.get_end_effector_link(), trajectory_pub)

            # Execute the trajectory and wait for 0.5 seconds
            group.execute(plan, wait=True)
            rospy.sleep(0.5)

            # Update temporary position
            temp_position = cube_pos

            # Stop the group and clear constraints and targets
            group.stop()
            group.clear_path_constraints()
            group.clear_pose_targets()

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    main()
    moveit_commander.roscpp_shutdown()