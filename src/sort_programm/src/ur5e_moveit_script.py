#!/usr/bin/env python3

from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

# First initialize moveit_commander and a rospy node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_tutorial", anonymous=True) #Kann den Node Namen später noch abändern

# Instantiate a RobotCommander object
robot = moveit_commander.RobotCommander()

# Instantiate a PlanningSceneInterface object
scene = moveit_commander.PlanningSceneInterface()

# Instantiate a MoveGroupCommander object
group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)

# Create a DisplayTrajectory ROS publisher which is used to display trajectories in Rviz
display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)

# Getting Basic Information

# We can get the name of the reference frame for this robot:
planning_frame = move_group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = move_group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names())

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print("============ Printing robot state")
print(robot.get_current_state())
print("")

# We get the joint values from the group and change them to the 'test' position values:
joint_goal = move_group.get_current_joint_values()
joint_goal[0] = -1.7453  # shoulder_pan_joint
joint_goal[1] = -2.4435  # shoulder_lift_joint #-2.1991
joint_goal[2] = -1.0996  # elbow_joint
joint_goal[3] = -0.1047  # wrist_1_joint
joint_goal[4] = 1.6057   # wrist_2_joint
joint_goal[5] = 0        # wrist_3_joint

# Drucken Sie den joint_goal, um sicherzustellen, dass die Werte korrekt sind
print("Geplanter joint_goal:", joint_goal)

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
#move_group.go(joint_goal, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
#move_group.stop()

#----------Der obere Teil war dazu da eine Position test über die Python-API anzufahren. Der untere Teil um die geplante Bewegung in Rviz zu zeigen-------------

#Einstellen der Geschwindigkeit und Beschleunigung
move_group.set_max_velocity_scaling_factor(0.5)
move_group.set_max_acceleration_scaling_factor(0.5)

# Zielwerte setzen
move_group.set_joint_value_target(joint_goal)

planning_result = move_group.plan()
if planning_result[0]:
    plan = planning_result[1]
    print("Plan erfolgreich erstellt!")
else:
    print("Planung fehlgeschlagen!")
    sys.exit()


# Anzeigen der Trajektorie in RViz
display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan)
# Veröffentliche die Trajektorie
display_trajectory_publisher.publish(display_trajectory)

# Gib RViz etwas Zeit, um die Trajektorie zu visualisieren. Passe die Schlafzeit bei Bedarf an.
rospy.sleep(5)

# Frage den Benutzer, ob er den Plan ausführen möchte
input("Drücke Enter, um den Plan auszuführen oder Ctrl+C zum Abbrechen...")

# Ausführen des Plans
move_group.execute(plan, wait=True)



