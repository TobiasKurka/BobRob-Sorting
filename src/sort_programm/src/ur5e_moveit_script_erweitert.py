#!/usr/bin/env python3

from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
import actionlib
import numpy as np 
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from color_detection.msg import LegoBrick

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from control_msgs.msg import GripperCommand
from geometry_msgs.msg import PointStamped

# Die Initialisierung von moveit_commander und des rospy Node wird zuerst gemacht
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_tutorial", anonymous=True) # Node Namen können Sie später noch anpassen

global lego_brick_data
lego_brick_data = None


def lego_brick_callback(msg):
    global lego_brick_data
    lego_brick_data = msg 
    rospy.loginfo("Empfangener LegoBrick hinzugefügt: Position - x=%f, y=%f, z=%f, Farbe - %s",
                  msg.position.point.x, msg.position.point.y, msg.position.point.z, msg.color.data)


lego_brick_sub = rospy.Subscriber('/lego_brick_info', LegoBrick, lego_brick_callback)

def lego_brick_callback(msg):
    global lego_brick_position
    lego_brick_position = msg.point
    rospy.loginfo("Empfangene Position des Legosteins: x=%f, y=%f, z=%f", msg.point.x, msg.point.y, msg.point.z)
    
# Initalisieren des Subscribers für topic '/lego_brick_coords'
#lego_brick_sub = rospy.Subscriber('/lego_brick_coords', geometry_msgs.msg.PointStamped, lego_brick_callback)

# Initialisieren des ActionClients für den Greifer
gripper_client = actionlib.SimpleActionClient('/gripper_controller/gripper_action', GripperCommandAction)
gripper_client.wait_for_server()

def lego_brick_color_callback(msg):
    global lego_brick_colors
    lego_brick_colors.append(msg.data)
    rospy.loginfo("Empfangene Farbe des Legosteins: %s", msg.data)

# Initalisieren des Subscribers für topic '/lego_brick_color'
lego_brick_color_sub = rospy.Subscriber('/lego_brick_color', String, lego_brick_color_callback)

def move_gripper(position, max_effort):
    # Erstellen des Ziels für den Greifer
    goal = GripperCommandGoal()
    goal.command.position = position
    goal.command.max_effort = max_effort

    # Senden des Ziels zum ActionServer und warten auf das Ende der Aktion
    gripper_client.send_goal(goal)
    gripper_client.wait_for_result(rospy.Duration.from_sec(5.0))
    result = gripper_client.get_result()

    return result


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

def move_gripper(position, max_effort):
    goal = GripperCommandGoal()
    goal.command.position = position
    goal.command.max_effort = max_effort
    gripper_client.send_goal(goal)
    gripper_client.wait_for_result(rospy.Duration.from_sec(5.0))
    return gripper_client.get_result()

# Funktion zum Bewegen des Roboters
def move_robot(waypoints):
    (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
    if fraction > 0:
        move_group.execute(plan, wait=True)

# Hauptfunktion
def main():
    global lego_brick_data  # Global deklariert
    while not rospy.is_shutdown():
        if lego_brick_data:
            # Berechnung der Wegpunkte
            waypoints = []
            roll = 0
            pitch = 0
            yaw = np.pi
            quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

            pose_above_lego = geometry_msgs.msg.Pose()
            pose_above_lego.orientation.x = quaternion[0]
            pose_above_lego.orientation.y = quaternion[1]
            pose_above_lego.orientation.z = quaternion[2]
            pose_above_lego.orientation.w = quaternion[3]
            pose_above_lego.position.x = lego_brick_data.position.point.x
            pose_above_lego.position.y = lego_brick_data.position.point.y
            pose_above_lego.position.z = 0.4
            waypoints.append(copy.deepcopy(pose_above_lego))

            pose_at_lego = copy.deepcopy(pose_above_lego)
            pose_at_lego.position.z = lego_brick_data.position.point.z
            waypoints.append(copy.deepcopy(pose_at_lego))

            color = lego_brick_data.color.data
            pose_drop_off = copy.deepcopy(pose_above_lego)
            if color == "yellow":
                pose_drop_off.position.x = 0.3
                pose_drop_off.position.y = 0.678
                pose_drop_off.position.z = 0.1
            elif color == "red":
                pose_drop_off.position.x = 0.3
                pose_drop_off.position.y = 0.678 - 0.12
                pose_drop_off.position.z = 0.1
            elif color == "green":
                pose_drop_off.position.x = 0.3
                pose_drop_off.position.y = 0.678 - 0.24
                pose_drop_off.position.z = 0.1                
            # Weitere Farben hier hinzufügen

            waypoints.append(copy.deepcopy(pose_drop_off))

            # Bewegung des Roboters
            move_robot(waypoints[:2])
            move_gripper(1.0, 0.0)  # Greifen
            move_robot([pose_above_lego, pose_drop_off])  # Zur Ablageposition bewegen
            move_gripper(0.0, 0.0)  # Loslassen

            lego_brick_data = None  # Zurücksetzen der Position für den nächsten Stein
        else:
            rospy.sleep(0.1)

if __name__ == '__main__':
    main()










