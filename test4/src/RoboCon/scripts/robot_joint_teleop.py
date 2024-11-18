#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys, select, termios, tty

msg = """
Control Your Robot Joints!
---------------------------
Control keys:
   q: Increase link1_joint
   a: Decrease link1_joint
   w: Increase link2_joint
   s: Decrease link2_joint
   e: Increase right_gripper_joint
   d: Decrease right_gripper_joint
   e: Increase left_gripper_joint
   d: Decrease left_gripper_joint
   r: Increase right_gripper_joint
   f: Decrease right_gripper_joint
   x: Stop all movement
CTRL-C to quit
"""

# Define key bindings for joint control with smaller increments for finer control
key_bindings = {
    'q': ('link1_joint', 0.05),   # Smaller increment
    'a': ('link1_joint', -0.05),  # Smaller decrement
    'w': ('link2_joint', 0.05),
    's': ('link2_joint', -0.05),
    'e': ('right_gripper_joint', 0.0005),
    'd': ('right_gripper_joint', -0.0005),
    'r': ('left_gripper_joint', -0.0005),
    'f': ('left_gripper_joint', 0.0005)
}

# Initialize joint positions
current_positions = {
    'left_gripper_joint': 0,
    'link1_joint': 0,
    'link2_joint': 0,
    'right_gripper_joint': 0
}

joint_limits = {
    'left_gripper_joint': (0.0, 0.03),
    'link1_joint': (0.81, 2.21),
    'link2_joint': (3.14, 6.3),
    'right_gripper_joint': (0.0, 0.03)
}
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
   
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def send_joint_trajectory(joint_name, increment):
    global current_positions

    # Update the current position for the joint
    current_positions[joint_name] += increment

    # Create the trajectory message
    traj = JointTrajectory()
    traj.header.stamp = rospy.Time.now()
    traj.joint_names = ['left_gripper_joint', 'link1_joint', 'link2_joint', 'right_gripper_joint']
    point = JointTrajectoryPoint()
    point.positions = [
        current_positions['left_gripper_joint'],
        current_positions['link1_joint'],
        current_positions['link2_joint'],
        current_positions['right_gripper_joint']
    ]
    point.time_from_start = rospy.Duration(0.1)  # Short duration for real-time effect
    traj.points.append(point)

    rospy.loginfo(f"Sending trajectory message: {traj}")
    pub.publish(traj)
    rospy.loginfo(f"Published trajectory to {joint_name} with increment {increment}")

def stop_all_movement():
    global current_positions

    # Create the trajectory message to maintain current positions
    traj = JointTrajectory()
    traj.header.stamp = rospy.Time.now()
    traj.joint_names = ['left_gripper_joint', 'link1_joint', 'link2_joint', 'right_gripper_joint']
    point = JointTrajectoryPoint()
    point.positions = [
        current_positions['left_gripper_joint'],
        current_positions['link1_joint'],
        current_positions['link2_joint'],
        current_positions['right_gripper_joint']
    ]
    point.time_from_start = rospy.Duration(0.1)  # Short duration for real-time effect
    traj.points.append(point)

    rospy.loginfo("Stopping all joints.")
    pub.publish(traj)
    rospy.loginfo("All joint movements stopped.")

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
   
    rospy.init_node('robot_joint_teleop')
    pub = rospy.Publisher('/urdf7/arm_controller/command', JointTrajectory, queue_size=10)
   
    try:
        print(msg)
        while True:
            key = getKey()
            if key in key_bindings:
                joint_name, increment = key_bindings[key]
                send_joint_trajectory(joint_name, increment)
                stop_all_movement()  # Ensure movement stops after increment
            elif key == 'x':  # Stop all movement
                stop_all_movement()
            if key == '\x03':  # CTRL-C to quit
                break

    except Exception as e:
        print(e)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
