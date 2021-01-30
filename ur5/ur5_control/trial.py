import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import time
import threading


# A publisher defined globally, will keep on publishing whenever the message is published
pub = rospy.Publisher('box_pose', geometry_msgs.msg.Pose, queue_size=10, latch=True)

# A flag to determine if the box is attached or not, which decides if the message should be published
attached_box = False


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


class Pick_and_place(object):
    def __init__(self, group):
        """
        Used to initialize multiple variables and initialize the node.

        Args:
            group ([type]): [description]
        """
        super(Pick_and_place, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        attachements = moveit_msgs.msg.AttachedCollisionObject()

        group_name = group
        move_group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()

        self.box_name = 'box'
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.attachements = attachements

    # def set_home_position(self, list):
    def set_home_position(self, list):
        """
        Sets home position of the robot arm

        Returns:
            joint goal positions
            end effector position and orientation
        """
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        # list = [0, 0, 0, 0, 0, 0]
        for i in range(len(list)):
            joint_goal[i] = list[i]
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def close_gripper(self):
        move_group = self.move_group
        list1 = [0.3, 0.3, 0.3, 0.3, 0.3, 0.3]
        joint_goal = move_group.get_current_joint_values()
        for i in range(len(list1)):
            joint_goal[i] = list1[i]

        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def open_gripper(self):
        move_group = self.move_group
        list2 = [0, 0, 0, 0, 0, 0]
        joint_goal = move_group.get_current_joint_values()

        for i in range(len(list2)):
            joint_goal[i] = list2[i]
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    # def add_box(self, list, timeout=4):
    def add_box(self, box_pose_list, timeout=4):
        """
        Adds box in the scene and places it at certain distance based on input
        """
        box_name = self.box_name
        scene = self.scene
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 1
        box_pose.pose.position.x = box_pose_list[0]
        box_pose.pose.position.y = box_pose_list[1]
        # slightly above the end effector
        box_pose.pose.position.z = box_pose_list[2]
        # print("The position of box is {0}".format(box_pose.pose))
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.05, 0.05, 0.05))
        

    def go_near_pose_goal(self, box_pose_list):
        """
        This method is used to take the end effector near pose goal which is closer to the box

        """
        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1
        pose_goal.orientation.x = 0
        pose_goal.orientation.y = 0.003
        pose_goal.orientation.z = 0
        pose_goal.position.x = box_pose_list[0] - 0.075
        pose_goal.position.y = box_pose_list[1]
        pose_goal.position.z = box_pose_list[2]

        pub.publish(pose_goal)
        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        """
        Updates the state of attached object in the scene
        """

        box_name = self.box_name
        scene = self.scene
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():

            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0
            is_known = box_name in scene.get_known_object_names()
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True
            rospy.sleep(0.1)
            seconds = rospy.get_time()
        return False

    def move_closer(self, box_pose_list, scale=1):
        """
        This method is used to get closer to the object by cartesian path planning
        """
        move_group = self.move_group
        waypoints = []
        wpose = move_group.get_current_pose().pose
        wpose.position.x = box_pose_list[0]-0.01
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0)         # jump_threshold
        return plan, fraction

    def display_trajectory(self, plan):
        """
        This method is used to display the cartesian path planning
        """

        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        move_group = self.move_group
        move_group.execute(plan, wait=True)

    def attach_box(self, timeout=5):
        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names
        move_group = self.move_group

        grasping_group = 'gripper'
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box("tool_tip", box_name, touch_links=touch_links)
        global attached_box
        attached_box = True

        p_x = move_group.get_current_pose("tool_tip").pose.position.x
        p_y = move_group.get_current_pose("tool_tip").pose.position.y
        p_z = move_group.get_current_pose("tool_tip").pose.position.z

        o_w = move_group.get_current_pose("tool_tip").pose.orientation.w
        o_x = move_group.get_current_pose("tool_tip").pose.orientation.x
        o_y = move_group.get_current_pose("tool_tip").pose.orientation.y
        o_z = move_group.get_current_pose("tool_tip").pose.orientation.z

        print("----------------------------------------"+ '\n'*4)
        
        print("Starting Position:  x= {0},y= {1}, z= {2} \n".format(
            p_x+0.02, p_y, p_z))
        print("Starting Orientation: x= {0},y= {1}, z= {2}, w= {3}".format(
            o_x, o_y, o_z, o_w))
        
        print("----------------------------------------")

        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def box_pose_publisher(self):
        print(pub.get_num_connections())
        rate = rospy.Rate(2)
        while attached_box:
            print(attached_box)
            box_pose = moveit_commander.MoveGroupCommander("group1")
            box_pose.get_current_pose("tool_tip").pose.position.x = box_pose.get_current_pose(
                "tool_tip").pose.position.x + 0.05
            x = box_pose.get_current_pose("tool_tip").pose
            pub.publish(x)
            rate.sleep()

    def go_drop_off(self, goal_pose_list):

        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()

        pose_goal.orientation.w = 0
        pose_goal.orientation.x = -0.003
        pose_goal.orientation.y = -1
        pose_goal.orientation.z = 0
        pose_goal.position.x = goal_pose_list[0]
        pose_goal.position.y = goal_pose_list[1]
        pose_goal.position.z = goal_pose_list[2]

        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def detach_box(self, timeout=4):

        move_group = self.move_group
        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link

        scene.remove_attached_object(eef_link, name=box_name)
        global attached_box
        attached_box = False

        p_x = move_group.get_current_pose("tool_tip").pose.position.x
        p_y = move_group.get_current_pose("tool_tip").pose.position.y
        p_z = move_group.get_current_pose("tool_tip").pose.position.z

        o_w = move_group.get_current_pose("tool_tip").pose.orientation.w
        o_x = move_group.get_current_pose("tool_tip").pose.orientation.x
        o_y = move_group.get_current_pose("tool_tip").pose.orientation.y
        o_z = move_group.get_current_pose("tool_tip").pose.orientation.z

        print("----------------------------------------")

        print("----------------------------------------"+'\n')
        print("Ending Position:  x= {0},y= {1}, z= {2} \n".format(
            p_x, p_y, p_z))
        print("Ending Orientation: x= {0},y= {1}, z= {2}, w= {3}".format(
            o_x, o_y, o_z, o_w))    

        print('\n'+"----------------------------------------"+"\n")

        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def remove_box(self, timeout=4):

        box_name = self.box_name
        scene = self.scene
        scene.remove_world_object(box_name)
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

# def execute_pick_place(list_joint, box_pose_list):


def execute_pick_place(joint_goal_list, box_pose_list, goal_pose_list):
    robot_arm_motion = Pick_and_place("group1")
    robot_gripper = Pick_and_place("gripper")

    robot_arm_motion.set_home_position(joint_goal_list)
    robot_arm_motion.add_box(box_pose_list)

    robot_arm_motion.go_near_pose_goal(box_pose_list)

    cartesian_plan, fraction = robot_arm_motion.move_closer(box_pose_list)
    robot_arm_motion.display_trajectory(cartesian_plan)
    robot_arm_motion.execute_plan(cartesian_plan)

    robot_gripper.close_gripper()
    robot_arm_motion.attach_box()
    begin_time = rospy.Time.now()

    t1 = threading.Thread(
        target=robot_arm_motion.go_drop_off, args=[goal_pose_list])
    t2 = threading.Thread(target=robot_arm_motion.box_pose_publisher)

    t1.start()
    t2.start()
    t1.join()

    robot_gripper.open_gripper()
    robot_arm_motion.detach_box()
    end_time = rospy.Time.now()
    duration = end_time.secs - begin_time.secs
    print("----------------------------------------"+"\n")
    print("Execution time is {} secs".format(duration))
    print('\n'+"----------------------------------------"+"\n")

    robot_arm_motion.remove_box()
    robot_arm_motion.set_home_position(joint_goal_list)

    print("Done")


if __name__ == '__main__':
    Pick_and_place.execute_pick_place()
