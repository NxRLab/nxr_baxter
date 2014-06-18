#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>

#include <moveit/planning_interface/planning_interface.h>
// #include <move_group.h>
#include <moveit/move_group_interface/move_group.h>
// #include <moveit/robot_state/robot_state.h>


/**
 * Try to do cartesian moveit planning with cpp insted of python.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "cpp_test_moveit");
    ros::NodeHandle n;
    ROS_INFO("Trying cpp_test_moveit");

    moveit::planning_interface::MoveGroup right_arm_group("right_arm");

    ROS_INFO("Move group created.");

    ROS_INFO("Reference frame: %s", right_arm_group.getPlanningFrame().c_str());

    ROS_INFO("End effector link: %s", right_arm_group.getEndEffectorLink().c_str());

    geometry_msgs::Pose target_pose;
    target_pose.position.x = 0.815;
    target_pose.position.y = -1.01;
    target_pose.position.z = 0.321;
    target_pose.orientation.x = 0.271;
    target_pose.orientation.y = 0.653;
    target_pose.orientation.z = -0.271;
    target_pose.orientation.w = 0.653;

    right_arm_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = right_arm_group.plan(my_plan);

    ROS_INFO("Plan 1 pose goal %s", success?"":"FAILED");
    if(success)
	right_arm_group.move();

    sleep(5.0);

    // ROS_INFO("Try joint-space goal");

    // std::vector<double> group_var_values;
    // right_arm_group.getCurrentState()->copyJointGroupPositions(right_arm_group.getCurrentState()->getRobotModel()->getJointModelGroup(right_arm_group.getName()), group_var_values);

    // group_var_values[0] = -1.0;
    // right_arm_group.setJointValueTarget(group_var_values);
    // right_arm_group.move();
    
    return 0;
}
