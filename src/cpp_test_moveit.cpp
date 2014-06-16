#include "ros/ros.h"
#include "std_msgs/String.h"


/**
 * Try to do cartesian moveit planning with cpp insted of python.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "cpp_test_moveit");
    ros::NodeHandle n;

    moveit::planning_interface::MoveGroup right_arm_group("right_arm");

    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

    ROS_INFO("End effector link: %s", group.getEndEffectorLink().c_str());

    geometry_msgs::Pose target_pose;
    target_pose.position.x = 0.815;
    target_pose.position.y = -1.01;
    target_pose.position.z = 0.321;
    target_pose.orientation.x = 0.271;
    target_pose.orientation.y = 0.653;
    target_pose.orientation.z = -0.271;
    target_pose.orientation.w = 0.653;

    return 0;
}
