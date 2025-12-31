#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
#include "arm_controller/control.h"

int main(int argc, char *argv[])
{
    // setlocale(LC_ALL,"");
    ros::init(argc,argv,"pose_swiftpro");
    ros::NodeHandle nh;
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    ros::Publisher 	 pub = nh.advertise<arm_controller::control>("arm_controller/position_info", 1);

    ros::Rate rate(10);
    sleep(1);
    while (ros::ok())
    {
        try
        {
            geometry_msgs::TransformStamped tfs = buffer.lookupTransform("Base","virtual_end_effector",ros::Time(0));
            arm_controller::control pose;
            pose.position.x = tfs.transform.translation.x;
            pose.position.y = tfs.transform.translation.y;
            pose.position.z = tfs.transform.translation.z;
            pub.publish(pose);
        }
        catch(const std::exception& e)
        {
            ROS_INFO("error:%s",e.what());
        }
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}