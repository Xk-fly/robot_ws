#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "arm_controller/move.h"
#include "arm_controller/control.h"
#include "arm_controller/PickPlace.h"
#include "std_srvs/Empty.h"

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <math.h>       /* tan */

using namespace std;

#define CAMREA_X_MAX 200
#define CAMREA_Y_MAX 0
#define CAMREA_Z_MAX 170

class PickAr
{
public:
    PickAr();

    ros::NodeHandle n;
	ros::ServiceServer pick_server,place_server;
    ros::ServiceClient client_pick,client_goto,client_place;
	ros::Subscriber sub;		
    tf::TransformListener listener;
    tf::TransformBroadcaster robot_broadcaster;
    ros::Time current_time, last_time;

    float position[4];
    void position_callback(const arm_controller::control& msg);
    bool pick_callback( arm_controller::PickPlace::Request &req,
                        arm_controller::PickPlace::Response &res);
    bool place_callback( arm_controller::move::Request &req,
                        arm_controller::move::Response &res);

private:
    /* data */
};

//pos_info回调函数
void PickAr::position_callback(const arm_controller::control& msg)
{
    //info当前位置
	position[0] = msg.position.x * 1000;
	position[1] = msg.position.y * 1000;
	position[2] = msg.position.z * 1000;
}

//pick_ar回调函数
bool PickAr::pick_callback(arm_controller::PickPlace::Request &req,
                        arm_controller::PickPlace::Response &res)
{
    //获取字符串“ar_marker_number”
    int a=req.number;
    stringstream ss;
    string stra="";
    ss<<"/ar_marker_";
    ss<<a;
    ss>>stra;
    const string &strb=stra;
    
    arm_controller::move pos;
    if(req.mode==0)//mode=0，则机械臂跑到盒子上面抓取物体
    {
        //走到摄像头固定位置-盒子的上方
        pos.request.pose.position.x=CAMREA_X_MAX;
        pos.request.pose.position.y=CAMREA_Y_MAX;
        pos.request.pose.position.z=CAMREA_Z_MAX;
        if (!client_goto.call(pos))
        {
            res.message=" error !";
            res.success=false;
            return true;
        }
    }else if(req.mode==1){

    }else{
        res.message="  Pattern error  !";
        res.success=false;
        return true;
    }
    sleep(1);
    //监听tf
    tf::StampedTransform transform;
    try
    {
        listener.waitForTransform("Base",strb, ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("Base",strb, ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) 
    {
        ROS_ERROR("TF may have some problems.\n %s",ex.what());
        ros::Duration(1.0).sleep();
        return true;
    }

    float x_,y_,z_;
    x_=transform.getOrigin().x()*1000.0;
    y_=transform.getOrigin().y()*1000.0;
    z_=transform.getOrigin().z()*1000.0;
    ROS_INFO("r-a X:  %.3f  Y:  %.3f Z:  %.3f",x_,y_,z_);

    //走到ar码上方
    pos.request.pose.position.x=x_;
    pos.request.pose.position.y=y_;
    pos.request.pose.position.z=z_+50;
    client_goto.call(pos);
    // 抓取
    arm_controller::move pick;
    pick.request.pose.position.x=x_;
    pick.request.pose.position.y=y_;
    pick.request.pose.position.z=z_;

    if(client_goto.call(pick))
    {
        std_srvs::Empty pump;
        client_pick.call(pump);
        if(pick.response.success==true)
            res.success=true;
        else
        {
            res.message=" Manipulator unattainable !";
            res.success=false;
        }
    }
    else
    {
        res.success=false;
    }

    sleep(0.5);//延时一段时间不能频繁call/goto服务

    //回到ar码上方
    pos.request.pose.position.x=x_;
    pos.request.pose.position.y=y_;
    pos.request.pose.position.z=z_+50;
    client_goto.call(pos);

    sleep(1);//延时一段时间不能频繁call/goto服务

    if(!((abs(req.pose.position.x)<0.001)&&(abs(req.pose.position.y)<0.001)&&(abs(req.pose.position.z)<0.001))){
        //走到放置位置上方
        ROS_INFO("goto");	
        pos.request.pose.position.x=req.pose.position.x;
        pos.request.pose.position.y=req.pose.position.y;
        pos.request.pose.position.z=req.pose.position.z+50;
        client_goto.call(pos);
        //ROS_INFO("gooto end");
        
        //下去放下
        arm_controller::move place;
        place.request.pose.position.x=req.pose.position.x;
        place.request.pose.position.y=req.pose.position.y;
        place.request.pose.position.z=req.pose.position.z+10;
        client_goto.call(place);
        std_srvs::Empty pump;
        client_place.call(pump);
	    ROS_INFO("place end");
    }else{
        if(req.mode==0)
        {
            ROS_INFO("固定位置");

            //回到摄像头固定位置
            pos.request.pose.position.x=CAMREA_X_MAX;
            pos.request.pose.position.y=CAMREA_Y_MAX;
            pos.request.pose.position.z=CAMREA_Z_MAX;
            client_goto.call(pos);
        }else if(req.mode==1){
            pos.request.pose.position.x=position[0];
            pos.request.pose.position.y=position[1];
            pos.request.pose.position.z=position[2];
            client_goto.call(pos);
            ROS_INFO("mode == 1");
        }
    }

    return true;	
}

//place_ar回调函数
bool PickAr::place_callback(arm_controller::move::Request &req,
                                 arm_controller::move::Response &res)
{
    //走到放置位置上方
    arm_controller::move pos;
    pos.request.pose.position.x=req.pose.position.x;
    pos.request.pose.position.y=req.pose.position.y;
    pos.request.pose.position.z=req.pose.position.z+50;
    client_goto.call(pos);    
    //下去放下
    arm_controller::move place;
    place.request.pose.position.x=req.pose.position.x;
    place.request.pose.position.y=req.pose.position.y;
    place.request.pose.position.z=req.pose.position.z;
    client_goto.call(place);
    std_srvs::Empty pump;
    client_place.call(pump);

	res.success = true;
    return true;
}

PickAr::PickAr()
{
    pick_server=n.advertiseService("pick_ar",&PickAr::pick_callback,this);
    place_server=n.advertiseService("place_ar",&PickAr::place_callback,this);

    client_pick=n.serviceClient<std_srvs::Empty>("swiftpro/on");
    client_place=n.serviceClient<std_srvs::Empty>("swiftpro/off");
    client_goto=n.serviceClient<arm_controller::move>("goto_position");

    sub = n.subscribe("arm_controller/position_info",100,&PickAr::position_callback,this);
    current_time = ros::Time::now();
    ROS_INFO("INIT OK");
}

int main(int argc,char** argv)
{
    //节点初始化
    ros::init(argc,argv,"robot_pick");
    //实例化类PickAr
    PickAr PickAr;
    //多线程接收
    ros::AsyncSpinner spinner(2); 
    spinner.start();
    ros::waitForShutdown();
    return 0;
}

