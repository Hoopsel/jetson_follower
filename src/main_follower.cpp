#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <follower.h>

#define DIST_K      	0.5
#define DIST_DEADBAND	0.5
#define ERR_K       	-1.0
#define ERR_DEADBAND	0.1

const string robot_name="batman";

using namespace std;

int main(int argc, char* argv[])
{
    //Init ros node
    init(argc, argv, "forbot_follower");

    //Ros variables
    NodeHandle n("~");
    ros::Rate rate(5);
    Publisher pub_vel = n.advertise<geometry_msgs::Twist>(string("/") + robot_name + string("/aria/cmd_vel"), 10);

    //Follower constructor
    Follower follower(n);

    //Loop
    while(ros::ok())
    {
        //Spin
        ros::spinOnce();

        //Find it!
        Follower::Position pos = follower.findMarker(true);

        // Distance component of vel
        float dist_comp=0.0;
        if (pos.second>DIST_DEADBAND) dist_comp = (pos.second - DIST_DEADBAND)*DIST_K;

        // Error component of vel
        float err_comp = 0.0;
        if (abs(pos.first)>ERR_DEADBAND) err_comp = pos.first*ERR_K;

        // Check if we saw sth
        if (pos.second < 0)
        {
            dist_comp = 0.0;
            err_comp = 0.0;
            cout << "I'm blind!" << endl;
        } else cout << "GO GO GO! " << dist_comp << '\t' << err_comp << endl;

        // cmd_vel
        geometry_msgs::Twist vel;
        vel.linear.x = dist_comp;
        vel.angular.z = err_comp;
        pub_vel.publish(vel);

        //Sleep
        rate.sleep();
    }

    // Stahp the robot
    geometry_msgs::Twist vel;
    vel.linear.x = 0;
    vel.angular.z = 0;
    pub_vel.publish(vel);

    return 0;
}
