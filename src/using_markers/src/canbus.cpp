#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

double vx=0;
double vz=0;

double vx1=0;
double vz1=0;
double jiaoyan=0;

void cmd_velCallback(const geometry_msgs::Twist &harry)
{

 vx=harry.linear.x*10;
 vz=harry.angular.z;
 jiaoyan=0;
}
void cmd_velCallback2(const geometry_msgs::TwistStamped &harry)
{

 vx1=harry.twist.linear.x;
 vz1=harry.twist.angular.z;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "twist_cmd");
  ros::NodeHandle nh;
  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("twist_cmd", 50);
  ros::Subscriber control_sub = nh.subscribe("cmd_vel", 50, cmd_velCallback);
  ros::Subscriber control_sub1 = nh.subscribe("twist_cmd1", 50, cmd_velCallback2);
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    jiaoyan+=1;
    if(jiaoyan>65536)jiaoyan=5;
    ROS_INFO("jiaoyan  is %f", jiaoyan);

    ros::spinOnce();
    geometry_msgs::TwistStamped  twist_cmd11;

    if(jiaoyan<5)
    {
    twist_cmd11.twist.linear.x=vx;
    twist_cmd11.twist.linear.y=0;
    twist_cmd11.twist.linear.z=0;
    twist_cmd11.twist.angular.x=0;
    twist_cmd11.twist.angular.y=0;
    twist_cmd11.twist.angular.z=vz;
    }

    else
    {

      twist_cmd11.twist.linear.x=vx1;
      twist_cmd11.twist.linear.y=0;
      twist_cmd11.twist.linear.z=0;
      twist_cmd11.twist.angular.x=0;
      twist_cmd11.twist.angular.y=0;
      twist_cmd11.twist.angular.z=vz1;

    }
    cmd_vel_pub.publish(twist_cmd11);
    loop_rate.sleep();
    }

}
