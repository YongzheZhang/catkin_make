#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/String.h"




ros::Publisher posePoseStamped_pub;


  void chatterCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
  {
         //Vertical parking_lot

   geometry_msgs::PoseStamped posestamped1,posestamped2,posestamped3,posestamped4,posestamped5;
    posestamped1.header.frame_id="/myframe_v";
    posestamped1.header.stamp=ros::Time::now();
    posestamped1.pose.position.x= -4.14;
    posestamped1.pose.position.y= -0.84;
    posestamped1.pose.position.z= 0;
    posestamped1.pose.orientation.x=0;
    posestamped1.pose.orientation.y=0;
    posestamped1.pose.orientation.z=-0.7;
    posestamped1.pose.orientation.w=0.7;



    posestamped2.header.frame_id="/myframe_v";
    posestamped2.header.stamp=ros::Time::now();
    posestamped2.pose.position.x= -1.98;
    posestamped2.pose.position.y= -0.84;
    posestamped2.pose.position.z= 0;
    posestamped2.pose.orientation.x=0;
    posestamped2.pose.orientation.y=0;
    posestamped2.pose.orientation.z=-0.7;
    posestamped2.pose.orientation.w=0.7;




    posestamped3.header.frame_id="/myframe_v";
    posestamped3.header.stamp=ros::Time::now();
    posestamped3.pose.position.x= 0;
    posestamped3.pose.position.y= -0.84;
    posestamped3.pose.position.z= 0;
    posestamped3.pose.orientation.x=0;
    posestamped3.pose.orientation.y=0;
    posestamped3.pose.orientation.z=-0.7;
    posestamped3.pose.orientation.w=0.7;





    posestamped4.header.frame_id="/myframe_v";
    posestamped4.header.stamp=ros::Time::now();
    posestamped4.pose.position.x= 1.98;
    posestamped4.pose.position.y= -0.84;
    posestamped4.pose.position.z= 0;
    posestamped4.pose.orientation.x=0;
    posestamped4.pose.orientation.y=0;
    posestamped4.pose.orientation.z=-0.7;
    posestamped4.pose.orientation.w=0.7;





    posestamped5.header.frame_id="/myframe_v";
    posestamped5.header.stamp=ros::Time::now();
    posestamped5.pose.position.x= 4.14;
    posestamped5.pose.position.y= -0.84;
    posestamped5.pose.position.z= 0;
    posestamped5.pose.orientation.x=0;
    posestamped5.pose.orientation.y=0;
    posestamped5.pose.orientation.z=-0.7;
    posestamped5.pose.orientation.w=0.7;





   if(msg->point.x>=-5.2&&msg->point.x<=-3.2&&msg->point.y>=-3.21&msg->point.y<=0)
   {
    posePoseStamped_pub.publish(posestamped1);
   }
  else if(msg->point.x>=-3.1&&msg->point.x<=-1.1&&msg->point.y>=-3.21&msg->point.y<=0)
   {
    posePoseStamped_pub.publish(posestamped2);
   }
  else if(msg->point.x>=-1&&msg->point.x<=1&&msg->point.y>=-3.21&msg->point.y<=0)
   {
    posePoseStamped_pub.publish(posestamped3);
   }
  else if(msg->point.x>=1.1&&msg->point.x<=3.1&&msg->point.y>=-3.21&msg->point.y<=0)
   {
    posePoseStamped_pub.publish(posestamped4);
   }
  else if(msg->point.x>=3.2&&msg->point.x<=5.2&&msg->point.y>=-3.21&msg->point.y<=0)
   {
    posePoseStamped_pub.publish(posestamped5);
   }




  else if(msg->point.x<-5.2&&msg->point.x>5.2&&msg->point.y<-3.21&msg->point.y>0)
   {

     ROS_INFO("The parking point is out of parking lots!! Please, reset it");

   }










  //Parallel parking_lot

    geometry_msgs::PoseStamped posestamped6,posestamped7,posestamped8;
    posestamped6.header.frame_id="/myframe_p";
    posestamped6.header.stamp=ros::Time::now();
    posestamped6.pose.position.x= -0.77;
    posestamped6.pose.position.y= -1.12;
    posestamped6.pose.position.z= 0;
    posestamped6.pose.orientation.x=0;
    posestamped6.pose.orientation.y=0;
    posestamped6.pose.orientation.z=0;
    posestamped6.pose.orientation.w=1;



   posestamped7.header.frame_id="/myframe_p";
   posestamped7.header.stamp=ros::Time::now();
   posestamped7.pose.position.x= 2.75;
   posestamped7.pose.position.y= -0.96;
   posestamped7.pose.position.z= 0;
   posestamped7.pose.orientation.x=0;
   posestamped7.pose.orientation.y=0;
   posestamped7.pose.orientation.z=0;
   posestamped7.pose.orientation.w=1;




   posestamped8.header.frame_id="/myframe_p";
   posestamped8.header.stamp=ros::Time::now();
   posestamped8.pose.position.x= -4.24;
   posestamped8.pose.position.y= -1.06;
   posestamped8.pose.position.z= 0;
   posestamped8.pose.orientation.x=0;
   posestamped8.pose.orientation.y=0;
   posestamped8.pose.orientation.z=0;
   posestamped8.pose.orientation.w=1;





   if(msg->point.x>=-5.2&&msg->point.x<=-1.79&&msg->point.y>=-2.1&msg->point.y<=0)
  {
   posePoseStamped_pub.publish(posestamped8);
  }
   else if(msg->point.x>=-1.69&&msg->point.x<=1.72&&msg->point.y>=-2.1&msg->point.y<=0)
  {
   posePoseStamped_pub.publish(posestamped6);
  }
   else if(msg->point.x>=1.82&&msg->point.x<=5.22&&msg->point.y>=-2.1&msg->point.y<=0)
  {
   posePoseStamped_pub.publish(posestamped7);
  }




  else if(msg->point.x<-5.2&&msg->point.x>5.22&&msg->point.y<-2.1&msg->point.y>0)
  {

   ROS_INFO("The parking point is out of parking lots!! Please, reset it");

  }

  }




  int main( int argc, char** argv )
  {
      ros::init(argc, argv, "parking_goal");
      ros::NodeHandle n;
      ros::Publisher marker_v_pub = n.advertise<visualization_msgs::Marker>("visualization_marker_v", 10);
      ros::Publisher marker_p_pub = n.advertise<visualization_msgs::Marker>("visualization_marker_p", 10);
      ros::Subscriber sub = n.subscribe("clicked_point", 10, chatterCallback);
      posePoseStamped_pub= n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);


     ros::Rate r(30);


  while (ros::ok())
  {

      visualization_msgs::Marker points_v,points_p ,line_strip_v,line_strip_p;
      points_v.header.frame_id = line_strip_v.header.frame_id = "/myframe_v";
      points_p.header.frame_id = line_strip_p.header.frame_id = "/myframe_p";
      points_v.header.stamp = line_strip_v.header.stamp = ros::Time::now();
      points_p.header.stamp = line_strip_p.header.stamp = ros::Time::now();
      points_v.ns = line_strip_v.ns  =  points_p.ns = line_strip_p.ns  = "parking_goal";
      points_v.pose.orientation.w = line_strip_v.pose.orientation.w = 1.0;
      points_p.pose.orientation.w = line_strip_p.pose.orientation.w = 1.0;


      points_v.id = 0;
      line_strip_v.id = 1;
      points_p.id = 0;
      line_strip_p.id = 1;




      points_v.type = visualization_msgs::Marker::POINTS;
      line_strip_v.type = visualization_msgs::Marker::LINE_STRIP;
      points_p.type = visualization_msgs::Marker::POINTS;
      line_strip_p.type = visualization_msgs::Marker::LINE_STRIP;



      // POINTS markers use x and y scale for width/height respectively
      points_v.scale.x = 0.05;
      points_v.scale.y = 0.05;
      points_p.scale.x = 0.05;
      points_p.scale.y = 0.05;
      // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width

      line_strip_v.scale.x = 0.1;
      line_strip_p.scale.x = 0.1;




      // Points are green
      points_v.color.g = 1.0f;
      points_v.color.a = 0.7;
      points_p.color.g = 1.0f;
      points_p.color.a = 0.7;
      // Line strip is blmapue
      line_strip_v.color.b = 1.0;
      line_strip_v.color.a = 1.0;
      line_strip_p.color.b = 1.0;
      line_strip_p.color.a = 1.0;







     //Vertical parking_lot
      geometry_msgs::Point p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p12,p13,p14,p15,p16,p17,p18,p19,p20;
      p1.x = -1.0;
      p1.y = 0;
      p1.z = 0;
   
      p2.x=-1.1;
      p2.y=0;
      p2.z=0;

      p3.x=-3.1;
      p3.y=0;
      p3.z=0;

      p4.x=-3.1;
      p4.y=-3.21;
      p4.z=0;

      p5.x=-1.1;
      p5.y=-3.21;
      p5.z=0;

      p6.x=-1.0;
      p6.y=-3.21;
      p6.z=0;

      p7.x=1.0;
      p7.y=-3.21;
      p7.z=0;

      p8.x=1.1;
      p8.y=-3.21;
      p8.z=0;

      p9.x=3.1;
      p9.y=-3.21;
      p9.z=0;

      p10.x=3.1;
      p10.y=0;
      p10.z=0;

      p11.x=1.1;
      p11.y=0;
      p11.z=0;

      p12.x=1.0;
      p12.y=0;
      p12.z=0;


      p13.x=-3.2;
      p13.y=0;
      p13.z=0;


      p14.x=-5.2;
      p14.y=0;
      p14.z=0;


      p15.x=-5.2;
      p15.y=-3.21;
      p15.z=0;


      p16.x=-3.2;
      p16.y=-3.21;
      p16.z=0;


      p17.x=3.2;
      p17.y=0;
      p17.z=0;


      p18.x=5.2;
      p18.y=0;
      p18.z=0;


      p19.x=5.2;
      p19.y=-3.21;
      p19.z=0;


      p20.x=3.2;
      p20.y=-3.21;
      p20.z=0;

     




     


    
   
        

      points_v.points.push_back(p1);
      points_v.points.push_back(p2);
      points_v.points.push_back(p3);
      points_v.points.push_back(p4);
      points_v.points.push_back(p5);
      points_v.points.push_back(p6);
      points_v.points.push_back(p7);
      points_v.points.push_back(p8);
      points_v.points.push_back(p9);
      points_v.points.push_back(p10);
      points_v.points.push_back(p11);
      points_v.points.push_back(p12);
      points_v.points.push_back(p13);
      points_v.points.push_back(p14);
      points_v.points.push_back(p15);
      points_v.points.push_back(p16);
      points_v.points.push_back(p17);
      points_v.points.push_back(p18);
      points_v.points.push_back(p19);
      points_v.points.push_back(p20);






      line_strip_v.points.push_back(p15);
      line_strip_v.points.push_back(p14);
      line_strip_v.points.push_back(p13);
      line_strip_v.points.push_back(p16);
       line_strip_v.points.push_back(p4);
      line_strip_v.points.push_back(p3);
      line_strip_v.points.push_back(p2);
      line_strip_v.points.push_back(p5);
      line_strip_v.points.push_back(p6);
      line_strip_v.points.push_back(p1);
      line_strip_v.points.push_back(p12);
      line_strip_v.points.push_back(p7);
      line_strip_v.points.push_back(p8);
      line_strip_v.points.push_back(p11);
      line_strip_v.points.push_back(p10);
      line_strip_v.points.push_back(p9);
      line_strip_v.points.push_back(p20);
      line_strip_v.points.push_back(p17);
      line_strip_v.points.push_back(p18);
      line_strip_v.points.push_back(p19);
      
      
       ros::Publisher marker_p_pub = n.advertise<visualization_msgs::Marker>("visualization_marker_p", 10);
      
      


    marker_v_pub.publish(points_v);
    marker_v_pub.publish(line_strip_v);
    //marker_pub.publish(line_list);














    //Parallel parking_lot

    geometry_msgs::Point p21,p22,p23,p24,p25,p26,p27,p28,p29,p30,p31,p32;
        p21.x = -1.705;
        p21.y = 0;
        p21.z = 0;

        p22.x=-1.805;
        p22.y=0;
        p22.z=0;

        p23.x=-5.215;
        p23.y=0;
        p23.z=0;

        p24.x=-5.215;
        p24.y=-2.1;
        p24.z=0;

        p25.x=-1.805;
        p25.y=-2.1;
        p25.z=0;

        p26.x=-1.705;
        p26.y=-2.1;
        p26.z=0;

        p27.x=1.705;
        p27.y=-2.1;
        p27.z=0;

        p28.x=1.805;
        p28.y=-2.1;
        p28.z=0;

        p29.x=5.215;
        p29.y=-2.1;
        p29.z=0;

        p30.x=5.215;
        p30.y=0;
        p30.z=0;

        p31.x=1.805;
        p31.y=0;
        p31.z=0;

        p32.x=1.705;
        p32.y=0;
        p32.z=0;





        points_p.points.push_back(p21);
        points_p.points.push_back(p22);
        points_p.points.push_back(p23);
        points_p.points.push_back(p24);
        points_p.points.push_back(p25);
        points_p.points.push_back(p26);
        points_p.points.push_back(p27);
        points_p.points.push_back(p28);
        points_p.points.push_back(p29);
        points_p.points.push_back(p30);
        points_p.points.push_back(p31);
        points_p.points.push_back(p32);







        line_strip_p.points.push_back(p24);
        line_strip_p.points.push_back(p23);
        line_strip_p.points.push_back(p22);
        line_strip_p.points.push_back(p25);
        line_strip_p.points.push_back(p26);
        line_strip_p.points.push_back(p21);
        line_strip_p.points.push_back(p32);
        line_strip_p.points.push_back(p27);
        line_strip_p.points.push_back(p28);
        line_strip_p.points.push_back(p31);
        line_strip_p.points.push_back(p30);
        line_strip_p.points.push_back(p29);





        marker_p_pub.publish(points_p);
        marker_p_pub.publish(line_strip_p);
        //marker_pub.publish(line_list);

        //ros::spin();
        ros::spinOnce();
        r.sleep();

  }

  }

