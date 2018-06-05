#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::Rate r(30);

  float f = 0.0;
  while (ros::ok())
  {

    visualization_msgs::Marker points, line_strip, line_list;
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/myframe";
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = "points_and_lines";
    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;



    points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;



    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;



    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.05;
    points.scale.y = 0.05;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.03;
    line_list.scale.x = 0.1;



    // Points are green
    points.color.g = 1.0f;
    points.color.a = 0.7;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;



    // Create the vertices for the points and lines
  //  for (uint32_t i = 0; i < 100; ++i)
    //{
      //float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
     // float z = 5 * cos(f + i / 100.0f * 2 * M_PI);

      geometry_msgs::Point p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p12,p13,p14,p15,p16,p17,p18,p19,p20;
      p1.x = -1.15;
      p1.y = 0;
      p1.z = 0;
   
      p2.x=-1.25;
      p2.y=0;
      p2.z=0;

      p3.x=-3.55;
      p3.y=0;
      p3.z=0;

      p4.x=-3.55;
      p4.y=-4.8;
      p4.z=0;

      p5.x=-1.25;
      p5.y=-4.8;
      p5.z=0;

      p6.x=-1.15;
      p6.y=-4.8;
      p6.z=0;

      p7.x=1.15;
      p7.y=-4.8;
      p7.z=0;

      p8.x=1.25;
      p8.y=-4.8;
      p8.z=0;

      p9.x=3.55;
      p9.y=-4.8;
      p9.z=0;

      p10.x=3.55;
      p10.y=0;
      p10.z=0;

      p11.x=1.25;
      p11.y=0;
      p11.z=0;

      p12.x=1.15;
      p12.y=0;
      p12.z=0;

    
      p13.x=-3.65;
       p13.y=0;
       p13.z=0;


       p14.x=-5.95;
       p14.y=0;
       p14.z=0;



       p15.x=-5.95;
       p15.y=-4.8;
       p15.z=0;


       p16.x=-3.65;
       p16.y=-4.8;
       p16.z=0;


       p17.x=3.65;
       p17.y=0;
       p17.z=0;


       p18.x=5.95;
       p18.y=0;
       p18.z=0;


       p19.x=5.95;
       p19.y=-4.8;
       p19.z=0;


       p20.x= 3.65;
       p20.y= -4.8;
       p20.z= 0;
        

      points.points.push_back(p1);
      points.points.push_back(p2);
      points.points.push_back(p3);
      points.points.push_back(p4);
      points.points.push_back(p5);
      points.points.push_back(p6);
      points.points.push_back(p7);
      points.points.push_back(p8);
      points.points.push_back(p9);
      points.points.push_back(p10);
      points.points.push_back(p11);
      points.points.push_back(p12);
      points.points.push_back(p13);
      points.points.push_back(p14);
      points.points.push_back(p15);
      points.points.push_back(p16);
      points.points.push_back(p17);
      points.points.push_back(p18);
      points.points.push_back(p19);
      points.points.push_back(p20);






      line_strip.points.push_back(p15);
      line_strip.points.push_back(p14);
      line_strip.points.push_back(p13);
      line_strip.points.push_back(p16);
       line_strip.points.push_back(p4);
      line_strip.points.push_back(p3);
      line_strip.points.push_back(p2);
      line_strip.points.push_back(p5);
      line_strip.points.push_back(p6);      
      line_strip.points.push_back(p1);
      line_strip.points.push_back(p12);
      line_strip.points.push_back(p7);      
      line_strip.points.push_back(p8);
      line_strip.points.push_back(p11);
      line_strip.points.push_back(p10);
      line_strip.points.push_back(p9);
      line_strip.points.push_back(p20);
      line_strip.points.push_back(p17);
      line_strip.points.push_back(p18);
      line_strip.points.push_back(p19);
      
      
      
      
      


 
    

      // The line list needs two points for each line
  //    line_list.points.push_back(p1);
      p1.z += 1.0;
    //  line_list.points.push_back(p1);
    //}


    marker_pub.publish(points);
    marker_pub.publish(line_strip);
    //marker_pub.publish(line_list);

    r.sleep();

  //  f += 0.04;
  }
}
