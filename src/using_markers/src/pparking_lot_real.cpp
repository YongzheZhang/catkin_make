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
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/myframe_p";
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

      geometry_msgs::Point p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p12;
      p1.x = -2.65;
      p1.y = 0;
      p1.z = 0;
   
      p2.x=-2.75;
      p2.y=0;
      p2.z=0;

      p3.x=-8.05;
      p3.y=0;
      p3.z=0;

      p4.x=-8.05;
      p4.y=-2.4;
      p4.z=0;

      p5.x=-2.75;
      p5.y=-2.4;
      p5.z=0;

      p6.x=-2.65;
      p6.y=-2.4;
      p6.z=0;

      p7.x=2.65;
      p7.y=-2.4;
      p7.z=0;

      p8.x=2.75;
      p8.y=-2.4;
      p8.z=0;

      p9.x=8.05;
      p9.y=-2.4;
      p9.z=0;

      p10.x=8.05;
      p10.y=0;
      p10.z=0;

      p11.x=2.75;
      p11.y=0;
      p11.z=0;

      p12.x=2.65;
      p12.y=0;
      p12.z=0;

    
   
        

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
