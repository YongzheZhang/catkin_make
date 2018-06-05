#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include <unistd.h>
#define PI 3.1416




   struct street_line
  {
   int number;
   double length_T;
   double distance;
   double length;
   double width;
   geometry_msgs::Pose origin;
   visualization_msgs::Marker point_array;
   visualization_msgs::Marker line_list;
  };

 street_line street_line1,street_line2, street_line3, street_line4;




 struct decoration_trees
 {
  double radius;
  double height;
  geometry_msgs::Pose origin;
  visualization_msgs::Marker Cylinder;
 };

decoration_trees tree_1,tree_2,tree_3,tree_4;



   int main( int argc, char** argv )
  {

    ros::init(argc, argv, "Decoration_AP_node");
    ros::NodeHandle private_nhh ("");


    ros::Publisher tree_1_pub = private_nhh.advertise<visualization_msgs::Marker>("tree_1", 10);
    ros::Publisher tree_2_pub = private_nhh.advertise<visualization_msgs::Marker>("tree_2", 10);
    ros::Publisher tree_3_pub = private_nhh.advertise<visualization_msgs::Marker>("tree_3", 10);
    ros::Publisher tree_4_pub = private_nhh.advertise<visualization_msgs::Marker>("tree_4", 10);
    ros::Publisher street_line1_pub = private_nhh.advertise<visualization_msgs::Marker>("street_line1", 10);
    ros::Publisher street_point1_pub = private_nhh.advertise<visualization_msgs::Marker>("street_point1", 10);
    ros::Publisher street_line2_pub = private_nhh.advertise<visualization_msgs::Marker>("street_line2", 10);
    ros::Publisher street_line3_pub = private_nhh.advertise<visualization_msgs::Marker>("street_line3", 10);
    ros::Publisher street_line4_pub = private_nhh.advertise<visualization_msgs::Marker>("street_line4", 10);



    private_nhh.param("tree_1_height", tree_1.height, 5.0);
    private_nhh.param("tree_1_radius", tree_1.radius, 1.0);
    private_nhh.param("tree_1_position_x", tree_1.origin.position.x, -12.0);
    private_nhh.param("tree_1_position_y", tree_1.origin.position.y, 0.5);

    private_nhh.param("tree_2_position_x", tree_2.origin.position.x, -0.83);
    private_nhh.param("tree_2_position_y", tree_2.origin.position.y, 0.5);

    private_nhh.param("tree_3_position_x", tree_3.origin.position.x, 10.13);
    private_nhh.param("tree_3_position_y", tree_3.origin.position.y, 0.5);

    private_nhh.param("tree_4_position_x", tree_4.origin.position.x, 20.7);
    private_nhh.param("tree_4_position_y", tree_4.origin.position.y, 0.5);


    tree_1.Cylinder.type=tree_2.Cylinder.type=tree_3.Cylinder.type=tree_4.Cylinder.type=visualization_msgs::Marker::CYLINDER;
    tree_1.Cylinder.header.frame_id=tree_2.Cylinder.header.frame_id=tree_3.Cylinder.header.frame_id=tree_4.Cylinder.header.frame_id="/map";


      tree_2.Cylinder.color.g=tree_3.Cylinder.color.g=tree_4.Cylinder.color.g=1.0;
    tree_1.Cylinder.color.a=tree_2.Cylinder.color.a=tree_3.Cylinder.color.a=tree_4.Cylinder.color.a=1.0;

     tree_1.Cylinder.color.r=0.0;
     tree_1.Cylinder.color.g=0.0;
     tree_1.Cylinder.color.b=0.0;

    //Tree diameter=1
    tree_1.Cylinder.scale.x=tree_2.Cylinder.scale.x=tree_3.Cylinder.scale.x=tree_4.Cylinder.scale.x=2*tree_1.radius;
    tree_1.Cylinder.scale.y=tree_2.Cylinder.scale.y=tree_3.Cylinder.scale.y=tree_4.Cylinder.scale.y=2*tree_1.radius;
    //Tree height
    tree_1.Cylinder.scale.z=tree_2.Cylinder.scale.z=tree_3.Cylinder.scale.z=tree_4.Cylinder.scale.z=tree_1.height;


    //tree_1 location
    tree_1.Cylinder.pose.position.x=tree_1.origin.position.x;
    tree_1.Cylinder.pose.position.y=tree_1.origin.position.y;
    tree_1.Cylinder.pose.position.z=tree_1.height/2;

    //tree_2 location
    tree_2.Cylinder.pose.position.x=tree_2.origin.position.x;
    tree_2.Cylinder.pose.position.y=tree_2.origin.position.y;
    tree_2.Cylinder.pose.position.z=tree_1.height/2;

    //tree_3 location
    tree_3.Cylinder.pose.position.x=tree_3.origin.position.x;
    tree_3.Cylinder.pose.position.y=tree_3.origin.position.y;
    tree_3.Cylinder.pose.position.z=tree_1.height/2;

    //tree_4 location
    tree_4.Cylinder.pose.position.x=tree_4.origin.position.x;
    tree_4.Cylinder.pose.position.y=tree_4.origin.position.y;
    tree_4.Cylinder.pose.position.z=tree_1.height/2;




    // Tree decoration end


    private_nhh.param("origin_x", street_line1.origin.position.x, -5.0);
    private_nhh.param("origin_y", street_line1.origin.position.y, -7.0);
    private_nhh.param("Total_length", street_line1.length_T, 20.0);
    private_nhh.param("distance_between_each_lines", street_line1.distance, 1.0);
    private_nhh.param("Length_each_lines", street_line1.length, 2.0);
    private_nhh.param("Street_width", street_line1.width, 3.0);
    private_nhh.param("Numbers", street_line1.number, 10);

   //Line1
   street_line1.point_array.type= street_line3.point_array.type=visualization_msgs::Marker::POINTS;
   street_line1.line_list.type=street_line3.line_list.type=visualization_msgs::Marker::LINE_LIST;
   street_line1.point_array.header.frame_id=street_line1.line_list.header.frame_id
  =street_line3.point_array.header.frame_id=street_line3.line_list.header.frame_id="/map";
    street_line1.line_list.color.r = street_line3.line_list.color.r = 1.0;
    street_line1.line_list.color.g = street_line3.line_list.color.g =1.0;
    street_line1.line_list.color.b = street_line3.line_list.color.b = 1.0;
    street_line1.line_list.color.a = street_line3.line_list.color.a =1.0;
    street_line1.line_list.scale.x = street_line3.line_list.scale.x =0.2;
    street_line1.point_array.color.r = street_line3.point_array.color.r =0.0;
    street_line1.point_array.color.g = street_line3.point_array.color.g = 1.0;
    street_line1.point_array.color.b = street_line3.point_array.color.b =0.0;
    street_line1.point_array.color.a = street_line3.point_array.color.a =1.0;
    street_line1.point_array.scale.x =  street_line3.point_array.scale.x =0.1;
    street_line1.point_array.scale.y = street_line3.point_array.scale.y = 0.2;


    //Line2   #declare Gold = color red 0.8 green 0.498039 blue 0.196078
    street_line2.point_array.type=street_line4.point_array.type=visualization_msgs::Marker::POINTS;
    street_line2.line_list.type=street_line4.line_list.type=visualization_msgs::Marker::LINE_LIST;
    street_line2.point_array.header.frame_id=street_line2.line_list.header.frame_id=street_line4.point_array.header.frame_id=street_line4.line_list.header.frame_id="/map";
    street_line2.line_list.color.r = street_line4.line_list.color.r =1.0;
    street_line2.line_list.color.g = street_line4.line_list.color.g = 0.87;
    street_line2.line_list.color.b = street_line4.line_list.color.b =0.0;
    street_line2.line_list.color.a = street_line4.line_list.color.a =1.0;
    street_line2.line_list.scale.x =   street_line4.line_list.scale.x =0.1;
    street_line2.point_array.color.r =street_line4.point_array.color.r = 0.0;
    street_line2.point_array.color.g = street_line4.point_array.color.g =1.0;
    street_line2.point_array.color.b = street_line4.point_array.color.b =0.0;
    street_line2.point_array.color.a =street_line4.point_array.color.a = 1.0;
    street_line2.point_array.scale.x = street_line4.point_array.scale.x = 0.1;
    street_line2.point_array.scale.y = street_line4.point_array.scale.y =0.2;



    for(int i=0;i<street_line1.number;i++){
       if(i%2==0)
       {
        geometry_msgs::Point pt1;
        pt1.x = street_line1.origin.position.x+(street_line1.length+street_line1.distance)*i/2;
        pt1.y = street_line1.origin.position.y;
       street_line1.point_array.points.push_back(pt1);

       }

       else
       {
         geometry_msgs::Point pt1;
         pt1.x = street_line1.origin.position.x+street_line1.length+(street_line1.length+street_line1.distance)*(i-1)/2;
         pt1.y = street_line1.origin.position.y;
       street_line1.point_array.points.push_back(pt1);

       }
       }
     for(int i=0;i<street_line1.number;i=i+2){

             street_line1.line_list.points.push_back(street_line1.point_array.points.at(i));
             street_line1.line_list.points.push_back(street_line1.point_array.points.at(i+1));
     }
     geometry_msgs::Point pt2,pt3;
     pt2.x=street_line1.origin.position.x;
     pt2.y=street_line1.origin.position.y+street_line1.width;
     street_line2.point_array.points.push_back(pt2);
     street_line2.line_list.points.push_back(pt2);
     pt2.x=street_line1.origin.position.x+(street_line1.length+street_line1.distance)*street_line1.number/2;
     pt2.y=street_line1.origin.position.y+street_line1.width;
     street_line2.point_array.points.push_back(pt2);
      street_line2.line_list.points.push_back(pt2);


      pt3.x=street_line1.origin.position.x;
      pt3.y=street_line1.origin.position.y-street_line1.width;
      street_line2.point_array.points.push_back(pt3);
      street_line2.line_list.points.push_back(pt3);
      pt3.x=street_line1.origin.position.x+(street_line1.length+street_line1.distance)*street_line1.number/2;
      pt3.y=street_line1.origin.position.y-street_line1.width;
      street_line2.point_array.points.push_back(pt3);
       street_line2.line_list.points.push_back(pt3);

  //Line1 done








       //Line3 and Line4
           private_nhh.param("s_origin_x", street_line3.origin.position.x, -5.0);
           private_nhh.param("s_origin_y", street_line3.origin.position.y, -7.0);
           private_nhh.param("s_Total_length", street_line3.length_T, 20.0);
           private_nhh.param("s_distance_between_each_lines", street_line3.distance, 1.0);
           private_nhh.param("s_Length_each_lines", street_line3.length, 2.0);
           private_nhh.param("s_Street_width", street_line3.width, 3.0);
           private_nhh.param("s_Numbers", street_line3.number, 10);



           for(int i=0;i<street_line3.number;i++){
              if(i%2==0)
              {
               geometry_msgs::Point pt1;
               pt1.x = street_line3.origin.position.x+(street_line3.length+street_line3.distance)*i/2;
               pt1.y = street_line3.origin.position.y;
              street_line3.point_array.points.push_back(pt1);

              }

              else
              {
                geometry_msgs::Point pt1;
                pt1.x = street_line3.origin.position.x+street_line3.length+(street_line3.length+street_line3.distance)*(i-1)/2;
                pt1.y = street_line3.origin.position.y;
              street_line3.point_array.points.push_back(pt1);

              }
              }
            for(int i=0;i<street_line3.number;i=i+2){

                    street_line3.line_list.points.push_back(street_line3.point_array.points.at(i));
                    street_line3.line_list.points.push_back(street_line3.point_array.points.at(i+1));
            }


            pt2.x=street_line3.origin.position.x;
            pt2.y=street_line3.origin.position.y+street_line3.width;
            street_line4.point_array.points.push_back(pt2);
            street_line4.line_list.points.push_back(pt2);
            pt2.x=street_line3.origin.position.x+(street_line3.length+street_line3.distance)*street_line3.number/2;
            pt2.y=street_line3.origin.position.y+street_line3.width;
            street_line4.point_array.points.push_back(pt2);
            street_line4.line_list.points.push_back(pt2);


             pt3.x=street_line3.origin.position.x;
             pt3.y=street_line3.origin.position.y-street_line3.width;
             street_line4.point_array.points.push_back(pt3);
             street_line4.line_list.points.push_back(pt3);
             pt3.x=street_line3.origin.position.x+(street_line3.length+street_line3.distance)*street_line3.number/2;
             pt3.y=street_line3.origin.position.y-street_line3.width;
             street_line4.point_array.points.push_back(pt3);
             street_line4.line_list.points.push_back(pt3);



    ros::Rate r(10);
   while(ros::ok()){

       tree_1_pub.publish(tree_1.Cylinder);
       tree_2_pub.publish(tree_2.Cylinder);
       tree_3_pub.publish(tree_3.Cylinder);
       tree_4_pub.publish(tree_4.Cylinder);
       street_line1_pub.publish(street_line1.line_list);
       street_point1_pub.publish(street_line1.point_array);
       street_line2_pub.publish(street_line2.line_list);
       street_line3_pub.publish(street_line3.line_list);
       street_line4_pub.publish(street_line4.line_list);
       r.sleep();
    }
    }

