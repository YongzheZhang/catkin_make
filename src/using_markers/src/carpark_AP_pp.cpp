#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include <geometry_msgs/PolygonStamped.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Point32.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "tf/tf.h"
#include <unistd.h>
#include <vector>
#include <costmap_2d/costmap_2d.h>
#define PI 3.1416


    ros::Publisher pose_pub;
    ros::Publisher half_circle_poses_pub,half_circle_poses_2_pub, half_circle_pub, half_circle_left_pub;
    ros::Subscriber goalReach_sub,costmap_sub;
    ros::Publisher car_polygon_pub,car_polygon_left_pub,car_polygon_vector_pub;

//    costmap_2d::Costmap2D map;
//    int occupied_min_value_=100;

    struct car_park{
    int number;                          // carpark lot numbers
    double width;                        // width of carpark lot
    double depth;                        // height of carpark lot
    double line_width;                   // line width
    double yaw;                         // yaw angle of carpark
    double offset_x;                     // x offset lot center
    double offset_y;                     // y offset lot center
    double offset_angle;
    geometry_msgs::Pose origin;          // origin in map coordinates
    visualization_msgs::Marker point_array;   // points of carpark lot
    visualization_msgs::Marker line_array;    // lines of carpark lot
    geometry_msgs::PoseArray lot_reference;   // carpark lot center points, this is for parking reference, not the actual center of carpark lot
    geometry_msgs::PoseArray lot_reference_2;
    geometry_msgs::PoseArray lot_center;
    geometry_msgs::PoseArray step1_left;
    geometry_msgs::PoseArray step1_right;
    nav_msgs::Odometry direction;
    };

    car_park carpark_vertical, carpark_parallel ,carpark_degree;

    double trianglearea(geometry_msgs::Point A , geometry_msgs::Point B , geometry_msgs::Point C)
    {

   //  Equation: Atriangle= 1/2(x1(y2−y3)+x2(y3−y1)+x3(y1−y2)

      double s=fabs((A.x*(B.y-C.y)+B.x*(C.y-A.y)+C.x*(A.y-B.y)))/2;
       std::cout<<"A is "<<A<<std::endl;
       std::cout<<"B is "<<B<<std::endl;
       std::cout<<"C is "<<C<<std::endl;
       std::cout<<"s is "<<s<<std::endl;
       return s;
    }


   double Vparking_lot_area;//=carpark_vertical.width*carpark_vertical.depth;
   double Pparking_lot_area;//=carpark_parallel.width*carpark_parallel.depth;
   double parking_lot_area; // 45degree parking

   double radius_min,angle_max ,angle_d,distance,angle_i;
   double radius_min_2,angle_max_2 ,angle_d_2,distance_2,angle_i_2;
   int circle_num;
   double steer_angle;

   double radius;
   geometry_msgs::PoseArray poses_car,poses_car_2;
   std::vector<geometry_msgs::Pose> points_on_circles_vector;


   void clickedptCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
   {

      poses_car.header.frame_id="/map";
      poses_car_2.header.frame_id="/map";




         geometry_msgs::PoseStamped goal1;
         geometry_msgs::PolygonStamped car_polygon,car_polygon_2;
         geometry_msgs::Point32 car_points, car_points_2;
         nav_msgs::Path car_poses;
         nav_msgs::Path car_poses_left;


        goal1.header.frame_id="/map";        
        car_polygon.header.frame_id="/map";
        car_polygon_2.header.frame_id="/map";       
        car_polygon.header.seq=1;
        car_poses.header.frame_id="/map";
        car_poses_left.header.frame_id="/map";


   // .....................................................................................
       int indexV_points, indexV_lot, indexP_points, indexP_lot,indexdegree_points, indexdegree_lot, select_lot = 3;
       double dist_to_lotcenter = 10.0;
       std::cout<<"car orientation "<<carpark_vertical.direction.pose.pose.orientation<<std::endl;
       std::cout<<"car orientation "<<carpark_parallel.direction.pose.pose.orientation<<std::endl;     

     
    for (int i=0; i<carpark_vertical.lot_center.poses.size();i++){
    double temp;
           temp=sqrt(pow(msg->point.x-carpark_vertical.lot_center.poses.at(i).position.x,2)+pow(msg->point.y-carpark_vertical.lot_center.poses.at(i).position.y,2));
     if(temp<dist_to_lotcenter){
           dist_to_lotcenter=temp;
           indexV_points=2*i;
           indexV_lot=i;
           select_lot=0;
   }
   }

    for (int i=0; i<carpark_parallel.lot_center.poses.size();i++){
   double temp;
          temp=sqrt(pow((msg->point.x-carpark_parallel.lot_center.poses.at(i).position.x),2)+pow((msg->point.y-carpark_parallel.lot_center.poses.at(i).position.y),2));
      if(temp<dist_to_lotcenter){
          dist_to_lotcenter=temp;
          indexP_points=2*i;
          indexP_lot=i;
          select_lot=1;
    }
    }


    for (int i=0; i<carpark_degree.lot_center.poses.size();i++){
   double temp;
          temp=sqrt(pow((msg->point.x-carpark_degree.lot_center.poses.at(i).position.x),2)+pow((msg->point.y-carpark_degree.lot_center.poses.at(i).position.y),2));
      if(temp<dist_to_lotcenter){
          dist_to_lotcenter=temp;
          indexdegree_points=2*i;
          indexdegree_lot=i;
          select_lot=2;
    }
    }
   //............................................................................................




   if(select_lot==0)
   {
      double area = trianglearea(carpark_vertical.point_array.points.at(indexV_points),carpark_vertical.point_array.points.at(indexV_points+1),msg->point)
                     +trianglearea(carpark_vertical.point_array.points.at(indexV_points+1),carpark_vertical.point_array.points.at(indexV_points+3),msg->point)
                     +trianglearea(carpark_vertical.point_array.points.at(indexV_points+2),carpark_vertical.point_array.points.at(indexV_points+3),msg->point)
                     +trianglearea(carpark_vertical.point_array.points.at(indexV_points),carpark_vertical.point_array.points.at(indexV_points+2),msg->point);

       std::cout<<"area is "<<area<<std::endl;
       std::cout<<"Vparking_lot_area is "<<Vparking_lot_area<<std::endl;

  if(fabs(area-Vparking_lot_area)<0.5)
        {

      if(carpark_vertical.direction.pose.pose.orientation.w>=0.0 && carpark_vertical.direction.pose.pose.orientation.w<=0.7)
      {
    if(carpark_vertical.lot_center.poses.at(1).orientation.z>=0 && carpark_vertical.lot_center.poses.at(1).orientation.z<=1)
       {


        goal1.pose=carpark_vertical.step1_left.poses.at(indexV_lot);
        pose_pub.publish(goal1);


    }
    else
    {
 
        goal1.pose=carpark_vertical.step1_right.poses.at(indexV_lot);
        pose_pub.publish(goal1);

    }
    }



      else if(carpark_vertical.direction.pose.pose.orientation.w>0.7 && carpark_vertical.direction.pose.pose.orientation.w<=1)
      {
       if(carpark_vertical.lot_center.poses.at(1).orientation.z>=0 && carpark_vertical.lot_center.poses.at(1).orientation.z<=1)
       {

          goal1.pose=carpark_vertical.step1_right.poses.at(indexV_lot);
          pose_pub.publish(goal1);
      }
       else
       {

           goal1.pose=carpark_vertical.step1_left.poses.at(indexV_lot);
           pose_pub.publish(goal1);


       }
      }
   }
}



    if(select_lot==1)
   {

    double area = trianglearea(carpark_parallel.point_array.points.at(indexP_points),carpark_parallel.point_array.points.at(indexP_points+1),msg->point)
                 +trianglearea(carpark_parallel.point_array.points.at(indexP_points+1),carpark_parallel.point_array.points.at(indexP_points+3),msg->point)
                 +trianglearea(carpark_parallel.point_array.points.at(indexP_points+2),carpark_parallel.point_array.points.at(indexP_points+3),msg->point)
                 +trianglearea(carpark_parallel.point_array.points.at(indexP_points),carpark_parallel.point_array.points.at(indexP_points+2),msg->point);

  std::cout<<"area is "<<area<<std::endl;
  std::cout<<"Pparking_lot_area is "<<Pparking_lot_area<<std::endl;



       //if(msg->point.x>carpark_parallel.point_array.points.at(indexP_points).x&&msg->point.x<carpark_parallel.point_array.points.at(indexP_points+2).x)



       int l=0;
       //One side-plot
      for(int t=0;t<=circle_num;t++)
      {
           geometry_msgs::Point points_on_circles_p;
          geometry_msgs::Point points;
    geometry_msgs::PoseStamped points_on_circles;
             std::vector<geometry_msgs::Point32> car_points_vector;
                std::vector<geometry_msgs::Point> points_on_circles_p_vector;
          //The center and radius of circles
           radius=radius_min+t*distance;
          points.x=msg->point.x-radius*sin(PI/2-steer_angle);   //*cos(carpark_vertical.yaw)
          points.y=msg->point.y+radius*cos(PI/2-steer_angle);    //sin(carpark_vertical.yaw)*



        for(double angle=angle_i;angle<angle_max;angle=angle+angle_d)
        {
            //Get the points on the circles

          points_on_circles.pose.position.x=points_on_circles_p.x=points.x+radius*cos(angle-carpark_vertical.yaw);
          points_on_circles.pose.position.y=points_on_circles_p.y=points.y-radius*sin(angle-carpark_vertical.yaw);
          tf::quaternionTFToMsg(tf::createQuaternionFromYaw(PI/2-(angle-carpark_vertical.yaw)+PI),points_on_circles.pose.orientation);

          std::cout<<"the angle is positive?  "<< (PI/2-(angle-carpark_vertical.yaw)+PI) <<std::endl;


          points_on_circles_p_vector.push_back(points_on_circles_p);
          poses_car.poses.push_back(points_on_circles.pose);
          car_poses.poses.push_back(points_on_circles);
           half_circle_pub.publish(car_poses);



          // car_poses.poses.clear();

          // car_poses.poses.reset();

          for(int w=0;w<=1;w++)
           {



              car_points_2.x=points_on_circles.pose.position.x-cos(0.521+(PI/2-(angle-carpark_vertical.yaw)+PI))*1.41+w*cos(PI/2-(angle-carpark_vertical.yaw)+PI)*2.44;
              car_points_2.y=points_on_circles.pose.position.y-sin(0.521+(PI/2-(angle-carpark_vertical.yaw)+PI))*1.41+w*sin(PI/2-(angle-carpark_vertical.yaw)+PI)*2.44;
              car_points_vector.push_back(car_points_2);



          }

       for(int w=0;w<=1;w++)
          {
           car_points.x=points_on_circles.pose.position.x-cos(0.521-(PI/2-(angle-carpark_vertical.yaw)+PI))*1.41+(1-w)*cos(PI/2-(angle-carpark_vertical.yaw)+PI)*2.44;
           car_points.y=points_on_circles.pose.position.y+sin(0.521-(PI/2-(angle-carpark_vertical.yaw)+PI))*1.41+(1-w)*sin(PI/2-(angle-carpark_vertical.yaw)+PI)*2.44;
           car_points_vector.push_back(car_points);

          }

       car_points_2.x=points_on_circles.pose.position.x-cos(0.521+(PI/2-(angle-carpark_vertical.yaw)+PI))*1.41;
       car_points_2.y=points_on_circles.pose.position.y-sin(0.521+(PI/2-(angle-carpark_vertical.yaw)+PI))*1.41;
       car_points_vector.push_back(car_points_2);

     for(int j=0; j<car_points_vector.size();j++)

     {
         std::cout<<"car_points_vector_size is  "<< car_points_vector.size() <<std::endl;

         geometry_msgs::Point32 fppt;
     fppt.x=car_points_vector.at(j).x;
     fppt.y=car_points_vector.at(j).y;
     car_polygon.polygon.points.push_back(fppt);
     }
     car_polygon.header.seq++;
     std::cout<<"car_polygon_sequence is "<<car_polygon.header.seq<<std::endl;
     car_polygon.header.stamp=ros::Time::now();
     car_polygon_pub.publish(car_polygon);
     car_polygon.polygon.points.clear();

     car_points_vector.clear();

    l=l+1;

         }
         }

      half_circle_poses_pub.publish(poses_car);
      poses_car.poses.clear();


       //The other side

      for(int t=0;t<=circle_num;t++)
      {
         geometry_msgs::Point points_on_circles_p;
         geometry_msgs::Point points;
         geometry_msgs::PoseStamped points_on_circles;
         std::vector<geometry_msgs::Point32> car_points_vector;
         std::vector<geometry_msgs::Point> points_on_circles_p_vector;
          //The center and radius of circles
          radius=radius_min_2+t*distance_2;
          points.x=msg->point.x+(radius)*sin(PI/2-steer_angle);
          points.y=msg->point.y-(radius)*cos(PI/2-steer_angle);



        for(double angle=angle_i_2; angle<angle_max_2;angle=angle+angle_d_2)
        {
            //Get the points on the circles

          points_on_circles.pose.position.x=points_on_circles_p.x=points.x-radius*cos(angle-carpark_vertical.yaw);
          points_on_circles.pose.position.y=points_on_circles_p.y=points.y-radius*sin(angle-carpark_vertical.yaw);
          tf::quaternionTFToMsg(tf::createQuaternionFromYaw(PI-((carpark_vertical.yaw-angle)-PI/2)),points_on_circles.pose.orientation);  //((angle-carpark_vertical.yaw)-PI/2)

             std::cout<<"the angle is negative?  "<<(PI-((carpark_vertical.yaw-angle)-PI/2))<<std::endl;


          points_on_circles_p_vector.push_back(points_on_circles_p);
          poses_car_2.poses.push_back(points_on_circles.pose);
          car_poses_left.poses.push_back(points_on_circles);


           half_circle_left_pub.publish(car_poses_left);



          // car_poses.poses.clear();




           //........................................

          for(int w=0;w<=1;w++)
           {
            car_points.x=points_on_circles.pose.position.x+cos(0.521+((carpark_vertical.yaw-angle)-PI/2))*1.41-w*cos((carpark_vertical.yaw-angle)-PI/2)*2.44;
            car_points.y=points_on_circles.pose.position.y-sin(0.521+((carpark_vertical.yaw-angle)-PI/2))*1.41+w*sin((carpark_vertical.yaw-angle)-PI/2)*2.44;
            car_points_vector.push_back(car_points);

            //std::cout<<"theta is  "<< ((carpark_vertical.yaw-angle)-PI/2)<<std::endl;
           // std::cout<<"angle_1 is  "<< (0.521+((carpark_vertical.yaw-angle)-PI/2)) <<std::endl;
             std::cout<<"car_points_x is  "<< car_points.y <<std::endl;
             std::cout<<"car_points_y is  "<< car_points.y <<std::endl;
          }


       for(int w=0;w<=1;w++)
          {
           car_points_2.x=points_on_circles.pose.position.x+cos(((carpark_vertical.yaw-angle)-PI/2)-0.521)*1.41-(1-w)*cos((carpark_vertical.yaw-angle)-PI/2)*2.44;
           car_points_2.y=points_on_circles.pose.position.y-sin(((carpark_vertical.yaw-angle)-PI/2)-0.521)*1.41+(1-w)*sin((carpark_vertical.yaw-angle)-PI/2)*2.44;
           car_points_vector.push_back(car_points_2);
         // std::cout<<"angle_2_x is  "<< (((carpark_vertical.yaw-angle)-PI/2)-0.521) <<std::endl;
           std::cout<<"car_points_2_x is  "<< car_points.y <<std::endl;
           std::cout<<"car_points_2_y is  "<< car_points.y <<std::endl;
          }

       car_points.x=points_on_circles.pose.position.x+cos(0.521+((carpark_vertical.yaw-angle)-PI/2))*1.41;
       car_points.y=points_on_circles.pose.position.y-sin(0.521+((carpark_vertical.yaw-angle)-PI/2))*1.41;
       car_points_vector.push_back(car_points);



     for(int j=0; j<car_points_vector.size();j++)

     {
        std::cout<<"car_points_vector_size_2 is  "<< car_points_vector.size() <<std::endl;

     geometry_msgs::Point32 fppt;
     fppt.x=car_points_vector.at(j).x;
     fppt.y=car_points_vector.at(j).y;
     car_polygon_2.polygon.points.push_back(fppt);
     }
     car_polygon_2.header.seq++;
     //std::cout<<"car_polygon_sequence is "<<car_polygon_2.header.seq<<std::endl;
     car_polygon_2.header.stamp=ros::Time::now();
     car_polygon_left_pub.publish(car_polygon_2);

     car_points_vector.clear();

    l=l+1;

         }
         }





      half_circle_poses_2_pub.publish(poses_car_2);
      poses_car_2.poses.clear();












    }





    if(select_lot==2)
    {



         double area = trianglearea(carpark_degree.point_array.points.at(indexdegree_points),carpark_degree.point_array.points.at(indexdegree_points+1),msg->point)
                      +trianglearea(carpark_degree.point_array.points.at(indexdegree_points+1),carpark_degree.point_array.points.at(indexdegree_points+3),msg->point)
                      +trianglearea(carpark_degree.point_array.points.at(indexdegree_points+2),carpark_degree.point_array.points.at(indexdegree_points+3),msg->point)
                      +trianglearea(carpark_degree.point_array.points.at(indexdegree_points),carpark_degree.point_array.points.at(indexdegree_points+2),msg->point);

        std::cout<<"area is "<<area<<std::endl;
        std::cout<<"parking_lot_area is "<<parking_lot_area<<std::endl;

   if(fabs(area-parking_lot_area)<0.5)
         {
         goal1.pose=carpark_degree.lot_reference.poses.at(indexdegree_lot);
         pose_pub.publish(goal1);
   }
   }
   }


   void step2Callback(const nav_msgs::Odometry::ConstPtr& msg)
   {
       geometry_msgs::PoseStamped goal1, goal2;
       goal1.header.frame_id
       =goal2.header.frame_id="/map";
       double dist_to_lotcenter = 10.0;
       int indexV_points, indexV_lot, indexP_points, indexP_lot, select_lot = 3;
       carpark_vertical.direction.pose.pose.orientation=msg->pose.pose.orientation;
       carpark_parallel.direction.pose.pose.orientation=msg->pose.pose.orientation;

           
    for (int i=0; i<carpark_vertical.lot_center.poses.size();i++){
    double temp;
           temp=sqrt(pow(msg->pose.pose.position.x-carpark_vertical.lot_center.poses.at(i).position.x,2)+pow(msg->pose.pose.position.y-carpark_vertical.lot_center.poses.at(i).position.y,2));
     if(temp<dist_to_lotcenter){
           dist_to_lotcenter=temp;
           indexV_points=2*i;
           indexV_lot=i;
           select_lot=0;
   }

   }
      for (int i=0; i<carpark_parallel.lot_center.poses.size();i++){
     double temp;
            temp=sqrt(pow((msg->pose.pose.position.x-carpark_parallel.lot_center.poses.at(i).position.x),2)+pow((msg->pose.pose.position.y-carpark_parallel.lot_center.poses.at(i).position.y),2));
        if(temp<dist_to_lotcenter){
            dist_to_lotcenter=temp;
            indexP_points=2*i;
            indexP_lot=i;
            select_lot=1;
    }
    }

   if(select_lot==0)
   {


          if(fabs(msg->pose.pose.position.x-carpark_vertical.step1_left.poses.at(abs(indexV_lot-1)).position.x) <0.1
           &&fabs(msg->pose.pose.position.y- carpark_vertical.step1_left.poses.at(abs(indexV_lot-1)).position.y)<0.1
           &&fabs(msg->pose.pose.orientation.w- carpark_vertical.step1_left.poses.at(abs(indexV_lot-1)).orientation.w)<0.05
           &&fabs(msg->pose.pose.orientation.z- carpark_vertical.step1_left.poses.at(abs(indexV_lot-1)).orientation.z)<0.05)
    {

       std::cout<<"step1_left.pose is "<<carpark_vertical.step1_left.poses.at(abs(indexV_lot-1))<<std::endl;

       goal2.pose=carpark_vertical.lot_reference.poses.at(abs(indexV_lot-1));
       ros::Duration(5.0).sleep();
       //usleep(5000000);
       pose_pub.publish(goal2);
   }

          if(fabs(msg->pose.pose.position.x-carpark_vertical.step1_left.poses.at(abs(carpark_vertical.lot_center.poses.size()-1)).position.x) <0.1
           &&fabs(msg->pose.pose.position.y- carpark_vertical.step1_left.poses.at(abs(carpark_vertical.lot_center.poses.size()-1)).position.y)<0.1
           &&fabs(msg->pose.pose.orientation.w- carpark_vertical.step1_left.poses.at(abs(carpark_vertical.lot_center.poses.size()-1)).orientation.w)<0.05
           &&fabs(msg->pose.pose.orientation.z- carpark_vertical.step1_left.poses.at(abs(carpark_vertical.lot_center.poses.size()-1)).orientation.z)<0.05)
     {

        goal2.pose=carpark_vertical.lot_reference.poses.at(abs(carpark_vertical.lot_center.poses.size()-1));
         ros::Duration(5.0).sleep();
        //usleep(5000000);
        pose_pub.publish(goal2);

   }


       if(indexV_lot!=carpark_vertical.lot_center.poses.size()-1)
       {

             if(fabs(msg->pose.pose.position.x-carpark_vertical.step1_right.poses.at(indexV_lot+1).position.x) <0.1
              &&fabs(msg->pose.pose.position.y- carpark_vertical.step1_right.poses.at(indexV_lot+1).position.y)<0.1
              &&fabs(msg->pose.pose.orientation.w- carpark_vertical.step1_right.poses.at(indexV_lot+1).orientation.w)<0.05
              &&fabs(msg->pose.pose.orientation.z- carpark_vertical.step1_right.poses.at(indexV_lot+1).orientation.z)<0.05)
       {

         std::cout<<"step1_right.pose is "<<carpark_vertical.step1_right.poses.at(indexV_lot+1)<<std::endl;

		
		  goal2.pose= carpark_vertical.lot_reference.poses.at(indexV_lot + 1);
           ros::Duration(5.0).sleep();
         // usleep(5000000);
          pose_pub.publish(goal2);
      }

             if(fabs(msg->pose.pose.position.x-carpark_vertical.step1_right.poses.at(0).position.x) <0.1
              &&fabs(msg->pose.pose.position.y- carpark_vertical.step1_right.poses.at(0).position.y)<0.1
              &&fabs(msg->pose.pose.orientation.w- carpark_vertical.step1_right.poses.at(0).orientation.w)<0.05
              &&fabs(msg->pose.pose.orientation.z- carpark_vertical.step1_right.poses.at(0).orientation.z)<0.05)
        {

           goal2.pose=carpark_vertical.lot_reference.poses.at(0);
            ros::Duration(5.0).sleep();
           //usleep(5000000);
           pose_pub.publish(goal2);

       }
       }
       }



   if(select_lot==1)
   {

 //std::cout<<"step1_left.pose is "<<carpark_parallel.step1_left.poses.at(abs(indexP_lot-1))<<std::endl;

          if(fabs(msg->pose.pose.position.x-carpark_parallel.step1_left.poses.at(abs(indexP_lot-1)).position.x) <0.1
           &&fabs(msg->pose.pose.position.y- carpark_parallel.step1_left.poses.at(abs(indexP_lot-1)).position.y)<0.1
           &&fabs(msg->pose.pose.orientation.w- carpark_parallel.step1_left.poses.at(abs(indexP_lot-1)).orientation.w)<0.05
           &&fabs(msg->pose.pose.orientation.z- carpark_parallel.step1_left.poses.at(abs(indexP_lot-1)).orientation.z)<0.05)
      {

       goal2.pose=carpark_parallel.lot_reference.poses.at(abs(indexP_lot-1));
       usleep(5000000);
       pose_pub.publish(goal2);
     }


         if(fabs(msg->pose.pose.position.x-carpark_parallel.step1_left.poses.at(abs(carpark_parallel.lot_center.poses.size()-1)).position.x) <0.1
          &&fabs(msg->pose.pose.position.y- carpark_parallel.step1_left.poses.at(abs(carpark_parallel.lot_center.poses.size()-1)).position.y)<0.1
          &&fabs(msg->pose.pose.orientation.w- carpark_parallel.step1_left.poses.at(abs(carpark_parallel.lot_center.poses.size()-1)).orientation.w)<0.05
          &&fabs(msg->pose.pose.orientation.z- carpark_parallel.step1_left.poses.at(abs(carpark_parallel.lot_center.poses.size()-1)).orientation.z)<0.05)
        {


       goal2.pose=carpark_parallel.lot_reference.poses.at(abs(carpark_parallel.lot_center.poses.size()-1));
       usleep(5000000);
       pose_pub.publish(goal2);

   }






   if(indexP_lot!=carpark_parallel.lot_center.poses.size()-1)
   {

         if(fabs(msg->pose.pose.position.x-carpark_parallel.step1_right.poses.at(indexP_lot+1).position.x) <0.1
          &&fabs(msg->pose.pose.position.y- carpark_parallel.step1_right.poses.at(indexP_lot+1).position.y)<0.1
          &&fabs(msg->pose.pose.orientation.w- carpark_parallel.step1_right.poses.at(indexP_lot+1).orientation.w)<0.05
          &&fabs(msg->pose.pose.orientation.z- carpark_parallel.step1_right.poses.at(indexP_lot+1).orientation.z)<0.05)
   {

     std::cout<<"step1_right.pose is "<<carpark_parallel.step1_right.poses.at(indexP_lot+1)<<std::endl;

      goal2.pose=carpark_parallel.lot_reference_2.poses.at(indexP_lot+1);
      usleep(5000000);
      pose_pub.publish(goal2);
  }

         if(fabs(msg->pose.pose.position.x-carpark_parallel.step1_right.poses.at(0).position.x) <0.1
          &&fabs(msg->pose.pose.position.y- carpark_parallel.step1_right.poses.at(0).position.y)<0.1
          &&fabs(msg->pose.pose.orientation.w- carpark_parallel.step1_right.poses.at(0).orientation.w)<0.05
          &&fabs(msg->pose.pose.orientation.z- carpark_parallel.step1_right.poses.at(0).orientation.z)<0.05)
    {

       goal2.pose=carpark_parallel.lot_reference_2.poses.at(0);
       usleep(5000000);
       pose_pub.publish(goal2);

   }
   }

   }

   }


   void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {

       ROS_INFO_ONCE("Got first costmap callback. This message will be printed once");

  }


   int main( int argc, char** argv )
  {

    ros::init(argc, argv, "carpark_AP_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh ("~");


    ros::Subscriber sub = private_nh.subscribe("clicked_point", 10, clickedptCallback);
//ros::Publisher tree_pub = private_nh.advertise<visualization_msgs::Marker>("tree_decoration", 10);
    ros::Publisher carpark_vertical_pt_pub = private_nh.advertise<visualization_msgs::Marker>("carpark_vertical_pt", 10);
    ros::Publisher carpark_vertical_li_pub = private_nh.advertise<visualization_msgs::Marker>("carpark_vertical_li", 10);
    ros::Publisher carpark_parallel_pt_pub =private_nh.advertise<visualization_msgs::Marker>("carpark_parallel_pt", 10);
    ros::Publisher carpark_parallel_li_pub =private_nh.advertise<visualization_msgs::Marker>("carpark_parallel_li", 10);
    ros::Publisher carpark_degree_pt_pub =private_nh.advertise<visualization_msgs::Marker>("carpark_degree_pt", 10);
    ros::Publisher carpark_degree_li_pub =private_nh.advertise<visualization_msgs::Marker>("carpark_degree_li", 10);
    pose_pub= private_nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);


    //plot car poses
     half_circle_left_pub =private_nh.advertise<nav_msgs::Path>("car_poses_left", 100);
     half_circle_pub =private_nh.advertise<nav_msgs::Path>("car_poses_right", 100);


       half_circle_poses_pub =private_nh.advertise<geometry_msgs::PoseArray>("half_circles_right", 100);
     half_circle_poses_2_pub =private_nh.advertise<geometry_msgs::PoseArray>("half_circles_left", 100);






     goalReach_sub=private_nh.subscribe("odom",10,step2Callback);

     car_polygon_pub=private_nh.advertise<geometry_msgs::PolygonStamped>("car_polygon",100);
     car_polygon_left_pub=private_nh.advertise<geometry_msgs::PolygonStamped>("car_polygon_left",100);
     car_polygon_vector_pub=private_nh.advertise<geometry_msgs::PolygonStamped>("car_polygon_vector",100);



     costmap_sub = private_nh.subscribe("/move_base/local_costmap/costmap", 10, costmapCallback);



     //private_nh.param("occupied_min_value", occupied_min_value_,100);




   private_nh.param("radius",radius_min,2.60);
   private_nh.param("initial_angle",angle_i,0.0);
   private_nh.param("angle_max",angle_max,PI/2);
   private_nh.param("angle_d",angle_d,0.1);
   private_nh.param("distance_between_two_centers",distance,0.5);

   private_nh.param("radius_2",radius_min_2,2.60);
   private_nh.param("initial_angle_2",angle_i_2,0.0);
   private_nh.param("angle_max_2",angle_max_2,PI/2);
   private_nh.param("angle_d_2",angle_d_2,0.1);
   private_nh.param("distance_between_two_centers_2",distance_2,0.5);

   private_nh.param("Cicle_numbers",circle_num,20);
   private_nh.param("Steering_angle",steer_angle,0.0);





    //Vertical parking lot

    private_nh.param("vp_number", carpark_vertical.number, 5);
    private_nh.param("vp_width",  carpark_vertical.width,  2.1);
    private_nh.param("vp_depth",  carpark_vertical.depth,  3.26);
    private_nh.param("vp_line_width", carpark_vertical.line_width, 0.1);
    private_nh.param("vp_origin_x", carpark_vertical.origin.position.x, 6.0);
    private_nh.param("vp_origin_y", carpark_vertical.origin.position.y, 6.0);
    private_nh.param("vp_yaw", carpark_vertical.yaw, PI);
    private_nh.param("offset_x", carpark_vertical.offset_x, 1.05);
    private_nh.param("offset_y", carpark_vertical.offset_y, 2.343);    //  vp_.depth+car_length)/2-0.49 and 0.49 is the distance between the car rear and tail 
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(carpark_vertical.yaw),carpark_vertical.origin.orientation);
    carpark_vertical.point_array.type = visualization_msgs::Marker::POINTS;
    carpark_vertical.point_array.scale.x = 0.2;
    carpark_vertical.point_array.scale.y = 0.2;
    carpark_vertical.point_array.color.r = 1.0;
    carpark_vertical.point_array.color.g = 1.0;
    carpark_vertical.point_array.color.b = 1.0;
    carpark_vertical.point_array.color.a = 1.0;
    carpark_vertical.line_array.type = visualization_msgs::Marker::LINE_LIST;
    carpark_vertical.line_array.scale.x = 0.1;
    carpark_vertical.line_array.scale.y = 0.0;
    carpark_vertical.line_array.color.r = 0.1;
    carpark_vertical.line_array.color.g = 0.1;
    carpark_vertical.line_array.color.b = 1.0;
    carpark_vertical.line_array.color.a = 1.0;

    // compute vertical carpark lot points and carpark lot center
     carpark_vertical.point_array.header.frame_id
    =carpark_vertical.line_array.header.frame_id
    =carpark_vertical.lot_reference.header.frame_id
    =carpark_vertical.step1_left.header.frame_id
    =carpark_vertical.step1_right.header.frame_id="/map";
    Vparking_lot_area=carpark_vertical.width*carpark_vertical.depth;


    for(int i=0;i<carpark_vertical.number*1+1;i++){

        geometry_msgs::Point pt, ptt;
         geometry_msgs::Pose lot;

        pt.x = carpark_vertical.origin.position.x + i*carpark_vertical.width;
        pt.y = carpark_vertical.origin.position.y;
        ptt.x = pt.x * cos(carpark_vertical.yaw) - pt.y * sin(carpark_vertical.yaw);
        ptt.y = pt.x * sin(carpark_vertical.yaw) + pt.y * cos(carpark_vertical.yaw);
        carpark_vertical.point_array.points.push_back(ptt);

        pt.x = carpark_vertical.origin.position.x + i*carpark_vertical.width;
        pt.y = carpark_vertical.origin.position.y + carpark_vertical.depth;
        ptt.x = pt.x * cos(carpark_vertical.yaw) - pt.y * sin(carpark_vertical.yaw);
        ptt.y = pt.x * sin(carpark_vertical.yaw) + pt.y * cos(carpark_vertical.yaw);
        carpark_vertical.point_array.points.push_back(ptt);

     if(i<carpark_vertical.number){

        pt.x = carpark_vertical.origin.position.x + carpark_vertical.offset_x + i*carpark_vertical.width;
        pt.y = carpark_vertical.origin.position.y + carpark_vertical.offset_y;
        lot.position.x = pt.x * cos(carpark_vertical.yaw) - pt.y * sin(carpark_vertical.yaw);
        lot.position.y = pt.x * sin(carpark_vertical.yaw) + pt.y * cos(carpark_vertical.yaw);
        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(carpark_vertical.yaw-PI/2),lot.orientation);
        carpark_vertical.lot_reference.poses.push_back(lot);




        pt.x = carpark_vertical.origin.position.x + carpark_vertical.width/2 + i*carpark_vertical.width;
        pt.y = carpark_vertical.origin.position.y + carpark_vertical.depth/2;
        lot.position.x = pt.x * cos(carpark_vertical.yaw) - pt.y * sin(carpark_vertical.yaw);
        lot.position.y = pt.x * sin(carpark_vertical.yaw) + pt.y * cos(carpark_vertical.yaw);
        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(carpark_vertical.yaw-PI/2),lot.orientation);
        carpark_vertical.lot_center.poses.push_back(lot);




        pt.x = carpark_vertical.origin.position.x + 3*carpark_vertical.width/2 + i*carpark_vertical.width;
        pt.y = carpark_vertical.origin.position.y - carpark_vertical.depth/2;
        lot.position.x = pt.x * cos(carpark_vertical.yaw) - pt.y * sin(carpark_vertical.yaw);
        lot.position.y = pt.x * sin(carpark_vertical.yaw) + pt.y * cos(carpark_vertical.yaw);
        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(carpark_vertical.yaw-1*PI/8),lot.orientation);
        carpark_vertical.step1_left.poses.push_back(lot);




        pt.x = carpark_vertical.origin.position.x - carpark_vertical.width/2 + i*carpark_vertical.width;
        pt.y = carpark_vertical.origin.position.y - carpark_vertical.depth/2;
        lot.position.x = pt.x * cos(carpark_vertical.yaw) - pt.y * sin(carpark_vertical.yaw);
        lot.position.y = pt.x * sin(carpark_vertical.yaw) + pt.y * cos(carpark_vertical.yaw);
        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(carpark_vertical.yaw-7*PI/8),lot.orientation);
        carpark_vertical.step1_right.poses.push_back(lot);

		// 90


    }
    }


    // compute vertical carpark lot lines
    for (int i=0; i<carpark_vertical.point_array.points.size();i=i+2){
        carpark_vertical.line_array.points.push_back(carpark_vertical.point_array.points.at(i));
        carpark_vertical.line_array.points.push_back(carpark_vertical.point_array.points.at(i+1));
    if(i<carpark_vertical.point_array.points.size()-2){
        carpark_vertical.line_array.points.push_back(carpark_vertical.point_array.points.at(i+1));
        carpark_vertical.line_array.points.push_back(carpark_vertical.point_array.points.at(i+3));
    }
    }


    // Parallel carpark lot


    private_nh.param("pp_number", carpark_parallel.number, 3);
    private_nh.param("pp_width",  carpark_parallel.width,  3.51);
    private_nh.param("pp_depth",  carpark_parallel.depth,  2.15);
    private_nh.param("pp_line_width", carpark_parallel.line_width, 0.1);
    private_nh.param("pp_origin_x", carpark_parallel.origin.position.x, 10.5);
    private_nh.param("pp_origin_y", carpark_parallel.origin.position.y, 7.0);
    private_nh.param("pp_yaw", carpark_parallel.yaw, 0.0);
    private_nh.param("pffset_x", carpark_parallel.offset_x, 0.80);       // pp_.width-car_length)/2+0.49-0.20 and 0.49 is the distance between the car rear and tail
    private_nh.param("pffset_y", carpark_parallel.offset_y, 1.075);      //  carpark_vertical.depth/2
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(carpark_parallel.yaw),carpark_parallel.origin.orientation);
    carpark_parallel.point_array.type = visualization_msgs::Marker::POINTS;
    carpark_parallel.point_array.scale.x = 0.2;
    carpark_parallel.point_array.scale.y = 0.2;
    carpark_parallel.point_array.color.r = 1.0;
    carpark_parallel.point_array.color.g = 1.0;
    carpark_parallel.point_array.color.b = 1.0;
    carpark_parallel.point_array.color.a = 1.0;
    carpark_parallel.line_array.type = visualization_msgs::Marker::LINE_LIST;
    carpark_parallel.line_array.scale.x = 0.1;
    carpark_parallel.line_array.scale.y = 0.0;
    carpark_parallel.line_array.color.r = 0.086;
    carpark_parallel.line_array.color.g = 0.059;
    carpark_parallel.line_array.color.b = 0.961;
    carpark_parallel.line_array.color.a = 1.0;

    // compute vertical carpark lot points and carpark lot center
     carpark_parallel.point_array.header.frame_id
    =carpark_parallel.line_array.header.frame_id
    =carpark_parallel.lot_reference.header.frame_id
    =carpark_parallel.step1_left.header.frame_id
    =carpark_parallel.step1_right.header.frame_id= "/map";


    Pparking_lot_area=carpark_parallel.width*carpark_parallel.depth;

    for(int i=0;i<carpark_parallel.number*1+1;i++){

        geometry_msgs::Point pt, ptt;
        geometry_msgs::Pose lot;
        pt.x = carpark_parallel.origin.position.x + i*carpark_parallel.width;
        pt.y = carpark_parallel.origin.position.y;
        ptt.x = pt.x * cos(carpark_parallel.yaw) - pt.y * sin(carpark_parallel.yaw) ;
        ptt.y = pt.x * sin(carpark_parallel.yaw) + pt.y * cos(carpark_parallel.yaw) ;
        carpark_parallel.point_array.points.push_back(ptt);



        pt.x = carpark_parallel.origin.position.x + i*carpark_parallel.width;
        pt.y = carpark_parallel.origin.position.y + carpark_parallel.depth;
        ptt.x = pt.x * cos(carpark_parallel.yaw) - pt.y * sin(carpark_parallel.yaw);
        ptt.y = pt.x * sin(carpark_parallel.yaw) + pt.y * cos(carpark_parallel.yaw);
        carpark_parallel.point_array.points.push_back(ptt);

     if(i<carpark_parallel.number){

        pt.x = carpark_parallel.origin.position.x + carpark_parallel.offset_x + i*carpark_parallel.width;
        pt.y = carpark_parallel.origin.position.y + carpark_parallel.offset_y;
        lot.position.x = pt.x * cos(carpark_parallel.yaw) - pt.y * sin(carpark_parallel.yaw);
        lot.position.y = pt.x * sin(carpark_parallel.yaw) + pt.y * cos(carpark_parallel.yaw);
        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(carpark_parallel.yaw),lot.orientation);
        carpark_parallel.lot_reference.poses.push_back(lot);


        pt.x = carpark_parallel.origin.position.x+carpark_parallel.width-carpark_parallel.offset_x + i*carpark_parallel.width;
        pt.y = carpark_parallel.origin.position.y + carpark_parallel.offset_y;
        lot.position.x = pt.x * cos(carpark_parallel.yaw) - pt.y * sin(carpark_parallel.yaw);
        lot.position.y = pt.x * sin(carpark_parallel.yaw) + pt.y * cos(carpark_parallel.yaw);
        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(carpark_parallel.yaw+PI),lot.orientation);
        carpark_parallel.lot_reference_2.poses.push_back(lot);



        pt.x = carpark_parallel.origin.position.x + carpark_parallel.width/2 + i*carpark_parallel.width;
        pt.y = carpark_parallel.origin.position.y + carpark_parallel.depth/2;
        lot.position.x = pt.x * cos(carpark_parallel.yaw) - pt.y * sin(carpark_parallel.yaw);
        lot.position.y = pt.x * sin(carpark_parallel.yaw) + pt.y * cos(carpark_parallel.yaw);
        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(carpark_parallel.yaw),lot.orientation);
        carpark_parallel.lot_center.poses.push_back(lot);



        pt.x = carpark_parallel.origin.position.x +1.5+carpark_parallel.width + i*carpark_parallel.width;
        pt.y = carpark_parallel.origin.position.y -1.7;
        lot.position.x = pt.x * cos(carpark_parallel.yaw) - pt.y * sin(carpark_parallel.yaw);
        lot.position.y = pt.x * sin(carpark_parallel.yaw) + pt.y * cos(carpark_parallel.yaw);
        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(carpark_parallel.yaw-0.12),lot.orientation);
        carpark_parallel.step1_left.poses.push_back(lot);


        pt.x = carpark_parallel.origin.position.x -1.5 + i*carpark_parallel.width;
        pt.y = carpark_parallel.origin.position.y -1.7;
        lot.position.x = pt.x * cos(carpark_parallel.yaw) - pt.y * sin(carpark_parallel.yaw);
        lot.position.y = pt.x * sin(carpark_parallel.yaw) + pt.y * cos(carpark_parallel.yaw);
        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(carpark_parallel.yaw+PI+0.12),lot.orientation);
        carpark_parallel.step1_right.poses.push_back(lot);

    }
    }

    // compute parallel carpark lot lines
    for (int i=0; i<carpark_parallel.point_array.points.size();i=i+2){
        carpark_parallel.line_array.points.push_back(carpark_parallel.point_array.points.at(i));
        carpark_parallel.line_array.points.push_back(carpark_parallel.point_array.points.at(i+1));
     if(i<carpark_parallel.point_array.points.size()-2){
        carpark_parallel.line_array.points.push_back(carpark_parallel.point_array.points.at(i+1));
        carpark_parallel.line_array.points.push_back(carpark_parallel.point_array.points.at(i+3));
    }
    }





    //parallel end





    //45degree parking lot
   //declare Gold = color red 0.8 green 0.498039 blue 0.196078
    private_nh.param("p_number", carpark_degree.number, 5);
    private_nh.param("p_width",  carpark_degree.width,  2.1);
    private_nh.param("p_depth",  carpark_degree.depth,  3.26);
    private_nh.param("p_line_width", carpark_degree.line_width, 0.1);
    private_nh.param("p_origin_x", carpark_degree.origin.position.x, 6.0);
    private_nh.param("p_origin_y", carpark_degree.origin.position.y, 6.0);
    private_nh.param("p_yaw", carpark_degree.yaw, PI);
    private_nh.param("ffset_x", carpark_degree.offset_x, 1.05);
    private_nh.param("ffset_y", carpark_degree.offset_y, 0.917);    //  vp_.depth-car_length)/2-0.49 and 0.49 is the distance between the car rear and tail
    private_nh.param("ffset_angle", carpark_degree.offset_angle, PI/2);
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(carpark_degree.yaw),carpark_degree.origin.orientation);
    carpark_degree.point_array.type = visualization_msgs::Marker::POINTS;
    carpark_degree.point_array.scale.x = 0.2;
    carpark_degree.point_array.scale.y = 0.2;
    carpark_degree.point_array.color.r = 1.0;
    carpark_degree.point_array.color.g = 1.0;
    carpark_degree.point_array.color.b = 1.0;
    carpark_degree.point_array.color.a = 1.0;
    carpark_degree.point_array.pose.orientation.w=1.0;
    carpark_degree.point_array.pose.orientation.w=-0.4;
    carpark_degree.line_array.type = visualization_msgs::Marker::LINE_LIST;
    carpark_degree.line_array.scale.x = 0.1;
    carpark_degree.line_array.scale.y = 0.0;
    carpark_degree.line_array.color.r = 0.173;
    carpark_degree.line_array.color.g = 0.64;
    carpark_degree.line_array.color.b = 0.42;
    carpark_degree.line_array.color.a = 1.0;

    // compute degree45 carpark lot points and carpark lot center
     carpark_degree.point_array.header.frame_id
    =carpark_degree.line_array.header.frame_id
    =carpark_degree.lot_reference.header.frame_id
    =carpark_degree.step1_left.header.frame_id
    =carpark_degree.step1_right.header.frame_id="/map";
    parking_lot_area=carpark_degree.width*carpark_degree.depth;



//    //------------------------

//    for(int i=0;i<carpark_vertical.number*1+1;i++){

//        geometry_msgs::Point pt, ptt;
//         geometry_msgs::Pose lot;

//        pt.x = carpark_vertical.origin.position.x + i*carpark_vertical.width;
//        pt.y = carpark_vertical.origin.position.y;
//        ptt.x = pt.x * cos(carpark_vertical.yaw) - pt.y * sin(carpark_vertical.yaw);
//        ptt.y = pt.x * sin(carpark_vertical.yaw) + pt.y * cos(carpark_vertical.yaw);
//        carpark_vertical.point_array.points.push_back(ptt);

//        pt.x = carpark_vertical.origin.position.x + i*carpark_vertical.width;
//        pt.y = carpark_vertical.origin.position.y + carpark_vertical.depth;
//        ptt.x = pt.x * cos(carpark_vertical.yaw) - pt.y * sin(carpark_vertical.yaw);
//        ptt.y = pt.x * sin(carpark_vertical.yaw) + pt.y * cos(carpark_vertical.yaw);
//        carpark_vertical.point_array.points.push_back(ptt);


//    //---------------------------
    for(int i=0;i<carpark_degree.number*1+1;i++){

        geometry_msgs::Point pt, ptt;
         geometry_msgs::Pose lot;

        pt.x = carpark_degree.origin.position.x + i*carpark_degree.width;
        pt.y = carpark_degree.origin.position.y;
        ptt.x = pt.x * cos(carpark_degree.yaw) - pt.y * sin(carpark_degree.yaw);
        ptt.y = pt.x * sin(carpark_degree.yaw) + pt.y * cos(carpark_degree.yaw);
        carpark_degree.point_array.points.push_back(ptt);

        pt.x = carpark_degree.origin.position.x + i*carpark_degree.width;
        pt.y = carpark_degree.origin.position.y + carpark_degree.depth;
        ptt.x = pt.x * cos(carpark_degree.yaw) - pt.y * sin(carpark_degree.yaw);
        ptt.y = pt.x * sin(carpark_degree.yaw) + pt.y * cos(carpark_degree.yaw);
        carpark_degree.point_array.points.push_back(ptt);

     if(i<carpark_degree.number){

        pt.x = carpark_degree.origin.position.x + carpark_degree.offset_x + i*carpark_degree.width;
        pt.y = carpark_degree.origin.position.y + carpark_degree.offset_y;
        lot.position.x = pt.x * cos(carpark_degree.yaw) - pt.y * sin(carpark_degree.yaw);
        lot.position.y = pt.x * sin(carpark_degree.yaw) + pt.y * cos(carpark_degree.yaw);
        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(carpark_degree.yaw+carpark_degree.offset_angle),lot.orientation);
        carpark_degree.lot_reference.poses.push_back(lot);




        pt.x = carpark_degree.origin.position.x + carpark_degree.width/2 + i*carpark_degree.width;
        pt.y = carpark_degree.origin.position.y + carpark_degree.depth/2;
        lot.position.x = pt.x * cos(carpark_degree.yaw) - pt.y * sin(carpark_degree.yaw);
        lot.position.y = pt.x * sin(carpark_degree.yaw) + pt.y * cos(carpark_degree.yaw);
        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(carpark_degree.yaw-PI/2),lot.orientation);
        carpark_degree.lot_center.poses.push_back(lot);
    }

    }


    // compute 45degree carpark lot lines
    for (int i=0; i<carpark_degree.point_array.points.size();i=i+2){
        carpark_degree.line_array.points.push_back(carpark_degree.point_array.points.at(i));
        carpark_degree.line_array.points.push_back(carpark_degree.point_array.points.at(i+1));
    if(i<carpark_degree.point_array.points.size()-2){
        carpark_degree.line_array.points.push_back(carpark_degree.point_array.points.at(i+1));
        carpark_degree.line_array.points.push_back(carpark_degree.point_array.points.at(i+3));
    }
    }






    //45degree end
    ros::Rate r(10);
   while(ros::ok()){
        carpark_vertical_pt_pub.publish(carpark_vertical.point_array);
        carpark_vertical_li_pub.publish(carpark_vertical.line_array);
        carpark_parallel_pt_pub.publish(carpark_parallel.point_array);
        carpark_parallel_li_pub.publish(carpark_parallel.line_array);
        carpark_degree_pt_pub.publish(carpark_degree.point_array);
        carpark_degree_li_pub.publish(carpark_degree.line_array);


        ros::spinOnce();
        r.sleep();
    }
    }

