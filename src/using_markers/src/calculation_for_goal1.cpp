#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
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




    struct car_park{
    int number;                          // carpark lot numbers
    double width;                        // width of carpark lot
    double depth;                        // height of carpark lot
    double line_width;                   // line width
    double yaw;                         // yaw angle of carpark
    double offset_x;                     // x offset lot center
    double offset_y;                     // y offset lot center
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

   int main( int argc, char** argv )
  {

    ros::init(argc, argv, "/calculation_goal1");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh ("");

    ros::Publisher carpark_vertical_pt_pub = private_nh.advertise<visualization_msgs::Marker>("carpark_vertical_pt", 10);
    ros::Publisher carpark_vertical_li_pub = private_nh.advertise<visualization_msgs::Marker>("carpark_vertical_li", 10);
    ros::Publisher carpark_parallel_pt_pub =private_nh.advertise<visualization_msgs::Marker>("carpark_parallel_pt", 10);
    ros::Publisher carpark_parallel_li_pub =private_nh.advertise<visualization_msgs::Marker>("carpark_parallel_li", 10);
    ros::Publisher carpark_degree_pt_pub =private_nh.advertise<visualization_msgs::Marker>("carpark_degree_pt", 10);
    ros::Publisher carpark_degree_li_pub =private_nh.advertise<visualization_msgs::Marker>("carpark_degree_li", 10);





    //ros::Publisher half_circle_2_pub =private_nh.advertise<visualization_msgs::MarkerArray>("half_circles_2", 10);
    //ros::Publisher half_circle_2_lines_pub =private_nh.advertise<visualization_msgs::MarkerArray>("half_circles_lines_2", 10);















    //Vertical parking lot

    private_nh.param("vp_number", carpark_vertical.number, 3);
    private_nh.param("vp_width",  carpark_vertical.width,  2.1);
    private_nh.param("vp_depth",  carpark_vertical.depth,  3.26);
    private_nh.param("vp_line_width", carpark_vertical.line_width, 0.1);
    private_nh.param("vp_origin_x", carpark_vertical.origin.position.x, 0.0);
    private_nh.param("vp_origin_y", carpark_vertical.origin.position.y, 0.0);
    private_nh.param("vp_yaw", carpark_vertical.yaw, PI/4);
    private_nh.param("offset_x", carpark_vertical.offset_x, 1.05);
    private_nh.param("offset_y", carpark_vertical.offset_y, 2.343);    //  vp_.depth+car_length)/2-0.49 and 0.49 is the distance between the car rear and tail
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(carpark_vertical.yaw),carpark_vertical.origin.orientation);
    carpark_vertical.point_array.type = visualization_msgs::Marker::POINTS;
    carpark_vertical.point_array.scale.x = 0.3;
    carpark_vertical.point_array.scale.y = 0.3;
    carpark_vertical.point_array.color.g = 1.0f;
    carpark_vertical.point_array.color.a = 0.7;
    carpark_vertical.line_array.type = visualization_msgs::Marker::LINE_LIST;
    carpark_vertical.line_array.scale.x = 0.1;
    carpark_vertical.line_array.scale.y = 0.0;
    carpark_vertical.line_array.color.r = 1.0;
    carpark_vertical.line_array.color.g = 0.3;
    carpark_vertical.line_array.color.b = 0.0;
    carpark_vertical.line_array.color.a = 1.0;

    // compute vertical carpark lot points and carpark lot center
     carpark_vertical.point_array.header.frame_id
    =carpark_vertical.line_array.header.frame_id
    =carpark_vertical.lot_reference.header.frame_id
    =carpark_vertical.step1_left.header.frame_id
    =carpark_vertical.step1_right.header.frame_id="/map";



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





    ros::Publisher lot_reference_pub =private_nh.advertise<geometry_msgs::PoseArray>("lot_reference", 100);

    //Half circles


    ros::Publisher half_circle_pub =private_nh.advertise<geometry_msgs::PoseArray>("half_circles", 100);



    ros::Publisher half_circle_2_pub =private_nh.advertise<geometry_msgs::PoseArray>("half_circles_2", 100);


    double radius_min,angle_max ,angle_d,distance;
    double radius;
    int lot_num , circle_num;
   private_nh.param("radius",radius_min,2.60);
   private_nh.param("angle_max",angle_max,PI/2);
   private_nh.param("angle_d",angle_d,0.1);
   private_nh.param("distance_between_two_centers",distance,0.5);
   private_nh.param("Cicle_numbers",circle_num,20);
   private_nh.param("lot_number",lot_num,0);

   geometry_msgs::PoseArray poses_car;
   poses_car.header.frame_id="/map";



   for(int i=0;i<=circle_num;i++)
   {       //One side
       geometry_msgs::Point points;
       points.x=carpark_vertical.lot_reference.poses.at(lot_num).position.x-(radius_min+i*distance)*cos(carpark_vertical.yaw);
       points.y=carpark_vertical.lot_reference.poses.at(lot_num).position.y-(radius_min+i*distance)*sin(carpark_vertical.yaw);
       radius=radius_min+i*distance;
        int t=0;
     for(double angle=0.0; angle<angle_max;angle=angle+angle_d)
     {
       geometry_msgs::Pose points_on_circles;
       geometry_msgs::Point points_on_circles_p;
       points_on_circles.position.x=points_on_circles_p.x=points.x+radius*cos(angle-carpark_vertical.yaw);
       points_on_circles.position.y=points_on_circles_p.y=points.y-radius*sin(angle-carpark_vertical.yaw);
       tf::quaternionTFToMsg(tf::createQuaternionFromYaw(PI/2-(angle-carpark_vertical.yaw)+PI),points_on_circles.orientation);
     // std::cout<<"pose is  "<< points_on_circles  <<std::endl;
     // std::cout<<"i is  "<< i  <<std::endl;
     // std::cout<<"t is  "<< t  <<std::endl;
      poses_car.header.frame_id="/map";
      poses_car.poses.push_back(points_on_circles);

     // std::cout<<"car_pose is  "<< poses_car <<std::endl;
      t=t+1;

      }
      }


   geometry_msgs::PoseArray poses_car_2;
   poses_car_2.header.frame_id="/map";


   for(int i=0;i<=circle_num;i++)
   {       //The other side
       geometry_msgs::Point points;
       points.x=carpark_vertical.lot_reference.poses.at(lot_num).position.x+(radius_min+i*distance)*cos(carpark_vertical.yaw);
       points.y=carpark_vertical.lot_reference.poses.at(lot_num).position.y+(radius_min+i*distance)*sin(carpark_vertical.yaw);
       radius=radius_min+i*distance;
        int t=0;
     for(double angle=0.0; angle<angle_max;angle=angle+angle_d)
     {
       geometry_msgs::Pose points_on_circles;
       geometry_msgs::Point points_on_circles_p;
       points_on_circles.position.x=points_on_circles_p.x=points.x-radius*cos(angle+carpark_vertical.yaw);
       points_on_circles.position.y=points_on_circles_p.y=points.y-radius*sin(angle+carpark_vertical.yaw);
       tf::quaternionTFToMsg(tf::createQuaternionFromYaw((angle+carpark_vertical.yaw)-PI/2),points_on_circles.orientation);
     // std::cout<<"pose is  "<< points_on_circles  <<std::endl;
     // std::cout<<"i is  "<< i  <<std::endl;
      //std::cout<<"t is  "<< t  <<std::endl;
      poses_car_2.poses.push_back(points_on_circles);

     // std::cout<<"car_pose_2 is  "<< poses_car <<std::endl;
      t=t+1;

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
    carpark_parallel.point_array.scale.x = 0.3;
    carpark_parallel.point_array.scale.y = 0.3;
    carpark_parallel.point_array.color.g = 1.0f;
    carpark_parallel.point_array.color.a = 0.7;
    carpark_parallel.line_array.type = visualization_msgs::Marker::LINE_LIST;
    carpark_parallel.line_array.scale.x = 0.1;
    carpark_parallel.line_array.scale.y = 0.0;
    carpark_parallel.line_array.color.r = 1.0;
    carpark_parallel.line_array.color.b = 1.0;
    carpark_parallel.line_array.color.a = 1.0;

    // compute vertical carpark lot points and carpark lot center
     carpark_parallel.point_array.header.frame_id
    =carpark_parallel.line_array.header.frame_id
    =carpark_parallel.lot_reference.header.frame_id
    =carpark_parallel.step1_left.header.frame_id
    =carpark_parallel.step1_right.header.frame_id= "/map";



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



        pt.x = carpark_parallel.origin.position.x +1.0+carpark_parallel.width + i*carpark_parallel.width;
        pt.y = carpark_parallel.origin.position.y -1.7;
        lot.position.x = pt.x * cos(carpark_parallel.yaw) - pt.y * sin(carpark_parallel.yaw);
        lot.position.y = pt.x * sin(carpark_parallel.yaw) + pt.y * cos(carpark_parallel.yaw);
        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(carpark_parallel.yaw),lot.orientation);
        carpark_parallel.step1_left.poses.push_back(lot);


        pt.x = carpark_parallel.origin.position.x -1.0 + i*carpark_parallel.width;
        pt.y = carpark_parallel.origin.position.y -1.7;
        lot.position.x = pt.x * cos(carpark_parallel.yaw) - pt.y * sin(carpark_parallel.yaw);
        lot.position.y = pt.x * sin(carpark_parallel.yaw) + pt.y * cos(carpark_parallel.yaw);
        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(carpark_parallel.yaw+PI),lot.orientation);
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

    private_nh.param("p_number", carpark_degree.number, 5);
    private_nh.param("p_width",  carpark_degree.width,  2.1);
    private_nh.param("p_depth",  carpark_degree.depth,  3.26);
    private_nh.param("p_line_width", carpark_degree.line_width, 0.1);
    private_nh.param("p_origin_x", carpark_degree.origin.position.x, 6.0);
    private_nh.param("p_origin_y", carpark_degree.origin.position.y, 6.0);
    private_nh.param("p_yaw", carpark_degree.yaw, PI);
    private_nh.param("ffset_x", carpark_degree.offset_x, 1.05);
    private_nh.param("ffset_y", carpark_degree.offset_y, 0.917);    //  vp_.depth-car_length)/2-0.49 and 0.49 is the distance between the car rear and tail
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(carpark_degree.yaw),carpark_degree.origin.orientation);
    carpark_degree.point_array.type = visualization_msgs::Marker::POINTS;
    carpark_degree.point_array.scale.x = 0.3;
    carpark_degree.point_array.scale.y = 0.3;
    carpark_degree.point_array.color.g = 1.0f;
    carpark_degree.point_array.color.a = 0.7;
    carpark_degree.line_array.type = visualization_msgs::Marker::LINE_LIST;
    carpark_degree.line_array.scale.x = 0.1;
    carpark_degree.line_array.scale.y = 0.0;
    carpark_degree.line_array.color.r = 0.0;
    carpark_degree.line_array.color.g = 0.0;
    carpark_degree.line_array.color.b = 1.0;
    carpark_degree.line_array.color.a = 1.0;

    // compute degree45 carpark lot points and carpark lot center
     carpark_degree.point_array.header.frame_id
    =carpark_degree.line_array.header.frame_id
    =carpark_degree.lot_reference.header.frame_id
    =carpark_degree.step1_left.header.frame_id
    =carpark_degree.step1_right.header.frame_id="/map";



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
        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(carpark_degree.yaw+PI/2),lot.orientation);
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
    ros::Rate r(1000);
   while(ros::ok()){
        carpark_vertical_pt_pub.publish(carpark_vertical.point_array);
        carpark_vertical_li_pub.publish(carpark_vertical.line_array);
        carpark_parallel_pt_pub.publish(carpark_parallel.point_array);
        carpark_parallel_li_pub.publish(carpark_parallel.line_array);
        carpark_degree_pt_pub.publish(carpark_degree.point_array);
        carpark_degree_li_pub.publish(carpark_degree.line_array);
        half_circle_pub.publish(poses_car);
        half_circle_2_pub.publish(poses_car_2);
         lot_reference_pub.publish(carpark_vertical.lot_reference);
        r.sleep();
    }
    }

