/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Mike Phillips
*********************************************************************/

#include <sbpl_lattice_planner/sbpl_lattice_planner.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Path.h>
#include <sbpl_lattice_planner/SBPLLatticePlannerStats.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <costmap_2d/inflation_layer.h>

using namespace std;
using namespace ros;


PLUGINLIB_DECLARE_CLASS(sbpl_latice_planner, SBPLLatticePlanner, sbpl_lattice_planner::SBPLLatticePlanner, nav_core::BaseGlobalPlanner);

namespace sbpl_lattice_planner{

class LatticeSCQ : public StateChangeQuery{
  public:
    LatticeSCQ(EnvironmentNAVXYTHETALAT* env, std::vector<nav2dcell_t> const & changedcellsV)
      : env_(env), changedcellsV_(changedcellsV) {
    }

    // lazy init, because we do not always end up calling this method
    virtual std::vector<int> const * getPredecessors() const{
      if(predsOfChangedCells_.empty() && !changedcellsV_.empty())
        env_->GetPredsofChangedEdges(&changedcellsV_, &predsOfChangedCells_);
      return &predsOfChangedCells_;
    }

    // lazy init, because we do not always end up calling this method
    virtual std::vector<int> const * getSuccessors() const{
      if(succsOfChangedCells_.empty() && !changedcellsV_.empty())
        env_->GetSuccsofChangedEdges(&changedcellsV_, &succsOfChangedCells_);
      return &succsOfChangedCells_;
    }

    EnvironmentNAVXYTHETALAT * env_;
    std::vector<nav2dcell_t> const & changedcellsV_;
    mutable std::vector<int> predsOfChangedCells_;
    mutable std::vector<int> succsOfChangedCells_;
};

SBPLLatticePlanner::SBPLLatticePlanner()
  : initialized_(false), costmap_ros_(NULL){
}

SBPLLatticePlanner::SBPLLatticePlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) 
  : initialized_(false), costmap_ros_(NULL){
  initialize(name, costmap_ros);
}


// added by Ben to cater for the fixed path scenario and multiple bus stop scenario
void SBPLLatticePlanner::stopCallback(const std_msgs::Char &msg){
    stop_ = msg;
    std::cout<<stop_.data<<std::endl;
}

void SBPLLatticePlanner::odomCallback(const nav_msgs::Odometry &msg){
    odom_ = msg;
}

double SBPLLatticePlanner::dist_points(double x1, double y1, double x2, double y2){
      double result;
      result = sqrt(pow(x1-x2,2) + pow(y1-y2,2));
      return result;
 }

// end*****************************************************************************
    
void SBPLLatticePlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
  if(!initialized_){
    ros::NodeHandle private_nh("~/"+name);
    ros::NodeHandle nh(name);
    
    ROS_INFO("Name is %s", name.c_str());
 
    private_nh.param("planner_type", planner_type_, string("ARAPlanner"));
    private_nh.param("allocated_time", allocated_time_, 10.0);
    private_nh.param("initial_epsilon",initial_epsilon_,3.0);
    private_nh.param("environment_type", environment_type_, string("XYThetaLattice"));
    private_nh.param("forward_search", forward_search_, bool(false));
    private_nh.param("primitive_filename",primitive_filename_,string(""));
    private_nh.param("force_scratch_limit",force_scratch_limit_,500);

    // added by Ben to cater for the fixed path scenario and multiple bus stop scenario
    private_nh.param("fix_path_flag", fix_path_flag_, false);
    private_nh.param("multi_stop_flag", multi_stop_flag_, false);
    private_nh.param("fix_path_topic", fix_path_topic_, std::string("fix_path"));
    private_nh.param("vir_rd_topic", vir_rd_topic_, std::string("virtual_road"));
    private_nh.param("odom_file", odom_file_, std::string("/home/gb/odom_loop.txt"));
    if(fix_path_flag_){
        fix_path.header.frame_id = "/map";
        fix_path_pub_ = private_nh.advertise<nav_msgs::Path>(fix_path_topic_, 100);
        vir_rd.header.frame_id = "/map";
        vir_rd_pub_ = private_nh.advertise<nav_msgs::Path>(vir_rd_topic_, 100);
        stopSub_ = private_nh.subscribe("/stop", 1, &SBPLLatticePlanner::stopCallback, this);
        odomSub_ = private_nh.subscribe("/toyota_odometry", 1, &SBPLLatticePlanner::odomCallback, this);
        stopMarker_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>( "stop_marker", 10);      // for debug visualization only
        stopmarkertext_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>( "stoptext_marker", 10);
    }
    // end****************************************************************************
  
    double nominalvel_mpersecs, timetoturn45degsinplace_secs;
    private_nh.param("nominalvel_mpersecs", nominalvel_mpersecs, 0.4);
    private_nh.param("timetoturn45degsinplace_secs", timetoturn45degsinplace_secs, 0.6);

    int lethal_obstacle;
    private_nh.param("lethal_obstacle",lethal_obstacle,20);
    lethal_obstacle_ = (unsigned char) lethal_obstacle;
    inscribed_inflated_obstacle_ = lethal_obstacle_-1;
    sbpl_cost_multiplier_ = (unsigned char) (costmap_2d::INSCRIBED_INFLATED_OBSTACLE/inscribed_inflated_obstacle_ + 1);
    ROS_DEBUG("SBPL: lethal: %uz, inscribed inflated: %uz, multiplier: %uz",lethal_obstacle,inscribed_inflated_obstacle_,sbpl_cost_multiplier_);
    
    costmap_ros_ = costmap_ros;

    std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();

    if ("XYThetaLattice" == environment_type_){
      ROS_DEBUG("Using a 3D costmap for theta lattice\n");
      env_ = new EnvironmentNAVXYTHETALAT();
    }
    else{
      ROS_ERROR("XYThetaLattice is currently the only supported environment!\n");
      exit(1);
    }

    // check if the costmap has an inflation layer
    // Warning: footprint updates after initialization are not supported here
    unsigned char cost_possibly_circumscribed_tresh = 0;
    for(std::vector<boost::shared_ptr<costmap_2d::Layer> >::const_iterator layer = costmap_ros_->getLayeredCostmap()->getPlugins()->begin();
        layer != costmap_ros_->getLayeredCostmap()->getPlugins()->end();
        ++layer) {
      boost::shared_ptr<costmap_2d::InflationLayer> inflation_layer = boost::dynamic_pointer_cast<costmap_2d::InflationLayer>(*layer);
      if (!inflation_layer) continue;

      cost_possibly_circumscribed_tresh = inflation_layer->computeCost(costmap_ros_->getLayeredCostmap()->getCircumscribedRadius());
    }

    if(!env_->SetEnvParameter("cost_inscribed_thresh",costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE))){
      ROS_ERROR("Failed to set cost_inscribed_thresh parameter");
      exit(1);
    }
    if(!env_->SetEnvParameter("cost_possibly_circumscribed_thresh", costMapCostToSBPLCost(cost_possibly_circumscribed_tresh))){
      ROS_ERROR("Failed to set cost_possibly_circumscribed_thresh parameter");
      exit(1);
    }
    int obst_cost_thresh = costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE);
    vector<sbpl_2Dpt_t> perimeterptsV;
    perimeterptsV.reserve(footprint.size());
    for (size_t ii(0); ii < footprint.size(); ++ii) {
      sbpl_2Dpt_t pt;
      pt.x = footprint[ii].x;
      pt.y = footprint[ii].y;
      perimeterptsV.push_back(pt);
    }

    bool ret;
    try{
      ret = env_->InitializeEnv(costmap_ros_->getCostmap()->getSizeInCellsX(), // width
                                costmap_ros_->getCostmap()->getSizeInCellsY(), // height
                                0, // mapdata
                                0, 0, 0, // start (x, y, theta, t)
                                0, 0, 0, // goal (x, y, theta)
                                0, 0, 0, //goal tolerance
                                perimeterptsV, costmap_ros_->getCostmap()->getResolution(), nominalvel_mpersecs,
                                timetoturn45degsinplace_secs, obst_cost_thresh,
                                primitive_filename_.c_str());
    }
    catch(SBPL_Exception e){
      ROS_ERROR("SBPL encountered a fatal exception!");
      ret = false;
    }
    if(!ret){
      ROS_ERROR("SBPL initialization failed!");
      exit(1);
    }
    for (ssize_t ix(0); ix < costmap_ros_->getCostmap()->getSizeInCellsX(); ++ix)
      for (ssize_t iy(0); iy < costmap_ros_->getCostmap()->getSizeInCellsY(); ++iy)
        env_->UpdateCost(ix, iy, costMapCostToSBPLCost(costmap_ros_->getCostmap()->getCost(ix,iy)));

    if ("ARAPlanner" == planner_type_){
      ROS_INFO("Planning with ARA*");
      planner_ = new ARAPlanner(env_, forward_search_);
    }
    else if ("ADPlanner" == planner_type_){
      ROS_INFO("Planning with AD*");
      planner_ = new ADPlanner(env_, forward_search_);
    }
    else{
      ROS_ERROR("ARAPlanner and ADPlanner are currently the only supported planners!\n");
      exit(1);
    }

    ROS_INFO("[sbpl_lattice_planner] Initialized successfully");
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
    stats_publisher_ = private_nh.advertise<sbpl_lattice_planner::SBPLLatticePlannerStats>("sbpl_lattice_planner_stats", 1);
    
    initialized_ = true;
  }
}
  
//Taken from Sachin's sbpl_cart_planner
//This rescales the costmap according to a rosparam which sets the obstacle cost
unsigned char SBPLLatticePlanner::costMapCostToSBPLCost(unsigned char newcost){
  if(newcost == costmap_2d::LETHAL_OBSTACLE)
    return lethal_obstacle_;
  else if(newcost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    return inscribed_inflated_obstacle_;
  else if(newcost == 0 || newcost == costmap_2d::NO_INFORMATION)
    return 0;
  else
    return (unsigned char) (newcost/sbpl_cost_multiplier_ + 0.5);
}

void SBPLLatticePlanner::publishStats(int solution_cost, int solution_size, 
                                      const geometry_msgs::PoseStamped& start, 
                                      const geometry_msgs::PoseStamped& goal){
  // Fill up statistics and publish
  sbpl_lattice_planner::SBPLLatticePlannerStats stats;
  stats.initial_epsilon = initial_epsilon_;
  stats.plan_to_first_solution = false;
  stats.final_number_of_expands = planner_->get_n_expands();
  stats.allocated_time = allocated_time_;

  stats.time_to_first_solution = planner_->get_initial_eps_planning_time();
  stats.actual_time = planner_->get_final_eps_planning_time();
  stats.number_of_expands_initial_solution = planner_->get_n_expands_init_solution();
  stats.final_epsilon = planner_->get_final_epsilon();

  stats.solution_cost = solution_cost;
  stats.path_size = solution_size;
  stats.start = start;
  stats.goal = goal;
  stats_publisher_.publish(stats);
}

bool SBPLLatticePlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                 const geometry_msgs::PoseStamped& goal,
                                 std::vector<geometry_msgs::PoseStamped>& plan){
  if(!initialized_){
    ROS_ERROR("Global planner is not initialized");
    return false;
  }

  plan.clear();

  // added by Ben**************************************************************************************************************
  if(fix_path_flag_){
      // pub fixed path, skip SBPL planner
      int StopID = 0;
      if(stop_.data!='A' && stop_.data!='B' && stop_.data!='C' && multi_stop_flag_){
          std::cout<<"No Stop ID entered"<<std::endl;
          return false;
      }
      int stopIDarray[3]={150,350,700};//******************************
      char stopChararray[3] = {'A', 'B', 'C'};
      if(stop_.data =='A') StopID = stopIDarray[0];
      else if (stop_.data =='B') StopID = stopIDarray[1];
      else if (stop_.data == 'C') StopID = stopIDarray[2];

      double temp[7]={0,0,0,0,0,0,1}, value;
      int i = 0, id = 0;
      odomFile.open(odom_file_.c_str(),std::ios::app);
      if(odomFile.is_open())
      {
          std::cout<<"odom file opened"<<std::endl;
          while(odomFile >> value){
              temp[i] = value;
              i++;
              if(i == 7){
                 geometry_msgs::PoseStamped pose;
                 pose.pose.position.x = temp[0];
                 pose.pose.position.y = temp[1];
                 pose.pose.position.z = 0.0;   // if height is needed, change back to temp[2]
                 pose.pose.orientation.x = temp[3];
                 pose.pose.orientation.y = temp[4];
                 pose.pose.orientation.z = temp[5];
                 pose.pose.orientation.w = temp[6];
                 pose.header.seq = id;
                 pose.header.frame_id = "/map";
                 pose.header.stamp = ros::Time::now();
                 plan.push_back(pose);
                 //fix_path.poses.push_back(pose);
                 vir_rd.poses.push_back(pose);
                 id++;
                 i = 0;
              }
          }
          odomFile.close();
          std::cout<<"odom file closed"<<std::endl;
      }
      else{
          std::cout<<"unable to open odom file"<<std::endl;
          return false;
      }

      //pub A-B-C path
      double dist = std::numeric_limits<double>::max();
      id = 0;
      for(int i=0; i<vir_rd.poses.size();i++){
          double temp = dist_points(odom_.pose.pose.position.x, odom_.pose.pose.position.y,
                                    vir_rd.poses.at(i).pose.position.x, vir_rd.poses.at(i).pose.position.y);
          if(temp < dist){
              dist = temp;
              id = i;  // change id name
          }
      }

      if(id>StopID && multi_stop_flag_){
          vector<geometry_msgs::PoseStamped>::const_iterator begin = vir_rd.poses.begin() + id;
          vector<geometry_msgs::PoseStamped>::const_iterator last = vir_rd.poses.begin() + StopID;
          vector<geometry_msgs::PoseStamped> temp1(begin, (vector<geometry_msgs::PoseStamped>::const_iterator)vir_rd.poses.end());
          vector<geometry_msgs::PoseStamped> temp2((vector<geometry_msgs::PoseStamped>::const_iterator) vir_rd.poses.begin(), last);
          temp1.insert(temp1.end(),temp2.begin(),temp2.end());
          fix_path.poses = temp1;
      }
      else if(id < StopID && multi_stop_flag_){  //
          vector<geometry_msgs::PoseStamped>::const_iterator begin = vir_rd.poses.begin() + id;
          vector<geometry_msgs::PoseStamped>::const_iterator last = vir_rd.poses.begin() + StopID;
          vector<geometry_msgs::PoseStamped> temp(begin, last);
          fix_path.poses = temp;
      }
      else if(!multi_stop_flag_){
          fix_path = vir_rd;
      }

      if(multi_stop_flag_){
          // pub stop markers
          visualization_msgs::MarkerArray stopmarker_array;
          visualization_msgs::MarkerArray stopmarker_array_text;
          for(int i=0;i<3;i++){
              visualization_msgs::Marker marker;
              visualization_msgs::Marker marker_text;
              marker.header.frame_id = "/map";
              marker.header.stamp = ros::Time::now();
              marker.id = i;
              marker.type = visualization_msgs::Marker::SPHERE;
              marker.action = visualization_msgs::Marker::ADD;
              marker.pose.position.x = vir_rd.poses.at(stopIDarray[i]).pose.position.x; marker.pose.position.y = vir_rd.poses.at(stopIDarray[i]).pose.position.y;
              marker.pose.position.z = vir_rd.poses.at(stopIDarray[i]).pose.position.z;
              marker.pose.orientation.x = 0.0;  marker.pose.orientation.y = 0.0;
              marker.pose.orientation.z = 0.0;  marker.pose.orientation.w = 1.0;
              marker.scale.x = 0.3; marker.scale.y = 0.3; marker.scale.z = 0.1;
              marker.color.a = 1.0;
              marker.color.r = 0.0; marker.color.g = 0.5; marker.color.b = 0.7;

              marker_text.header.frame_id = "/map";
              marker_text.header.stamp = ros::Time::now();
              marker_text.id = i;
              marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
              marker_text.action = visualization_msgs::Marker::ADD;
              marker_text.pose.position = marker.pose.position;
              marker_text.pose.orientation = marker.pose.orientation;
              marker_text.color.a = 1.0; marker_text.color.r = 0.9;
              marker_text.color.g = 0.2; marker_text.color.b = 0.4;
              marker_text.scale.x = 0.7; marker_text.scale.y = 0.7; marker_text.scale.z = 1.0;
              stringstream ss;
              ss << stopChararray[i];
              string index_string = ss.str();
              marker_text.text = "Stop " + index_string;

              stopmarker_array.markers.push_back(marker);
              stopmarker_array_text.markers.push_back(marker_text);
          }
          stopMarker_pub_.publish(stopmarker_array);
          stopmarkertext_pub_.publish(stopmarker_array_text);
          stopmarker_array.markers.clear();
          stopmarker_array_text.markers.clear();
      }

      vir_rd_pub_.publish(vir_rd);
      fix_path_pub_.publish(fix_path);

      vir_rd.poses.clear();
      fix_path.poses.clear();

      return true;
  }
  // end**********************************************************************************************************************

  ROS_INFO("[sbpl_lattice_planner] getting start point (%g,%g) goal point (%g,%g)",
           start.pose.position.x, start.pose.position.y,goal.pose.position.x, goal.pose.position.y);
  double theta_start = 2 * atan2(start.pose.orientation.z, start.pose.orientation.w);
  double theta_goal = 2 * atan2(goal.pose.orientation.z, goal.pose.orientation.w);

  try{
    int ret = env_->SetStart(start.pose.position.x - costmap_ros_->getCostmap()->getOriginX(), start.pose.position.y - costmap_ros_->getCostmap()->getOriginY(), theta_start);
    if(ret < 0 || planner_->set_start(ret) == 0){
      ROS_ERROR("ERROR: failed to set start state\n");
      return false;
    }
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL encountered a fatal exception while setting the start state");
    return false;
  }

  try{
    int ret = env_->SetGoal(goal.pose.position.x - costmap_ros_->getCostmap()->getOriginX(), goal.pose.position.y - costmap_ros_->getCostmap()->getOriginY(), theta_goal);
    if(ret < 0 || planner_->set_goal(ret) == 0){
      ROS_ERROR("ERROR: failed to set goal state\n");
      return false;
    }
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL encountered a fatal exception while setting the goal state");
    return false;
  }
  
  int offOnCount = 0;
  int onOffCount = 0;
  int allCount = 0;
  vector<nav2dcell_t> changedcellsV;

  for(unsigned int ix = 0; ix < costmap_ros_->getCostmap()->getSizeInCellsX(); ix++) {
    for(unsigned int iy = 0; iy < costmap_ros_->getCostmap()->getSizeInCellsY(); iy++) {

      unsigned char oldCost = env_->GetMapCost(ix,iy);
      unsigned char newCost = costMapCostToSBPLCost(costmap_ros_->getCostmap()->getCost(ix,iy));

      if(oldCost == newCost) continue;

      allCount++;

      //first case - off cell goes on

      if((oldCost != costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) && oldCost != costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE)) &&
          (newCost == costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) || newCost == costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE))) {
        offOnCount++;
      }

      if((oldCost == costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) || oldCost == costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE)) &&
          (newCost != costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) && newCost != costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE))) {
        onOffCount++;
      }
      env_->UpdateCost(ix, iy, costMapCostToSBPLCost(costmap_ros_->getCostmap()->getCost(ix,iy)));

      nav2dcell_t nav2dcell;
      nav2dcell.x = ix;
      nav2dcell.y = iy;
      changedcellsV.push_back(nav2dcell);
    }
  }

  try{
    if(!changedcellsV.empty()){
      StateChangeQuery* scq = new LatticeSCQ(env_, changedcellsV);
      planner_->costs_changed(*scq);
      delete scq;
    }

    if(allCount > force_scratch_limit_)
      planner_->force_planning_from_scratch();
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL failed to update the costmap");
    return false;
  }

  //setting planner parameters
  ROS_DEBUG("allocated:%f, init eps:%f\n",allocated_time_,initial_epsilon_);
  planner_->set_initialsolution_eps(initial_epsilon_);
  planner_->set_search_mode(false);

  ROS_DEBUG("[sbpl_lattice_planner] run planner");
  vector<int> solution_stateIDs;
  int solution_cost;
  try{
    int ret = planner_->replan(allocated_time_, &solution_stateIDs, &solution_cost);
    if(ret)
      ROS_DEBUG("Solution is found\n");
    else{
      ROS_INFO("Solution not found\n");
      publishStats(solution_cost, 0, start, goal);
      return false;
    }
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL encountered a fatal exception while planning");
    return false;
  }

  ROS_DEBUG("size of solution=%d", (int)solution_stateIDs.size());

  vector<EnvNAVXYTHETALAT3Dpt_t> sbpl_path;
  try{
    env_->ConvertStateIDPathintoXYThetaPath(&solution_stateIDs, &sbpl_path);
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL encountered a fatal exception while reconstructing the path");
    return false;
  }
  ROS_DEBUG("Plan has %d points.\n", (int)sbpl_path.size());
  ros::Time plan_time = ros::Time::now();

  //create a message for the plan 
  nav_msgs::Path gui_path;
  gui_path.poses.resize(sbpl_path.size());
  gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
  gui_path.header.stamp = plan_time;
  for(unsigned int i=0; i<sbpl_path.size(); i++){
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = plan_time;
    pose.header.frame_id = costmap_ros_->getGlobalFrameID();

    pose.pose.position.x = sbpl_path[i].x + costmap_ros_->getCostmap()->getOriginX();
    pose.pose.position.y = sbpl_path[i].y + costmap_ros_->getCostmap()->getOriginY();
    pose.pose.position.z = start.pose.position.z;

    tf::Quaternion temp;
    temp.setRPY(0,0,sbpl_path[i].theta);
    pose.pose.orientation.x = temp.getX();
    pose.pose.orientation.y = temp.getY();
    pose.pose.orientation.z = temp.getZ();
    pose.pose.orientation.w = temp.getW();

    plan.push_back(pose);

    gui_path.poses[i].pose.position.x = plan[i].pose.position.x;
    gui_path.poses[i].pose.position.y = plan[i].pose.position.y;
    gui_path.poses[i].pose.position.z = plan[i].pose.position.z;


    gui_path.poses[i].pose.orientation.x = plan[i].pose.orientation.x;
    gui_path.poses[i].pose.orientation.y = plan[i].pose.orientation.y;
    gui_path.poses[i].pose.orientation.z = plan[i].pose.orientation.z;
    gui_path.poses[i].pose.orientation.w = plan[i].pose.orientation.w;
  }
  plan_pub_.publish(gui_path);
  publishStats(solution_cost, sbpl_path.size(), start, goal);

  return true;
}

};
