#ifndef SBPL_LATTICE_PLANNER_H
#define SBPL_LATTICE_PLANNER_H

#include <iostream>
#include <vector>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Char.h>

#include <fstream>
using namespace std;

/** ROS **/
#include <ros/ros.h>

// Costmap used for the map representation
#include <costmap_2d/costmap_2d_ros.h>

// sbpl headers
#include <sbpl/headers.h>

//global representation
#include <nav_core/base_global_planner.h>

namespace sbpl_lattice_planner{

class SBPLLatticePlanner : public nav_core::BaseGlobalPlanner{
public:
  
  /**
   * @brief  Default constructor for the NavFnROS object
   */
  SBPLLatticePlanner();

  
  /**
   * @brief  Constructor for the SBPLLatticePlanner object
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
   */
  SBPLLatticePlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);


  /**
   * @brief  Initialization function for the SBPLLatticePlanner object
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
   */
  virtual void initialize(std::string name, 
                          costmap_2d::Costmap2DROS* costmap_ros);
  
  /**
   * @brief Given a goal pose in the world, compute a plan
   * @param start The start pose 
   * @param goal The goal pose 
   * @param plan The plan... filled by the planner
   * @return True if a valid plan was found, false otherwise
   */
  virtual bool makePlan(const geometry_msgs::PoseStamped& start, 
                        const geometry_msgs::PoseStamped& goal, 
                        std::vector<geometry_msgs::PoseStamped>& plan);

  virtual ~SBPLLatticePlanner(){};

private:
  unsigned char costMapCostToSBPLCost(unsigned char newcost);
  void publishStats(int solution_cost, int solution_size, 
                    const geometry_msgs::PoseStamped& start, 
                    const geometry_msgs::PoseStamped& goal);

  bool initialized_;

  SBPLPlanner* planner_;
  EnvironmentNAVXYTHETALAT* env_;
  
  std::string planner_type_; /**< sbpl method to use for planning.  choices are ARAPlanner and ADPlanner */

  double allocated_time_; /**< amount of time allowed for search */
  double initial_epsilon_; /**< initial epsilon for beginning the anytime search */

  std::string environment_type_; /** what type of environment in which to plan.  choices are 2D and XYThetaLattice. */ 
  std::string cost_map_topic_; /** what topic is being used for the costmap topic */

  bool forward_search_; /** whether to use forward or backward search */
  std::string primitive_filename_; /** where to find the motion primitives for the current robot */
  int force_scratch_limit_; /** the number of cells that have to be changed in the costmap to force the planner to plan from scratch even if its an incremental planner */

  unsigned char lethal_obstacle_;
  unsigned char inscribed_inflated_obstacle_;
  unsigned char sbpl_cost_multiplier_;

  // added by Ben to cater for the fixed path scenario and multiple bus stop scenario
  bool fix_path_flag_;
  bool multi_stop_flag_;
  std::string fix_path_topic_;
  ros::Publisher fix_path_pub_;
  nav_msgs::Path fix_path;
  std::ifstream odomFile;

  std::string vir_rd_topic_;
  std::string odom_file_;
  ros::Publisher vir_rd_pub_;
  ros::Publisher stopMarker_pub_;
  ros::Publisher stopmarkertext_pub_;
  nav_msgs::Path vir_rd;
  ros::Subscriber stopSub_;
  ros::Subscriber odomSub_;
  nav_msgs::Odometry odom_;
  std_msgs::Char stop_;
  void stopCallback(const std_msgs::Char& msg);
  void odomCallback(const nav_msgs::Odometry& msg);
  double dist_points(double x1, double y1, double x2, double y2);
  // end*****************************************************************************

  costmap_2d::Costmap2DROS* costmap_ros_; /**< manages the cost map for us */

  ros::Publisher plan_pub_;
  ros::Publisher stats_publisher_;
  
  std::vector<geometry_msgs::Point> footprint_;

};
};

#endif

