#ifndef MOVEIT_MOTION_PLANNING_RVIZ_PLUGIN_MOTION_PLANNING_FRAME_
#define MOVEIT_MOTION_PLANNING_RVIZ_PLUGIN_MOTION_PLANNING_FRAME_

#include <moveit_floating_robot_planning/actions.h>

#include "ui_floating_robot_planning_frame.h"

// STANDARD
#include <map>
#include <string>
#include <vector>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <octomap_server/OctomapServer.h>
#include <topic_tools/shape_shifter.h>


// Riz
#include <rviz/panel.h>

//QT
#include <QWidget>
#include <QTableView>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/background_processing/background_processing.h>


// #include <QTreeWidgetItem>
// #include <QListWidgetItem>

// #ifndef Q_MOC_RUN
// #include <moveit/macros/class_forward.h>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit/planning_scene_monitor/planning_scene_monitor.h>
// #include <moveit/robot_interaction/robot_interaction.h>
// #include <moveit/robot_interaction/interaction_handler.h>
// #include <moveit/semantic_world/semantic_world.h>
// #include <interactive_markers/interactive_marker_server.h>
// #include <rviz/default_plugin/interactive_markers/interactive_marker.h>
// #include <moveit_msgs/MotionPlanRequest.h>
// #include <actionlib/client/simple_action_client.h>
// #include <object_recognition_msgs/ObjectRecognitionAction.h>

// #include <std_msgs/Bool.h>
// #include <std_msgs/Empty.h>
// #endif



class QLineEdit;


namespace moveit_floating_robot_planning {


class Display: public rviz::Panel
{

Q_OBJECT

public:

  Display( QWidget* parent = 0 );
  ~Display();

 
  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

  Ui::FloatingRobotPlanningUI* ui_;
  

protected:

  moveit::tools::BackgroundProcessing background_process_;

private Q_SLOTS:

  // General
  void tabChanged(int index);

  // XML Generation Tab
  void addActionButtonClicked();
  void removeActionButtonClicked();
  void generateIntermediateWaypoints();
  void importXMLButtonClicked();
  void exportXMLButtonClicked();
  void centerToRobotClicked();
  void actionTableSelectionChanged(const QItemSelection &, const QItemSelection &);
  void actionIndexChanged(int index);

  // Octomap Tab
  void loadOctomapButtonClicked();
  void removeOctomapButtonClicked();
  void octomapsTableSelectionChanged(const QItemSelection &, const QItemSelection &);


private:

  ros::NodeHandle nh_;

  // TODO get
  std::string frame_id_ = "world_ned";

  moveit::planning_interface::MoveGroupInterface * move_group;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


  std::vector<ActionType> actionTypes;
  std::vector<Action> actions_;
  moveit_msgs::DisplayTrajectory plan_;

  ros::AsyncSpinner spinner;

  // Initialization
  void loadActionTypes(const std::string &, const bool & append = true);

  // XML tab
  geometry_msgs::Pose getRobotPosition();

  std::string missionSavePath;
  std::string missionAppendPath;

  bool loadXMLMissionFile(const QString &);
  void addAction(const ActionType &);
  void addAction(const ActionType &, const double & x, const double & y, const double & z);
  void addAction(const ActionType &, const geometry_msgs::Pose & );

  QStandardItemModel * actions_model_;
  QStandardItemModel * parameters_model_;
    
  void appendToTable(const std::string & , QStandardItemModel * );
  void appendToTable(const QString &, QStandardItemModel *);
  void insertToTable(const std::string & , const int & , QStandardItemModel * );
  void insertToTable(const QString & , const int & , QStandardItemModel * );

  std::string actionsPublisher_publishTopic_;
  ros::Publisher actionsPublisher_adv_;
  ros::Publisher trajectory_pub;


  void publishAction(const Action &);

  void plan(geometry_msgs::Pose & start, geometry_msgs::Pose & goal, bool CREATE_TRAJECTORY=false);

  //Octomaps tab
  bool loadOctomap(const QString &, const bool & publish = true);
  QStandardItemModel * octomaps_model_;
  std::vector<std::string> octomaps_filePaths;
  void publishSelectedOctomap(const std::string &);

  octomap_server::OctomapServer * octomapServer_;

  void octomapsPublisherHandleMessage(const topic_tools::ShapeShifter::ConstPtr& msg);
  void publishToPlanningScene(const octomap_msgs::Octomap & msg);
  ros::Subscriber octomapsPlanningScene_subs_;
  ros::Publisher octomapsPlanningScene_pub_;
  ros::Publisher octomapsMoveGroupPlanningScene_pub_;

  //Helpers

  // str to save the workspace path
  std::string workspacePath_;
  std::string worldFrame_;
  float markerSize_;
  std::string base_link;
  std::string robot_group;

  static const int8_t DESCRIPTIONS_SIZE;
  static const std::string DESCRIPTIONS[];
  std::vector<int> freeIDs;
  
  bool strEndsWith(std::string const &a, std::string const &b);

  int centeredActionID_;
  void centerUIToAction(const Action &);
  void centerUIToSelectedAction();
  void updateColorOfAction(Action &);
  int getTypeOfAction(const std::string &);

};

} // namespace

#endif
