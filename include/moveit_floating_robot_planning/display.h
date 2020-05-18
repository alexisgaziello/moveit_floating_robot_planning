#ifndef MOVEIT_MOTION_PLANNING_RVIZ_PLUGIN_MOTION_PLANNING_FRAME_
#define MOVEIT_MOTION_PLANNING_RVIZ_PLUGIN_MOTION_PLANNING_FRAME_

#include <ros/ros.h>

#include <rviz/panel.h>

#include <QWidget>

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

#include <map>
#include <string>
#include <vector>
// #include <memory>

#include <moveit_floating_robot_planning/actions.h>

#include <geometry_msgs/Pose.h>
#include <octomap_server/OctomapServer.h>


#include <QTableView>

#include "ui_floating_robot_planning_frame.h"



class QLineEdit;


namespace moveit_floating_robot_planning {


class Display: public rviz::Panel
{

Q_OBJECT

public:
  // MotionPlanningFrame(MotionPlanningDisplay* pdisplay, rviz::DisplayContext* context, QWidget* parent = 0);
  // ~MotionPlanningFrame();

  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  Display( QWidget* parent = 0 );
  ~Display();

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;



  // static const int ITEM_TYPE_SCENE = 1;
  // static const int ITEM_TYPE_QUERY = 2;

  // void constructPlanningRequest(moveit_msgs::MotionPlanRequest& mreq);

  // void updateSceneMarkers(float wall_dt, float ros_dt);

  // void updateExternalCommunication();

  // MotionPlanningDisplay* planning_display_;
  // rviz::DisplayContext* context_;
  Ui::FloatingRobotPlanningUI* ui_;

  // moveit::planning_interface::MoveGroupInterfacePtr move_group_;
  // moveit::planning_interface::PlanningSceneInterfacePtr planning_scene_interface_;
  // moveit::semantic_world::SemanticWorldPtr semantic_world_;

  // moveit::planning_interface::MoveGroupInterface::PlanPtr current_plan_;
  // moveit_warehouse::PlanningSceneStoragePtr planning_scene_storage_;
  // moveit_warehouse::ConstraintsStoragePtr constraints_storage_;
  // moveit_warehouse::RobotStateStoragePtr robot_state_storage_;

  // std::shared_ptr<rviz::InteractiveMarker> scene_marker_;

  // typedef std::map<std::string, moveit_msgs::RobotState> RobotStateMap;
  // typedef std::pair<std::string, moveit_msgs::RobotState> RobotStatePair;
  // RobotStateMap robot_states_;
/*
Q_SIGNALS:
  // void planningFinished();

private Q_SLOTS:

  // General
  void tabChanged(int index);

  // XML Generation Tab
  void addActionButtonClicked();
  void removeActionButtonClicked();
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

  std::vector<ActionType> actionTypes;
  std::vector<Action> actions_;

  // Initialization
  void loadActionTypes(const std::string &, const bool & append = true);

  // XML tab
  std::string missionSavePath;
  std::string missionAppendPath;

  bool loadXMLMissionFile(const QString &);
  void addAction(const ActionType &);
  void addAction(const ActionType &, const double & x, const double & y, const double & z);
  void addAction(const ActionType &, geometry_msgs::Pose & );

  QStandardItemModel * actions_model_;
  QStandardItemModel * parameters_model_;
  
  void loadActionsTable();
  
  void appendToTable(const std::string & , QStandardItemModel * );
  void appendToTable(const QString &, QStandardItemModel *);
  void insertToTable(const std::string & , const int & , QStandardItemModel * );
  void insertToTable(const QString & , const int & , QStandardItemModel * );

  std::string actionsPublisher_publishTopic_;
  ros::Publisher actionsPublisher_adv_;

  void publishAction(const Action &);

  //Octomaps tab
  bool loadOctomap(const QString &, const bool & publish = true);
  void loadOctomapsTable();
  QStandardItemModel * octomaps_model_;
  std::vector<std::string> octomaps_filePaths;
  void publishSelectedOctomap(const std::string &);

  octomap_server::OctomapServer * octomapServer_;

  //Helpers

  // str to save the workspace path
  std::string workspacePath_;
  std::string worldFrame_;
  float markerSize_;

  static const int8_t DESCRIPTIONS_SIZE;
  static const std::string DESCRIPTIONS[];
  std::vector<int> freeIDs;
  
  bool strEndsWith(std::string const &a, std::string const &b);

  int centeredActionID_;
  void centerUIToAction(const Action &);
  void centerUIToSelectedAction();
  void updateColorOfAction(Action &);
  int getTypeOfAction(const std::string &);
*/
};

/*
class FloatingRobotPlanningDisplay;

// const std::string OBJECT_RECOGNITION_ACTION = "/recognize_objects";

// static const std::string TAB_CONTEXT = "Context";
// static const std::string TAB_PLANNING = "Planning";
// static const std::string TAB_MANIPULATION = "Manipulation";
// static const std::string TAB_OBJECTS = "Scene Objects";
// static const std::string TAB_SCENES = "Stored Scenes";
// static const std::string TAB_STATES = "Stored States";
// static const std::string TAB_STATUS = "Status";


class FloatingRobotPlanningFrame : public QWidget
{
  friend class MotionPlanningDisplay;
  Q_OBJECT

public:
  MotionPlanningFrame(MotionPlanningDisplay* pdisplay, rviz::DisplayContext* context, QWidget* parent = 0);
  ~MotionPlanningFrame();

  void changePlanningGroup();
  void enable();
  void disable();
  void sceneUpdate(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type);

protected:
  static const int ITEM_TYPE_SCENE = 1;
  static const int ITEM_TYPE_QUERY = 2;

  void constructPlanningRequest(moveit_msgs::MotionPlanRequest& mreq);

  void updateSceneMarkers(float wall_dt, float ros_dt);

  void updateExternalCommunication();

  MotionPlanningDisplay* planning_display_;
  rviz::DisplayContext* context_;
  Ui::MotionPlanningUI* ui_;

  moveit::planning_interface::MoveGroupInterfacePtr move_group_;
  moveit::planning_interface::PlanningSceneInterfacePtr planning_scene_interface_;
  moveit::semantic_world::SemanticWorldPtr semantic_world_;

  moveit::planning_interface::MoveGroupInterface::PlanPtr current_plan_;
  moveit_warehouse::PlanningSceneStoragePtr planning_scene_storage_;
  moveit_warehouse::ConstraintsStoragePtr constraints_storage_;
  moveit_warehouse::RobotStateStoragePtr robot_state_storage_;

  std::shared_ptr<rviz::InteractiveMarker> scene_marker_;

  typedef std::map<std::string, moveit_msgs::RobotState> RobotStateMap;
  typedef std::pair<std::string, moveit_msgs::RobotState> RobotStatePair;
  RobotStateMap robot_states_;

Q_SIGNALS:
  void planningFinished();

private Q_SLOTS:

  // Context tab
  void databaseConnectButtonClicked();
  void publishSceneButtonClicked();
  void planningAlgorithmIndexChanged(int index);
  void resetDbButtonClicked();
  void approximateIKChanged(int state);

  // Planning tab
  void planButtonClicked();
  void executeButtonClicked();
  void planAndExecuteButtonClicked();
  void stopButtonClicked();
  void allowReplanningToggled(bool checked);
  void allowLookingToggled(bool checked);
  void allowExternalProgramCommunication(bool enable);
  void pathConstraintsIndexChanged(int index);
  void useStartStateButtonClicked();
  void useGoalStateButtonClicked();
  void onClearOctomapClicked();

  // Scene Objects tab
  void importFileButtonClicked();
  void importUrlButtonClicked();
  void clearSceneButtonClicked();
  void sceneScaleChanged(int value);
  void sceneScaleStartChange();
  void sceneScaleEndChange();
  void removeObjectButtonClicked();
  void selectedCollisionObjectChanged();
  void objectPoseValueChanged(double value);
  void collisionObjectChanged(QListWidgetItem* item);
  void imProcessFeedback(visualization_msgs::InteractiveMarkerFeedback& feedback);
  void copySelectedCollisionObject();
  void exportAsTextButtonClicked();
  void importFromTextButtonClicked();

  // Stored scenes tab
  void saveSceneButtonClicked();
  void planningSceneItemClicked();
  void saveQueryButtonClicked();
  void deleteSceneButtonClicked();
  void deleteQueryButtonClicked();
  void loadSceneButtonClicked();
  void loadQueryButtonClicked();
  void warehouseItemNameChanged(QTreeWidgetItem* item, int column);

  // States tab
  void loadStateButtonClicked();
  void saveStartStateButtonClicked();
  void saveGoalStateButtonClicked();
  void removeStateButtonClicked();
  void clearStatesButtonClicked();
  void setAsStartStateButtonClicked();
  void setAsGoalStateButtonClicked();

  // Pick and place
  void detectObjectsButtonClicked();
  void pickObjectButtonClicked();
  void placeObjectButtonClicked();
  void selectedDetectedObjectChanged();
  void detectedObjectChanged(QListWidgetItem* item);
  void selectedSupportSurfaceChanged();

  // General
  void tabChanged(int index);

  // XML Generation Tab
  void addActionButtonClicked();
  void removeActionButtonClicked();
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
  // Context tab
  void computeDatabaseConnectButtonClicked();
  void computeDatabaseConnectButtonClickedHelper(int mode);
  void computeResetDbButtonClicked(const std::string& db);
  void populatePlannersList(const moveit_msgs::PlannerInterfaceDescription& desc);

  // Planning tab
  void computePlanButtonClicked();
  void computeExecuteButtonClicked();
  void computePlanAndExecuteButtonClicked();
  void computePlanAndExecuteButtonClickedDisplayHelper();
  void computeStopButtonClicked();
  void onFinishedExecution(bool success);
  void populateConstraintsList();
  void populateConstraintsList(const std::vector<std::string>& constr);
  void configureForPlanning();
  void configureWorkspace();
  void updateQueryStateHelper(robot_state::RobotState& state, const std::string& v);
  void fillStateSelectionOptions();
  void useStartStateButtonExec();
  void useGoalStateButtonExec();

  // Scene objects tab
  void addObject(const collision_detection::WorldPtr& world, const std::string& id, const shapes::ShapeConstPtr& shape,
                 const Eigen::Affine3d& pose);
  void updateCollisionObjectPose(bool update_marker_position);
  void createSceneInteractiveMarker();
  void renameCollisionObject(QListWidgetItem* item);
  void attachDetachCollisionObject(QListWidgetItem* item);
  void populateCollisionObjectsList();
  void computeImportFromText(const std::string& path);
  void computeExportAsText(const std::string& path);

  // Stored scenes tab
  void computeSaveSceneButtonClicked();
  void computeSaveQueryButtonClicked(const std::string& scene, const std::string& query_name);
  void computeLoadSceneButtonClicked();
  void computeLoadQueryButtonClicked();
  void populatePlanningSceneTreeView();
  void computeDeleteSceneButtonClicked();
  void computeDeleteQueryButtonClicked();
  void computeDeleteQueryButtonClickedHelper(QTreeWidgetItem* s);
  void checkPlanningSceneTreeEnabledButtons();

  // States tab
  void saveRobotStateButtonClicked(const robot_state::RobotState& state);
  void populateRobotStatesList();

  // Pick and place
  void processDetectedObjects();
  void updateDetectedObjectsList(const std::vector<std::string>& object_ids, const std::vector<std::string>& objects);
  void publishTables();
  void updateSupportSurfacesList();
  ros::Publisher object_recognition_trigger_publisher_;
  std::map<std::string, std::string> pick_object_name_;
  std::string place_object_name_;
  std::vector<geometry_msgs::PoseStamped> place_poses_;
  void pickObject();
  void placeObject();
  void triggerObjectDetection();
  void updateTables();
  std::string support_surface_name_;
  // For coloring
  std::string selected_object_name_;
  std::string selected_support_surface_name_;

  std::unique_ptr<actionlib::SimpleActionClient<object_recognition_msgs::ObjectRecognitionAction> >
      object_recognition_client_;
  template <typename T>
  void waitForAction(const T& action, const ros::NodeHandle& node_handle, const ros::Duration& wait_for_server,
                     const std::string& name);
  void listenDetectedObjects(const object_recognition_msgs::RecognizedObjectArrayPtr& msg);
  ros::Subscriber object_recognition_subscriber_;

  ros::Subscriber plan_subscriber_;
  ros::Subscriber execute_subscriber_;
  ros::Subscriber stop_subscriber_;
  ros::Subscriber update_start_state_subscriber_;
  ros::Subscriber update_goal_state_subscriber_;
  ros::Subscriber update_custom_start_state_subscriber_;
  ros::Subscriber update_custom_goal_state_subscriber_;
  // General
  void changePlanningGroupHelper();
  void importResource(const std::string& path);
  void loadStoredStates(const std::string& pattern);

  void remotePlanCallback(const std_msgs::EmptyConstPtr& msg);
  void remoteExecuteCallback(const std_msgs::EmptyConstPtr& msg);
  void remoteStopCallback(const std_msgs::EmptyConstPtr& msg);
  void remoteUpdateStartStateCallback(const std_msgs::EmptyConstPtr& msg);
  void remoteUpdateGoalStateCallback(const std_msgs::EmptyConstPtr& msg);
  void remoteUpdateCustomStartStateCallback(const moveit_msgs::RobotStateConstPtr& msg);
  void remoteUpdateCustomGoalStateCallback(const moveit_msgs::RobotStateConstPtr& msg);

  // Selects or unselects a item in a list by the item name 
  void setItemSelectionInList(const std::string& item_name, bool selection, QListWidget* list);

  ros::NodeHandle nh_;
  ros::Publisher planning_scene_publisher_;
  ros::Publisher planning_scene_world_publisher_;

  collision_detection::CollisionWorld::ObjectConstPtr scaled_object_;

  std::vector<std::pair<std::string, bool> > known_collision_objects_;
  long unsigned int known_collision_objects_version_;
  bool first_time_;
  ros::ServiceClient clear_octomap_service_client_;


  // UNEXMIN Functions //
  std::vector<ActionType> actionTypes;
  std::vector<Action> actions_;

  // Initialization
  void loadActionTypes(const std::string &, const bool & append = true);

  // XML tab
  std::string missionSavePath;
  std::string missionAppendPath;

  bool loadXMLMissionFile(const QString &);
  void addAction(const ActionType &);
  void addAction(const ActionType &, const double & x, const double & y, const double & z);
  void addAction(const ActionType &, geometry_msgs::Pose & );

  QStandardItemModel * actions_model_;
  QStandardItemModel * parameters_model_;
  
  void loadActionsTable();
  
  void appendToTable(const std::string & , QStandardItemModel * );
  void appendToTable(const QString &, QStandardItemModel *);
  void insertToTable(const std::string & , const int & , QStandardItemModel * );
  void insertToTable(const QString & , const int & , QStandardItemModel * );

  std::string actionsPublisher_publishTopic_;
  ros::Publisher actionsPublisher_adv_;

  void publishAction(const Action &);

  //Octomaps tab
  bool loadOctomap(const QString &, const bool & publish = true);
  void loadOctomapsTable();
  QStandardItemModel * octomaps_model_;
  std::vector<std::string> octomaps_filePaths;
  void publishSelectedOctomap(const std::string &);

  octomap_server::OctomapServer * octomapServer_;

  //Helpers

  // str to save the workspace path
  std::string workspacePath_;
  std::string worldFrame_;
  float markerSize_;

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

*/

} // namespace

#endif
