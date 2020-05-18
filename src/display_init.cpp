
#include <moveit_floating_robot_planning/display.h>

#include <ros/package.h>

#include <tinyxml.h>


namespace moveit_floating_robot_planning
{

// BEGIN_TUTORIAL
// Here is the implementation of the Display class.  Display
// has these responsibilities:
//
// - Act as a container for GUI elements DriveWidget and QLineEdit.
// - Publish command velocities 10 times per second (whether 0 or not).
// - Saving and restoring internal state from a config file.
//
// We start with the constructor, doing the standard Qt thing of
// passing the optional *parent* argument on to the superclass
// constructor, and also zero-ing the velocities we will be
// publishing.
Display::Display(QWidget* parent)
  : rviz::Panel( parent )
  , ui_(new Ui::FloatingRobotPlanningUI())
  // , actions_model_(nullptr)
  // , octomaps_model_(nullptr)
  // , centeredActionID_(-1)
{
  ui_->setupUi(this);

/*
   // UNEXMIN
  connect(ui_->add_action_button, SIGNAL(clicked()), this, SLOT(addActionButtonClicked()));
  connect(ui_->remove_action_button, SIGNAL(clicked()), this, SLOT(removeActionButtonClicked()));
  connect(ui_->import_xml_button, SIGNAL(clicked()), this, SLOT(importXMLButtonClicked()));
  connect(ui_->export_xml_button, SIGNAL(clicked()), this, SLOT(exportXMLButtonClicked()));
  connect(ui_->center_to_robot, SIGNAL(clicked()), this, SLOT(centerToRobotClicked()));
  connect(ui_->load_octomap_button, SIGNAL(clicked()), this, SLOT(loadOctomapButtonClicked()));
  connect(ui_->remove_octomap_button, SIGNAL(clicked()), this, SLOT(removeOctomapButtonClicked()));
  connect(ui_->action_type_combo_box, SIGNAL(currentIndexChanged(int)), this,
          SLOT(actionIndexChanged(int)));

  //Init workspacePath_
  // /home/alexis/catkin_ws/src/unexmin_mission_planning_ws/moveit/moveit_ros/visualization
  std::string workspacePath = ros::package::getPath("moveit_ros_visualization");
  //Take off: /src/unexmin_mission_planning_ws/moveit/moveit_ros/visualization
  workspacePath_ = workspacePath.substr(0, workspacePath.find("/src/")+1);


  //Parameters
  // TODO GLOBAL VS RELATIVE
  const std::string param_frame = "frame_id";
  if (!nh_.getParam(param_frame, worldFrame_))
  {
    worldFrame_ = "/world_ned";
    ROS_INFO("Could not get param '%s'. Setting frame to '%s'.", param_frame.c_str(), worldFrame_.c_str());
    nh_.setParam(param_frame, worldFrame_);
  }

  const std::string param_actionPublish = "action_markers_topic";
  if (!nh_.getParam(param_actionPublish, actionsPublisher_publishTopic_))
  {
    actionsPublisher_publishTopic_ = "action_markers";
    ROS_INFO("Could not get param to publish actions '%s'. Setting topic to '%s'.", param_actionPublish.c_str(), actionsPublisher_publishTopic_.c_str());
    nh_.setParam(param_actionPublish, actionsPublisher_publishTopic_);
  }

  const std::string param_marker = "action_markers_size";
  if (!nh_.getParam(param_marker, markerSize_))
  {
    markerSize_ = 0.3;
    ROS_INFO("Could not get param '%s'. Setting marker size to %fm.", param_marker.c_str(), markerSize_);
    nh_.setParam(param_marker, markerSize_);
  }

  const std::string save_mission_param = "mission_save_path";
  if (!nh_.getParam(save_mission_param, missionSavePath))
  {
    missionSavePath = workspacePath_ + "/resources/mission.xml";
    ROS_INFO("Could not get param '%s'. mission.xml will be save in: '%s'.", save_mission_param.c_str(), missionSavePath.c_str());
  }

  const std::string mission_append_param = "mission_append_path";
  if (!nh_.getParam(mission_append_param, missionAppendPath))
  {
    missionAppendPath = "";
    ROS_INFO("Could not get param '%s'. Information will be appended if found in the 'resources/append.xml' file.", mission_append_param.c_str());
  }

  // const std::string param_markerArrayTopic = "publish_topic_markersArray";
  // std::string markerArrayTopic = "occupied_cells_vis_array";
  // if (!nh_.getParam(param_markerArrayTopic, markerArrayTopic))
  // {
  //   ROS_INFO("Could not get param '%s'. Setting marker size to default: '%s'.", param_markerArrayTopic.c_str(), markerArrayTopic.c_str());
  //   nh_.setParam(param_markerArrayTopic, markerArrayTopic);
  // }

  // Other
  loadActionsTable();
  loadOctomapsTable();
  actionsPublisher_adv_ = nh_.advertise<visualization_msgs::Marker>(actionsPublisher_publishTopic_, 1000);
  octomapServer_ = new octomap_server::OctomapServer(nh_);

  // Load all octomaps from resources/octomaps/
  std::string pathToOctomaps = workspacePath_ + "resources/octomaps/";
  if(boost::filesystem::is_directory(pathToOctomaps)) {
    for(auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(pathToOctomaps), {})){
      loadOctomap(QString::fromStdString(entry.path().string()), false);
    }
    ROS_INFO("Loaded all octomaps found in %s", pathToOctomaps.c_str());
  } else {
    ROS_INFO("Directory %s not existing\nCould not load any octomaps.", pathToOctomaps.c_str());
  }

  // Load octomaps from parameter if specified
  if (!nh_.getParam("octomaps_path", pathToOctomaps))
  {
    ROS_INFO("Could not get param 'octomaps_path'. Loading Octomaps only from default folder: 'resources/octomaps'.");
  } else {
    // if is a file (must ending with proper extension)
    if (strEndsWith(pathToOctomaps, ".bt") || strEndsWith(pathToOctomaps, ".ot")) {
      loadOctomap(QString::fromStdString(pathToOctomaps), false);
    } else {// else we suppose is a directory
      // Load all octomaps from resources/octomaps/
      if(boost::filesystem::is_directory(pathToOctomaps)) {
        for(auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(pathToOctomaps), {})){
          loadOctomap(QString::fromStdString(entry.path().string()), false);
        }
        ROS_INFO("Loaded all octomaps found in %s", pathToOctomaps.c_str());
      } else {
        ROS_INFO("Directory %s not existing\nCould not load any octomaps.", pathToOctomaps.c_str());
      }
    }
  }


  // Load all action types 
  std::string pathToActionsDefinition = workspacePath_ + "resources/actions_definitions.xml";
  loadActionTypes(pathToActionsDefinition);

   // Load action types from parameter if specified
  if (!nh_.getParam("actions_definitions_path", pathToActionsDefinition))
  {
    ROS_INFO("Could not get param 'actions_definitions_path'. Loading actions definitions only from default folder: 'resources'.");
  } else {
    loadActionTypes(pathToActionsDefinition);
  }

  // Add Waypoint as default. if len <1
  if (actionTypes.size() == 0){
    ROS_ERROR("No actions defined. Adding Waypoint as default.");
    actionTypes.emplace_back(ActionType("Waypoint", 2, 0, 0, 1, 1));
    ui_->action_type_combo_box->addItem(QString("Waypoint"));
  }*/

}

Display::~Display(){
  delete ui_;
  // delete actions_model_;
  // delete octomaps_model_;
}


// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void Display::save( rviz::Config config ) const
{
  // rviz::Panel::save( config );
  // config.mapSetValue( "Topic", output_topic_ );
}

// Load all configuration data for this panel from the given Config object.
void Display::load( const rviz::Config& config )
{
  // rviz::Panel::load( config );
  // QString topic;
  // if( config.mapGetString( "Topic", &topic ))
  // {
  //   output_topic_editor_->setText( topic );
  //   updateTopic();
  // }
}

/*

#include <moveit/motion_planning_rviz_plugin/motion_planning_frame.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>
#include <moveit/move_group/capability_names.h>

#include <geometric_shapes/shape_operations.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>

#include <std_srvs/Empty.h>

#include <QMessageBox>
#include <QInputDialog>
#include <QShortcut>

#include "ui_motion_planning_rviz_plugin_frame.h"

// UNEXMIN
#include <tinyxml.h>

namespace moveit_floating_robot_planning
{
Display::MotionPlanningFrame(MotionPlanningDisplay* pdisplay, rviz::DisplayContext* context,
                                         QWidget* parent)
  : QWidget(parent)
  , planning_display_(pdisplay)
  , context_(context)
  , ui_(new Ui::MotionPlanningUI())
  , first_time_(true)
  , clear_octomap_service_client_(nh_.serviceClient<std_srvs::Empty>(move_group::CLEAR_OCTOMAP_SERVICE_NAME))
  // UNEXMIN
  , actions_model_(nullptr)
  , octomaps_model_(nullptr)
  , centeredActionID_(-1)

{
  // set up the GUI
  ui_->setupUi(this);

  // connect bottons to actions; each action usually registers the function pointer for the actual computation,
  // to keep the GUI more responsive (using the background job processing)
  connect(ui_->plan_button, SIGNAL(clicked()), this, SLOT(planButtonClicked()));
  connect(ui_->execute_button, SIGNAL(clicked()), this, SLOT(executeButtonClicked()));
  connect(ui_->plan_and_execute_button, SIGNAL(clicked()), this, SLOT(planAndExecuteButtonClicked()));
  connect(ui_->stop_button, SIGNAL(clicked()), this, SLOT(stopButtonClicked()));
  connect(ui_->use_start_state_button, SIGNAL(clicked()), this, SLOT(useStartStateButtonClicked()));
  connect(ui_->use_goal_state_button, SIGNAL(clicked()), this, SLOT(useGoalStateButtonClicked()));
  connect(ui_->database_connect_button, SIGNAL(clicked()), this, SLOT(databaseConnectButtonClicked()));
  connect(ui_->save_scene_button, SIGNAL(clicked()), this, SLOT(saveSceneButtonClicked()));
  connect(ui_->save_query_button, SIGNAL(clicked()), this, SLOT(saveQueryButtonClicked()));
  connect(ui_->delete_scene_button, SIGNAL(clicked()), this, SLOT(deleteSceneButtonClicked()));
  connect(ui_->delete_query_button, SIGNAL(clicked()), this, SLOT(deleteQueryButtonClicked()));
  connect(ui_->planning_scene_tree, SIGNAL(itemSelectionChanged()), this, SLOT(planningSceneItemClicked()));
  connect(ui_->load_scene_button, SIGNAL(clicked()), this, SLOT(loadSceneButtonClicked()));
  connect(ui_->load_query_button, SIGNAL(clicked()), this, SLOT(loadQueryButtonClicked()));
  connect(ui_->allow_looking, SIGNAL(toggled(bool)), this, SLOT(allowLookingToggled(bool)));
  connect(ui_->allow_replanning, SIGNAL(toggled(bool)), this, SLOT(allowReplanningToggled(bool)));
  connect(ui_->allow_external_program, SIGNAL(toggled(bool)), this, SLOT(allowExternalProgramCommunication(bool)));
  connect(ui_->planning_algorithm_combo_box, SIGNAL(currentIndexChanged(int)), this,
          SLOT(planningAlgorithmIndexChanged(int)));
  connect(ui_->planning_algorithm_combo_box, SIGNAL(currentIndexChanged(int)), this,
          SLOT(planningAlgorithmIndexChanged(int)));
  connect(ui_->import_file_button, SIGNAL(clicked()), this, SLOT(importFileButtonClicked()));
  connect(ui_->import_url_button, SIGNAL(clicked()), this, SLOT(importUrlButtonClicked()));
  connect(ui_->clear_scene_button, SIGNAL(clicked()), this, SLOT(clearSceneButtonClicked()));
  connect(ui_->scene_scale, SIGNAL(valueChanged(int)), this, SLOT(sceneScaleChanged(int)));
  connect(ui_->scene_scale, SIGNAL(sliderPressed()), this, SLOT(sceneScaleStartChange()));
  connect(ui_->scene_scale, SIGNAL(sliderReleased()), this, SLOT(sceneScaleEndChange()));
  connect(ui_->remove_object_button, SIGNAL(clicked()), this, SLOT(removeObjectButtonClicked()));
  connect(ui_->object_x, SIGNAL(valueChanged(double)), this, SLOT(objectPoseValueChanged(double)));
  connect(ui_->object_y, SIGNAL(valueChanged(double)), this, SLOT(objectPoseValueChanged(double)));
  connect(ui_->object_z, SIGNAL(valueChanged(double)), this, SLOT(objectPoseValueChanged(double)));
  connect(ui_->object_rx, SIGNAL(valueChanged(double)), this, SLOT(objectPoseValueChanged(double)));
  connect(ui_->object_ry, SIGNAL(valueChanged(double)), this, SLOT(objectPoseValueChanged(double)));
  connect(ui_->object_rz, SIGNAL(valueChanged(double)), this, SLOT(objectPoseValueChanged(double)));
  connect(ui_->publish_current_scene_button, SIGNAL(clicked()), this, SLOT(publishSceneButtonClicked()));
  connect(ui_->collision_objects_list, SIGNAL(itemSelectionChanged()), this, SLOT(selectedCollisionObjectChanged()));
  connect(ui_->collision_objects_list, SIGNAL(itemChanged(QListWidgetItem*)), this,
          SLOT(collisionObjectChanged(QListWidgetItem*)));
  connect(ui_->path_constraints_combo_box, SIGNAL(currentIndexChanged(int)), this,
          SLOT(pathConstraintsIndexChanged(int)));
  connect(ui_->clear_octomap_button, SIGNAL(clicked()), this, SLOT(onClearOctomapClicked()));
  connect(ui_->planning_scene_tree, SIGNAL(itemChanged(QTreeWidgetItem*, int)), this,
          SLOT(warehouseItemNameChanged(QTreeWidgetItem*, int)));
  connect(ui_->reset_db_button, SIGNAL(clicked()), this, SLOT(resetDbButtonClicked()));
  connect(ui_->export_scene_text_button, SIGNAL(clicked()), this, SLOT(exportAsTextButtonClicked()));
  connect(ui_->import_scene_text_button, SIGNAL(clicked()), this, SLOT(importFromTextButtonClicked()));
  connect(ui_->load_state_button, SIGNAL(clicked()), this, SLOT(loadStateButtonClicked()));
  connect(ui_->save_start_state_button, SIGNAL(clicked()), this, SLOT(saveStartStateButtonClicked()));
  connect(ui_->save_goal_state_button, SIGNAL(clicked()), this, SLOT(saveGoalStateButtonClicked()));
  connect(ui_->set_as_start_state_button, SIGNAL(clicked()), this, SLOT(setAsStartStateButtonClicked()));
  connect(ui_->set_as_goal_state_button, SIGNAL(clicked()), this, SLOT(setAsGoalStateButtonClicked()));
  connect(ui_->remove_state_button, SIGNAL(clicked()), this, SLOT(removeStateButtonClicked()));
  connect(ui_->clear_states_button, SIGNAL(clicked()), this, SLOT(clearStatesButtonClicked()));
  connect(ui_->approximate_ik, SIGNAL(stateChanged(int)), this, SLOT(approximateIKChanged(int)));

  connect(ui_->detect_objects_button, SIGNAL(clicked()), this, SLOT(detectObjectsButtonClicked()));
  connect(ui_->pick_button, SIGNAL(clicked()), this, SLOT(pickObjectButtonClicked()));
  connect(ui_->place_button, SIGNAL(clicked()), this, SLOT(placeObjectButtonClicked()));
  connect(ui_->detected_objects_list, SIGNAL(itemSelectionChanged()), this, SLOT(selectedDetectedObjectChanged()));
  connect(ui_->detected_objects_list, SIGNAL(itemChanged(QListWidgetItem*)), this,
          SLOT(detectedObjectChanged(QListWidgetItem*)));
  connect(ui_->support_surfaces_list, SIGNAL(itemSelectionChanged()), this, SLOT(selectedSupportSurfaceChanged()));

  connect(ui_->tabWidget, SIGNAL(currentChanged(int)), this, SLOT(tabChanged(int)));

  // UNEXMIN
  connect(ui_->add_action_button, SIGNAL(clicked()), this, SLOT(addActionButtonClicked()));
  connect(ui_->remove_action_button, SIGNAL(clicked()), this, SLOT(removeActionButtonClicked()));
  connect(ui_->import_xml_button, SIGNAL(clicked()), this, SLOT(importXMLButtonClicked()));
  connect(ui_->export_xml_button, SIGNAL(clicked()), this, SLOT(exportXMLButtonClicked()));
  connect(ui_->center_to_robot, SIGNAL(clicked()), this, SLOT(centerToRobotClicked()));
  connect(ui_->load_octomap_button, SIGNAL(clicked()), this, SLOT(loadOctomapButtonClicked()));
  connect(ui_->remove_octomap_button, SIGNAL(clicked()), this, SLOT(removeOctomapButtonClicked()));
  connect(ui_->action_type_combo_box, SIGNAL(currentIndexChanged(int)), this,
          SLOT(actionIndexChanged(int)));

  //Init workspacePath_
  // /home/alexis/catkin_ws/src/unexmin_mission_planning_ws/moveit/moveit_ros/visualization
  std::string workspacePath = ros::package::getPath("moveit_ros_visualization");
  //Take off: /src/unexmin_mission_planning_ws/moveit/moveit_ros/visualization
  workspacePath_ = workspacePath.substr(0, workspacePath.find("/src/")+1);


  QShortcut* copy_object_shortcut = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_C), ui_->collision_objects_list);
  connect(copy_object_shortcut, SIGNAL(activated()), this, SLOT(copySelectedCollisionObject()));

  ui_->reset_db_button->hide();
  ui_->background_job_progress->hide();
  ui_->background_job_progress->setMaximum(0);

  ui_->tabWidget->setCurrentIndex(0);

  known_collision_objects_version_ = 0;

  planning_scene_publisher_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  planning_scene_world_publisher_ = nh_.advertise<moveit_msgs::PlanningSceneWorld>("planning_scene_world", 1);

  // object_recognition_trigger_publisher_ = nh_.advertise<std_msgs::Bool>("recognize_objects_switch", 1);
  object_recognition_client_.reset(new actionlib::SimpleActionClient<object_recognition_msgs::ObjectRecognitionAction>(
      OBJECT_RECOGNITION_ACTION, false));
  object_recognition_subscriber_ =
      nh_.subscribe("recognized_object_array", 1, &Display::listenDetectedObjects, this);

  if (object_recognition_client_)
  {
    try
    {
      waitForAction(object_recognition_client_, nh_, ros::Duration(3.0), OBJECT_RECOGNITION_ACTION);
    }
    catch (std::exception& ex)
    {
      // ROS_ERROR("Object recognition action: %s", ex.what());
      object_recognition_client_.reset();
    }
  }
  try
  {
    planning_scene_interface_.reset(new moveit::planning_interface::PlanningSceneInterface());
  }
  catch (std::exception& ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  try
  {
    const planning_scene_monitor::LockedPlanningSceneRO& ps = planning_display_->getPlanningSceneRO();
    if (ps)
    {
      semantic_world_.reset(new moveit::semantic_world::SemanticWorld(ps));
    }
    else
      semantic_world_.reset();
    if (semantic_world_)
    {
      semantic_world_->addTableCallback(boost::bind(&Display::updateTables, this));
    }
  }
  catch (std::exception& ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  //UNEXMIN

  //Parameters
  // TODO GLOBAL VS RELATIVE
  const std::string param_frame = "frame_id";
  if (!nh_.getParam(param_frame, worldFrame_))
  {
    worldFrame_ = "/world_ned";
    ROS_INFO("Could not get param '%s'. Setting frame to '%s'.", param_frame.c_str(), worldFrame_.c_str());
    nh_.setParam(param_frame, worldFrame_);
  }

  const std::string param_actionPublish = "action_markers_topic";
  if (!nh_.getParam(param_actionPublish, actionsPublisher_publishTopic_))
  {
    actionsPublisher_publishTopic_ = "action_markers";
    ROS_INFO("Could not get param to publish actions '%s'. Setting topic to '%s'.", param_actionPublish.c_str(), actionsPublisher_publishTopic_.c_str());
    nh_.setParam(param_actionPublish, actionsPublisher_publishTopic_);
  }

  const std::string param_marker = "action_markers_size";
  if (!nh_.getParam(param_marker, markerSize_))
  {
    markerSize_ = 0.3;
    ROS_INFO("Could not get param '%s'. Setting marker size to %fm.", param_marker.c_str(), markerSize_);
    nh_.setParam(param_marker, markerSize_);
  }

  const std::string save_mission_param = "mission_save_path";
  if (!nh_.getParam(save_mission_param, missionSavePath))
  {
    missionSavePath = workspacePath_ + "/resources/mission.xml";
    ROS_INFO("Could not get param '%s'. mission.xml will be save in: '%s'.", save_mission_param.c_str(), missionSavePath.c_str());
  }

  const std::string mission_append_param = "mission_append_path";
  if (!nh_.getParam(mission_append_param, missionAppendPath))
  {
    missionAppendPath = "";
    ROS_INFO("Could not get param '%s'. Information will be appended if found in the 'resources/append.xml' file.", mission_append_param.c_str());
  }

  // const std::string param_markerArrayTopic = "publish_topic_markersArray";
  // std::string markerArrayTopic = "occupied_cells_vis_array";
  // if (!nh_.getParam(param_markerArrayTopic, markerArrayTopic))
  // {
  //   ROS_INFO("Could not get param '%s'. Setting marker size to default: '%s'.", param_markerArrayTopic.c_str(), markerArrayTopic.c_str());
  //   nh_.setParam(param_markerArrayTopic, markerArrayTopic);
  // }

  // Other
  loadActionsTable();
  loadOctomapsTable();
  actionsPublisher_adv_ = nh_.advertise<visualization_msgs::Marker>(actionsPublisher_publishTopic_, 1000);
  octomapServer_ = new octomap_server::OctomapServer(nh_);

  // Load all octomaps from resources/octomaps/
  std::string pathToOctomaps = workspacePath_ + "resources/octomaps/";
  if(boost::filesystem::is_directory(pathToOctomaps)) {
    for(auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(pathToOctomaps), {})){
      loadOctomap(QString::fromStdString(entry.path().string()), false);
    }
    ROS_INFO("Loaded all octomaps found in %s", pathToOctomaps.c_str());
  } else {
    ROS_INFO("Directory %s not existing\nCould not load any octomaps.", pathToOctomaps.c_str());
  }

  // Load octomaps from parameter if specified
  if (!nh_.getParam("octomaps_path", pathToOctomaps))
  {
    ROS_INFO("Could not get param 'octomaps_path'. Loading Octomaps only from default folder: 'resources/octomaps'.");
  } else {
    // if is a file (must ending with proper extension)
    if (strEndsWith(pathToOctomaps, ".bt") || strEndsWith(pathToOctomaps, ".ot")) {
      loadOctomap(QString::fromStdString(pathToOctomaps), false);
    } else {// else we suppose is a directory
      // Load all octomaps from resources/octomaps/
      if(boost::filesystem::is_directory(pathToOctomaps)) {
        for(auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(pathToOctomaps), {})){
          loadOctomap(QString::fromStdString(entry.path().string()), false);
        }
        ROS_INFO("Loaded all octomaps found in %s", pathToOctomaps.c_str());
      } else {
        ROS_INFO("Directory %s not existing\nCould not load any octomaps.", pathToOctomaps.c_str());
      }
    }
  }


  // Load all action types 
  std::string pathToActionsDefinition = workspacePath_ + "resources/actions_definitions.xml";
  loadActionTypes(pathToActionsDefinition);

   // Load action types from parameter if specified
  if (!nh_.getParam("actions_definitions_path", pathToActionsDefinition))
  {
    ROS_INFO("Could not get param 'actions_definitions_path'. Loading actions definitions only from default folder: 'resources'.");
  } else {
    loadActionTypes(pathToActionsDefinition);
  }

  // Add Waypoint as default. if len <1
  if (actionTypes.size() == 0){
    ROS_ERROR("No actions defined. Adding Waypoint as default.");
    actionTypes.emplace_back(ActionType("Waypoint", 2, 0, 0, 1, 1));
    ui_->action_type_combo_box->addItem(QString("Waypoint"));
  }

}


void Display::loadActionTypes(const std::string & pathToActionsDefinition,  const bool & append){
  if(!append){
    actionTypes.clear();
    ui_->action_type_combo_box->clear();
  }

  TiXmlDocument doc(pathToActionsDefinition);
  if(doc.LoadFile()){
    TiXmlElement *pActions;
    pActions = doc.FirstChildElement("actions");
    if(pActions) {       
      // Search all tasks
      TiXmlElement *pTask;
      pTask = pActions->FirstChildElement("task");
      while(pTask) {
        const char * description = pTask->Attribute("description");
        
        if (description==NULL){
          ROS_ERROR("Task with undefined description. Skipping.");
        } else {
          
          const std::string name(description);

          // Default = sphere
          int shape = 2;
          
          const char * representation = pTask->Attribute("representation");
          if(representation){
            switch (*representation){
              case 'A': // Arrow
              case 'a':
              case '0':
                shape = 0;
                break;
              case 'C':
              case 'c':
                switch (*(representation+1)){
                  case 'Y': // Cylinder
                  case 'y':
                    shape = 3;
                    break;
                  case 'U': // Cube
                  case 'u':
                    shape = 1;
                    break;
                  default: // Cx means cube
                    shape = 1;
                    break;
                }
                break;
              case '1':
                shape = 1;
                break;
              case '3':
                shape = 3;
                break;
              case 'S': // Sphere
              case 's':
              case '2':
                shape = 2;
                break;
              case 'L': // Line
              case 'l':
              case '4':
                shape = 4;
                break;
              case 'P': // Point
              case 'p':
              case '8':
                shape = 8;
                break;
              default:
                ROS_ERROR("Unrecognized representation: %s\nLists are unsupported.\nUse same format as RViz markers.\nSet to SPHERE.", representation);
                //Default alreade set to int type = 2;
                break;
                              
            }
          }


          // Colors
          float r = 0;
          float g = 0;
          float b = 0;
          float a = 1;     
          
          const char * pR = pTask->Attribute("r");
          if(pR){
            const std::string r_str(pR);
            // get numbers
            r = std::stof(r_str);
            // If > 1, it was written in 255 format. Let's change it
            if (r>1) r=r/255;
          }
          const char * pG = pTask->Attribute("g");
          if(pG){
            const std::string g_str(pG);
            // get numbers
            g = std::stof(g_str);
            // If > 1, it was written in 255 format. Let's change it
            if (g>1) g=g/255;
          }
          const char * pB = pTask->Attribute("b");
          if(pB){
            const std::string b_str(pB);
            // get numbers
            b = std::stof(b_str);
            // If > 1, it was written in 255 format. Let's change it
            if (b>1) b=b/255;
          }
          if (pR == NULL && pG == NULL && pB == NULL){
            // Default to blue
            b=1;
          }
          const char * pA = pTask->Attribute("a");
          if(pA){
            const std::string a_str(pA);
            // get numbers
            a = std::stof(a_str);
            // If > 1, it was written in 255 format. Let's change it
            if (a>1) a=a/255;
          }            
          ActionType actionType(name, shape, r, g, b, a);

          // Parameters
          TiXmlElement *pParam;
          pParam = pTask->FirstChildElement("param");
          while(pParam){
            // Add parameter
            const char * param_desc = pParam->Attribute("description");
            if (param_desc){
              const std::string param_name(param_desc);
              std::string param_value = "";
              if (pParam->GetText()){
                param_value = pParam->GetText();
              }
              actionType.addParam(param_name, param_value);
            } else {
              ROS_ERROR("Parameter with undefined description. Skipping.");
            }
            pParam = pParam->NextSiblingElement("param");
          }

          actionTypes.emplace_back(actionType);
          ui_->action_type_combo_box->addItem(QString::fromStdString(actionType.name));

        }
        pTask = pTask->NextSiblingElement("task");
      }
      
      // General Parameters
      TiXmlElement *pParam;
      pParam = pActions->FirstChildElement("param");
      while(pParam){
        // Add parameter
        const char * param_desc = pParam->Attribute("description");
        if (param_desc){
          const std::string param_name(param_desc);
          std::string param_value = "";
          if (pParam->GetText()){
            param_value = pParam->GetText();
          }
          // Add to all actions types
          for(int i=0; i < actionTypes.size(); i++){
            actionTypes[i].addParam(param_name, param_value);
          }
        } else {
          ROS_ERROR("Parameter with undefined description. Skipping.");
        }
        pParam = pParam->NextSiblingElement("param");
      }

      ROS_INFO("Added %zd actions type(s) (tasks) from XML File: %s", actionTypes.size(), pathToActionsDefinition.c_str());
    
    } else {
      ROS_ERROR("Could not find any actions definition in the file: %s", pathToActionsDefinition.c_str());
    }
  } else {
    ROS_ERROR("Could not open file: %s", pathToActionsDefinition.c_str());
  }
  

  actionIndexChanged(ui_->action_type_combo_box->currentIndex());
}

bool Display::strEndsWith(std::string const &a, std::string const &b) {
    auto len = b.length();
    auto pos = a.length() - len;
    if (pos < 0)
        return false;
    auto pos_a = &a[pos];
    auto pos_b = &b[0];
    while (*pos_a)
        if (*pos_a++ != *pos_b++)
            return false;
    return true;
}



*/
}  // namespace

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(moveit_floating_robot_planning::Display, rviz::Panel)