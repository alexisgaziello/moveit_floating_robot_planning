
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
  , actions_model_(new QStandardItemModel(this))
  , octomaps_model_(new QStandardItemModel(this))
  , centeredActionID_(-1)
  , spinner(1)
{
  ui_->setupUi(this);
  
 

   // UNEXMIN
  connect(ui_->add_action_button, SIGNAL(clicked()), this, SLOT(addActionButtonClicked()));
  connect(ui_->remove_action_button, SIGNAL(clicked()), this, SLOT(removeActionButtonClicked()));
  connect(ui_->intermediate_waypoints_button, SIGNAL(clicked()), this, SLOT(generateIntermediateWaypoints()));
  connect(ui_->import_xml_button, SIGNAL(clicked()), this, SLOT(importXMLButtonClicked()));
  connect(ui_->export_xml_button, SIGNAL(clicked()), this, SLOT(exportXMLButtonClicked()));
  connect(ui_->center_to_robot, SIGNAL(clicked()), this, SLOT(centerToRobotClicked()));
  connect(ui_->load_octomap_button, SIGNAL(clicked()), this, SLOT(loadOctomapButtonClicked()));
  connect(ui_->remove_octomap_button, SIGNAL(clicked()), this, SLOT(removeOctomapButtonClicked()));
  connect(ui_->action_type_combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(actionIndexChanged(int)));


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

  const std::string base_link_param = "base_link";
  if (!nh_.getParam(base_link_param, base_link))
  {
    base_link = "body";
    ROS_INFO("Could not get param '%s'. Taking default value as '%s'.", base_link_param.c_str(), base_link.c_str());
  }

  const std::string robot_group_param = "planning_group";
  if (!nh_.getParam(robot_group_param, robot_group))
  {
    robot_group = "robot_group";
    ROS_INFO("Could not get param '%s'. Taking default value as '%s'.", robot_group_param.c_str(), robot_group.c_str());
  }

  // const std::string param_markerArrayTopic = "publish_topic_markersArray";
  // std::string markerArrayTopic = "occupied_cells_vis_array";
  // if (!nh_.getParam(param_markerArrayTopic, markerArrayTopic))
  // {
  //   ROS_INFO("Could not get param '%s'. Setting marker size to default: '%s'.", param_markerArrayTopic.c_str(), markerArrayTopic.c_str());
  //   nh_.setParam(param_markerArrayTopic, markerArrayTopic);
  // }

  // Load actions table
  actions_model_->setColumnCount(1);
  actions_model_->setHorizontalHeaderLabels(QStringList() << "Actions");
  ui_->actions_table->setModel(actions_model_);
  ui_->actions_table->setColumnWidth(0, ui_->actions_table->width());
  connect(ui_->actions_table->selectionModel(),
          SIGNAL(selectionChanged(const QItemSelection &, const QItemSelection &)),
          this,
          SLOT(actionTableSelectionChanged(const QItemSelection &, const QItemSelection &))
          );

  // Load octomaps table
  octomaps_model_->setColumnCount(1);
  octomaps_model_->setHorizontalHeaderLabels(QStringList() << "Octomaps:");
  ui_->octomaps_table->setModel(octomaps_model_);
  ui_->octomaps_table->setColumnWidth(0, ui_->octomaps_table->width());
  connect(ui_->octomaps_table->selectionModel(),
          SIGNAL(selectionChanged(const QItemSelection &, const QItemSelection &)),
          this,
          SLOT(octomapsTableSelectionChanged(const QItemSelection &, const QItemSelection &))
          );

  

  // Octomap server
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

Display::~Display(){
  delete ui_;
  delete actions_model_;
  delete octomaps_model_;
}


// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void Display::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  // config.mapSetValue( "Topic", output_topic_ );
}

// Load all configuration data for this panel from the given Config object.
void Display::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  // QString topic;
  // if( config.mapGetString( "Topic", &topic ))
  // {
  //   output_topic_editor_->setText( topic );
  //   updateTopic();
  // }
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




}  // namespace
