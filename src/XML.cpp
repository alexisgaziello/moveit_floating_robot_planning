
#include <moveit_floating_robot_planning/display.h>

#include <fstream>
#include <sstream>

#include <QFileDialog>

#include <rviz/visualization_manager.h>
#include <rviz/view_manager.h>
#include <rviz/view_controller.h>
#include <rviz/window_manager_interface.h>

#include <topic_tools/shape_shifter.h>
#include <visualization_msgs/InteractiveMarkerInit.h>

#include <tinyxml.h>

namespace moveit_floating_robot_planning
{
// XML Generation Tab

void Display::addActionButtonClicked()
{
  // Get action from selected option
  const int index = ui_->action_type_combo_box->currentIndex();
  addAction(actionTypes[index]);
}

void Display::addAction(const ActionType & actionType)
{
  
  // const robot_state::RobotStateConstPtr & robotState = planning_display_->getQueryGoalState();
  // const Eigen::Affine3d & end_effector_state = robotState->getGlobalLinkTransform("body");
  // const Eigen::Affine3d & end_effector_state = planning_display_->getQueryGoalState()->getGlobalLinkTransform("body");
  // Eigen::Quaterniond end_effector_quaternion(end_effector_state.linear());
  // end_effector_quaternion.normalize();

  // TODO CONSIDER BINDING
  // moveit::core::RobotStatePtr robotState = move_group.getCurrentState();
  // if (robotState == nullptr){
  //   ROS_ERROR("Could not get robot position from moveit. Cannot add actions.");
  // }
  // const Eigen::Affine3d & end_effector_state = robotState->getGlobalLinkTransform("body");
  // Eigen::Quaterniond end_effector_quaternion(end_effector_state.linear());
  // end_effector_quaternion.normalize();
  // geometry_msgs::Pose pose = move_group.getCurrentPose().pose;

  // geometry_msgs::Pose pose;
  // pose.position.x = end_effector_state(0,3);
  // pose.position.y = end_effector_state(1,3);
  // pose.position.z = end_effector_state(2,3);
  // pose.orientation.x = end_effector_quaternion.x();
  // pose.orientation.y = end_effector_quaternion.y();
  // pose.orientation.z = end_effector_quaternion.z();
  // pose.orientation.w = end_effector_quaternion.w();

  // const geometry_msgs::PoseStamped current_state = move_group.getPoseTarget(END_EFFECTOR_LINK);
  // const geometry_msgs::Pose pose(current_state.pose);
    
  // goalPositionSubscriber = nh_.subscribe<topic_tools::ShapeShifter>("/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update_full"
  // ,1 , &Display::goalPositionCallback, this);


  addAction(actionType, getRobotPosition());

}

geometry_msgs::Pose Display::getRobotPosition(){

  geometry_msgs::Pose pose; 

  const topic_tools::ShapeShifter::ConstPtr & msg = ros::topic::waitForMessage<topic_tools::ShapeShifter>(
      "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update_full"
      , nh_);

  if (msg->getDataType() == ros::message_traits::datatype<visualization_msgs::InteractiveMarkerInit>()) {
    const visualization_msgs::InteractiveMarkerInit::ConstPtr & markersInit = msg->instantiate<visualization_msgs::InteractiveMarkerInit>();
    
    for(size_t i = 0; i < markersInit->markers.size(); ++i) {
              
      if (markersInit->markers.at(i).name.compare("JJ:goal_body") == 0){ // JJ:start_body for starting position
        pose.position.x = markersInit->markers.at(i).pose.position.x;
        pose.position.y = markersInit->markers.at(i).pose.position.y;
        pose.position.z = markersInit->markers.at(i).pose.position.z;
        pose.orientation.x = markersInit->markers.at(i).pose.orientation.x;
        pose.orientation.y = markersInit->markers.at(i).pose.orientation.y;
        pose.orientation.z = markersInit->markers.at(i).pose.orientation.z;
        pose.orientation.w = markersInit->markers.at(i).pose.orientation.w;
      }
    }
  } else {
    ROS_ERROR("Unknown message type (%s) for goal position topic. Should be visualization_msgs/InteractiveMarkerInit.", msg->getDataType().c_str());
  }  

  return pose;
}

// void Display::goalPositionCallback(const topic_tools::ShapeShifter::ConstPtr& msg)
// {
//   if (msg->getDataType() == ros::message_traits::datatype<visualization_msgs::InteractiveMarkerInit>()) {
//     visualization_msgs::InteractiveMarkerInit * markersInit = msg->instantiate<visualization_msgs::InteractiveMarkerInit>().get();
    
//     for(size_t i = 0; i < markersInit->markers.size(); ++i) {

//     // for(auto && marker : markersInit->markers){
//       // if (marker.ns.compare("JJ:start_body")){
        
//       // }
              
//       ROS_ERROR("new marker");
//       //if (markersInit->markers.at(i).name.compare("JJ:goal_body")){
//         ROS_ERROR("NEW POSE");
//         // lastGoalPose = markersInit->markers.at(i).pose;
//     // }
//     }
//   }
//   else
//   {
//     ROS_ERROR("Unknown message type (%s) for goal position topic. Should be visualization_msgs/InteractiveMarkerInit.", msg->getDataType().c_str());
//   }
// }


void Display::addAction(const ActionType & actionType, const double & x, const double & y, const double & z)
{
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  addAction(actionType, pose);
}

//Creates all types of "Markers/actions"
void Display::addAction(const ActionType & actionType, const geometry_msgs::Pose & pose)
{
  Action action(actionType);

  action.marker.pose = pose;
  
  action.marker.header.stamp = ros::Time::now();
  action.marker.header.frame_id = worldFrame_;
  action.marker.action = visualization_msgs::Marker::ADD;

  action.marker.scale.x = markerSize_;
  action.marker.scale.y = markerSize_;
  action.marker.scale.z = markerSize_;

  updateColorOfAction(action);
  action.marker.type = action.actionType.type;

  action.marker.lifetime = ros::Duration();

  const int qty = freeIDs.size();
  if (qty == 0){
    action.marker.id = actions_.size();
  } else {
    action.marker.id = freeIDs[qty-1];
    freeIDs.pop_back();
  }

  // We check if any marker is selected
  const QModelIndexList selectedActions = ui_->actions_table->selectionModel()->selectedRows();
  // if only 1 is selected we save it because we will add this marker just before.
  if (selectedActions.count() == 1){
    const int indexToInsert = selectedActions[0].row();
    actions_.insert(actions_.begin()+indexToInsert, action);
    insertToTable(action.actionType.name, indexToInsert, actions_model_);
  } else { // None or more than 1 selected. We append it
    actions_.emplace_back(action);
    appendToTable(action.actionType.name, actions_model_);
  }

  publishAction(action);
}

void Display::publishAction(const Action & action){
  actionsPublisher_adv_.publish(action.marker);
  if (centeredActionID_ == action.marker.id){
    centerUIToAction(action);
  }
}

void Display::removeActionButtonClicked()
{
  // Retrieve ID
  // We get how many markers are selected
  const int itemsToRemove = ui_->actions_table->selectionModel()->selectedRows().count();
  // if only 1 is selected we save it
  for (int i=0; i<itemsToRemove; i++){
    const QModelIndexList selectedActions = ui_->actions_table->selectionModel()->selectedRows();
    const int index = selectedActions[0].row();

    // Publish empty marker
    actions_[index].marker.action = visualization_msgs::Marker::DELETE;
    actionsPublisher_adv_.publish(actions_[index].marker);

    // Save ID and eliminate from vector and table
    freeIDs.emplace_back(actions_[index].marker.id);

    actions_.erase(actions_.begin()+index);
    actions_model_->removeRow(index);

  }
}

// Function to manage the seletion of the items in the table
void Display::actionTableSelectionChanged(const QItemSelection & selected, const QItemSelection & deselected)
{
  // // Center to action location
  centerUIToSelectedAction();

  // //Change color to white to selected markers
  for (int i = 0; i < selected.indexes().size(); i++){
    const int index = selected.indexes()[i].row();
    actions_[index].selected=true;
    updateColorOfAction(actions_[index]);
    actionsPublisher_adv_.publish(actions_[index].marker);
  }

  //Change color to default to deselected markers
  for (int i = 0; i < deselected.indexes().size(); i++){
    const int index = deselected.indexes()[i].row();
    actions_[index].selected = false;
    updateColorOfAction(actions_[index]);
    //Republish with default color
    actionsPublisher_adv_.publish(actions_[index].marker);
  }
}


void Display::appendToTable(const std::string & str, QStandardItemModel * model)
{
  appendToTable(QString::fromStdString(str), model);
}

void Display::appendToTable(const QString & str, QStandardItemModel * model)
{
  QList<QStandardItem *> standardItemsList;
  standardItemsList.append(new QStandardItem(str));
  model->appendRow(standardItemsList);
}

void Display::insertToTable(const std::string & str, const int & index, QStandardItemModel * model)
{
  insertToTable(QString::fromStdString(str), index, model);
}

void Display::insertToTable(const QString & str, const int & index, QStandardItemModel * model)
{
  QList<QStandardItem *> standardItemsList;
  standardItemsList.append(new QStandardItem(str));
  model->insertRow(index, standardItemsList);
}


// Combo box, when index changes, update parameters
void Display::actionIndexChanged(int index)
{
  ui_->actions_parameters_table->setModel(actionTypes[index].parameters);
  ui_->actions_parameters_table->setColumnWidth(0, ui_->actions_parameters_table->width()*0.45);
}


void Display::importXMLButtonClicked()
{
  // Variable para guardar el path del archivo seleccionado
  QString filePath;

  // Directorio por defecto para la busqueda del archivo
  std::string directorioBusqueda = "/";

  // Obtiene el path del archivo a partir de un buscador
  filePath = QFileDialog::getOpenFileName(this,
                                            "Select mission file",
                                            directorioBusqueda.c_str(),
                                            "XML Files (*.xml)");
  if(filePath.isNull()) {
    // If user cancels
    return;
  }

  if (!loadXMLMissionFile(filePath)){
    ROS_ERROR("Error trying to load %s", filePath.toStdString().c_str());
  }
}


bool Display::loadXMLMissionFile(const QString & filePath)
{
  int i = ui_->action_type_combo_box->currentIndex();
  ROS_INFO("%s", actionTypes[i].name.c_str());
  
  const std::string str_mission = "mission";
  const std::string str_task = "task";
  TiXmlDocument doc(filePath.toStdString());
  if(doc.LoadFile())
  {
      TiXmlElement *pRoot;
      pRoot = doc.FirstChildElement(str_mission);
      if(pRoot)
      {
          TiXmlElement *pAction;
          pAction = pRoot->FirstChildElement(str_task);
          int i = 0; // for sorting the entries
          while(pAction)
          {
              bool foundError = false;
              const char * description = pAction->Attribute("description");
              int actionTypeIndex = 0;
              if (description) {
                const std::string str_description(description);
                actionTypeIndex = getTypeOfAction(str_description);
              } else {
                ROS_ERROR("Could not get description of action from action number %d of XML file.\nSetting to default: '%s'.\nBe careful: parameters will be lost.", i, ActionType(actionTypes[0]).name.c_str());
              }
              ActionType actionType(actionTypes[actionTypeIndex]);


              TiXmlElement *pPoint, *pX, *pY, *pZ;
              pPoint = pAction->FirstChildElement("point");
              if (!pPoint){
                ROS_ERROR("Could not find point from action number %d of XML file.", i);
                foundError = true;
              }
              double x, y, z;

              pX = pPoint->FirstChildElement("x");
              if (pX){
                const char *x_char = pX->GetText();
                if (x_char) {
                  x = atof(x_char);
                } else {
                  ROS_ERROR("Could not parse x position from action number %d of XML file.", i);
                  foundError = true;
                }
              } else {
                ROS_ERROR("Could not find x position from action number %d of XML file.", i);
                foundError = true;
              }

              pY = pPoint->FirstChildElement("y");
              if (pY){
                const char *y_char = pY->GetText();
                if (y_char) {
                  y = atof(y_char);
                } else {
                  ROS_ERROR("Could not parse y position from action number %d of XML file.", i);
                  foundError = true;
                }
              } else {
                ROS_ERROR("Could not find y position from action number %d of XML file.", i);
                foundError = true;
              }

              pZ = pPoint->FirstChildElement("z");
              if (pZ){
                const char *z_char = pZ->GetText();
                if (z_char) {
                  z = atof(z_char);
                } else {
                  ROS_ERROR("Could not parse z position from action number %d of XML file.", i);
                  foundError = true;
                }
              } else {
                ROS_ERROR("Could not find z position from action number %d of XML file.", i);
                foundError = true;
              }

              // Parameters
              for (int i=0 ; i<actionType.parameters->rowCount() ; i++){
                const std::string name = actionType.parameters->data(actionType.parameters->index(i,0)).toString().toStdString();
                TiXmlElement *pParam;
                pParam = pAction->FirstChildElement(name);

                if (pParam) {
                  const char *param_char = pParam->GetText();
                  if (param_char) {
                    actionType.parameters->setItem(i,1,new QStandardItem(QString::fromUtf8(param_char)));
                  } else {
                    actionType.parameters->setItem(i,1,new QStandardItem(QString()));
                  }
                } else {
                  ROS_ERROR("Could not find any parameter named '%s' for action number %d. Setting to default.", name.c_str(), i);
                }
              }

              //TODO rotation?

              if (!foundError){
                addAction(actionType, x, y, z);
              }

              pAction = pAction->NextSiblingElement(str_task);
              i++;
          }
          ROS_INFO("Added %d actions from XML File: %s", i, filePath.toUtf8().constData());
      }

  } else {
      ROS_ERROR("Could not load XML File: %s", filePath.toUtf8().constData());
      return false;
  }
  return true;
}


void Display::exportXMLButtonClicked()
{
  std::stringstream stream;
  stream << "<?xml version=\"1.0\"?>\n";
  stream << "<mission description=\"Waypoint navigation\">\n";
  for (int i=0; i<actions_.size();i++){
    stream << "	 <task description=\"" << actions_[i].actionType.name<< "\">\n";
    stream << "    <point>\n";
    stream << "      <x>" << std::to_string(actions_[i].marker.pose.position.x) << "</x>\n";
    stream << "      <y>" << std::to_string(actions_[i].marker.pose.position.y) << "</y>\n";
    stream << "      <z>" << std::to_string(actions_[i].marker.pose.position.z) << "</z>\n";
    stream << "    </point>\n";
    for (int j=0; j<actions_[i].actionType.parameters->rowCount(); j++){
      QAbstractItemModel * model =  actions_[i].actionType.parameters;
      QString name = model->data(model->index(j, 0)).toString();
      QString value = model->data(model->index(j, 1)).toString();
      stream << "      <" << name.toStdString() << ">" << value.toStdString() << "</" << name.toStdString() << ">\n";
    }
    stream << "  </task>\n";
  }

  // Add any other text from append.xml
  std::ifstream appendFile;
  appendFile.open ( workspacePath_ + "/resources/append.xml");
  if (appendFile.is_open())
  { 
    std::string firstLine; 
    getline(appendFile, firstLine);

    if (firstLine.substr(0,5).compare("<?xml")){
      // They are different
      // Forgot to add the xml version tag
      stream << firstLine << '\n';
    } // else
      // They are equal
      // We dont want this line. Just do nothing
    
    // Add the rest of the text
    stream << appendFile.rdbuf();;

    appendFile.close();
  }

  if(!missionAppendPath.empty()){
    appendFile.open(missionAppendPath);
    if (appendFile.is_open())
    { 
      std::string firstLine; 
      getline(appendFile, firstLine);

      if (firstLine.substr(0,5).compare("<?xml")){
        // They are different
        // Forgot to add the xml version tag
        stream << firstLine << '\n';
      } // else
        // They are equal
        // We dont want this line. Just do nothing
      
      // Add the rest of the text
      stream << appendFile.rdbuf();;

      appendFile.close();
    }
  }

  stream << "</mission>";

  std::ofstream file;
  file.open (missionSavePath);
  file << stream.str();
  file.close();

}

// Function to initialize the table

void Display::updateColorOfAction(Action & action)
{
  if (action.selected){
    //White
    action.marker.color.r = 1;
    action.marker.color.g = 1;
    action.marker.color.b = 1;
    action.marker.color.a = 1;
    return;
  } // else 

  // We add a color depending on type
  action.marker.color.r = action.actionType.r;
  action.marker.color.g = action.actionType.g;
  action.marker.color.b = action.actionType.b;
  action.marker.color.a = action.actionType.a;
}

int Display::getTypeOfAction(const std::string & typeDescription){
  for(int i=0; i<actionTypes.size(); i++){
    if (typeDescription == actionTypes[i].name){
      return i;
    }
  }
  ROS_ERROR("Could not find an action with that description.\nSetting to default: '%s'.\nBe careful: parameters will be lost.", ActionType(actionTypes[0]).name.c_str());
  return 0;
}

// Center interface
void Display::centerUIToAction(const Action & action)
{
  vis_manager_->getViewManager()->getCurrent()->lookAt(
      action.marker.pose.position.x
    , action.marker.pose.position.y
    , action.marker.pose.position.z
  );
}

void Display::centerToRobotClicked()
{
  geometry_msgs::Pose pose = getRobotPosition();
  vis_manager_->getViewManager()->getCurrent()->lookAt(
      pose.position.x
    , pose.position.y
    , pose.position.z
  );
}

void Display::centerUIToSelectedAction()
{
  if (ui_->center_to_action->isChecked()){
    QModelIndexList selectedActions = ui_->actions_table->selectionModel()->selectedRows();
    if (selectedActions.count() == 1){
      centeredActionID_ = selectedActions[0].row();
      centerUIToAction(actions_[centeredActionID_]);
    } else {
      centeredActionID_ = -1;
    }
  }
}

} // Namespace
