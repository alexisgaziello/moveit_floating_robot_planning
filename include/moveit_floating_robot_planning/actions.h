#ifndef MOVEIT_MOTION_PLANNING_RVIZ_PLUGIN_ACTIONS_UNEXMIN_
#define MOVEIT_MOTION_PLANNING_RVIZ_PLUGIN_ACTIONS_UNEXMIN_


#include <QStandardItemModel>
#include <string>
#include <visualization_msgs/Marker.h>

// UNEXMIN CLASSES
namespace moveit_floating_robot_planning
{

class ActionType {
  public:
    std::string name;
    int type;
    float r;
    float g;
    float b;
    float a;
    QStandardItemModel * parameters;

    ActionType(const std::string _name, const int _type
        , const float _r, const float _g, const float _b, const float _a
        );
    
    ActionType(const ActionType &); 

    ActionType& operator=(const ActionType & actionType);  

    ~ActionType();

    void addParam(const std::string name, const std::string value);
  };

class Action {
  public:
    visualization_msgs::Marker marker;
    ActionType actionType;
    bool selected;

    Action(const ActionType & _actionType)
    : actionType(ActionType(_actionType))
    , selected(false)
    {}
  };

}

#endif
