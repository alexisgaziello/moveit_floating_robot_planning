#include <moveit_floating_robot_planning/actions.h>
#include <algorithm>

namespace moveit_floating_robot_planning
{

ActionType::ActionType(const std::string _name, const int _type
      , const float _r, const float _g, const float _b, const float _a
      )
      : name(_name)
      , type(_type)
      , r(_r)
      , g(_g)
      , b(_b)
      , a(_a)
      , parameters(new QStandardItemModel())
{
      parameters->setColumnCount(2);
      parameters->setHorizontalHeaderLabels(QStringList() << "Name" << "Value");
}

ActionType::ActionType(const ActionType & actionType)
     : name(actionType.name)
      , type(actionType.type)
      , r(actionType.r)
      , g(actionType.g)
      , b(actionType.b)
      , a(actionType.a)
      , parameters(new QStandardItemModel())

{
      parameters->setColumnCount(2);
      parameters->setHorizontalHeaderLabels(QStringList() << "Name" << "Value");
      // Copy stuff inside pointer
      for (int i=0 ; i<actionType.parameters->rowCount() ; i++){
            QList<QStandardItem *> standardItemsList;
            for (int j=0; j<actionType.parameters->columnCount() ; j++){
                  standardItemsList.append(actionType.parameters->item(i,j)->clone());
            }
            parameters->appendRow(standardItemsList);
      }
} 

ActionType& ActionType::operator=(const ActionType & actionType)
{
      if (this != &actionType)
      {
            name = actionType.name;
            type = actionType.type;
            r = actionType.r;
            g = actionType.g;
            b = actionType.b;
            a = actionType.a;

            delete parameters;
            parameters = new QStandardItemModel();
            parameters->setColumnCount(2);
            parameters->setHorizontalHeaderLabels(QStringList() << "Name" << "Value");
            // Copy stuff inside pointer
            for (int i=0 ; i<actionType.parameters->rowCount() ; i++){
                  QList<QStandardItem *> standardItemsList;
                  for (int j=0; j<actionType.parameters->columnCount() ; j++){
                        standardItemsList.append(actionType.parameters->item(i,j)->clone());
                  }
                  parameters->appendRow(standardItemsList);
            }
      }
      return *this;
}

// ActionType(ActionType&& actionType) noexcept ††
//       : ActionType() // initialize via default constructor, C++11 only
// {
//       std::swap(*this, other);
// }

ActionType::~ActionType(){
      delete parameters;
}      

void ActionType::addParam(const std::string name, const std::string value){      
      QList<QStandardItem *> standardItemsList;
      standardItemsList.append(new QStandardItem(QString::fromStdString(name)));
      standardItemsList.append(new QStandardItem(QString::fromStdString(value)));
      parameters->appendRow(standardItemsList);
}

}