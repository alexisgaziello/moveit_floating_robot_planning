#include <moveit_floating_robot_planning/display.h>

#include <fstream>
#include <sstream>

#include <QFileDialog>
#include <QDir>


namespace moveit_floating_robot_planning
{

// Octomap Tab

void Display::loadOctomapButtonClicked()
{
  // Variable para guardar el path del archivo seleccionado
  QString filePath;

  // Directorio por defecto para la busqueda del archivo
  std::string directorioBusqueda = "/";

  // Obtiene el path del archivo a partir de un buscador
  filePath = QFileDialog::getOpenFileName(this,
                                            "Select Octomap file",
                                            directorioBusqueda.c_str(),
                                            "Octomap Files (*.bt)");
  if(filePath.isNull()) {
    // If user cancels
    return;
  }

  if (!loadOctomap(filePath)){
    ROS_ERROR("Error trying to load %s", filePath.toStdString().c_str());
  }
}

bool Display::loadOctomap(const QString & filePath, const bool & publish)
{
  // If someday it was compiled in windows.
  #ifdef _WIN32
  const QChar sep = '\\';
  #else
  const QChar sep = '/';
  #endif

  // Get file name from path
  const int pos = filePath.lastIndexOf(sep);
  QString fileName;
  if (pos != -1){
    fileName = filePath.right(filePath.length()-pos-1);
  } else {
    fileName = filePath;
  }

  // Take off extension if it has it
  if (fileName.endsWith(".bt")){
    fileName = fileName.left(fileName.length()-4);
  }

  appendToTable(fileName, octomaps_model_);
  octomaps_filePaths.push_back(filePath.toStdString());

  if (publish){
    publishSelectedOctomap(filePath.toStdString());
  }
}

void Display::publishSelectedOctomap(const std::string & filePath)
{
  if (!octomapServer_->openFile(filePath)){
    ROS_ERROR("Could not open file %s", filePath.c_str());
  }
}

void Display::removeOctomapButtonClicked()
{
  // Retrieve ID
  // We get how many markers are selected
  const int itemsToRemove = ui_->octomaps_table->selectionModel()->selectedRows().count();
  // if only 1 is selected we save it
  for (int i=0; i<itemsToRemove; i++){
    const QModelIndexList selectedOctomaps = ui_->octomaps_table->selectionModel()->selectedRows();
    const int index = selectedOctomaps[0].row();
    // Publish empty marker
    //TODO UNPUBLISH IF PUBLISHED
    // Save ID and eliminate from vector and table
    octomaps_filePaths.erase(octomaps_filePaths.begin() + index);
    octomaps_model_->removeRow(index);
  }
}

void Display::loadOctomapsTable()
{
  delete octomaps_model_;
  octomaps_model_ = new QStandardItemModel(this);
  octomaps_model_->setColumnCount(1);
  octomaps_model_->setHorizontalHeaderLabels(QStringList() << "Octomaps:");
  ui_->octomaps_table->setModel(octomaps_model_);
  ui_->octomaps_table->setColumnWidth(0, ui_->octomaps_table->width());
  connect(ui_->octomaps_table->selectionModel(),
          SIGNAL(selectionChanged(const QItemSelection &, const QItemSelection &)),
          this,
          SLOT(octomapsTableSelectionChanged(const QItemSelection &, const QItemSelection &))
          );
}

void Display::octomapsTableSelectionChanged(const QItemSelection & selected, const QItemSelection & deselected)
{
  const QModelIndexList selectedOctomaps = ui_->octomaps_table->selectionModel()->selectedRows();
  if (selectedOctomaps.count() == 1){
    // Publish
    const int index = selectedOctomaps[0].row();
    publishSelectedOctomap(octomaps_filePaths[index]);

  }
  // Is it really necessary?? You could unpublish it by error.
  // There is no point in not having an octomap.
  // You could always unselect the poincloud/markersArray in RViz
  // else if (selectedOctomaps.count() == 0) {
  //   // Unpublish
  //
  // }

}

} // Namespace
