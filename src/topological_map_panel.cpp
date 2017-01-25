#include "topological_map_panel.h"

#include <QLabel>
#include <QListWidget>
#include <QComboBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QInputDialog>

namespace topological_rviz_tools
{
TopologicalMapPanel::TopologicalMapPanel(QWidget* parent)
  : rviz::Panel(parent)
  , topmap_man_(NULL)
{
  properties_view_ = new rviz::PropertyTreeWidget();

  ros::NodeHandle nh;
  delNodeSrv_ = nh.serviceClient<topological_rviz_tools::DeleteNode>("/topmap_interface/delete_node", true);

  QPushButton* remove_button = new QPushButton("Remove");

  QHBoxLayout* button_layout = new QHBoxLayout;
  button_layout->addWidget(remove_button);
  button_layout->setContentsMargins(2, 0, 2, 2);

  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->setContentsMargins(0,0,0,0);
  main_layout->addWidget(properties_view_);
  main_layout->addLayout(button_layout);
  setLayout(main_layout);

  connect(remove_button, SIGNAL(clicked()), this, SLOT(onDeleteClicked()));
  // connect(properties_view_, SIGNAL(clicked(const QModelIndex&)), this, SLOT(setCurrentViewFromIndex(const QModelIndex&)));
  // connect(properties_view_, SIGNAL(activated(const QModelIndex&)), this, SLOT(setCurrentViewFromIndex(const QModelIndex&)));
}

void TopologicalMapPanel::onInitialize()
{
  ROS_INFO("Topmapmanel::OnInitialise");
  setTopmapManager(new TopmapManager(NULL));
}

void TopologicalMapPanel::setTopmapManager(TopmapManager* topmap_man)
{
  ROS_INFO("Setting model");
  properties_view_->setModel(topmap_man->getPropertyModel());
  topmap_man_ = topmap_man;

  // connect(camera_type_selector_, SIGNAL(activated(int)), this, SLOT(onTypeSelectorChanged(int)));
  // connect(topmap_man_, SIGNAL(currentChanged()), this, SLOT(onCurrentChanged()));
  // onCurrentChanged();
}

void TopologicalMapPanel::onDeleteClicked()
{
  QList<NodeProperty*> nodes_to_delete = properties_view_->getSelectedObjects<NodeProperty>();
  NodeController* controller = topmap_man_->getController();
  
  for(int i = 0; i < nodes_to_delete.size(); i++)
  {
    topological_rviz_tools::DeleteNode srv;
    srv.request.node_name = nodes_to_delete[i]->getValue().toString().toStdString().c_str();
    
    if (delNodeSrv_.call(srv)) {
      if (srv.response.success) {
	delete controller->takeChild(nodes_to_delete[i]);
	ROS_INFO("Successfully removed node %s", srv.request.node_name.c_str());
      } else {
	ROS_INFO("Failed to remove node %s: %s", srv.request.node_name.c_str(), srv.response.message.c_str());
      }
    } else {
      ROS_INFO("Failed to remove node %s: %s", srv.request.node_name.c_str(), srv.response.message.c_str());
    }
  }
}

void TopologicalMapPanel::renameSelected()
{
  // QList<Node*> views_to_rename = properties_view_->getSelectedObjects<NodeController>();
  // if(views_to_rename.size() == 1)
  // {
  //   NodeProperty* view = views_to_rename[ 0 ];

  //   // TODO: should eventually move to a scheme where the CURRENT view
  //   // is not in the same list as the saved views, at which point this
  //   // check can go away.
  //   if(view == topmap_man_->getCurrent())
  //   {
  //     return;
  //   }

  //   QString old_name = view->getName();
  //   QString new_name = QInputDialog::getText(this, "Rename View", "New Name?", QLineEdit::Normal, old_name);

  //   if(new_name.isEmpty() || new_name == old_name)
  //   {
  //     return;
  //   }

  //   view->setName(new_name);
  // }
}

void TopologicalMapPanel::onCurrentChanged()
{
  //QString formatted_class_id = NodeController::formatClassId(topmap_man_->getCurrent()->getClassId());
  // ROS_INFO("CUrrent changed");
  // properties_view_->setAnimated(false);
  // topmap_man_->getCurrent()->expand();
  // properties_view_->setAnimated(true);
}

void TopologicalMapPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
  properties_view_->save(config);
}

void TopologicalMapPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
  properties_view_->load(config);
}

} // namespace topological_rviz_tools

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(topological_rviz_tools::TopologicalMapPanel, rviz::Panel)
