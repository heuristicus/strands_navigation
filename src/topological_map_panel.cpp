/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <QLabel>
#include <QListWidget>
#include <QComboBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QInputDialog>

#include "rviz/properties/property_tree_widget.h"
#include "rviz/view_controller.h"
#include "rviz/view_manager.h"
#include "rviz/visualization_manager.h"

#include "topological_map_panel.h"

namespace rviz_topmap
{

TopologicalMapPanel::TopologicalMapPanel(QWidget* parent)
  : Panel(parent)
  , view_man_(NULL)
{
  ros::NodeHandle nh;
  top_map_sub = nh.subscribe("/topological_map", 5, &TopologicalMapPanel::topmapCallback, this);

  properties_view_ = new rviz::PropertyTreeWidget();

  QPushButton* add_button = new QPushButton("Add");
  QPushButton* remove_button = new QPushButton("Remove");
  QPushButton* rename_button = new QPushButton("Rename");

  QHBoxLayout* button_layout = new QHBoxLayout;
  button_layout->addWidget(add_button);
  button_layout->addWidget(remove_button);
  button_layout->addWidget(rename_button);
  button_layout->setContentsMargins(2, 0, 2, 2);

  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->setContentsMargins(0,0,0,0);
  main_layout->addWidget(properties_view_);
  main_layout->addLayout(button_layout);
  setLayout(main_layout);

  connect(remove_button, SIGNAL(clicked()), this, SLOT(onDeleteClicked()));
  connect(rename_button, SIGNAL(clicked()), this, SLOT(renameSelected()));
  connect(properties_view_, SIGNAL(clicked(const QModelIndex&)), this, SLOT(setCurrentNodeFromIndex(const QModelIndex&)));
  connect(properties_view_, SIGNAL(activated(const QModelIndex&)), this, SLOT(setCurrentNodeFromIndex(const QModelIndex&)));
}

void TopologicalMapPanel::onInitialize()
{
  ROS_INFO("Initialised map panel");
}

void TopologicalMapPanel::topmapCallback(const strands_navigation_msgs::TopologicalMap::ConstPtr& msg){
  ROS_INFO("Got topological map");
}

// void TopologicalMapPanel::setViewManager(rviz::ViewManager* view_man)
// {
//   if(view_man_)
//   {
//     disconnect(save_button_, SIGNAL(clicked()), view_man_, SLOT(copyCurrentToList()));
//     disconnect(map_selector_, SIGNAL(activated(int)), this, SLOT(onTypeSelectorChanged(int)));
//     disconnect(view_man_, SIGNAL(currentChanged()), this, SLOT(onCurrentChanged()));
//   }
//   view_man_ = view_man;
//   map_selector_->clear();
//   if(view_man_)
//   {
//     properties_view_->setModel(view_man_->getPropertyModel());

//     QStringList ids = view_man_->getFactory()->getDeclaredClassIds();
//     for(int i = 0; i < ids.size(); i++)
//     {
//       const QString& id = ids[ i ];
//       map_selector_->addItem(rviz::ViewController::formatClassId(id), id); // send the regular-formatted id as userData.
//     }

//     connect(save_button_, SIGNAL(clicked()), view_man_, SLOT(copyCurrentToList()));
//     connect(map_selector_, SIGNAL(activated(int)), this, SLOT(onTypeSelectorChanged(int)));
//     connect(view_man_, SIGNAL(currentChanged()), this, SLOT(onCurrentChanged()));
//   }
//   else
//   {
//     properties_view_->setModel(NULL);
//   }
//   onCurrentChanged();
// }

void TopologicalMapPanel::setCurrentNodeFromIndex(const QModelIndex& index)
{
  rviz::Property* prop = view_man_->getPropertyModel()->getProp(index);
}

void TopologicalMapPanel::onDeleteClicked()
{
  QList<rviz::ViewController*> nodes_to_delete = properties_view_->getSelectedObjects<rviz::ViewController>();

  for(int i = 0; i < nodes_to_delete.size(); i++)
  {
    // TODO: should eventually move to a scheme where the CURRENT view
    // is not in the same list as the saved views, at which point this
    // check can go away.
    if(nodes_to_delete[ i ] != view_man_->getCurrent())
    {
      delete nodes_to_delete[ i ];
    }
  }
}

void TopologicalMapPanel::renameSelected()
{
  QList<rviz::ViewController*> nodes_to_rename = properties_view_->getSelectedObjects<rviz::ViewController>();
  if(nodes_to_rename.size() == 1)
  {
    rviz::ViewController* view = nodes_to_rename[ 0 ];

    // TODO: should eventually move to a scheme where the CURRENT view
    // is not in the same list as the saved views, at which point this
    // check can go away.
    if(view == view_man_->getCurrent())
    {
      return;
    }

    QString old_name = view->getName();
    QString new_name = QInputDialog::getText(this, "Rename View", "New Name?", QLineEdit::Normal, old_name);

    if(new_name.isEmpty() || new_name == old_name)
    {
      return;
    }

    view->setName(new_name);
  }
}

void TopologicalMapPanel::onCurrentChanged()
{
  QString formatted_class_id = rviz::ViewController::formatClassId(view_man_->getCurrent()->getClassId());
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

} // namespace rviz_topmap

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_topmap::TopologicalMapPanel, rviz::Panel)
