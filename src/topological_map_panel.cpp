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

#include "topological_map_panel.h"

#include <QLabel>
#include <QListWidget>
#include <QComboBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QInputDialog>

namespace rviz_topmap
{
TopologicalMapPanel::TopologicalMapPanel(QWidget* parent)
  : rviz::Panel(parent)
  , topmap_man_(NULL)
{
  properties_view_ = new rviz::PropertyTreeWidget();

  QPushButton* add_button = new QPushButton("Add");
  QPushButton* remove_button = new QPushButton("Remove");

  QHBoxLayout* button_layout = new QHBoxLayout;
  button_layout->addWidget(add_button);
  button_layout->addWidget(remove_button);
  button_layout->setContentsMargins(2, 0, 2, 2);

  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->setContentsMargins(0,0,0,0);
  main_layout->addWidget(properties_view_);
  main_layout->addLayout(button_layout);
  setLayout(main_layout);

  connect(add_button, SIGNAL(clicked()), this, SLOT(addNew()));
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
  QList<NodeController*> views_to_delete = properties_view_->getSelectedObjects<NodeController>();

  // for(int i = 0; i < views_to_delete.size(); i++)
  // {
  //   // TODO: should eventually move to a scheme where the CURRENT view
  //   // is not in the same list as the saved views, at which point this
  //   // check can go away.
  //   if(views_to_delete[ i ] != topmap_man_->getCurrent())
  //   {
  //     delete views_to_delete[ i ];
  //   }
  // }
}

void TopologicalMapPanel::addNew()
{
  QList<NodeController*> views_to_rename = properties_view_->getSelectedObjects<NodeController>(); 
}

void TopologicalMapPanel::renameSelected()
{
  QList<NodeController*> views_to_rename = properties_view_->getSelectedObjects<NodeController>();
  if(views_to_rename.size() == 1)
  {
    NodeController* view = views_to_rename[ 0 ];

    // TODO: should eventually move to a scheme where the CURRENT view
    // is not in the same list as the saved views, at which point this
    // check can go away.
    if(view == topmap_man_->getCurrent())
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

} // namespace rviz_topmap

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_topmap::TopologicalMapPanel, rviz::Panel)
