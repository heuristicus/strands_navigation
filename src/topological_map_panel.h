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

#ifndef TOPMAP_PANEL_H
#define TOPMAP_PANEL_H

#include "rviz/panel.h"
#include "node_manager.h"
#include "ros/ros.h"

#include "rviz/properties/property_tree_widget.h"

class QComboBox;
class QModelIndex;
class QPushButton;

namespace rviz_topmap {
/**
 * @brief Panel for choosing the view controller and saving and restoring
 * viewpoints.
RR */
class TopologicalMapPanel: public rviz::Panel
{
Q_OBJECT
public:
  TopologicalMapPanel(QWidget* parent = 0);
  virtual ~TopologicalMapPanel() {}

  /** @brief Overridden from TopologicalMapPanel.  Just calls setMan() with vis_manager_->getNodeManager(). */
  virtual void onInitialize();

  /** @brief Set the NodeManager which this panel should display and edit.
   *
   * If this TopologicalMapPanel is to be used with a NodeManager other than
   * the one in the NodeManager sent in through
   * TopologicalMapPanel::initialize(), either TopologicalMapPanelel::initialize() must not be
   * called or setNodeManager() must be called after
   * TopologicalMapPanel::initialize(). */
  void setNodeManager(NodeManager* node_man);

  /** @brief Returns the current NodeManager. */
  NodeManager* getNodeManager() const { return node_man_; }

  /** @brief Load configuration data, specifically the PropertyTreeWidget view settings. */
  virtual void load(const rviz::Config& config);

  /** @brief Save configuration data, specifically the PropertyTreeWidget view settings. */
  virtual void save(rviz::Config config) const;

private Q_SLOTS:
  void onDeleteClicked();
  void renameSelected();
  void addNew();
  void onCurrentChanged();
private:
  NodeManager* node_man_;
  rviz::PropertyTreeWidget* properties_view_;
};

} // namespace rviz_topmap

#endif // TOPMAP_PANEL_H
