//
// Created by matej on 23/09/23.
//

#ifndef PLANT_GUARD_BAGTUBE_PANEL_HPP
#define PLANT_GUARD_BAGTUBE_PANEL_HPP

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "bagtube_msgs/srv/get_bag_list.hpp"
#include "bagtube_msgs/srv/control_playback.hpp"
#include "bagtube_msgs/action/play_bag.hpp"
#include "bagtube_msgs/action/record_bag.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <QSlider>
#include <QTableWidget>
#include <QLineEdit>
#include <QLabel>
#include <QPushButton>

namespace bagtube_rviz_plugins {

class BagtubePanel : public rviz_common::Panel {
  Q_OBJECT
public:
  BagtubePanel( QWidget* parent = 0 );

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  void onInitialize() override;
  virtual void load( const rviz_common::Config& config ) override;
  virtual void save( rviz_common::Config config ) const override;
  virtual void setVisible(bool visible) override;

protected Q_SLOTS:

  void onBagSelected(int row, int column);
  void onBagsFilterChanged();
  void onPlaybackSliderChanged(int value);
  void onPlayPauseButtonClicked();
  void onLivestreamButtonClicked(bool checked);
  void onRecordNameChanged(const QString &text);
  void onRecordButtonClicked(bool checked);
  void reloadBags();
protected:

  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr sync_response_executor_;
  rclcpp::CallbackGroup::SharedPtr sync_response_cb_group_;

  rclcpp::Client<bagtube_msgs::srv::GetBagList>::SharedPtr list_bags_cl_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr livestream_cl_;
  rclcpp::Client<bagtube_msgs::srv::ControlPlayback>::SharedPtr control_playback_cl_;
  rclcpp_action::Client<bagtube_msgs::action::PlayBag>::SharedPtr play_bag_ac_;
  rclcpp_action::Client<bagtube_msgs::action::RecordBag>::SharedPtr record_bag_ac_;

  rclcpp_action::ClientGoalHandle<bagtube_msgs::action::PlayBag>::SharedPtr play_bag_goal_handle_;
  rclcpp_action::ClientGoalHandle<bagtube_msgs::action::RecordBag>::SharedPtr record_bag_goal_handle_;

  std::vector<bagtube_msgs::msg::BagInfo> bags_;
  bagtube_msgs::msg::BagInfo selected_bag_;
  bagtube_msgs::msg::BagInfo bag_being_played_;

  QPushButton* livestream_button_;
  QLineEdit* record_name_edit_;
  QPushButton* record_button_;

  QSlider* playback_slider_;
  QLabel* playback_time_label_;
  QLabel* playback_duration_label_;
  QPushButton *play_pause_button_;

  QTableWidget* bags_table_;
  QLineEdit* bags_filter_edit_;

  std::recursive_mutex action_mutex_;

  QString durationToString(double duration);
  void onLivestreamEnabledChanged();

  template<class MsgT>
  typename MsgT::Response::SharedPtr callService(typename rclcpp::Client<MsgT>::SharedPtr client, const typename MsgT::Request::SharedPtr& request);

};

} // bagtube_rviz_plugins

#endif //PLANT_GUARD_BAGTUBE_PANEL_HPP
