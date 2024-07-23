//
// Created by matej on 23/09/23.
//

#include "bagtube_rviz_plugins/bagtube_panel.hpp"
#include "rviz_common/display_context.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMouseEvent>

#include <QProxyStyle>

using namespace std::chrono_literals;

const auto LOGGER = rclcpp::get_logger("BagtubePanel");

class SliderAbsoluteSetClickStyle : public QProxyStyle {
public:
  using QProxyStyle::QProxyStyle;

  int styleHint(QStyle::StyleHint hint, const QStyleOption *option = 0, const QWidget *widget = 0,
                QStyleHintReturn *returnData = 0) const {
    if (hint == QStyle::SH_Slider_AbsoluteSetButtons)
      return (Qt::LeftButton | Qt::MiddleButton | Qt::RightButton);
    return QProxyStyle::styleHint(hint, option, widget, returnData);
  }
};

namespace bagtube_rviz_plugins {

void BagtubeTableWidget::mouseReleaseEvent(QMouseEvent *event)
{
  if (event->button() == Qt::MouseButton::LeftButton) {
    auto item = itemAt(event->pos());
    if (item == nullptr) {
      clearSelection();
      emit cellClicked(-1, -1);
    }
  }
  QTableWidget::mouseReleaseEvent(event);
}
void BagtubeTableWidget::keyPressEvent(QKeyEvent *e)
{
  if(e->key()==Qt::Key_Delete)
  {
    emit deletePressed(this->currentRow(),this->currentColumn());
  }
  else { QTableWidget::keyPressEvent(e); }
}

BagtubePanel::BagtubePanel(QWidget *parent)
    : rviz_common::Panel(parent) {
  auto layout = new QVBoxLayout;

  auto livestream_layout = new QHBoxLayout;
  livestream_button_ = new QPushButton("Live");
  livestream_button_->setCheckable(true);
  connect(livestream_button_, SIGNAL(clicked(bool)), this, SLOT(onLivestreamButtonClicked(bool)));
  livestream_layout->addWidget(livestream_button_);
  livestream_layout->addStretch();

  record_name_edit_ = new QLineEdit;
  record_name_edit_->setPlaceholderText("Name");
  record_name_edit_->setVisible(false);
  connect(record_name_edit_, SIGNAL(textChanged(const QString &)), this, SLOT(onRecordNameChanged(const QString &)));
  livestream_layout->addWidget(record_name_edit_);

  record_button_ = new QPushButton("Record");
  record_button_->setCheckable(true);
  record_button_->setVisible(false);
  record_button_->setEnabled(false);
  connect(record_button_, SIGNAL(clicked(bool)), this, SLOT(onRecordButtonClicked(bool)));
  livestream_layout->addWidget(record_button_);

  layout->addLayout(livestream_layout);

  // bags list
  bags_table_ = new BagtubeTableWidget;
  bags_table_->setColumnCount(3);
  bags_table_->setHorizontalHeaderLabels({"Name", "Timestamp", "Duration"});
  bags_table_->setSelectionBehavior(QAbstractItemView::SelectionBehavior::SelectRows);
  connect(bags_table_, SIGNAL(cellClicked(int, int)), this, SLOT(onBagSelected(int, int)));
  connect(bags_table_, SIGNAL(deletePressed(int, int)), this, SLOT(onBagDelete(int, int)));
  connect(bags_table_, SIGNAL(cellChanged(int, int)), this, SLOT(onBagNameEdited(int, int)));
  layout->addWidget(bags_table_);

  auto bags_control_layout = new QHBoxLayout;
  auto name_label = new QLabel("Name: ");
  bags_control_layout->addWidget(name_label);

  bags_filter_edit_ = new QLineEdit;
  connect(bags_filter_edit_, SIGNAL(editingFinished()), this, SLOT(onBagsFilterChanged()));
  bags_control_layout->addWidget(bags_filter_edit_);

  auto load_button = new QPushButton("Load list");
  connect(load_button, SIGNAL(clicked()), this, SLOT(reloadBags()));
  bags_control_layout->addWidget(load_button);

  layout->addLayout(bags_control_layout);

  // playback control
  playback_slider_ = new QSlider(Qt::Horizontal);
  playback_slider_->setStyle(new SliderAbsoluteSetClickStyle(this->style()));
  playback_slider_->setTracking(false);
  playback_slider_->setSingleStep(std::numeric_limits<int>::max());
  connect(playback_slider_, SIGNAL(valueChanged(int)), this, SLOT(onPlaybackSliderChanged(int)));
  layout->addWidget(playback_slider_);
  playback_slider_->setEnabled(false);

  auto playback_control_layout = new QHBoxLayout;
  playback_time_label_ = new QLabel(durationToString(0.0));
  playback_control_layout->addWidget(playback_time_label_);
  playback_control_layout->addStretch();

  play_pause_button_ = new QPushButton("▶");  // "❚❚"
  connect(play_pause_button_, SIGNAL(clicked()), this, SLOT(onPlayPauseButtonClicked()));
  playback_control_layout->addWidget(play_pause_button_);
  play_pause_button_->setEnabled(false);

  playback_control_layout->addStretch();
  playback_speed_combo_ = new QComboBox;
  playback_speed_combo_->addItems({"0.25x", "0.5x", "1x", "2x", "4x", "8x", "16x"});
  playback_speed_combo_->setCurrentIndex(2);
  connect(playback_speed_combo_, SIGNAL(currentIndexChanged(int)), this, SLOT(onPlaybackSpeedChanged(int)));
  playback_control_layout->addWidget(playback_speed_combo_);

  playback_duration_label_ = new QLabel(durationToString(0.0));
  playback_control_layout->addWidget(playback_duration_label_);

  layout->addLayout(playback_control_layout);

  setLayout(layout);
}

void BagtubePanel::onInitialize() {
  rviz_ros_node_ = getDisplayContext()->getRosNodeAbstraction();
  auto node_lock = rviz_ros_node_.lock();
  auto node = node_lock->get_raw_node();

  sync_response_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  sync_response_cb_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  sync_response_executor_->add_callback_group(sync_response_cb_group_, node->get_node_base_interface());

  list_bags_cl_ = node->create_client<bagtube_msgs::srv::GetBagList>(
      "get_bag_list",
      rclcpp::ServicesQoS(),
      sync_response_cb_group_
  );

  edit_bag_cl_ = node->create_client<bagtube_msgs::srv::EditBag>(
      "edit_bag",
      rclcpp::ServicesQoS(),
      sync_response_cb_group_
  );

  livestream_cl_ = node->create_client<std_srvs::srv::SetBool>(
      "enable_livestream",
      rclcpp::ServicesQoS(),
      sync_response_cb_group_
  );

  control_playback_cl_ = node->create_client<bagtube_msgs::srv::ControlPlayback>(
      "control_playback",
      rclcpp::ServicesQoS(),
      sync_response_cb_group_
  );

  play_bag_ac_ = rclcpp_action::create_client<bagtube_msgs::action::PlayBag>(node, "play_bag");
  record_bag_ac_ = rclcpp_action::create_client<bagtube_msgs::action::RecordBag>(node, "record_bag");

  reloadBags();
}

void BagtubePanel::setVisible(bool visible) {
  if (visible != isVisible()) {
    if (!rviz_ros_node_.lock())
      return;
    if (visible)  // update list of bags
      reloadBags();
    else
      onBagSelected(-1, -1);
  }

  rviz_common::Panel::setVisible(visible);
}

void BagtubePanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
  config.mapSetValue( "Record name", record_name_edit_->text() );
  config.mapSetValue( "Filter name", bags_filter_edit_->text() );
  config.mapSetValue( "Playback rate", playback_speed_combo_->currentText() );
}

// Load all configuration data for this panel from the given Config object.
void BagtubePanel::load(const rviz_common::Config &config) {
  rviz_common::Panel::load(config);

  QString record_name;
  if (config.mapGetString( "Record name", &record_name ))
    record_name_edit_->setText( record_name );
  QString filter_name;
  if (config.mapGetString( "Filter name", &filter_name ))
    bags_filter_edit_->setText( filter_name );
  QString playback_rate;
  if (config.mapGetString( "Playback rate", &playback_rate ))
    playback_speed_combo_->setCurrentText( playback_rate );

  reloadBags();
}

QString BagtubePanel::durationToString(double duration) {
  // convert from rclcpp::Duration to hh:mm:ss.sss
  char buf[30];
  snprintf(buf, sizeof(buf), "%02d:%02d:%02d.%03d",
           (int)duration/3600, ((int)duration%3600)/60, (int)duration%60,
           (int)(duration*1000)%1000);
  return QString(buf);
}

template<typename MsgT>
typename MsgT::Response::SharedPtr BagtubePanel::callService(typename rclcpp::Client<MsgT>::SharedPtr client, const typename MsgT::Request::SharedPtr& request)
{
  bool connected = client->wait_for_service(800ms);
  if (!connected)
    return nullptr;
  auto result = client->async_send_request(request);
  typename MsgT::Response::SharedPtr response;
  if (sync_response_executor_->spin_until_future_complete(result, 800ms) == rclcpp::FutureReturnCode::SUCCESS &&
      (response = result.get())) {
    return response;
  }
  return nullptr;
}

void BagtubePanel::onBagSelected(int row, int /*column*/) {
  std::unique_lock<std::recursive_mutex> lock(action_mutex_);
  if (row == -1 ? selected_bag_ == bagtube_msgs::msg::BagInfo() : selected_bag_ == bags_[row])
    return;

  if (play_bag_goal_handle_)
    play_bag_ac_->async_cancel_goal(play_bag_goal_handle_);

  if (row < 0 || static_cast<size_t>(row) >= bags_.size()) {
    selected_bag_ = bagtube_msgs::msg::BagInfo();
    playback_slider_->setEnabled(false);
    play_pause_button_->setEnabled(false);
  }
  else {
    selected_bag_ = bags_[row];
    playback_slider_->setEnabled(true);
    play_pause_button_->setEnabled(true);
  }

  playback_slider_->blockSignals(true);
  playback_slider_->setRange(0, selected_bag_.duration*1000);
  playback_slider_->setValue(0);
  playback_slider_->blockSignals(false);
  playback_time_label_->setText(durationToString(0.0));
  playback_duration_label_->setText(durationToString(selected_bag_.duration));
  bool wait_for_messages_to_arrive = livestream_button_->isChecked();
  if (livestream_button_->isChecked()) {
    livestream_button_->setChecked(false);
    onLivestreamButtonClicked(false);
  }

  // in case a bag is being played we will request a preview after the playback is canceled
  if (!play_bag_goal_handle_ && selected_bag_.duration != 0) {
    if (wait_for_messages_to_arrive) {
      RCLCPP_INFO(LOGGER, "Waiting for livestream messages to arrive before requesting selected bag preview");
      std::this_thread::sleep_for(100ms);
    }
    // get a preview of the selected bag
    auto request = std::make_shared<bagtube_msgs::srv::ControlPlayback::Request>();
    request->command = bagtube_msgs::srv::ControlPlayback::Request::SEEK;
    request->offset = selected_bag_.duration/2.0;
    request->snapshot_bag_name = selected_bag_.name;
    request->snapshot_bag_stamp = selected_bag_.stamp;

    callService<bagtube_msgs::srv::ControlPlayback>(control_playback_cl_, request);
  }
}

void BagtubePanel::onBagDelete(int row, int /*column*/)
{
  std::unique_lock<std::recursive_mutex> lock(action_mutex_);
  if (row < 0 || row >= static_cast<int>(bags_.size()))
    return;
  if (play_bag_goal_handle_)
    play_bag_ac_->async_cancel_goal(play_bag_goal_handle_);

  auto request = std::make_shared<bagtube_msgs::srv::EditBag::Request>();
  request->command = bagtube_msgs::srv::EditBag::Request::DELETE;
  request->name = bags_[row].name;
  request->stamp = bags_[row].stamp;
  if (auto response = callService<bagtube_msgs::srv::EditBag>(edit_bag_cl_, request); response && response->success) {
    reloadBags();
    onBagSelected(bags_table_->currentRow(), bags_table_->currentColumn());
  }
  else
    RCLCPP_ERROR(LOGGER, "Failed to delete bag: %s", response->message.c_str());
}

void BagtubePanel::onBagNameEdited(int row, int column)
{
  std::unique_lock<std::recursive_mutex> lock(action_mutex_);
  if (row < 0 || row >= static_cast<int>(bags_.size()))
    return;
  std::string new_name = bags_table_->item(row, column)->text().toStdString();
  if (new_name == bags_[row].name || new_name.empty()) {
    bags_table_->item(row, column)->setText(QString::fromStdString(bags_[row].name));
    return;
  }
  if (play_bag_goal_handle_)
    play_bag_ac_->async_cancel_goal(play_bag_goal_handle_);

  auto request = std::make_shared<bagtube_msgs::srv::EditBag::Request>();
  request->command = bagtube_msgs::srv::EditBag::Request::RENAME;
  request->name = bags_[row].name;
  request->stamp = bags_[row].stamp;
  request->new_name = new_name;
  if (auto response = callService<bagtube_msgs::srv::EditBag>(edit_bag_cl_, request); response && response->success)
    bags_[row].name = new_name;
  else
    RCLCPP_ERROR(LOGGER, "Failed to rename bag: %s", response->message.c_str());
}

void BagtubePanel::reloadBags()
{
  std::unique_lock<std::recursive_mutex> lock(action_mutex_);
  bags_table_->blockSignals(true);
  auto request = std::make_shared<bagtube_msgs::srv::GetBagList::Request>();
  request->name = bags_filter_edit_->text().toStdString();
  if (auto response = callService<bagtube_msgs::srv::GetBagList>(list_bags_cl_, request)) {
    bags_ = response->bags;
    bags_table_->setRowCount(response->bags.size());
    for (size_t i = 0; i < response->bags.size(); ++i) {
      auto &bag = response->bags[i];
      auto name_item = new QTableWidgetItem(QString::fromStdString(bag.name));
      name_item->setFlags(name_item->flags() | Qt::ItemIsEditable);
      bags_table_->setItem(i, 0, name_item);

      {
        // convert from rclcpp::Time to yyyy-mm-dd hh:mm:ss
        auto time = std::chrono::system_clock::time_point(
            std::chrono::seconds(bag.stamp.sec) + std::chrono::nanoseconds(bag.stamp.nanosec));
        auto time_t = std::chrono::system_clock::to_time_t(time);
        auto tm = std::localtime(&time_t);
        char buf[30];
        strftime(buf, sizeof(buf), "%F %T", tm);
        auto stamp_item = new QTableWidgetItem(buf);
        stamp_item->setFlags(stamp_item->flags() & ~Qt::ItemIsEditable);
        bags_table_->setItem(i, 1, stamp_item);
      }

      {
        auto duration_item = new QTableWidgetItem(durationToString(bag.duration));
        duration_item->setFlags(duration_item->flags() & ~Qt::ItemIsEditable);
        bags_table_->setItem(i, 2, duration_item);
      }
    }

    auto sel_it = std::find(bags_.begin(), bags_.end(), selected_bag_);
    if (sel_it != bags_.end()) {
      bags_table_->selectRow(sel_it - bags_.begin());
    } else {
      onBagSelected(-1, -1);
    }
  }
  bags_table_->resizeColumnsToContents();
  bags_table_->blockSignals(false);
}

void BagtubePanel::onBagsFilterChanged() {
  reloadBags();
  Q_EMIT configChanged();
}

void BagtubePanel::onPlaybackSliderChanged(int value) {
  std::unique_lock<std::recursive_mutex> lock(action_mutex_);
  if (selected_bag_.duration == 0) {
    playback_slider_->blockSignals(true);
    playback_slider_->setValue(0);
    playback_slider_->blockSignals(false);
    return;
  }

  playback_time_label_->setText(durationToString(value/1000.0));
  auto request = std::make_shared<bagtube_msgs::srv::ControlPlayback::Request>();
  request->command = bagtube_msgs::srv::ControlPlayback::Request::SEEK;
  request->offset = value/1000.0;
  // request publishing snapshot at selected time (even if playback is paused)
  request->snapshot_bag_name = selected_bag_.name;
  request->snapshot_bag_stamp = selected_bag_.stamp;

  if (auto response = callService<bagtube_msgs::srv::ControlPlayback>(control_playback_cl_, request); response && response->success) {
    // do nothing
  } else if (play_bag_goal_handle_) {
    RCLCPP_ERROR(LOGGER, "Failed to seek playback");
  }
}

void BagtubePanel::onPlayPauseButtonClicked() {
  std::unique_lock<std::recursive_mutex> lock(action_mutex_);
  livestream_button_->setChecked(false);
  onLivestreamButtonClicked(false);

  if (play_bag_goal_handle_) {
    // TODO: don't cancel the goal, pause/resume instead
    auto request = std::make_shared<bagtube_msgs::srv::ControlPlayback::Request>();
    request->command = play_pause_button_->text() == "▶" ? bagtube_msgs::srv::ControlPlayback::Request::RESUME :
                       bagtube_msgs::srv::ControlPlayback::Request::PAUSE;
    if (auto response = callService<bagtube_msgs::srv::ControlPlayback>(control_playback_cl_, request); response && response->success) {
      play_pause_button_->setText(request->command == bagtube_msgs::srv::ControlPlayback::Request::RESUME ? "❚❚" : "▶");
    } else {
      RCLCPP_ERROR(LOGGER, "Failed to %s playback",
                   request->command == bagtube_msgs::srv::ControlPlayback::Request::RESUME ? "resume" : "pause");
    }
  }
  else {
    if (selected_bag_.duration == 0)
      return;

    play_pause_button_->setText("❚❚");
    play_pause_button_->setEnabled(false);

    auto node_lock = rviz_ros_node_.lock();
    auto node = node_lock->get_raw_node();
    play_bag_ac_->wait_for_action_server(0.8s);
    if (!play_bag_ac_->action_server_is_ready()) {
      RCLCPP_ERROR(LOGGER, "Playback action server not available after waiting");
      return;
    }
    auto send_goal_opts = rclcpp_action::Client<bagtube_msgs::action::PlayBag>::SendGoalOptions();
    send_goal_opts.goal_response_callback =
        [&](const rclcpp_action::ClientGoalHandle<bagtube_msgs::action::PlayBag>::SharedPtr &goal_handle) {
          std::unique_lock<std::recursive_mutex> lock(action_mutex_);
          play_bag_goal_handle_ = goal_handle;
          play_pause_button_->setEnabled(true);
          if (!goal_handle) {
            play_pause_button_->setText("▶");
            RCLCPP_INFO(LOGGER, "Playback request rejected");
          } else
            RCLCPP_INFO(LOGGER, "Playback request accepted");
        };
    send_goal_opts.result_callback =
        [&](const rclcpp_action::ClientGoalHandle<bagtube_msgs::action::PlayBag>::WrappedResult &result) {
          std::unique_lock<std::recursive_mutex> lock(action_mutex_);
          bool wait_for_messages_to_arrive = play_pause_button_->text() == "❚❚";
          play_pause_button_->setText("▶");
          play_bag_goal_handle_ = nullptr;
          playback_slider_->blockSignals(true);
          playback_slider_->setValue(0);
          playback_slider_->blockSignals(false);

          switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
              RCLCPP_INFO(LOGGER, "Playback request complete!");
              break;
            case rclcpp_action::ResultCode::ABORTED:
              RCLCPP_INFO(LOGGER, "Playback request aborted");
              break;
            case rclcpp_action::ResultCode::CANCELED:
              RCLCPP_INFO(LOGGER, "Playback request canceled");
              break;
            default:
              RCLCPP_INFO(LOGGER, "Playback request unknown result code");
              break;
          }

          if (selected_bag_ != bag_being_played_ && selected_bag_.duration != 0) {
            // wait for previous playback messages to arrive so we can get a clear preview of the newly selected bag
            if (wait_for_messages_to_arrive) {
              RCLCPP_INFO(LOGGER, "Waiting for playback messages to arrive before requesting selected bag preview");
              std::this_thread::sleep_for(100ms);
            }
            // get a preview of the selected bag
            auto request = std::make_shared<bagtube_msgs::srv::ControlPlayback::Request>();
            request->command = bagtube_msgs::srv::ControlPlayback::Request::SEEK;
            request->offset = selected_bag_.duration/2.0;
            request->snapshot_bag_name = selected_bag_.name;
            request->snapshot_bag_stamp = selected_bag_.stamp;

            callService<bagtube_msgs::srv::ControlPlayback>(control_playback_cl_, request);
          }
        };
    send_goal_opts.feedback_callback =
        [&](const rclcpp_action::ClientGoalHandle<bagtube_msgs::action::PlayBag>::SharedPtr &goal_handle,
            const std::shared_ptr<const bagtube_msgs::action::PlayBag::Feedback> &feedback) {
          std::unique_lock<std::recursive_mutex> lock(action_mutex_);
          if (goal_handle != play_bag_goal_handle_)
            return;
          if (playback_slider_->isSliderDown())
            return;
          playback_slider_->blockSignals(true);
          playback_slider_->setValue(feedback->current_offset*1000);
          playback_slider_->blockSignals(false);
          playback_time_label_->setText(durationToString(feedback->current_offset));
        };
    bag_being_played_ = selected_bag_;
    bagtube_msgs::action::PlayBag::Goal goal;
    goal.stamp = selected_bag_.stamp;
    goal.name = selected_bag_.name;
    std::string rate_str = playback_speed_combo_->itemText(playback_speed_combo_->currentIndex()).toStdString();
    goal.rate = std::stod(rate_str.substr(0, rate_str.size()-1));
    goal.start_offset = playback_slider_->value()/1000.0;
    play_bag_ac_->async_send_goal(goal, send_goal_opts);
  }
}

void BagtubePanel::onLivestreamButtonClicked(bool checked) {
  std::unique_lock<std::recursive_mutex> lock(action_mutex_);
  if (checked && play_bag_goal_handle_)
    play_bag_ac_->async_cancel_goal(play_bag_goal_handle_);
  onLivestreamEnabledChanged();
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = checked;
  if (auto response = callService<std_srvs::srv::SetBool>(livestream_cl_, request); !response || !response->success) {
    livestream_button_->setChecked(!checked);
    onLivestreamEnabledChanged();
    RCLCPP_ERROR(LOGGER, "Failed to %s livestream",
                 checked ? "enable" : "disable");
  }
}

void BagtubePanel::onRecordNameChanged(const QString &text) {
  record_button_->setEnabled(!text.isEmpty());
  Q_EMIT configChanged();
}

void BagtubePanel::onRecordButtonClicked(bool checked) {
  std::unique_lock<std::recursive_mutex> lock(action_mutex_);

  if (checked) {
    if (record_bag_goal_handle_)
      record_bag_ac_->async_cancel_goal(record_bag_goal_handle_);
    auto node_lock = rviz_ros_node_.lock();
    auto node = node_lock->get_raw_node();
    record_bag_ac_->wait_for_action_server(0.8s);
    if (!record_bag_ac_->action_server_is_ready()) {
      RCLCPP_ERROR(LOGGER, "Record action server not available after waiting");
      return;
    }
    auto send_goal_opts = rclcpp_action::Client<bagtube_msgs::action::RecordBag>::SendGoalOptions();
    send_goal_opts.goal_response_callback =
        [&](const rclcpp_action::ClientGoalHandle<bagtube_msgs::action::RecordBag>::SharedPtr &goal_handle) {
          std::unique_lock<std::recursive_mutex> lock(action_mutex_);
          record_bag_goal_handle_ = goal_handle;
          if (!goal_handle) {
            record_button_->setChecked(false);
            RCLCPP_INFO(LOGGER, "Record request rejected");
          } else
            RCLCPP_INFO(LOGGER, "Record request accepted");
        };
    send_goal_opts.result_callback =
        [&](const rclcpp_action::ClientGoalHandle<bagtube_msgs::action::RecordBag>::WrappedResult &result) {
          std::unique_lock<std::recursive_mutex> lock(action_mutex_);
          record_button_->setChecked(false);
          record_bag_goal_handle_ = nullptr;
          if (result.code == rclcpp_action::ResultCode::SUCCEEDED || result.code == rclcpp_action::ResultCode::CANCELED)
            RCLCPP_INFO(LOGGER, "Record request complete!");
          else
            RCLCPP_INFO(LOGGER, "Record request failed");
          reloadBags();
        };
    bagtube_msgs::action::RecordBag::Goal goal;
    goal.name = record_name_edit_->text().toStdString();
    record_bag_ac_->async_send_goal(goal, send_goal_opts);
  }
  else {
    if (record_bag_goal_handle_)
      record_bag_ac_->async_cancel_goal(record_bag_goal_handle_);
  }
}

void BagtubePanel::onLivestreamEnabledChanged() {
  std::unique_lock<std::recursive_mutex> lock(action_mutex_);
  record_name_edit_->setVisible(livestream_button_->isChecked());
  record_button_->setVisible(livestream_button_->isChecked());
  if (!livestream_button_->isChecked() && record_bag_goal_handle_)
    record_bag_ac_->async_cancel_goal(record_bag_goal_handle_);
}

void BagtubePanel::onPlaybackSpeedChanged(int index) {
  std::unique_lock<std::recursive_mutex> lock(action_mutex_);
  Q_EMIT configChanged();
  if (!play_bag_goal_handle_)
    return;

  auto request = std::make_shared<bagtube_msgs::srv::ControlPlayback::Request>();
  request->command = bagtube_msgs::srv::ControlPlayback::Request::SET_RATE;
  std::string rate_str = playback_speed_combo_->itemText(index).toStdString();
  request->rate = std::stod(rate_str.substr(0, rate_str.size()-1));
  if (auto response = callService<bagtube_msgs::srv::ControlPlayback>(control_playback_cl_, request); response && response->success) {
    // do nothing
  } else
    RCLCPP_ERROR(LOGGER, "Failed to set playback rate");
}

} // bagtube_rviz_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(bagtube_rviz_plugins::BagtubePanel, rviz_common::Panel)
