#ifndef RVIZ2_TWIST_PLUGIN__TWIST_PANEL_HPP
#define RVIZ2_TWIST_PLUGIN__TWIST_PANEL_HPP

#include <cmath>

#include <QSlider>
#include <QLineEdit>
#include <QListWidget>
#include <QCheckBox>
#include <QTimer>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <analog_stick_widget.hpp>

using namespace geometry_msgs::msg;

namespace rviz2_twist_plugin
{

class TwistPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  TwistPanel(QWidget* parent = nullptr);

private Q_SLOTS:
  void send_twist();
  void add_topic();
  void remove_selected_topic();
  void update_limits();
  void update_velocity_stick(double linear, double angular);
  void update_velocity_slider();
  void round(double& value, uint precision);

private:
  rclcpp::Node::SharedPtr node_;

  QLabel* linear_velocity_value_;
  QLabel* angular_velocity_value_;
  QLineEdit* topic_edit_;
  QListWidget* topic_list_;
  QSlider* linear_slider_;
  QSlider* angular_slider_;
  QLineEdit* max_linear_edit_;
  QLineEdit* max_angular_edit_;
  QCheckBox* on_off_checkbox_;
  QTimer* timer_;
  QList<QString> topics_;
  QList<rclcpp::Publisher<Twist>::SharedPtr> publishers_;

  AnalogStickWidget* analog_stick_;

  double max_linear_, max_angular_;
  double linear_velocity_, angular_velocity_;
};

}  // namespace rviz2_twist_plugin

#endif  // RVIZ2_TWIST_PLUGIN__TWIST_PANEL_HPP
