#include <twist_panel.hpp>
#include <QHBoxLayout>

namespace rviz2_twist_plugin
{

TwistPanel::TwistPanel(QWidget* parent)
  : rviz_common::Panel(parent), max_linear_(1.0), max_angular_(1.0)
{
  // Create ROS2 node
  node_ = std::make_shared<rclcpp::Node>("twist_panel");

  // Create UI elements
  QVBoxLayout* layout = new QVBoxLayout;

  // Topic management section
  QLabel* topic_label = new QLabel("Topics:");
  layout->addWidget(topic_label);

  QHBoxLayout* topic_input_layout = new QHBoxLayout;

  topic_edit_ = new QLineEdit();
  topic_edit_->setPlaceholderText("Enter topic name");
  topic_input_layout->addWidget(topic_edit_);

  QPushButton* add_topic_button = new QPushButton("Add");
  topic_input_layout->addWidget(add_topic_button);
  connect(add_topic_button, &QPushButton::clicked, this, &TwistPanel::add_topic);

  layout->addLayout(topic_input_layout);

  topic_list_ = new QListWidget();
  layout->addWidget(topic_list_);

  QString topic_name = "/cmd_vel";
  topic_list_->addItem(topic_name);
  auto pub = node_->create_publisher<geometry_msgs::msg::Twist>(topic_name.toStdString(), 10);
  publishers_.append(pub);

  QPushButton* remove_topic_button = new QPushButton("Remove selected");
  layout->addWidget(remove_topic_button);
  connect(remove_topic_button, &QPushButton::clicked, this, &TwistPanel::remove_selected_topic);

  // Max velocity fields (side by side)
  QHBoxLayout* max_velocity_layout = new QHBoxLayout;

  QLabel* max_linear_label = new QLabel("Max m/s:");
  max_velocity_layout->addWidget(max_linear_label);

  max_linear_edit_ = new QLineEdit("1.0");
  max_linear_edit_->setMaximumWidth(50); // Limit text field width
  max_velocity_layout->addWidget(max_linear_edit_);
  connect(max_linear_edit_, &QLineEdit::editingFinished, this, &TwistPanel::update_limits);

  QLabel* max_angular_label = new QLabel("Max rad/s:");
  max_velocity_layout->addWidget(max_angular_label);

  max_angular_edit_ = new QLineEdit("1.0");
  max_angular_edit_->setMaximumWidth(50); // Limit text field width
  max_velocity_layout->addWidget(max_angular_edit_);
  connect(max_angular_edit_, &QLineEdit::editingFinished, this, &TwistPanel::update_limits);

  layout->addLayout(max_velocity_layout);

  // Analog Stick Widget
  QHBoxLayout* user_layout = new QHBoxLayout;

  analog_stick_ = new AnalogStickWidget();
  connect(analog_stick_, &AnalogStickWidget::velocity_changed, this, &TwistPanel::update_velocity_stick);
  user_layout->addWidget(analog_stick_);

  QVBoxLayout* sliders_layout = new QVBoxLayout;


  // Linear velocity slider
  QHBoxLayout* linear_velocity_layout = new QHBoxLayout;
  QLabel* linear_label = new QLabel("Linear velocity:");
  linear_velocity_layout->addWidget(linear_label);

  linear_velocity_value_ = new QLabel("0.0 m/s");
  linear_velocity_layout->addWidget(linear_velocity_value_);

  sliders_layout->addLayout(linear_velocity_layout);

  linear_slider_ = new QSlider(Qt::Horizontal);
  linear_slider_->setRange(-100, 100);
  connect(linear_slider_, &QSlider::valueChanged, this, &TwistPanel::update_velocity_slider);
  sliders_layout->addWidget(linear_slider_);

  // Angular velocity slider
  QHBoxLayout* angular_velocity_layout = new QHBoxLayout;
  QLabel* angular_label = new QLabel("Angular velocity:");
  angular_velocity_layout->addWidget(angular_label);

  angular_velocity_value_ = new QLabel("0.0 rad/s");
  angular_velocity_layout->addWidget(angular_velocity_value_);

  sliders_layout->addLayout(angular_velocity_layout);

  angular_slider_ = new QSlider(Qt::Horizontal);
  angular_slider_->setRange(-100, 100);
  connect(angular_slider_, &QSlider::valueChanged, this, &TwistPanel::update_velocity_slider);
  sliders_layout->addWidget(angular_slider_);

  // Stop button
  QPushButton* stop_button = new QPushButton("Stop");
  connect(stop_button, &QPushButton::clicked, [this]() {
    linear_slider_->setValue(0);
    angular_slider_->setValue(0);
  });
  sliders_layout->addWidget(stop_button);

  user_layout->addLayout(sliders_layout);
  layout->addLayout(user_layout);

  // On/Off checkbox
  on_off_checkbox_ = new QCheckBox("Enable publishing");
  on_off_checkbox_->setChecked(false);
  layout->addWidget(on_off_checkbox_);
  // only when state becomes false
  connect(on_off_checkbox_, &QCheckBox::stateChanged, [this](int state) {
    if (state == Qt::Unchecked) {
      linear_slider_->setValue(0);
      angular_slider_->setValue(0);

      auto twist = Twist();
      for (const auto& pub : publishers_) {
        pub->publish(twist);
      }
    }
  });

  setLayout(layout);

  // Timer for periodic publishing
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &TwistPanel::send_twist);
  timer_->start(100); // Publish every 100 ms
}

void TwistPanel::update_velocity_stick(double linear, double angular)
{
  linear_slider_->setValue(std::round(linear * 100));
  angular_slider_->setValue(std::round(angular * 100));
}

void TwistPanel::update_velocity_slider()
{
  // Scale by max velocities
  linear_velocity_ = linear_slider_->value() / 100.0 * max_linear_;
  angular_velocity_ = angular_slider_->value() / 100.0 * max_angular_;

  linear_velocity_value_->setText(QString::number(linear_velocity_, 'f', 2) + " m/s");
  angular_velocity_value_->setText(QString::number(angular_velocity_, 'f', 2) + " rad/s");
}

void TwistPanel::send_twist()
{
  if (!on_off_checkbox_->isChecked()) {
    return;
  }

  auto twist = Twist();
  twist.linear.x = std::round(linear_velocity_ * 100.0) / 100.0;
  twist.angular.z = std::round(angular_velocity_ * 100.0) / 100.0;

  for (const auto& pub : publishers_) {
    pub->publish(twist);
  }
}

void TwistPanel::add_topic()
{
  QString topic_name = topic_edit_->text();
  if (topic_name.isEmpty()) {
    return; // Ignore empty input
  }

  if (!topics_.contains(topic_name)) {
    topics_.append(topic_name);
    topic_list_->addItem(topic_name);

    // Create a new publisher for this topic
    auto pub = node_->create_publisher<Twist>(topic_name.toStdString(), 10);
    publishers_.append(pub);
  }
  topic_edit_->clear(); // Clear input field
}

void TwistPanel::remove_selected_topic()
{
  QList<QListWidgetItem*> selected_items = topic_list_->selectedItems();
  for (QListWidgetItem* item : selected_items) {
    QString topic_name = item->text();
    int index = topics_.indexOf(topic_name);

    if (index != -1) {
      topics_.removeAt(index);
      publishers_.removeAt(index);
    }

    delete topic_list_->takeItem(topic_list_->row(item));
  }
}

void TwistPanel::update_limits()
{
  max_linear_ = max_linear_edit_->text().toDouble();
  max_angular_ = max_angular_edit_->text().toDouble();
}

void TwistPanel::round(double& value, uint precision)
{
  value = std::round(value * std::pow(10, precision)) / std::pow(10, precision);
}

}  // namespace rviz2_twist_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz2_twist_plugin::TwistPanel, rviz_common::Panel)
