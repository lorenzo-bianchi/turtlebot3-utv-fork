#include <analog_stick_widget.hpp>

AnalogStickWidget::AnalogStickWidget(QWidget* parent)
  : QWidget(parent), x_(0), y_(0)
{
  setFixedSize(150, 150);
}

double AnalogStickWidget::get_linear_velocity() const
{
  return y_;
}

double AnalogStickWidget::get_angular_velocity() const
{
  return x_;
}

void AnalogStickWidget::set_linear_velocity(double linear)
{
  y_ = std::min(1.0, std::max(-1.0, linear));
  update();
}

void AnalogStickWidget::set_angular_velocity(double angular)
{
  x_ = std::min(1.0, std::max(-1.0, angular));
  update();
}

void AnalogStickWidget::paintEvent(QPaintEvent*)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  // Draw the boundary
  painter.setBrush(QColor(200, 200, 200));
  painter.drawRect(rect());

  // Draw the stick
  int stick_radius = 10;
  int boundary_size = qMin(width(), height());
  int center_x = width() / 2 + x_ * (boundary_size / 2 - stick_radius);
  int center_y = height() / 2 - y_ * (boundary_size / 2 - stick_radius); // Invert Y
  painter.setBrush(Qt::blue);
  painter.drawEllipse(QPointF(center_x, center_y), stick_radius, stick_radius);
}

void AnalogStickWidget::mousePressEvent(QMouseEvent* event)
{
  update_stick_position(event->pos());
}

void AnalogStickWidget::mouseMoveEvent(QMouseEvent* event)
{
  update_stick_position(event->pos());
}

void AnalogStickWidget::mouseReleaseEvent(QMouseEvent*)
{
  x_ = 0;
  y_ = 0;
  emit velocity_changed(y_, x_);
  update();
}

void AnalogStickWidget::update_stick_position(const QPoint& pos)
{
  x_ = std::min(1.0, std::max(-1.0, 2.0 * pos.x() / width() - 1.0));
  y_ = std::min(1.0, std::max(-1.0, 1.0 - 2.0 * pos.y() / height()));

  emit velocity_changed(y_, x_);
  update();
}
