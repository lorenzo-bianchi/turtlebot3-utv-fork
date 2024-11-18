#ifndef RVIZ2_TWIST_PLUGIN__ANALOG_STICK_WIDGET_HPP
#define RVIZ2_TWIST_PLUGIN__ANALOG_STICK_WIDGET_HPP

#include <QWidget>
#include <QPainter>
#include <QMouseEvent>
#include <cmath>

class AnalogStickWidget : public QWidget
{
  Q_OBJECT

public:
  explicit AnalogStickWidget(QWidget* parent = nullptr);

  double get_linear_velocity() const;
  double get_angular_velocity() const;
  void set_linear_velocity(double linear);
  void set_angular_velocity(double angular);

signals:
  void velocity_changed(double linear, double angular);

protected:
  void paintEvent(QPaintEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;

private:
  void update_stick_position(const QPoint& pos);

  double x_;
  double y_;
};

#endif // RVIZ2_TWIST_PLUGIN__ANALOG_STICK_WIDGET_HPP
