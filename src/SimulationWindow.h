#pragma once

#include <QWidget>

QT_BEGIN_NAMESPACE
namespace Ui { class SimulationWindow; }
QT_END_NAMESPACE

class SimulationWindow : public QWidget
{
    Q_OBJECT

public:
    SimulationWindow(QWidget *parent = nullptr);
    ~SimulationWindow();

signals:
    void onServoAngle(size_t servoIndex, double servoAngle);
    void onLegPosition(size_t legIndex, double radialCoordinate, double angularCoordinate);
    void onPlatformSpeed(double moveForwardSpeed, double rotateClockwiseSpeed);

private:
    void connectServoGroup();
    void connectLegGroup();
    void connectPlatformGroup();

    void notifySelectedLegs(double radialCoordinate, double angularCoordinate);

private:
    Ui::SimulationWindow *ui;
};
