#include "SimulationWindow.h"
#include "ui_simulationwindow.h"

SimulationWindow::SimulationWindow(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::SimulationWindow)
{
    ui->setupUi(this);

    QObject::connect(ui->servoAngleDial, &QDial::valueChanged, [&](int dialValue)
    {
        const size_t servoIndex = ui->servoIndexComboBox->currentIndex();
        const double servoAngle = static_cast<double>(dialValue);
        emit onServoAngle(servoIndex, servoAngle);
    });
}

SimulationWindow::~SimulationWindow()
{
    delete ui;
}
