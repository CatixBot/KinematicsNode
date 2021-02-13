#include "SimulationWindow.h"
#include "ui_simulationwindow.h"

//------------------------------------------------------------------------

double convertDegreesToRadians(double angleInDegrees)
{
    return angleInDegrees * M_PI / 180.0;
}

//------------------------------------------------------------------------

SimulationWindow::SimulationWindow(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::SimulationWindow)
{
    ui->setupUi(this);

    this->connectServoGroup();
    this->connectLegGroup();
    this->connectPlatformGroup();
}

SimulationWindow::~SimulationWindow()
{
    delete ui;
}

void SimulationWindow::connectServoGroup()
{
    QObject::connect(ui->servoAngleDial, &QDial::valueChanged, [&](int dialValue)
    {
        if (!ui->servoEnabledCheckBox->isChecked())
        {
            return;
        }

        const size_t servoIndex = ui->servoIndexComboBox->currentIndex();
        const double servoAngle = convertDegreesToRadians(dialValue);
        emit onServoAngle(servoIndex, servoAngle);
    });

    QObject::connect(ui->servoAngleDial, &QDial::valueChanged, [&](int dialValue)
    {
        this->ui->servoAngleSpinBox->setValue(dialValue);
    });

    QObject::connect(ui->servoEnabledCheckBox, &QCheckBox::toggled, [&](bool isChecked)
    {
        if (!isChecked)
        {
            return;
        }

        const size_t servoIndex = ui->servoIndexComboBox->currentIndex();
        const double servoAngle = convertDegreesToRadians(ui->servoAngleDial->value());
        emit onServoAngle(servoIndex, servoAngle);
    });
}

void SimulationWindow::connectLegGroup()
{
    QObject::connect(ui->radialCoordinateDial, &QDial::valueChanged, [&](int dialValue)
    {
        if (!ui->legEnabledCheckBox->isChecked())
        {
            return;
        }

        const double radialCoordinate = dialValue / 1000.0;
        const double angularCoordinate = convertDegreesToRadians(ui->angularCoordinateDial->value());
        notifySelectedLegs(radialCoordinate, angularCoordinate);
    });

    QObject::connect(ui->radialCoordinateDial, &QDial::valueChanged, [&](int dialValue)
    {
        this->ui->radialCoordinateSpinBox->setValue(dialValue);
    });

    QObject::connect(ui->angularCoordinateDial, &QDial::valueChanged, [&](int dialValue)
    {
        if (!ui->legEnabledCheckBox->isChecked())
        {
            return;
        }

        const double radialCoordinate = ui->radialCoordinateDial->value() / 1000.0;
        const double angularCoordinate = convertDegreesToRadians(dialValue);
        notifySelectedLegs(radialCoordinate, angularCoordinate);
    });

    QObject::connect(ui->angularCoordinateDial, &QDial::valueChanged, [&](int dialValue)
    {
        this->ui->angularCoordinateSpinBox->setValue(dialValue);
    });

    QObject::connect(ui->legEnabledCheckBox, &QCheckBox::toggled, [&](bool isChecked)
    {
        if (!isChecked)
        {
            return;
        }

        const double radialCoordinate = ui->radialCoordinateDial->value() / 1000.0;
        const double angularCoordinate = convertDegreesToRadians(ui->angularCoordinateDial->value());
        notifySelectedLegs(radialCoordinate, angularCoordinate);
    });
}

void SimulationWindow::connectPlatformGroup()
{
    // TODO: Connect platform signals
}

void SimulationWindow::notifySelectedLegs(double radialCoordinate, double angularCoordinate)
{
    std::vector<size_t> selectedLegIndecies;
    if (ui->zeroLegCheckBox->isChecked())
    {
        selectedLegIndecies.push_back(0);
    }
    if (ui->firstLegCheckBox->isChecked())
    {
        selectedLegIndecies.push_back(1);
    }
    if (ui->secondLegCheckBox->isChecked())
    {
        selectedLegIndecies.push_back(2);
    }
    if (ui->thirdLegCheckBox->isChecked())
    {
        selectedLegIndecies.push_back(3);
    }

    for (const auto legIndex : selectedLegIndecies)
    {
        emit onLegPosition(legIndex, radialCoordinate, angularCoordinate);
    }
}
