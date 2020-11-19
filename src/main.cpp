#include "CatixKinematics.h"

#include <QApplication>

int main(int argc, char** argv) 
{
    QApplication app(argc, argv);

    ros::init(argc, argv, "CatixKinematics");
    if (!ros::master::check())
    {
        std::cerr << "ROS Master is not running" << std::endl;
        return 1;
    }

    CatixKinematics catixKinematics;

	return app.exec();
}

