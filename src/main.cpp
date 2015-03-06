#include <sigue_lineas.h>



int main(int argc, char **argv)
{
    sigue_lineas robot;
    ros::init(argc, argv, "sigue_lineas");
    while (ros::ok())
    {
        robot.follow();
    }
    return 0;
}

