#include <ros/ros.h>
#include "motor_control/TalonNode.h"

using namespace motor_control;

static std::shared_ptr<ros::NodeHandle> nh;
static std::vector<std::unique_ptr<TalonNode>> talons;

static TalonConfig createConfig(ros::NodeHandle& nh){
    motor_control::TalonConfig config;
    config.interface = nh.getParam("interface", config.interface);
    config.id = nh.getParam("id", config.id);
    config.inverted = nh.getParam("inverted", config.inverted);
    config.peak_voltage = nh.getParam("peak_voltage", config.peak_voltage);
    config.P = nh.getParam("P", config.P);
    config.I = nh.getParam("I", config.I);
    config.D = nh.getParam("D", config.D);
    config.F = nh.getParam("F", config.F);
    return config;
}

static void createTalon(std::string name){
    auto node = ros::NodeHandle(*nh, name);
    talons.push_back(std::unique_ptr<TalonNode>(new TalonNode(node, name, createConfig(node))));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "motor_control");
    nh = std::make_shared<ros::NodeHandle>();

    createTalon("fl");
    createTalon("fr");
    createTalon("br");
    createTalon("bl");

    ROS_INFO("Spinning node");
    ros::Rate loop_rate(50);
    while(ros::ok()){
        std::for_each(talons.begin(), talons.end(), [](std::unique_ptr<TalonNode>& talon){
           talon->update();
        });

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}