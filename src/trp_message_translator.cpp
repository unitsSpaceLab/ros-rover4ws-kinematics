#include "../include/trp_message_translator.h"


TrpMessageTranslator::TrpMessageTranslator()
{
    this -> pub1 = nh.advertise<std_msgs::Float64>("command1", 1000);
    this -> pub2 = nh.advertise<std_msgs::Float64>("command2", 1000);
    this -> pub3 = nh.advertise<std_msgs::Float64>("command3", 1000);
    this -> pub4 = nh.advertise<std_msgs::Float64>("command4", 1000);
    this -> pub5 = nh.advertise<std_msgs::Float64>("command5", 1000);
    this -> pub6 = nh.advertise<std_msgs::Float64>("command6", 1000);
    this -> pub7 = nh.advertise<std_msgs::Float64>("command7", 1000);
    this -> pub8 = nh.advertise<std_msgs::Float64>("command8", 1000);

    this -> sub = nh.subscribe("cmd_vel_motors",1000, &TrpMessageTranslator::subCallback, this);

}

TrpMessageTranslator::~TrpMessageTranslator()
{

}

void TrpMessageTranslator::publish(void)
{

}


void TrpMessageTranslator::subCallback(const trp_msgs::TrpCommands::ConstPtr& msg)
{
    std_msgs::Float64 float_msg;

    float_msg.data = msg -> fl_drive_command;
    this -> pub1.publish(float_msg);

    float_msg.data = msg -> fr_drive_command;
    this -> pub2.publish(float_msg);

    float_msg.data = msg -> br_drive_command;
    this -> pub3.publish(float_msg);

    float_msg.data = msg -> bl_drive_command;
    this -> pub4.publish(float_msg);

    float_msg.data = msg -> fl_steer_command;
    this -> pub5.publish(float_msg);

    float_msg.data = msg -> fr_steer_command;
    this -> pub6.publish(float_msg);

    float_msg.data = msg -> br_steer_command;
    this -> pub7.publish(float_msg);

    float_msg.data = msg -> bl_steer_command;
    this -> pub8.publish(float_msg);

}





int main(int argc, char **argv)
{
    ros::init(argc, argv, "trp_msgs_translator");   

    TrpMessageTranslator node;
    
    
    ros::spin();
}
