#include "hand_controller_node.h"
#include <iostream>
#include <queue>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"


using namespace std;

Serial_t Serial;
std::mutex Serial_Lock;
uint8_t ID = 1;

int main(int argc, char **argv)
{
    cout << "Hello World!" << endl;

    if(argc == 2)
    {
        string dev_dir(argv[1]);
        Serial_t Serial(dev_dir);
    }
    else
    {
        string dev_dir(UART_DEV);
        Serial_t Serial(dev_dir);
    }
    Hand_t Hand(argc,argv);
    return 0;
}





















/* 明日任务
** 1、完成ros接入
** 2、定义角度反馈和触感反馈消息
 */