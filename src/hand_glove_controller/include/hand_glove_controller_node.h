#ifndef HAND_CONTROLLER
#define HAND_CONTROLLER
#include <stdio.h>      /*标准输入输出定义*/
#include <stdlib.h>     /*标准函数库定义*/
#include <unistd.h>     /*Unix 标准函数定义*/
#include <sys/types.h> 
#include <sys/stat.h>
#include <fcntl.h>      /*文件控制定义*/
#include <termios.h>    /*PPSIX 终端控制定义*/
#include <string>
#include <iostream>
#include <queue>
#include "ros/ros.h"
#include <thread>
#include <mutex>
#include "signal.h"
//消息文件

#include "sensor_msgs/JointState.h"

using namespace std;

#define UART_DEV "/dev/ttyUSB0"

#define HAND_GLOVE_ID 0x11
#define ANGLE_ACT 0x1546
#define FORCE_SET 0x1498

uint8_t Hand_Force_Set[12];


class Send_Hand_Data_t
{
public:
    Send_Hand_Data_t(uint8_t Hand_ID, uint16_t Reg_Address, uint8_t Register_Length,uint8_t *Data, uint8_t Read_Write)
    {
        if(Read_Write == 0)
        {
            queue<uint8_t> Temp_Data;
            Temp_Data.push(0xEB);
            Temp_Data.push(0x90);
            Temp_Data.push(Hand_ID);
            Temp_Data.push(0x04);
            Temp_Data.push(0x11);
            Temp_Data.push(Reg_Address & 0xff);
            Temp_Data.push((Reg_Address >> 8) & 0xff);
            Temp_Data.push(Register_Length);

            uint16_t Check_Sum = 0;
            uint8_t Data_Length = Temp_Data.size();
            for(uint8_t i = 0; i < Data_Length; i++)
            {
                Check_Sum += Temp_Data.front();
                Send_Data_Buffer.push(Temp_Data.front());
                Temp_Data.pop();
            }
            Check_Sum -= 0xEB;
            Check_Sum -= 0x90;
            Send_Data_Buffer.push(Check_Sum & 0xff);
        }
        else if(Read_Write == 1)
        {
            queue<uint8_t> Temp_Data;
            Temp_Data.push(0xEB);
            Temp_Data.push(0x90);
            Temp_Data.push(Hand_ID);
            Temp_Data.push(Register_Length + 3);
            Temp_Data.push(0x12);
            Temp_Data.push(Reg_Address & 0xff);
            Temp_Data.push((Reg_Address >> 8) & 0xff);
            
            if(Data != nullptr)
            {
                for(uint8_t i = 0; i < Register_Length; i ++)
                {
                    Temp_Data.push(*(Data + i));
                }
            }

            uint16_t Check_Sum = 0;
            uint8_t Data_Length = Temp_Data.size();
            for(uint8_t i = 0; i < Data_Length; i++)
            {
                Check_Sum += Temp_Data.front();
                Send_Data_Buffer.push(Temp_Data.front());
                Temp_Data.pop();
            }
            Check_Sum -= 0xEB;
            Check_Sum -= 0x90;
            Send_Data_Buffer.push(Check_Sum & 0xff); 
        }
        else if(Read_Write == 2 )
        {
            queue<uint8_t> Temp_Data;
            Temp_Data.push(0xEB);
            Temp_Data.push(0x90);
            Temp_Data.push(Hand_ID);
            Temp_Data.push(Register_Length + 1);
            Temp_Data.push(0x31);
            
            if(Data != nullptr)
            {
                for(uint8_t i = 0; i < Register_Length; i ++)
                {
                    Temp_Data.push(*(Data + i));
                }
            }

            uint16_t Check_Sum = 0;
            uint8_t Data_Length = Temp_Data.size();
            for(uint8_t i = 0; i < Data_Length; i++)
            {
                Check_Sum += Temp_Data.front();
                Send_Data_Buffer.push(Temp_Data.front());
                Temp_Data.pop();
            }
            Check_Sum -= 0xEB;
            Check_Sum -= 0x90;
            Send_Data_Buffer.push(Check_Sum & 0xff); 
        }
    }

    void Get_Send_Data(queue<uint8_t> &ptr)
    {
        queue<uint8_t> temp_data(Send_Data_Buffer);

        while(!temp_data.empty())
        {
            ptr.push(temp_data.front());
            temp_data.pop();
        }
    }

    void Get_Send_Data(uint8_t *ptr,size_t *data_size)
    {
        queue<uint8_t> temp_data(Send_Data_Buffer);

        uint16_t i = 0;
        *data_size = temp_data.size();
        //将数据从queue中取出
        while(!temp_data.empty())
        {
            ptr[i] = temp_data.front();
            i++;
            temp_data.pop();
        }
    }

public:
queue<uint8_t> Send_Data_Buffer;
};


class Serial_t
{

public:
    Serial_t(string serial_name)
    {
        fd = open(serial_name.c_str(), O_RDWR|O_NOCTTY|O_NDELAY);

        if(fd < 0)
        {
            cout<< "Open failed: "<< serial_name << endl;
        }
        else
        {
            //==========串口打开============//
            printf("COM (%s) Open Success ! Watting recv...\n\n",UART_DEV);

            cout<< "Serial Open Success!"<< endl;

            tcgetattr(fd, &options);         //获取终端控制属性
            Set_Serial();
        }
    }
    Serial_t()
    {
        fd = open(UART_DEV, O_RDWR|O_NOCTTY|O_NDELAY);
    }

    void Set_Serial()
    {
        //==========配置串口============//
    
        cfsetispeed(& options, B115200);  //指定输入波特率(若不设置系统默认9600bps)
        cfsetospeed(& options, B115200);  //指定输出波特率(若不设置系统默认9600bps)
    
        /* c_lflag 本地模式 */
        options.c_cflag &= ~ INPCK;           //不启用输入奇偶检测
        options.c_cflag |= (CLOCAL |  CREAD); //CLOCAL忽略 modem 控制线,CREAD打开接受者
    
        /* c_lflag 本地模式 */
        options.c_lflag &= ~(ICANON | ECHO | ECHOE |  ISIG); //ICANON启用标准模式;ECHO回显输入字符;ECHOE如果同时设置了 ICANON，字符 ERASE 擦除前一个输入字符，WERASE 擦除前一个词;ISIG当接受到字符 INTR, QUIT, SUSP, 或 DSUSP 时，产生相应的信号
    
        /* c_oflag 输出模式 */
        options.c_oflag &= ~ OPOST;             //OPOST启用具体实现自行定义的输出处理
        options.c_oflag &= ~(ONLCR | OCRNL);    //ONLCR将输出中的新行符映射为回车-换行,OCRNL将输出中的回车映射为新行符
    
        /* c_iflag 输入模式 */
        options.c_iflag &= ~(ICRNL | IGNCR);  //ICRNL将输入中的回车翻译为新行 (除非设置了 IGNCR),INLCR将输入中的 NL 翻译为 CR
        options.c_iflag &= ~(IXON | IXOFF | IXANY);    //IXON启用输出的 XON/XOFF流控制,IXOFF启用输入的 XON/XOFF流控制,IXANY(不属于 POSIX.1；XSI) 允许任何字符来重新开始输出
    
        /* c_cflag 控制模式 */
        options.c_cflag &= ~CSIZE;     //字符长度掩码,取值为 CS5, CS6, CS7, 或 CS8,加~就是无
        options.c_cflag |=  CS8;        //数据宽度是8bit
        options.c_cflag &= ~CSTOPB;    //CSTOPB设置两个停止位，而不是一个,加~就是设置一个停止位
        options.c_cflag &= ~PARENB;    //PARENB允许输出产生奇偶信息以及输入的奇偶校验,加~就是无校验
    
        /* c_cc[NCCS] 控制字符 */
        options.c_cc[VTIME] = 0.5;   //等待数据时间(10秒的倍数),每个单位是0.1秒  若20就是2秒
        options.c_cc[VMIN] = 18;    //最少可读数据,非规范模式读取时的最小字符数，设为0则为非阻塞，如果设为其它值则阻塞，直到读到到对应的数据,就像一个阀值一样，比如设为8，如果只接收到3个数据，那么它是不会返回的，只有凑齐8个数据后一齐才READ返回，阻塞在那儿
        /* 
        如果这样设置，就完全阻塞了，只有串口收到至少8个数据才会对READ立即返回，或才少于8个数据时，超时2秒也会有返回
        另外特别注意的是当设置VTIME后，如果read第三个参数小于VMIN ，将会将VMIN 修改为read的第三个参数*/
    
        /*TCIFLUSH  刷清输入队列
        TCOFLUSH  刷清输出队列
        TCIOFLUSH 刷清输入、输出队列*/
        tcflush(fd, TCIOFLUSH);         //刷串口清缓存
        tcsetattr(fd, TCSANOW, &options);   //设置终端控制属性,TCSANOW：不等数据传输完毕就立即改变属性
    }

    void Get_Data(uint8_t *ptr, uint8_t *Rx_Data_Len)
    {
        uint16_t RxLen = 0;
        //==========串口接收(字符串)============//
        while(((RxLen = read(fd, RxBuff,sizeof(RxBuff))) > 0))
        {            
            if(RxLen == 18)
            {
                memcpy(ptr,RxBuff,18);
                *Rx_Data_Len = 18;
                return ;
            }
        }
    }

    void Send_Data(uint8_t *Tx_Data,size_t Tx_Data_Len)
    {
        write(fd,Tx_Data,Tx_Data_Len);
    }

    ~Serial_t()
    {

    }

private:
    int fd;
    struct termios options;
    uint8_t TxBuff[1024] = {0};
    uint8_t RxBuff[1024] = {0};

};

extern Serial_t Serial;
extern std::mutex Serial_Lock;
extern uint8_t ID;

class Hand_t
{
public:
    Hand_t(int argc_, char **argv_)
    {
        //状态初始化
        //*ID = 1;

        ros::init(argc_,argv_,"hand_glove");

        hand_glove_controller_node = new ros::NodeHandle;

        pub_hand_glove_status = hand_glove_controller_node->advertise<sensor_msgs::JointState>("/hand_control", 1000); //把这个节点设置成发布者，并把发布主题的类型告诉节点管理器，第一个参数是消息名称。

        //订阅灵巧手的控制回调，每当有控制值到来时触发，控制topic为hand_controll
        sub_hand_glove_controller = hand_glove_controller_node->subscribe("/hand_status", 1, Hand_Glove_Control_topic_callback);

        //创建手势获取任务
        pthread_create(&threads[0], NULL, Get_Hand_Glove_Status_Task, NULL);

        //创建手势发布任务
        pthread_create(&threads[1], NULL, Pub_Hand_Glove_Status_Task, this);

        signal(SIGINT, ros_exit);
        
        ros::spin();
    }

    /*ROS控制消息回调函数*/
    static void Hand_Glove_Control_topic_callback(const sensor_msgs::JointState &msg)
    {
        //获取控制值
        for(uint8_t i = 0; i < msg.effort.size(); i ++)
        {
            Hand_Force_Set[2*i + 0] = (uint8_t) ((int16_t)(0.7 * msg.effort[i]) & 0xff);
            Hand_Force_Set[2*i + 1] = (uint8_t)(((int16_t)(0.7 * msg.effort[i]) >> 8) & 0xff);
            
            // cout << "*******" << endl;
            // cout << msg.effort[i] << endl;
            // cout << hex <<unsigned(Hand_Force_Set[2*i + 1]) << " " << unsigned(Hand_Force_Set[2*i + 0]) << endl;
        }
    }

    /*灵巧手状态获取任务*/
    static void *Get_Hand_Glove_Status_Task(void *)
    {
        uint8_t Rx_Data[1024],Rx_Data_Len;
        while(1)
        {
            // cout << "111111111111" << endl;
            //得到获取状态指令
            Send_Hand_Data_t Get_Status_Command(HAND_GLOVE_ID, FORCE_SET, 0x0D, Hand_Force_Set, 2);
            uint8_t *Tx_Data = new uint8_t(32);
            size_t Tx_Data_Len = 0;
            Get_Status_Command.Get_Send_Data(Tx_Data, &Tx_Data_Len);
            // Serial_Lock.lock();//等待锁
            Serial.Send_Data(Tx_Data, Tx_Data_Len);
            // Serial_Lock.unlock();//解锁
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

    }

    static void *Pub_Hand_Glove_Status_Task(void *hand_ptr)
    {
        Hand_t *hand = static_cast<Hand_t*>(hand_ptr);
        static uint8_t Rx_Data[1024],Rx_Data_Len;
        sensor_msgs::JointState *hand_status;
        uint8_t Flag;
        while(1)
        {
            Serial.Get_Data(Rx_Data,&Rx_Data_Len);
            hand_status = new sensor_msgs::JointState;
            Solve_Date(Rx_Data, Rx_Data_Len, hand_status, &Flag);
            if(Flag == true)
            {
                hand->pub_hand_glove_status.publish(*hand_status);
                delete hand_status;
                Flag = false;
            }
        }
    }

    static void Solve_Date(uint8_t *Rx_Data,uint8_t Rx_Data_Len, sensor_msgs::JointState *hand_status_temp, uint8_t *Flag)
    {
            //         cout << unsigned(Rx_Data_Len) << endl;
            // for(uint16_t i = 0; i < Rx_Data_Len; i++ )
            // {
            //     cout << hex << unsigned(Rx_Data[i]) << " ";
            // }
            // cout << endl;

        if((Rx_Data[0] << 8 | Rx_Data[1]) == 0x90EB)
        {
            uint8_t Temp_Data = 0;
            for(uint8_t i = 2; i < 18 - 1; i ++)
            {
                Temp_Data += Rx_Data[i];
            }
            if(Temp_Data == Rx_Data[18 - 1])
            {
                hand_status_temp->header.frame_id =  "Hand_Glove";
                hand_status_temp->header.seq = 1;
                hand_status_temp->header.stamp = ros::Time::now();
                for(uint8_t i = 0; i < 6; i++)
                {
                    hand_status_temp->position.push_back((int16_t) Rx_Data[6 + 2*i] << 8 | Rx_Data[5 + 2*i]);

                    //cout << "Joint:" << unsigned(i)  << " " << (int16_t)( Rx_Data[6 + 2*i] << 8 | Rx_Data[5 + 2*i] )<< endl;
                }
                *Flag = true;
            }
        }
    }
    static void ros_exit(int sig)
    {
        ROS_INFO("shutting down!");
        ros::shutdown();
        exit(0);
    }

public:    
    ros::NodeHandle *hand_glove_controller_node;
    ros::Publisher  pub_hand_glove_status;
    ros::Subscriber sub_hand_glove_controller;
    ros::Timer *timer;

    pthread_t threads[2];

};


#endif