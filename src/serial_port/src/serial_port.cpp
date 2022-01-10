#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 

#include <std_msgs/String.h> 
#include <std_msgs/Empty.h> 
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <iostream>

#include <nav_msgs/Odometry.h>
#include <sstream>
#include <tf/transform_broadcaster.h>

//创建一个serial类
serial::Serial ser;
serial::Serial ser_imu;

signed short left_vel, right_vel;       //单位：rps
const float wheel_dist = 0.52;             //轮间距0.52m
const float circumference = 0.628;          //周长0.628m
const float linesNum = 1024;

long tick=0;
uint8_t rcNum=0;
uint8_t rxbuffer[20]={0};
uint8_t txbuffer[20]={0};
uint8_t temp[1]={0};
uint8_t rcFlag=0;
uint8_t rdFlag=0;

uint8_t rcNum1=0;
uint8_t rxbuffer_imu[20]={0};
uint8_t temp1[1]={0};
uint8_t rcFlag1=0;
uint8_t rdFlag1=0;

float acc[3],gyro[3],rpy[3],Q4[4],temperature;

float speed[2];

//回调函数 
void write_callback(const geometry_msgs::Twist& msg) 
{ 
    ROS_INFO("Linear Components:[%f,%f,%f]",msg.linear.x,msg.linear.y,msg.linear.z);
    ROS_INFO("Angular Components:[%f,%f,%f]",msg.angular.x,msg.angular.y,msg.angular.z);

    int lx,ly,az;
    lx = msg.linear.x*1000;
    ly = msg.linear.y*1000;

    az = msg.angular.z*1000;

    txbuffer[0] = 0x78;//帧头
    txbuffer[1] = lx>>8;
    txbuffer[2] = lx;
    txbuffer[3] = ly>>8;
    txbuffer[4] = lx;
    txbuffer[5] = az>>8;
    txbuffer[6] = az;
    txbuffer[7] = 0;
    txbuffer[8] += 1;
    txbuffer[9] = 0x68;//帧尾
    //ser.write(txbuffer, 10);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_port");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle nh;

    //订阅主题，并配置回调函数 
    ros::Subscriber write_32_sub = nh.subscribe("/cmd_vel", 1000, write_callback); 
    //发布主题 
    //ros::Publisher read_32_pub = nh.advertise<std_msgs::String>("read", 1000); 

    ros::Publisher IMU_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 100);

    //ros::Publisher imu_raw_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 1000);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 500);
    
    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);

    //设置要打开的串口名称
    ser.setPort("/dev/ttyTHS0");
    //设置串口通信的波特率
    ser.setBaudrate(115200);
    //串口设置timeout
    ser.setTimeout(to);

    //设置要打开的串口名称
    ser_imu.setPort("/dev/ttyUSB1");
    //设置串口通信的波特率
    ser_imu.setBaudrate(115200);
    //串口设置timeout
    ser_imu.setTimeout(to);
 
    try
    {
        //打开串口
        ser.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    
    //判断串口是否打开成功
    if(ser.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyTHS0 is opened.");
    }
    else
    {
        return -1;
    }

    try
    {
        //打开串口
        ser_imu.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    
    //判断串口是否打开成功
    if(ser_imu.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB1 is opened.");
    }
    else
    {
        return -1;
    }
    
    float delta_dist, delta_speed, delta_x, delta_y, pos_x, pos_y; //单位：m


    static tf::TransformBroadcaster odom_broadcaster;//定义tf对象
    geometry_msgs::TransformStamped odom_trans;//创建一个tf发布需要使用的TransformStamped类型消息
    nav_msgs::Odometry odom;//定义里程计对象
    geometry_msgs::Quaternion odom_quat; //四元数变量

    float delta_th = 0, dt = 0;
    int count = 0;
    unsigned short int crc;
    float th = 0;

    float rpyzl;

    //std_msgs::String result; 
    ros::Time current_time = ros::Time::now();
    ros::Time last_time = ros::Time::now();

    ros::Rate loop_rate(200);
    while(ros::ok())
    {
        tick++;
        if(tick>=20000)tick=0;

        current_time = ros::Time::now();
        nav_msgs::Odometry odom;
        sensor_msgs::Imu imu_data;

        imu_data.header.stamp = ros::Time::now();
        imu_data.header.seq = count;
        imu_data.header.frame_id = "base_link";

        imu_data.orientation_covariance = {1000000.0, 0, 0,
                                               0, 1000000, 0,
                                               0, 0, 0.000001};
        imu_data.angular_velocity_covariance = imu_data.orientation_covariance;
        imu_data.linear_acceleration_covariance = {-1,0,0,0,0,0,0,0,0};

        odom.header.stamp = ros::Time::now();
        odom.header.seq = count;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.covariance = {0.001, 0, 0, 0, 0, 0,
                                0, 0.001, 0, 0, 0, 0,
                                0, 0, 1000000, 0, 0, 0,
                                0, 0, 0, 1000000, 0, 0,
                                0, 0, 0, 0, 1000000, 0,
                                0, 0, 0, 0, 0, 1000};
        odom.twist.covariance = odom.pose.covariance;

        //获取缓冲区内的字节数
        if(ser.available())
        {
            //读出数据
            ser.read(temp, 1);
            if((temp[0]==0x78)&&(rdFlag==0))
            {
                rdFlag = 1;
                rcNum = 0;
            }
            if(rdFlag==1)
            {
                rxbuffer[rcNum] = temp[0];
                rcNum++;
            }
            if(rcNum>=10)
            {
                rcNum = 0;
                rcFlag = 1;
                rdFlag = 0;
            }
            if(rcFlag==1)
            {
                rcFlag=0;
                speed[0] = ((short)(rxbuffer[1] << 8 | rxbuffer[2]))*1.0; 
                speed[1] = ((short)(rxbuffer[3] << 8 | rxbuffer[4]))*1.0; 
                ROS_INFO("robot speed: %f, %f",speed[0],speed[1]);
//                std::cout <<"stm ";
//                for(int i=0; i<10; i++)
//                {
//                    //16进制的方式打印到屏幕
//                    std::cout << std::hex << (rxbuffer[i] & 0xff) << " ";
//                }
//                std::cout << std::endl;
                //ROS_INFO_STREAM("Read stm32"); 
            }
            //result.data = ser.read(ser.available());
        }

        if(ser_imu.available())
        {
            //读出数据
            ser_imu.read(temp1, 1);
            if((temp1[0]==0x55)&&(rdFlag1==0))
            {
                rdFlag1 = 1;
                rcNum1 = 0;
            }
            if(rdFlag1==1)
            {
                rxbuffer_imu[rcNum1] = temp1[0];
                rcNum1++;
            }
            if(rcNum1>=11)
            {
                rcNum1 = 0;
                rcFlag1 = 1;
                rdFlag1 = 0;
            }
            if(rcFlag1==1)
            {
                rcFlag1=0;
                switch (rxbuffer_imu[1])
                {
                case 0x51: //标识这个包是加速度包
                    acc[0] = ((short)(rxbuffer_imu[3] << 8 | rxbuffer_imu[2])) / 32768.0 * 16; //X轴加速度
                    acc[1] = ((short)(rxbuffer_imu[5] << 8 | rxbuffer_imu[4])) / 32768.0 * 16; //Y轴加速度
                    acc[2] = ((short)(rxbuffer_imu[7] << 8 | rxbuffer_imu[6])) / 32768.0 * 16; //Z轴加速度
                    temperature   = ((short)(rxbuffer_imu[9] << 8 | rxbuffer_imu[8])) / 100.0; //温度
                    break;
                case 0x52: //标识这个包是角速度包
                    gyro[0] = ((short)(rxbuffer_imu[3] << 8 | rxbuffer_imu[2])) / 32768.0 * 2000; //X轴角速度
                    gyro[1] = ((short)(rxbuffer_imu[5] << 8 | rxbuffer_imu[4])) / 32768.0 * 2000; //Y轴角速度
                    gyro[2] = ((short)(rxbuffer_imu[7] << 8 | rxbuffer_imu[6])) / 32768.0 * 2000; //Z轴角速度
                    temperature    = ((short)(rxbuffer_imu[9] << 8 | rxbuffer_imu[8])) / 100.0; //温度
                    break;
                case 0x53: //标识这个包是角度包
                    rpy[0] = ((short)(rxbuffer_imu[3] << 8 | rxbuffer_imu[2])) / 32768.0 * 180; //X轴滚转角（x 轴）
                    rpy[1] = ((short)(rxbuffer_imu[5] << 8 | rxbuffer_imu[4])) / 32768.0 * 180; //Y轴俯仰角（y 轴）
                    rpy[2] = ((short)(rxbuffer_imu[7] << 8 | rxbuffer_imu[6])) / 32768.0 * 180; //Z轴偏航角（z 轴）
                    temperature   = ((short)(rxbuffer_imu[9] << 8 | rxbuffer_imu[8])) / 100.0; //温度
                    break;
                case 0x59: //标识这个包是
                    Q4[0] = ((short)(rxbuffer_imu[3] << 8 | rxbuffer_imu[2])) / 32768.0; 
                    Q4[1] = ((short)(rxbuffer_imu[5] << 8 | rxbuffer_imu[4])) / 32768.0;
                    Q4[2] = ((short)(rxbuffer_imu[7] << 8 | rxbuffer_imu[6])) / 32768.0;
                    Q4[3] = ((short)(rxbuffer_imu[9] << 8 | rxbuffer_imu[8])) / 32768.0;
                    break;
                }
                ROS_INFO_STREAM("Read imu"); 
                switch (rxbuffer_imu[1])
                {
                    case 0x51:
                    {
                        //线加速度
                        imu_data.linear_acceleration.x = acc[0]; 
                        imu_data.linear_acceleration.y = acc[1];
                        imu_data.linear_acceleration.z = acc[2];  
                        break;                      
                    }
                    case 0x52:
                    {                
                        //角速度
                        imu_data.angular_velocity.x = gyro[0]; 
                        imu_data.angular_velocity.y = gyro[1]; 
                        imu_data.angular_velocity.z = gyro[2];
                        break;    
                    }
                    case 0x59:
                    {                
                        //四元数
                        imu_data.orientation.x = Q4[0];
                        imu_data.orientation.y = Q4[1];
                        imu_data.orientation.z = Q4[2];
                        imu_data.orientation.w = Q4[3];
                        break;    
                    }
                }

                dt = (current_time - last_time).toSec();
                delta_th = imu_data.angular_velocity.z  * dt;
                //rpyzl = rpy[2];
                delta_dist = (speed[0] + speed[1])/linesNum * circumference / 2;
        //                    delta_speed = delta_dist / dt;
                delta_x = cos(delta_th) * delta_dist;
                delta_y = -sin(delta_th) * delta_dist;
                //th = rpy[2];// 
                th += delta_th;
                pos_x += (cos(th) * delta_x - sin(th) * delta_y);
                pos_y += (sin(th) * delta_x + cos(th) * delta_y);
                odom_quat = tf::createQuaternionMsgFromYaw(th);

                ROS_INFO("th: %f, pos_x: %f, pos_y: %f", th, pos_x, pos_y);
                odom.pose.pose.position.x = pos_x;
                odom.pose.pose.position.y = pos_y;
                odom.pose.pose.position.z = 0;
                odom.pose.pose.orientation = odom_quat;
                odom.twist.twist.linear.x = 0;
                odom.twist.twist.linear.y = 0;
                odom.twist.twist.angular.z = imu_data.angular_velocity.z;

            }
            //result.data = ser.read(ser.available());
        }
        //把数据发送回去
        ser.write(txbuffer, 10);

        count++;

        IMU_pub.publish(imu_data);
        odom_pub.publish(odom);
        //处理ROS的信息，比如订阅消息,并调用回调函数 
        ros::spinOnce(); 
        loop_rate.sleep(); 
    }
    
    //关闭串口
    ser.close();
    ser_imu.close();
 
    return 0;
}