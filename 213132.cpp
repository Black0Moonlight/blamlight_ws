
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sstream>
#include <tf/transform_broadcaster.h>

typedef union
{
    float  f;
    unsigned char u[4];
}Float4Byte;

signed short left_vel, right_vel;       //单位：rps
const float wheel_dist = 0.52;             //轮间距0.52m
const float circumference = 0.628;          //周长0.628m
const float linesNum = 1024;

serial::Serial ser;

void write_callback(const geometry_msgs::Twist& cmd_vel)
{

    left_vel = (cmd_vel.linear.x - cmd_vel.angular.z * wheel_dist / 2) / circumference;
    right_vel = (cmd_vel.linear.x + cmd_vel.angular.z * wheel_dist / 2) / circumference;

    ROS_INFO("I heard linear velocity: x-[%f],y-[%f],",cmd_vel.linear.x,cmd_vel.linear.y);
    ROS_INFO("I heard angular velocity: [%f]",cmd_vel.angular.z);
//    ser.write(msg->data);   //发送串口数据,用来控制底盘运动
}


unsigned int modbus_CRC(unsigned char *arr_buff, unsigned char len)
{
    unsigned short int crc=0xFFFF;
    unsigned char i, j, Data;
    for( j=0; j < len; j++)
    {
        crc=crc ^*arr_buff++;
        for ( i=0; i<8; i++)
        {
            if( ( crc&0x0001) >0)
            {
                crc=crc>>1;
                crc=crc^ 0xa001;
            }
            else
                crc=crc>>1;
        }
    }
    return crc;
}


int main (int argc, char** argv){
    ros::init(argc, argv, "serial_imu_node");
    ros::NodeHandle nh;
    ros::Subscriber IMU_write_pub = nh.subscribe("cmd_vel", 1000, write_callback);
    ros::Publisher imu_raw_pub = nh.advertise<sensor_msgs::Imu>("imu_data_raw", 1000);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom_raw", 500);

    Float4Byte quaternion_q0, quaternion_q1, quaternion_q2, quaternion_q3;
//    Float4Byte vx, vy;  //单位：m/s
    float delta_dist, delta_speed, delta_x, delta_y, pos_x, pos_y; //单位：m
    signed short encoder_right = 0, encoder_left = 0;
    signed short angular_velocity_x, angular_velocity_y, angular_velocity_z;
    signed short linear_acceleration_x, linear_acceleration_y, linear_acceleration_z;

    geometry_msgs::Quaternion odom_quat;
    float delta_th = 0, dt = 0;
    int count = 0;
    unsigned short int crc;
    float th = 0;

    try {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(460800);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e){
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else{
        return -1;
    }


    ros::Time current_time = ros::Time::now();
    ros::Time last_time = ros::Time::now();
    ros::Rate loop_rate(200);

    while(ros::ok()){
        //处理从串口来的Imu数据
        //串口缓存字符数
        unsigned char  data_size;
        if(data_size = ser.available()) { //ser.available(当串口没有缓存时，这个函数会一直等到有缓存才返回字符数

            unsigned char tmpdata[data_size];
            current_time = ros::Time::now();

            //打包IMU数据
            sensor_msgs::Imu imu_data_raw;
            nav_msgs::Odometry odom;

            imu_data_raw.header.stamp = ros::Time::now();
            imu_data_raw.header.seq = count;
            imu_data_raw.header.frame_id = "base_link";

            imu_data_raw.orientation_covariance = {1000000.0, 0, 0,
                                                   0, 1000000, 0,
                                                   0, 0, 0.000001};
            imu_data_raw.angular_velocity_covariance = imu_data_raw.orientation_covariance;
            imu_data_raw.linear_acceleration_covariance = {-1,0,0,0,0,0,0,0,0};

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

            ser.read(tmpdata, data_size);
            if((tmpdata[0] == 0xA5) && (tmpdata[1] == 0x5A))
            {
                crc = modbus_CRC(tmpdata, 34);
                if((crc>>8 == tmpdata[35]) && (crc|0xFF == tmpdata[34]))
                {
                    quaternion_q0.u[0] = tmpdata[2]; quaternion_q0.u[1] = tmpdata[3]; quaternion_q0.u[2] = tmpdata[4];
                    quaternion_q0.u[3] = tmpdata[5];
                    quaternion_q1.u[0] = tmpdata[6]; quaternion_q1.u[1] = tmpdata[7]; quaternion_q1.u[2] = tmpdata[8];
                    quaternion_q1.u[3] = tmpdata[9];
                    quaternion_q2.u[0] = tmpdata[10]; quaternion_q2.u[1] = tmpdata[11]; quaternion_q2.u[2] = tmpdata[12];
                    quaternion_q2.u[3] = tmpdata[13];
                    quaternion_q3.u[0] = tmpdata[14]; quaternion_q3.u[1] = tmpdata[15]; quaternion_q3.u[2] = tmpdata[16];
                    quaternion_q3.u[3] = tmpdata[17];
                    linear_acceleration_x = (tmpdata[19] << 8) | tmpdata[18];
                    linear_acceleration_y = (tmpdata[21] << 8) | tmpdata[20];
                    linear_acceleration_z = (tmpdata[23] << 8) | tmpdata[22];
                    angular_velocity_x    = (tmpdata[25] << 8) | tmpdata[24];
                    angular_velocity_y    = (tmpdata[27] << 8) | tmpdata[26];
                    angular_velocity_z    = (tmpdata[29] << 8) | tmpdata[28];
                    encoder_right         = (tmpdata[31] << 8) | tmpdata[30];
                    encoder_left          = (tmpdata[33] << 8) | tmpdata[32];
//                    vx.f = (((tmpdata[30] << 8) | tmpdata[31]) + ((tmpdata[32] << 8) | tmpdata[33]))/1024 * 0.628;
//                    vy.f = 0;
//                    ROS_INFO("linear_acceleration_x: [%d], linear_acceleration_y: [%d], linear_acceleration_z: [%d]", \
//                            linear_acceleration_x, linear_acceleration_y, linear_acceleration_z);
//                    ROS_INFO("angular_velocity_x: [%d], angular_velocity_y: [%d], angular_velocity_z: [%d]", \
                            angular_velocity_x, angular_velocity_y, angular_velocity_z);
                    imu_data_raw.orientation.x = quaternion_q0.f;
                    imu_data_raw.orientation.y = quaternion_q1.f;
                    imu_data_raw.orientation.z = quaternion_q2.f;
                    imu_data_raw.orientation.w = quaternion_q3.f;
                    imu_data_raw.angular_velocity.x = ((float)(angular_velocity_x)/131.0)*3.1415/180.0;
                    imu_data_raw.angular_velocity.y = ((float)(angular_velocity_y)/131.0)*3.1415/180.0;
                    imu_data_raw.angular_velocity.z = ((float)(angular_velocity_z)/131.0)*3.1415/180.0;
                    imu_data_raw.linear_acceleration.x = ((float)(linear_acceleration_x)/16386.0)*9.80665;
                    imu_data_raw.linear_acceleration.y = ((float)(linear_acceleration_y)/16386.0)*9.80665;
                    imu_data_raw.linear_acceleration.z = ((float)(linear_acceleration_z)/16386.0)*9.80665;

//                    encoder_right = 50;
//                    encoder_left = 50;

                    dt = (current_time - last_time).toSec();
                    delta_th = imu_data_raw.angular_velocity.z  * dt;
                    delta_dist = (encoder_right + encoder_left)/linesNum * circumference / 2;
//                    delta_speed = delta_dist / dt;
                    delta_x = cos(delta_th) * delta_dist;
                    delta_y = -sin(delta_th) * delta_dist;
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
                    odom.twist.twist.angular.z = imu_data_raw.angular_velocity.z;

                    count++;
                    imu_raw_pub.publish(imu_data_raw); //imu_raw_pub 节点发布消息至imu_data_raw topic
                    odom_pub.publish(odom);
                }
            }
            last_time = current_time;
            ros::spinOnce();
        }
    }
}
