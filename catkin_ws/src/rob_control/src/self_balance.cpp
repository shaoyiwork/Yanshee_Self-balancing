
/*******************************************************************
 *Copyright (c) <2019>, <qibiao huang>　
 *All rights reserved.
 * 
 *Aiming to using PID controller to handle the imu data and achieve the self-balance of the robot
 *******************************************************************/


#include <ros/ros.h>
#include <iostream>
//the imu data are defined in the including file 
#include <ubt_msgs/gyro_report.h>
//17 servo data message 
#include <ubt_msgs/angles_set.h>

#define P_CONTROL
//#define PI_CONTROL
//#define PID_CONTROL


class SubcribeAndPublish
{
public:
	SubcribeAndPublish()
	{
		//publish topic
		 servo_data_pub = n.advertise<ubt_msgs::angles_set>("hal_angles_set", 1000);
		 sub = n.subscribe("hal_gyro_report", 1000, &SubcribeAndPublish::dataCallback,this);		
	}

private:
	ros::NodeHandle n;	
	ros::Subscriber sub;
	ros::Publisher servo_data_pub;
	ubt_msgs::gyro_report pre_angle;
	ubt_msgs::gyro_report angle;
        ubt_msgs::gyro_report last_angle;
	int count=0;

void dataCallback(const ubt_msgs::gyro_report::ConstPtr& msg)
{
	//ubt_msgs::gyro_report pre_angle;
	pre_angle.euler_data.resize(3);
	pre_angle.euler_data[0]=91.95;
	pre_angle.euler_data[1]=1.94;
	pre_angle.euler_data[2]=4.66;
        
       //ubt_msgs::gyro_report last_angle;
	last_angle.euler_data.resize(3);
	last_angle.euler_data[0]=91.95;
	last_angle.euler_data[1]=1.94;
	last_angle.euler_data[2]=4.66;

	//ubt_msgs::gyro_report angle;
	angle.euler_data.resize(3);

	angle.euler_data[0]=msg->euler_data[0];
	angle.euler_data[1]=msg->euler_data[1];
	angle.euler_data[2]=msg->euler_data[2];

	ubt_msgs::angles_set servo_data; //send the servo data to Raspberry Pi

		servo_data.angles.resize(17);
		//right arm
		servo_data.angles[0]=int(90*2048/180);
		servo_data.angles[1]=int(90*2048/180);//130
		servo_data.angles[2]=int(90*2048/180);//179
		//left arm
		servo_data.angles[3]=int(90*2048/180);
		servo_data.angles[4]=int(90*2048/180);//40
		servo_data.angles[5]=int(90*2048/180);//15

		servo_data.angles[6]=int(90*2048/180);//90
		servo_data.angles[7]=int(60*2048/180);//60
		servo_data.angles[8]=int(76*2048/180);//76
		servo_data.angles[9]=int(110*2048/180);//110
		servo_data.angles[10]=int(90*2048/180);//90

		servo_data.angles[11]=int(90*2048/180);//90
		servo_data.angles[12]=int(120*2048/180);//120
		servo_data.angles[13]=int(104*2048/180);//104
		servo_data.angles[14]=int(70*2048/180);//70
		servo_data.angles[15]=int(90*2048/180);//90
		servo_data.angles[16]=int(90*2048/180);//90

	//PID control
	float Data_right_servo=0,Data_left_servo=0;
	// kp ki kd 
	float kp=0.8,ki=0.015,kd=0.2,error=0,error_integral=0,error_last=0;
#ifdef P_CONTROL
	error=(angle.euler_data[0]-pre_angle.euler_data[0]);
        if (last_angle.euler_data[0] - angle.euler_data[0] >5|| last_angle.euler_data[0] - angle.euler_data[0] <-5){
        //kp control
        Data_right_servo=-kp*error+110;
	Data_left_servo=kp*error+70;
	ROS_INFO("error data: %f ", error);
	ROS_INFO("right leg servo data: %f ", Data_right_servo);
	ROS_INFO("left leg servo data: %f ", Data_left_servo);
	servo_data.angles[9]=int(Data_right_servo*2048/180);		
	servo_data.angles[14]=int(Data_left_servo*2048/180);
        ROS_INFO("transform angle data: %d %d", servo_data.angles[10],servo_data.angles[15]);
	servo_data_pub.publish(servo_data);
	ROS_INFO("receive pre euler angle data: %f %f %f", pre_angle.euler_data[0],pre_angle.euler_data[1],pre_angle.euler_data[2]);	
	ROS_INFO("receive euler angle data: %f %f %f", angle.euler_data[0],angle.euler_data[1],angle.euler_data[2]);
        last_angle.euler_data[0] = angle.euler_data[0];
        }
#endif
#ifdef PI_CONTROL
	error=(angle.euler_data[0]-pre_angle.euler_data[0]);
	error_integral+=error;
        if (last_angle.euler_data[0] - angle.euler_data[0] >5|| last_angle.euler_data[0] - angle.euler_data[0] <-5){
	//kp+ki control
	Data_right_servo=kp*error+ki*error_integral+110;
	Data_left_servo=-kp*error+ki*error_integral+70;
	
	ROS_INFO("error data: %f ", error);
	ROS_INFO("right leg servo data: %f ", Data_right_servo);
	ROS_INFO("left leg servo data: %f ", Data_left_servo);
	servo_data.angles[9]=int(Data_right_servo*2048/180);		
	servo_data.angles[14]=int(Data_left_servo*2048/180);
        ROS_INFO("transform angle data: %d %d", servo_data.angles[9],servo_data.angles[14]);
	servo_data_pub.publish(servo_data);
	ROS_INFO("receive pre euler angle data: %f %f %f", pre_angle.euler_data[0],pre_angle.euler_data[1],pre_angle.euler_data[2]);	
	ROS_INFO("receive euler angle data: %f %f %f", angle.euler_data[0],angle.euler_data[1],angle.euler_data[2]);
        last_angle.euler_data[0] = angle.euler_data[0];
        }
#endif
#ifdef PID_CONTROL
	error=angle.euler_data[0]-pre_angle.euler_data[0];
	error_integral+=error;
        if (last_angle.euler_data[0] - angle.euler_data[0] >2|| last_angle.euler_data[0] - angle.euler_data[0] <-2){
	//kp+ki+kd control
	Data_right_servo=kp*error+ki*error_integral+kd*(error-error_last)+110;
	Data_left_servo=-kp*error+ki*error_integral+kd*(error-error_last)+70;

	error_last=error;
	ROS_INFO("error data: %f ", error);
	ROS_INFO("right leg servo data: %f ", Data_right_servo);
	ROS_INFO("left leg servo data: %f ", Data_left_servo);
	servo_data.angles[9]=int(Data_right_servo*2048/180);		
	servo_data.angles[14]=int(Data_left_servo*2048/180);
	ROS_INFO("transform angle data: %d %d", servo_data.angles[9],servo_data.angles[14]);
	servo_data_pub.publish(servo_data);
	ROS_INFO("receive pre euler angle data: %f %f %f", pre_angle.euler_data[0],pre_angle.euler_data[1],pre_angle.euler_data[2]);	
	ROS_INFO("receive euler angle data: %f %f %f", angle.euler_data[0],angle.euler_data[1],angle.euler_data[2]);
         last_angle.euler_data[0] = angle.euler_data[0];
    }
#endif
}

};

int main(int argc, char **argv)
{
	//init ROS
 	ros::init(argc, argv, "self_balance_control");
	ROS_INFO("start to receive data");
	SubcribeAndPublish SAPObject;
        ros::spin();
	return 0;
}
