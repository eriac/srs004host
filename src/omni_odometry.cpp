#include "ros/ros.h"
  
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_broadcaster.h"

#include "math.h"
#include <sstream>
#include <string>

float wheel_odometry[3]={0};
void odometry0_callback(const std_msgs::Float32& float_msg){
	wheel_odometry[0]=float_msg.data;
}
void odometry1_callback(const std_msgs::Float32& float_msg){
	wheel_odometry[1]=float_msg.data;
}
void odometry2_callback(const std_msgs::Float32& float_msg){
	wheel_odometry[2]=float_msg.data;
}


float wheel_base=0.100;
float wheel_radius=0.20;

void wheel_invert(float *out, float *in){
	float wheel_base=0.0972;
	float wheel_radius=0.019;
	float lv=1.0/wheel_radius;
	float av=wheel_base/wheel_radius;
	float r3=sqrt(3);
	out[0]=+(1.0/r3/lv)*in[0] +0.0       *in[1] -(1.0/r3/lv)*in[2];
	out[1]=-(1.0/3/lv) *in[0] +(2.0/3/lv)*in[1] -(1.0/3/lv) *in[2];
	out[2]=-(1.0/3/av) *in[0] -(1.0/3/av)*in[1] -(1.0/3/av) *in[2];
}
void set_robot(float *position, float *direction){
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(position[0], position[1], position[2]) );
	tf::Quaternion q;
	q.setRPY(direction[0], direction[1], direction[2]);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "omni_driver");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	//publish
	ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1000);


	//Subscribe
	ros::Subscriber odometry0 = n.subscribe("odometry0", 10, odometry0_callback); 
	ros::Subscriber odometry1 = n.subscribe("odometry1", 10, odometry1_callback); 
	ros::Subscriber odometry2 = n.subscribe("odometry2", 10, odometry2_callback); 

	pn.getParam("wheel_base",    wheel_base);
	pn.getParam("wheel_radius",  wheel_radius);
	//for(int i=0;i<3;i++)wheel_normal[i]=wheel[i]-M_PI/2;

	float dt=1.0/20;
	ros::Rate loop_rate(20); 
	while (ros::ok()){
		//publish joint states
		
		sensor_msgs::JointState js0;
		js0.header.stamp = ros::Time::now();
		js0.name.resize(3);
		js0.name[0]="wheel0_housing_joint";
		js0.name[1]="wheel1_housing_joint";
		js0.name[2]="wheel2_housing_joint";
		js0.position.resize(3);
		js0.position[0]=wheel_odometry[0];
		js0.position[1]=wheel_odometry[1];
		js0.position[2]=wheel_odometry[2];
		joint_pub.publish(js0);


		//publish base_link tf
		float roll[3]={0};
		static float wheel_odometry_last[3]={0};
		for(int i=0;i<3;i++){
			roll[i]=wheel_odometry[i]-wheel_odometry_last[i];
			wheel_odometry_last[i]=wheel_odometry[i];
		}
		float move[3]={0};
		wheel_invert(move,roll);

		static float position[3]={0};
		static float direction[3]={0};
		position[0]+=cos(direction[2])*move[0]-sin(direction[2])*move[1];
		position[1]+=sin(direction[2])*move[0]+cos(direction[2])*move[1];
		position[2]=0.019;
		direction[0]=0.0;
		direction[1]=0.0;
		direction[2]+=move[2];
		set_robot(position,direction);
		
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

