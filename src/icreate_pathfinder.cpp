#include <iostream>
#include <cmath>
#include <vector>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>

#include "world_object.h"

class RobotDriver
{
	private:
		//! The node handle we'll be using
		ros::NodeHandle nh_;
		//! We will be publishing to the "cmd_vel" topic to issue commands
		ros::Publisher cmd_vel_pub_;
		//! We will be listening to TF transforms as well
		tf::TransformListener listener_;

		ros::Subscriber scan_sub_;

		boost::mutex mutex_;

		laser_geometry::LaserProjection projector;
		tf::TransformListener listener;
	public:
		//! ROS node initialization
		RobotDriver(ros::NodeHandle &nh)
		{
			nh_ = nh;

			//set up the publisher for the cmd_vel topic
			cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 1);

      		scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("scan", 1, boost::bind( &RobotDriver::scanCallback, this, _1 ) );
		}

		//! Drive forward a specified distance based on odometry information
		bool driveForwardOdom(double distance)
		{
			//wait for the listener to get the first message
			listener_.waitForTransform("base_footprint", "odom", 
					ros::Time(0), ros::Duration(1.0));

			//we will record transforms here
			tf::StampedTransform start_transform;
			tf::StampedTransform current_transform;

			//record the starting transform from the odometry to the base frame
			listener_.lookupTransform("base_footprint", "odom", 
					ros::Time(0), start_transform);

			//we will be sending commands of type "twist"
			geometry_msgs::Twist base_cmd;
			//the command will be to go forward at 0.25 m/s
			base_cmd.linear.y = base_cmd.angular.z = 0;
			base_cmd.linear.x = 0.25;

			ros::Rate rate(1.0);
			bool done = false;
			while (!done && nh_.ok())
			{
				//send the drive command
				cmd_vel_pub_.publish(base_cmd);
				rate.sleep();
				//get the current transform
				try
				{
					listener_.lookupTransform("base_footprint", "odom", 
							ros::Time(0), current_transform);
				}
				catch (tf::TransformException ex)
				{
					ROS_ERROR("%s",ex.what());
					break;
				}
				//see how far we've traveled
				tf::Transform relative_transform = 
					start_transform.inverse() * current_transform;
				double dist_moved = relative_transform.getOrigin().length();

				if(dist_moved > distance) done = true;
			}
			if (done) return true;
			return false;
		}

		bool turnOdom(bool clockwise, double radians)
		{
			while(radians < 0) radians += 2*M_PI;
			while(radians > 2*M_PI) radians -= 2*M_PI;

			//wait for the listener to get the first message
			listener_.waitForTransform("base_footprint", "odom", 
					ros::Time(0), ros::Duration(1.0));

			//we will record transforms here
			tf::StampedTransform start_transform;
			tf::StampedTransform current_transform;

			//record the starting transform from the odometry to the base frame
			listener_.lookupTransform("base_footprint", "odom", 
					ros::Time(0), start_transform);

			//we will be sending commands of type "twist"
			geometry_msgs::Twist base_cmd;
			//the command will be to turn at 0.75 rad/s
			base_cmd.linear.x = base_cmd.linear.y = 0.0;
			base_cmd.angular.z = 0.25;
			if (clockwise) base_cmd.angular.z = -base_cmd.angular.z;

			//the axis we want to be rotating by
			tf::Vector3 desired_turn_axis(0,0,1);
			if (!clockwise) desired_turn_axis = -desired_turn_axis;

			ros::Rate rate(1.0);
			bool done = false;
			while (!done && nh_.ok())
			{
				//send the drive command
				cmd_vel_pub_.publish(base_cmd);
				rate.sleep();
				//get the current transform
				try
				{
					listener_.lookupTransform("base_footprint", "odom", 
							ros::Time(0), current_transform);
				}
				catch (tf::TransformException ex)
				{
					ROS_ERROR("%s",ex.what());
					break;
				}
				tf::Transform relative_transform = 
					start_transform.inverse() * current_transform;
				tf::Vector3 actual_turn_axis = 
					relative_transform.getRotation().getAxis();
				double angle_turned = relative_transform.getRotation().getAngle();
				if ( fabs(angle_turned) < 1.0e-2) continue;

				if ( actual_turn_axis.dot( desired_turn_axis ) < 0 ) 
					angle_turned = 2 * M_PI - angle_turned;

				if (angle_turned > radians) done = true;
			}
			if (done) return true;
			return false;
		}

	void MoveCyclically( double radius, double velocity, int times )
	{
		geometry_msgs::Twist base_cmd;
		
		base_cmd.linear.y = 0;
		base_cmd.linear.x = velocity;
		base_cmd.angular.z = velocity / radius;

		double rateFrequency = 10.0;

		ros::Rate rate( rateFrequency );

		times = times * ( int )( rateFrequency * 2 * M_PI * radius / velocity );
		while ( --times > 0 )
		{
			boost::mutex::scoped_lock lock( mutex_ );
			cmd_vel_pub_.publish(base_cmd);
			rate.sleep();
			lock.unlock();
		}
	}

	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in) 
    {
		sensor_msgs::PointCloud cloud;

		std::cout << scan_in->header.frame_id << std::endl;
		if(!listener_.waitForTransform(
				scan_in->header.frame_id,
				"/base_link",
				scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
				ros::Duration(1.0)))
		{
			return;
		}

		projector.transformLaserScanToPointCloud("base_link", *scan_in, cloud, listener);	

		std::vector< WorldObject > objects;

		geometry_msgs::Point32 point;

		for (int i = 0; i < cloud.points.size(); ++i)
		{
			double min_dist = objects.empty() ? 0 : objects.front().GetDistanceToObject( cloud.points[i] );

			std::vector< WorldObject >::iterator it_min = objects.begin();
			for ( std::vector< WorldObject >::iterator it = objects.begin(); it != objects.end(); ++it )
			{
				double cur_dist = it->GetDistanceToObject( cloud.points[i] );
				if( cur_dist < min_dist )
				{
					min_dist = cur_dist;
					it_min = it;
				}
			}

			if ( objects.empty() || min_dist > CRITICAL_DISTANCE_BETWEEN_OBJECTS )
			{
				WorldObject object;
				object.AddPoint( cloud.points[i] );
				objects.push_back( object );
			}
			else
			{
				it_min->AddPoint( cloud.points[i] );
			}
		}

		// Get closest object

		double dist_to_closest_obj = 10000000;
		std::vector< WorldObject >::iterator closest_it = objects.begin();
		geometry_msgs::Point32 closest_point;
		closest_point.x = 0;
		closest_point.y = 0;
		for ( std::vector< WorldObject >::iterator it = objects.begin(); it != objects.end(); ++it )
		{
			geometry_msgs::Point32 center_point = it->GetObjectCenter();
			double cur_dist = sqrt( pow( center_point.x, 2 ) + pow( center_point.y, 2 ) );

			if( cur_dist < 2 * CRITICAL_DISTANCE_BETWEEN_OBJECTS )
				continue;

			if( cur_dist < dist_to_closest_obj )
			{
				dist_to_closest_obj = cur_dist;
				closest_point = center_point;
			}
		}

		std::cout << "x: " << closest_point.x << ", y: " << closest_point.y << std::endl;
		
		if( closest_point.x == 0 || fabs( closest_point.y/closest_point.x ) > 0.1 )
		{
			turnOdom( closest_point.y < 0, M_PI / 100 );
			std::cout << "turn" << std::endl;
		}
		else
		{
			driveForwardOdom( 0.2 );
//			driveForwardOdom( closest_point.x );
		}
    }

	void GetDistanceToClosest()
	{
		//wait for the listener to get the first message
		listener_.waitForTransform("base_footprint", "odom", 
				ros::Time(0), ros::Duration(1.0));

		//we will record transforms here
		tf::StampedTransform start_transform;
		tf::StampedTransform current_transform;

		//record the starting transform from the odometry to the base frame
		listener_.lookupTransform("base_footprint", "odom", 
				ros::Time(0), start_transform);

		ros::Rate rate( 10 );
		
		while (true)
		{
			boost::mutex::scoped_lock lock( mutex_ );

			//see how far we've traveled
			try
			{
					listener_.lookupTransform("base_footprint", "odom", 
									ros::Time(0), current_transform);
			}
			catch (tf::TransformException ex)
			{
					ROS_ERROR("%s",ex.what());
					break;
			}
			tf::Transform relative_transform = 
					start_transform.inverse() * current_transform;
			
			double angle_turned = relative_transform.getRotation().getAngle();
			double dist_moved = relative_transform.getOrigin().length();

			lock.unlock();
		}
	}
};

int main(int argc, char** argv)
{
	//init the ROS node
	ros::init(argc, argv, "robot_driver");
	ros::NodeHandle nh;

	RobotDriver driver(nh);
//	sleep(5);
	ros::spin();

//	driver.MoveCyclically( 0.5, 1, 10 );
//	boost::thread coordThread( boost::bind( &RobotDriver::GetRobotCoords, &driver ) );
//	boost::thread movingThread( boost::bind( &RobotDriver::MoveCyclically, &driver, 0.5, 1, 10 ) );
	
//	coordThread.join();
//	movingThread.join();
}
