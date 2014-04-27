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
#include <nav_msgs/OccupancyGrid.h>

struct Pos
{
	int x;
	int y;
};

class PosComp
{
	public:
		bool operator()( const Pos pos1, const Pos pos2 )
		{
			return ( pos1.x * 100000 + pos1.y ) < ( pos2.x * 100000 + pos2.y );
		}
};

class WorldObject
{
public:
	
	WorldObject( const Pos& pos )
	{
		AddPosition( pos );
	}

	double DistBetweenPos( const Pos& pos1, const Pos& pos2 ) const 
	{
		return sqrt( pow( ( pos1.x - pos2.x ), 2 ) + pow( ( pos1.y - pos2.y ), 2 ) );
	}

	double GetDistToObj( const Pos& pos ) const
	{
		double minDist = DistBetweenPos( positions.front(), pos );

		for( std::vector< Pos >::const_iterator it = positions.begin(); it != positions.end(); ++it )
		{
			double curDist = DistBetweenPos( *it, pos );
			if ( curDist < minDist )
				minDist = curDist;
		}

		return minDist;
	}

	Pos GetClosestPointTo( const Pos& pos ) const
	{
		double minDist = DistBetweenPos( positions.front(), pos );

		std::vector< Pos >::const_iterator minIt = positions.begin();
		for( std::vector< Pos >::const_iterator it = positions.begin(); it != positions.end(); ++it )
		{
			double curDist = DistBetweenPos( *it, pos );
			if ( curDist < minDist )
			{
				minDist = curDist;
				minIt = it;
			}
		}
		
		return *minIt;
	}

	void AddPosition( const Pos& pos )
	{
		positions.push_back( pos );
	}

	void PrintObjectPositions() const
	{
		for( std::vector< Pos >::const_iterator it = positions.begin(); it != positions.end(); ++it )
		{
			std::cout << "(" << it->x << ", " << it->y << "), ";
		}
	}

private:

	std::vector< Pos > positions;
};

class RobotDriver
{
	private:
		//! The node handle we'll be using
		ros::NodeHandle nh_;
		//! We will be publishing to the "cmd_vel" topic to issue commands
		ros::Publisher cmd_vel_pub_;
		//! We will be listening to TF transforms as well
		tf::TransformListener listener_;

		ros::Subscriber map_sub_;

		laser_geometry::LaserProjection projector;

		std::vector< WorldObject > objects;

	public:


		//! ROS node initialization
		RobotDriver(ros::NodeHandle &nh)
		{
			nh_ = nh;

			//set up the publisher for the cmd_vel topic
			cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

			map_sub_ = nh_.subscribe("/map", 1, &RobotDriver::mapCallback, this );
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
			cmd_vel_pub_.publish(base_cmd);
			rate.sleep();
		}
	}

	void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
	{
		objects.clear();

		for( int i = 0; i < msg->data.size(); ++i )
		{
			// Если пусто либо неизвестно, то не обрабатываем
			if (msg->data[i] <= 0)
				continue;

			Pos pos;
			pos.x = i % msg->info.width;
			pos.y = i / msg->info.width;

			if (objects.empty())
			{
				objects.push_back( WorldObject( pos ) );	
				continue;
			}
			
			double distToClosestObject = objects.front().GetDistToObj( pos );
			std::vector< WorldObject >::iterator closestIt = objects.begin();

			for ( std::vector< WorldObject >::iterator it = objects.begin(); it != objects.end(); ++it )
			{
				double curDist = it->GetDistToObj( pos );
				if( curDist < distToClosestObject )
				{
					closestIt = it;
					distToClosestObject = curDist;
				}
			}

			if( distToClosestObject > 15 )
			{
				objects.push_back( WorldObject( pos ) );
			}
			else
			{
				closestIt->AddPosition( pos );
			}
		}
	
		std::cout << "New Map" << std::endl;
		int i = 0;
		for ( std::vector< WorldObject >::const_iterator it = objects.begin(); it != objects.end(); ++it )
		{
			std::cout << "Object " << ++i << std::endl;
			it->PrintObjectPositions();
			std::cout << std::endl;
		}

		// estimate robot position
		listener_.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));

		tf::StampedTransform transform;
		ros::Rate rate(1.0);
		rate.sleep();

		listener_.lookupTransform( "map", "base_link", ros::Time( 0 ), transform );

		// estimate robot cell on map
		Pos robot_pose;
		robot_pose.x = (int)( ( transform.getOrigin().x() - msg->info.origin.position.x ) / msg->info.resolution );
		robot_pose.y = (int)( ( transform.getOrigin().y() - msg->info.origin.position.y ) / msg->info.resolution );

		std::cout << "Robot pose: " << robot_pose.x << ", " << robot_pose.y << std::endl;
		std::cout << "Robot angle: " << tf::getYaw( transform.getRotation() ) * 180 / 2 / M_PI << std::endl;

		i = 0;
		double min_dist = objects.front().GetDistToObj( robot_pose );
		std::vector< WorldObject >::const_iterator min_it = objects.begin();
		for ( std::vector< WorldObject >::const_iterator it = objects.begin(); it != objects.end(); ++it )
		{
			double cur_dist = it->GetDistToObj( robot_pose );
			if ( cur_dist < min_dist )
			{
				min_dist = cur_dist;
				min_it = it;
			}
		}
		
		Pos closest_pos = min_it->GetClosestPointTo( robot_pose );

		std::cout << "Closest pose: " << closest_pos.x << ", " << closest_pos.y << std::endl;

		// angle to rotate robot
		double alpha = std::atan2( closest_pos.y - robot_pose.y, closest_pos.x - robot_pose.x ) - tf::getYaw( transform.getRotation() );

		geometry_msgs::Twist base_cmd;
		
		base_cmd.linear.y = 0;
		base_cmd.linear.x = std::min( 0.5, min_dist * msg->info.resolution );
		base_cmd.angular.z = alpha;

		i = 0;

		ros::Rate new_rate( 30.0 );
		while( min_dist > 10.0 )
		{
			cmd_vel_pub_.publish(base_cmd);
			new_rate.sleep();
				
			try
			{
				listener_.lookupTransform( "map", "base_link", ros::Time( 0 ), transform );
			}
			catch (tf::TransformException ex)
			{
				ROS_ERROR("%s",ex.what());
				break;
			}

			robot_pose.x = (int)( ( transform.getOrigin().x() - msg->info.origin.position.x ) / msg->info.resolution );
			robot_pose.y = (int)( ( transform.getOrigin().y() - msg->info.origin.position.y ) / msg->info.resolution );

			min_dist = min_it->GetDistToObj( robot_pose );
			closest_pos = min_it->GetClosestPointTo( robot_pose );

			// angle to rotate robot
			alpha = std::atan2( closest_pos.y - robot_pose.y, closest_pos.x - robot_pose.x ) - tf::getYaw( transform.getRotation() );

			base_cmd.linear.x = std::min( 0.2, min_dist * msg->info.resolution );
			base_cmd.angular.z = alpha;
		}
	}
};

int main(int argc, char** argv)
{
	//init the ROS node
	ros::init(argc, argv, "robot_driver");
	ros::NodeHandle nh;

	RobotDriver driver(nh);

	ros::spin();
}
