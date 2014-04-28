#include <iostream>
#include <cmath>
#include <vector>
#include <set>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/OccupancyGrid.h>

#define MIN_DIST 10.0

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
		visited = false;
	}

	double DistBetweenPos( const Pos& pos1, const Pos& pos2 ) const 
	{
		return sqrt( pow( ( pos1.x - pos2.x ), 2 ) + pow( ( pos1.y - pos2.y ), 2 ) );
	}

	static double DistBetweenPosStatic( const Pos& pos1, const Pos& pos2 )
	{
		return sqrt( pow( ( pos1.x - pos2.x ), 2 ) + pow( ( pos1.y - pos2.y ), 2 ) );
	}


	double GetDistToObj( const Pos& pos ) const
	{
		double minDist = DistBetweenPos( *positions.begin(), pos );

		for( std::set< Pos >::const_iterator it = positions.begin(); it != positions.end(); ++it )
		{
			double curDist = DistBetweenPos( *it, pos );
			if ( curDist < minDist )
				minDist = curDist;
		}

		return minDist;
	}

	Pos GetClosestPointTo( const Pos& pos ) const
	{
		double minDist = DistBetweenPos( *positions.begin(), pos );

		std::set< Pos >::const_iterator minIt = positions.begin();
		for( std::set< Pos >::const_iterator it = positions.begin(); it != positions.end(); ++it )
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
		positions.insert( pos );
	}

	void RemovePosition( const Pos& pos )
	{
		std::set< Pos >::iterator it = positions.find( pos );

		if (it != positions.end())
			positions.erase( it );
	}

	void PrintObjectPositions() const
	{
		for( std::set< Pos >::const_iterator it = positions.begin(); it != positions.end(); ++it )
		{
			std::cout << "(" << it->x << ", " << it->y << "), ";
		}
	}
	
	void SetVisited() { visited = true; }
	bool isVisited() const { return visited; }

private:

	std::set< Pos, PosComp > positions;

	bool visited;
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

	std::set< Pos, PosComp > posCache;
	
	std::vector< Pos > pathPlan;

public:

	//! ROS node initialization
	RobotDriver(ros::NodeHandle &nh)
	{
		nh_ = nh;

		//set up the publisher for the cmd_vel topic
		cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

		map_sub_ = nh_.subscribe("/map", 1, &RobotDriver::mapCallback, this );
	}


	void ProcessMapPoints( const nav_msgs::OccupancyGrid::ConstPtr& msg )
	{
		for( int i = 0; i < msg->data.size(); ++i )
		{
			// Если пусто либо неизвестно, то не обрабатываем
			if (msg->data[i] < 0)
				continue;

			Pos pos;
			pos.x = i % msg->info.width;
			pos.y = i / msg->info.width;

			if (msg->data[i] == 0)
			{
				std::set< Pos >::iterator it = posCache.find( pos );
				if( it == posCache.end() )
					continue;

				posCache.erase( it );
			}
			else
			{
				if ( posCache.find( pos ) != posCache.end() )
					continue;
				
				posCache.insert( pos );
			}

			if (objects.empty() && msg->data[i] > 0)
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

			if( distToClosestObject > MIN_DIST && msg->data[i] > 0 )
			{
				objects.push_back( WorldObject( pos ) );
			}
			else if( msg->data[i] > 0 )
			{
				closestIt->AddPosition( pos );
			}
			else if( msg->data[i] == 0 )
			{
				closestIt->RemovePosition( pos );
			}
		}
	}

	void PathPlanning( const Pos & robot_pose )
	{
		double min_dist = 10000000;
		std::vector< WorldObject >::iterator min_it = objects.begin();

		bool allVisited = true;
		for ( std::vector< WorldObject >::iterator it = objects.begin(); it != objects.end(); ++it )
		{
			if ( it->isVisited() )
				continue;

			allVisited = false;

			double cur_dist = it->GetDistToObj( robot_pose );
			if ( cur_dist <= MIN_DIST )
			{
				it->SetVisited();
				continue;
			}

			if ( cur_dist < min_dist )
			{
				min_dist = cur_dist;
				min_it = it;
			}
		}

		if ( allVisited )
			return;
		
		Pos closest_pos = min_it->GetClosestPointTo( robot_pose );

		// calculate closest optimal point
		Pos target_pos;
		target_pos.x = closest_pos.x - MIN_DIST * cos( atan2( closest_pos.y - robot_pose.y, closest_pos.x - robot_pose.x ) );
		target_pos.y = closest_pos.y - MIN_DIST * sin( atan2( closest_pos.y - robot_pose.y, closest_pos.x - robot_pose.x ) );

		Pos detour_pos;
		if ( avoidObstacles(robot_pose, target_pos, detour_pos) )
			pathPlan.push_back( detour_pos );

		pathPlan.push_back( target_pos );

		min_it->SetVisited();

		PathPlanning( target_pos );
	}

	bool avoidObstacles( const Pos& robot_pos, const Pos& target_pos, Pos& detour_pos )
	{
		double k = ( target_pos.y - robot_pos.y ) * 1.0 / ( target_pos.x - robot_pos.x );
		double c = robot_pos.y - k * robot_pos.x;
		double alpha = atan( k );

		double min_d = MIN_DIST;
		std::set< Pos >::const_iterator min_it = posCache.begin();
		for( std::set< Pos >::const_iterator it = posCache.begin(); it != posCache.end(); ++it )
		{
			if ( it->x < robot_pos.x || it->x > target_pos.x
				 || it->y < robot_pos.y || it->y > target_pos.y )
				continue;

			double d = abs( it->y - k * it->x - c ) / pow( k, 2 );

			if ( d < min_d )
			{
				min_d = d;
				min_it = it;
			}
		}

		if( min_d < MIN_DIST )
		{
			if ( min_it->y > ( k * min_it->x + c ) )
			{
				detour_pos.x = min_it->x + MIN_DIST * cos( alpha ); 
				detour_pos.y = min_it->y - MIN_DIST * sin( alpha ); 
			}
			else
			{
				detour_pos.x = min_it->x - MIN_DIST * cos( alpha ); 
				detour_pos.y = min_it->y + MIN_DIST * sin( alpha ); 
			}

			return true;
		}

		return false;
	}

	void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
	{
		pathPlan.clear();

		// objects.clear();
		this->ProcessMapPoints( msg );
	
		std::cout << "New Map" << std::endl;
		int i = 0;
		for ( std::vector< WorldObject >::const_iterator it = objects.begin(); it != objects.end(); ++it )
		{
			std::cout << "Object " << ++i << ( it->isVisited() ? " visited" : "" ) << std::endl;
			// it->PrintObjectPositions();
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

		// collect pathPlan vector
		this->PathPlanning( robot_pose );

		// =========================================
		// Move robot by plan
		// =========================================
		for( std::vector< Pos >::iterator it = pathPlan.begin(); it != pathPlan.end(); ++it )
		{
			Pos closest_pos = *it;

			std::cout << "Closest pose: " << closest_pos.x << ", " << closest_pos.y << std::endl;

			double min_dist = WorldObject::DistBetweenPosStatic( closest_pos, robot_pose );

			double alpha = std::atan2( closest_pos.y - robot_pose.y, closest_pos.x - robot_pose.x ) - tf::getYaw( transform.getRotation() );

			geometry_msgs::Twist base_cmd;

			base_cmd.linear.y = 0;
			base_cmd.linear.x = std::min( 0.5, min_dist * msg->info.resolution );
			base_cmd.angular.z = alpha;

			ros::Rate new_rate( 30.0 );
			while( min_dist > MIN_DIST )
			{
				// std::cout << base_cmd.linear.x << ", " << base_cmd.angular.z << std::endl;
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

				min_dist = WorldObject::DistBetweenPosStatic( closest_pos, robot_pose );

				// angle to rotate robot
				alpha = std::atan2( closest_pos.y - robot_pose.y, closest_pos.x - robot_pose.x ) - tf::getYaw( transform.getRotation() );

				base_cmd.linear.x = std::min( 0.2, min_dist * msg->info.resolution );
				base_cmd.angular.z = alpha;
			}
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
