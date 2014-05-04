#include <iostream>
#include <cmath>
#include <vector>
#include <deque>
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
#define WALL -2
#define FREE_SPACE -1
#define UNREACHABLE -3

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

	std::vector< std::vector< int > > posCache;
	
	std::vector< Pos > pathPlan;

	int height;

	int width;
public:

	//! ROS node initialization
	RobotDriver(ros::NodeHandle &nh)
	{
		nh_ = nh;

		//set up the publisher for the cmd_vel topic
		cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

		map_sub_ = nh_.subscribe("/map", 1, &RobotDriver::mapCallback, this );
	}

	bool CheckWalkBounds(int y, int x)
	{
			return ( x >= 0 && x < width && y >= 0 && y < height );
	}

	// const Pos& robot_pos, const Pos& target_pos
	bool FindPath(const Pos& robot_pos, const Pos& target_pos, std::deque< Pos >& path)
	{
		int x_src = robot_pos.x;
		int y_src = robot_pos.y;
		int x_dst = target_pos.x;
		int y_dst = target_pos.y;

		// the incoming and outcoming vector should be empty
		if (!path.empty())
			return false;

		int** field = new int*[height];
		for( int i = 0; i < height; ++i )
			field[i] = new int[width];

		for( int y = 0; y < height; y++ )
		{
			for( int x = 0; x < width; x++ )
			{
				field[y][x] = ( posCache[y][x] <= 0 ) ? FREE_SPACE : WALL;
			}
		}

		// второй цикл для составления непроходимых участков учитывая размеры робота
		for( int y = 0; y < height; y++ )
		{
			for( int x = 0; x < width; x++ )
			{
				if( field[y][x] == WALL )
				{
					for( int ty = -MIN_DIST; ty < MIN_DIST; ++ty )
					{
						for( int tx = -MIN_DIST; tx < MIN_DIST; ++tx )
						{
							if (CheckWalkBounds(y + ty, x + tx) == false)
								continue;
							
							if( field[y + ty][x + tx] != WALL )
								field[y + ty][x + tx] = UNREACHABLE;
						}
					}
				}
			}
		}

		// пометить точку назначения что до нее можно добраться
		for( int ty = -MIN_DIST / 2; ty < MIN_DIST / 2; ++ty )
		{
			for( int tx = -MIN_DIST / 2; tx < MIN_DIST / 2; ++tx )
			{
				if (CheckWalkBounds(y_dst + ty, x_dst + tx) == false)
					continue;

				if( field[y_dst + ty][x_dst + tx] == UNREACHABLE )
					field[y_dst + ty][x_dst + tx] = FREE_SPACE;
			}
		}

		int dx[8] = {1, 0, -1, 0, 1, 1, -1, -1};   // смещения, соответствующие соседям ячейки
		int dy[8] = {0, 1, 0, -1, 1, -1, 1, -1};   // справа, снизу, слева и сверху

		field[y_src][x_src] = 0;
		bool stop = false;
		int d = 0;

		while(!stop && field[y_dst][x_dst] == FREE_SPACE)
		{
			stop = true;                              // предполагаем, что все свободные клетки уже помечены
			for ( int y = 0; y < height; y++ )
			{
				for ( int x = 0; x < width; x++ )
				{
					if ( field[y][x] == d )                              // ячейка (x, y) помечена числом d
					{
						for ( int k = 0; k < 8; k++ )                    // проходим по всем непомеченным соседям
						{
							if (CheckWalkBounds(y + dy[k], x + dx[k]) == false)
								continue;

							if ( field[y + dy[k]][x + dx[k]] == FREE_SPACE )
							{
								stop = false;                            // найдены непомеченные клетки
								field[y + dy[k]][x + dx[k]] = d + 1;     // распространяем волну
							}
						}
					}
				}
			}

			d++;
		}

		if (field[y_dst][x_dst] == FREE_SPACE) 
			return false;			// путь не найден

		// восстановление пути
		int len = field[y_dst][x_dst];              // длина кратчайшего пути из (ax, ay) в (bx, by)
		int x = x_dst;
		int y = y_dst;
		d = len;
		std::cout << "distance: " << d << std::endl;
		int last_k = -1;

		while ( d > 0 )
		{
			Pos pos;
			pos.x = x;
			pos.y = y;
			path.push_front(pos);

			d--;
			if( last_k >= 0 && 
				CheckWalkBounds( y + dy[ last_k ], x + dx[ last_k ] ) &&
				field[y + dy[last_k]][x + dx[last_k]] == d )
			{
				path.pop_front();
				x += dx[ last_k ];
				y += dy[ last_k ];
				continue;
			}

			for (int k = 0; k < 8; k++)
			{
				if (CheckWalkBounds(y + dy[k], x + dx[k]) == false)
					continue;

				if (field[y + dy[k]][x + dx[k]] == d)
				{
					x = x + dx[k];
					y = y + dy[k];           // переходим в ячейку, которая на 1 ближе к старту
					last_k = k;
					break;
				}
			}
		}

		for( int i = 0; i < height; ++i )
			delete[] field[i];

		delete[] field;

		return true;
	}

	void ProcessMapPoints( const nav_msgs::OccupancyGrid::ConstPtr& msg )
	{
		objects.clear();
		posCache.clear();

		posCache.reserve( msg->info.height );
		std::vector< int > temp( msg->info.width );

		for( int i = 0; i < msg->data.size(); ++i )
		{
			if ( i % msg->info.width == 0 )
			{
				memcpy( &temp[0], &msg->data[i], msg->info.width * sizeof( int ) );

				posCache.push_back( temp );
			}

			// Если неизвестно, то не обрабатываем
			if (msg->data[i] <= 0)
				continue;

			Pos pos;
			pos.x = i % msg->info.width;
			pos.y = i / msg->info.width;

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

			if( distToClosestObject > 1.5 * MIN_DIST && msg->data[i] > 0 )
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

		// avoid obstacles
		std::deque< Pos > path;
		FindPath( robot_pose, target_pos, path );

		std::cout << "path plan: " << std::endl;
		for( std::deque< Pos >::iterator it = path.begin(); it != path.end(); ++it )
		{
			std::cout << it->x << ", " << it->y << std::endl;
			pathPlan.push_back( *it );
		}

		min_it->SetVisited();

		PathPlanning( target_pos );
	}

	void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
	{
		pathPlan.clear();

		this->height = msg->info.height;
		this->width = msg->info.width;

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
