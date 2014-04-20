#include <vector>
#include <cmath>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>

#define CRITICAL_DISTANCE_BETWEEN_OBJECTS 0.5

double GetDist( geometry_msgs::Point32 point1, geometry_msgs::Point32 point2 )
{
	return sqrt( pow( ( point1.x - point2.x ), 2 ) + pow( ( point1.y - point2.y ), 2 ) );
}

class WorldObject
{
public:

	typedef std::vector< geometry_msgs::Point32 > PointsVec;
	WorldObject() { }
	~WorldObject() { }
	
	void AddPoint( geometry_msgs::Point32 point )
	{
		points.push_back( point );
	}

	double GetDistanceToObject( geometry_msgs::Point32 point )
	{
		double min_dist = points.empty() ? 0 : GetDist( point, points.front() );
		for( PointsVec::iterator it = points.begin(); it != points.end(); ++it )
		{
			double cur_dist = GetDist( point, *it );

			if( cur_dist < min_dist )
				min_dist = cur_dist;
		}

		return min_dist;
	}

	geometry_msgs::Point32 GetObjectCenter()
	{
		geometry_msgs::Point32 center_point;
		center_point.x = 0;
		center_point.y = 0;
		center_point.z = 0;

		for( PointsVec::iterator it = points.begin(); it != points.end(); ++it )
		{
			center_point.x += it->x;
			center_point.y += it->y;
		}

		center_point.x /= points.size();
		center_point.y /= points.size();

		return center_point;
	}
private:
	std::vector< geometry_msgs::Point32 > points; 
};
