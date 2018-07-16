#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

#include <cmath>

tf::TransformListener *listener; // transform betwween map frame and odom frame
ros::Publisher marker_pub;
const uint32_t shape = visualization_msgs::Marker::CUBE;
visualization_msgs::Marker marker;

ros::Subscriber odom_subscriber;
nav_msgs::Odometry odom_msg;
float distanceThreshold = 0.17;

// pickup and dropoff points in map frame and odom frame
float pickupMapX = 1.8;
float pickupMapY = 2.9;
float pickupW = 1;
float dropoffMapX = -1.2;
float dropoffMapY = 4.8;
float dropoffW = 0.29;
float pickupOdomX;
float pickupOdomY;
float dropoffOdomX;
float dropoffOdomY;

float object_scale = 0.25; // size of marker

bool objectPickedup = false;
bool objectDroppedoff = false;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	//odom_msg = *msg;

	//ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
	//ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	//ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x, msg->twist.twist.angular.z);

	float robotX = msg->pose.pose.position.x;
	float robotY = msg->pose.pose.position.y;

	// transform from map to odom
	// populate mapPosePickup
	geometry_msgs::PointStamped mapPosePickup;
	mapPosePickup.header.frame_id = "/map";
	mapPosePickup.header.stamp = ros::Time();
	mapPosePickup.point.x = pickupMapX;
	mapPosePickup.point.y = pickupMapY;
	mapPosePickup.point.z = 0.0;

	// populate mapPoseDropoff
	geometry_msgs::PointStamped mapPoseDropoff;
	mapPoseDropoff.header.frame_id = "/map";
	mapPoseDropoff.header.stamp = ros::Time();
	mapPoseDropoff.point.x = dropoffMapX;
	mapPoseDropoff.point.y = dropoffMapY;
	mapPoseDropoff.point.z = 0.0;

	geometry_msgs::PointStamped odomPosePickup;
	geometry_msgs::PointStamped odomPoseDropoff;
	try
	{
		listener->transformPoint("/odom", mapPosePickup, odomPosePickup);
		listener->transformPoint("/odom", mapPoseDropoff, odomPoseDropoff);
	}
	catch (tf::TransformException& ex)
	{
		ROS_ERROR("Received an exception trying to transform a point: %s", ex.what());
	}
	// get x,y of odom frame
	pickupOdomX = odomPosePickup.point.x;
	pickupOdomY = odomPosePickup.point.y;
	dropoffOdomX = odomPoseDropoff.point.x;
	dropoffOdomY = odomPoseDropoff.point.y;

	float distanceToPickup = sqrt(pow((robotX - pickupOdomX), 2) + pow((robotY - pickupOdomY), 2));
	float distanceToDropoff = sqrt(pow((robotX - dropoffOdomX), 2) + pow((robotY - dropoffOdomY), 2));
	ROS_INFO("\ndistanceToPickup: %f\ndistanceToDropoff: %f\ndistanceThreshold: %f\n", distanceToPickup, distanceToDropoff, distanceThreshold);

	//check if robot reached the pickup location
	if (distanceToPickup < distanceThreshold)
	{
		if (!objectPickedup)
		{
			// pick up the object
			objectPickedup = true;
			ROS_INFO("Picking up...");

			// Hide the marker
			marker.action = visualization_msgs::Marker::DELETE;
			marker_pub.publish(marker);
		}
	}

	//check if robot reached the droppoff location
	if (distanceToDropoff < distanceThreshold)
	{
		if (objectPickedup && !objectDroppedoff)
		{
			// drop off the object
			objectDroppedoff = true;

			// Publish the marker at the drop off zone
			// Set the marker type
			marker.type = shape;

			// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
			marker.action = visualization_msgs::Marker::ADD;

			// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
			marker.pose.position.x = dropoffMapX;
			marker.pose.position.y = dropoffMapY;
			marker.pose.position.z = 0;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = 0.0;
			marker.pose.orientation.w = dropoffW;

			marker_pub.publish(marker);
			ROS_INFO("Dropping off...");
		}
	}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "add_markers");
	ros::NodeHandle n;
	ros::Rate r(1);


	// create a transformer to transform from map to odom
	// to calculate distance to pickup and dropoff zones
	listener = new tf::TransformListener(ros::Duration(10));
	//listener.waitForTransform("/map", "/odom", ros::Time(0), ros::Duration(10.0));  

	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

	// subscribe to odometry data
	odom_subscriber = n.subscribe("/odom", 1000, odomCallback);

	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = "basic_shapes";
	marker.id = 0;

	// Set the marker type
	marker.type = shape;

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = pickupMapX;
	marker.pose.position.y = pickupMapY;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = pickupW;


	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = object_scale;
	marker.scale.y = object_scale;
	marker.scale.z = object_scale;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration(); // never auto delete the marker

									   // Publish the marker
	while (marker_pub.getNumSubscribers() < 1)
	{
		if (!ros::ok())
		{
			return 0;
		}
		ROS_WARN_ONCE("Please create a subscriber to the marker");
		sleep(1);
	}
	marker_pub.publish(marker);
	ROS_INFO("Marker published to pickup zone");

	while (ros::ok())
	{
		ros::spinOnce();
	}

	return 0;
}
