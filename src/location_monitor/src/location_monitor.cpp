#include "location_monitor/LandmarkDistance.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "gazebo_msgs/GetModelProperties.h"

#include <gazebo/physics/physics.hh>

using location_monitor::LandmarkDistance;
using std::string;
using std::vector;

class Landmark {
   public:
    Landmark(string name, double x, double y, double z) : name(name), x(x), y(y) {}
    string name;
    double x, y, z;
};

class LandmarkMonitor {
   public:
    LandmarkMonitor(const ros::Publisher& landmark_pub) : landmarks(), landmarks_publisher(landmark_pub) {
        InitLandmarks();
    }

    /**
     * @brief Return name of the Landmark closest to robot as well as the distance to that Landmark.
     * 
     * @param x Robot x coordinate
     * @param y Robot y coordinate
     * @return LandmarkDistance 
     */
    LandmarkDistance FindClosest(double x, double y) {
        LandmarkDistance result;
        result.distance = -1;

        for (size_t i = 0; i < landmarks.size(); i++) {
            const Landmark& landmark = landmarks[i];
            double xd = landmark.x - x;
            double yd = landmark.y - y;
            double distance = sqrt(xd * xd + yd * yd);

            if (result.distance < 0 || distance < result.distance) {
                result.distance = distance;
                result.name = landmark.name;
            }
        }
        return result;
    }

    void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // only the message is a pointer, but the pose is an object
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double z = msg->pose.pose.position.z;

        LandmarkDistance ld = FindClosest(x, y);

        // logging for roscpp
        ROS_INFO("x: %f, y: %f, z: %f, name: %s, d: %f", x, y, z, ld.name.c_str(), ld.distance);
        landmarks_publisher.publish(ld);
    }

   private:
    vector<Landmark> landmarks;
    ros::Publisher landmarks_publisher;

    void InitLandmarks() {
        // call service to get the obstacles location

        landmarks.push_back(Landmark("Cube", 1.00, -1.00, 9.99));
        landmarks.push_back(Landmark("Dumpster", 0.25, -0.25, 9.99));
        landmarks.push_back(Landmark("Cylinder", 0.30, -0.5, 9.99));
        landmarks.push_back(Landmark("Barrier", 0.50, -0.75, 9.99));
        landmarks.push_back(Landmark("Bookshelf", 0.75, -0.1, 9.99));
    }
};

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 * 
 * This is the callback function that will get called when a new message has arrived on the odom topic. 
 * 
 * The message is passed in a boost shared_ptr, which means you can store it off if you want, without worrying about it getting deleted underneath you, and without copying the underlying data.
 * 
 * boost::shared_ptr https://www.boost.org/doc/libs/1_37_0/libs/smart_ptr/shared_ptr.htm
 */

int main(int argc, char** argv) {
    /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
    ros::init(argc, argv, "location_monitor");

    /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
    ros::NodeHandle n;
    ros::Publisher landmark_pub = n.advertise<LandmarkDistance>("closest_landmark", 10);
    LandmarkMonitor monitor(landmark_pub);

    // TODO: get the collision coordinates on the map
    // gazebo::physics::BasePtr s = gazebo::physics::Entity::GetByName();
    // if (gazebo::physics::has_world("default")){
    //     gazebo::physics::get_world("default");
    // } else {
    //     ROS_INFO("no world");
    // }

    // if (gazebo::physics::worlds_running()){
    //     ROS_INFO("Detected worlds running.");
    //     gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
    //     gazebo::physics::ModelPtr obstacle = world->ModelByName("ros_symbol");
    //     gazebo::physics::LinkPtr link = obstacle->GetLink("symbol");
    //     vector<gazebo::physics::CollisionPtr> collisions = link->GetCollisions();
    //     for (int i = 0; i < collisions.size(); i++) {
    //         ROS_INFO("Collision name: %s", collisions[i]->GetName().c_str());
    //     }
    // } else {
    //     ROS_INFO("No world is running.");
    // }
    // ros::ServiceClient gazebo_model_client = n.serviceClient<gazebo_msgs::GetModelProperties>("/gazebo/get_model_properties");
    // gazebo_msgs::GetModelProperties srv;
    // srv.request.model_name = "ros_symbol";

    // if (gazebo_model_client.call(srv)) {
    //     for (int i = 0;i < srv.response.geom_names.size(); i++) {
    //         ROS_INFO("Child name %s", srv.response.geom_names[i].c_str());
    //     }
    // }else {
    //     ROS_ERROR("Failed to call service ");
    //     return 1;
    // }

    /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue. If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   * 
   * Subscribe to the Odom topic with the master. ROS will call the OdomCallback() function whenever a new message arrives. 
   * NodeHandle::subscribe() returns a ros::Subscriber object, that you must hold on to until you want to unsubscribe. 
   * When the Subscriber object is destructed, it will automatically unsubscribe from the Odom topic.
   * There are versions of the NodeHandle::subscribe() function which allow you to specify a class member function, or even anything callable by a Boost.Function object. 
   * The roscpp overview contains more information.
   */
    ros::Subscriber sub = n.subscribe("odom", 1000, &LandmarkMonitor::OdomCallback, &monitor);  // subcribing to odom topic

    /**
   * ros::spin() will enter a loop, calling message callbacks as fast as possible.  
   * If there's nothing for it to do it won't use much CPU. 
   * ros::spin() will exit once ros::ok() returns false, which means ros::shutdown() has been called, either by the default Ctrl-C handler, the master telling us to shutdown or it being called manually.
   * With this version, all callbacks will be called from within this thread (the main one).  
   * ros::spin() will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
    ros::spin();

    return 0;
}
