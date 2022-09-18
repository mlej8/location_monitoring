#include "ros/ros.h" // includes all headers necessary to use the most common public pieces of ROS
#include "std_msgs/String.h"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it. The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   * 
   * Allows us to do name remapping through the command line
   * 
   * The name used here must be a base name, ie. it cannot have a / or ~ in it.
   */
  ros::init(argc, argv, "talker");


  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node and cleanup any resources the node was using.
   */
  ros::NodeHandle n;


  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   * 
   * Tell the master that we are going to be publishing a message of type `std_msgs/String` on the topic "chatter". 
   * This lets the master tell any nodes listening on "chatter" that we are going to publish data on that topic.
   * 
   * NodeHandle::advertise() returns a ros::Publisher object, which serves two purposes: 1) it contains a publish() method that lets you publish messages onto the topic it was created with, and 2) when it goes out of scope, it will automatically unadvertise.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);


  /**
   * A ros::Rate object allows you to specify a frequency that you would like to loop at. It will keep track of how long it has been since the last call to Rate::sleep(), and sleep for the correct amount of time.
   * In this case we tell it we want to run at 10Hz.
   */
  ros::Rate loop_rate(10);


  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;

  /**
   * By default roscpp will install a SIGINT handler which provides Ctrl-C handling which will cause ros::ok() to return false if that happens.
   * 
   * ros::ok() will return false if:
   *    a SIGINT is received (Ctrl-C)
   *    we have been kicked off the network by another node with the same name
   *    ros::shutdown() has been called by another part of the application.
   *    all ros::NodeHandles have been destroyed
   * 
   * Once ros::ok() returns false, all ROS calls will fail.
   */
  while (ros::ok())
  {

    /**
     * This is a message object. You stuff it with data, and then publish it.
     * The standard String message has one member: "data".
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    // logging - replacement for printf/cout
    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     * 
     * Now we actually broadcast the message to anyone who is connected.
     */
    chatter_pub.publish(msg);

    /**
     * If you were to add a subscription into this application, and did not have ros::spinOnce() here, your callbacks would never get called.
    */
    ros::spinOnce();

    /**
     * Use the ros::Rate object to sleep for the time remaining to let us hit our 10Hz publish rate
     * Calling ros::spinOnce() here is not necessary for this simple program, because we are not receiving any 
     * callbacks. However, if you were to add a subscription into this application, and did not have ros::spinOnce() 
     * here, your callbacks would never get called. So, add it for good measure.
    */
    loop_rate.sleep();

    ++count;
  }

  return 0;
}
