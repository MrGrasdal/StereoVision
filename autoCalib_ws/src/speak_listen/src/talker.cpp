//
// Created by martin on 20.04.2021.
//

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <sys/types.h>
#include <dirent.h>
#include <vector>
#include <sstream>

using namespace std;
using namespace cv;


void read_directory(const std::string& name, vector<string>& v)
{
    DIR* dirp = opendir(name.c_str());
    struct dirent * dp;
    while ((dp = readdir(dirp)) != NULL) {
        v.push_back(dp->d_name);
    }

    v.erase(v.begin(), v.begin()+2);

    closedir(dirp);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);

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
     */
    image_transport::Publisher pubLeft = it.advertise("/camera_array/left/image_raw", 1000);
    image_transport::Publisher pubRight = it.advertise("/camera_array/right/image_raw", 1000);

    ros::Rate loop_rate(1);

    std::string path = "/media/martin/Samsung_T5/imgs/calibDay2/";

    vector<string> filenames;

    read_directory(path+"left", filenames);

    /**
     * A count of how many messages we have sent. This is used to create
     * a unique string for each message.
     */

    for (auto & fname : filenames)
    {
        /**
         * This is a message object. You stuff it with data, and then publish it.
         */

        Mat imgL = imread(path+"left/"+fname);
        Mat imgR = imread(path+"right/"+fname);

        sensor_msgs::ImagePtr msgL = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgL).toImageMsg();
        sensor_msgs::ImagePtr msgR = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgR).toImageMsg();


        /**
         * The publish() function is how you send messages. The parameter
         * is the message object. The type of this object must agree with the type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
         */


        pubLeft.publish(msgL);
        pubRight.publish(msgR);
        ros::spinOnce();

        loop_rate.sleep();

    }


    return 0;
}

