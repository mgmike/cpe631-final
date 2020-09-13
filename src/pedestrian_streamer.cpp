#include <ros/ros.h>
#include <ros/console.h>
#include <gazebo_msgs/ModelStates.h>
#include <std_msgs/Header.h>
#include <people_msgs/People.h>



class SubscribeAndPublish {
public:
    SubscribeAndPublish(){
        people_publisher = n.advertise<people_msgs::People>("people", 1000);
        people_subscriber = n.subscribe("gazebo/model_states", 1000, &SubscribeAndPublish::convert_people, this);
        count = 0;
        ROS_DEBUG_ONCE("SAP is initialized");
    }

    void convert_people(const gazebo_msgs::ModelStates modelStates) {
        ROS_DEBUG_ONCE("inside convert_people");
        if (ros::ok()) {
            people_msgs::People people;

            for (int i = 0; i < modelStates.name.size(); i++) {
                if (modelStates.name[i].size() > 16 && (modelStates.name[i].substr(0, 15).compare("person_standing")) == 0) {
                    people_msgs::Person person;
                    person.name = modelStates.name[i];
                    person.position = modelStates.pose[i].position;
                    person.velocity.x = modelStates.twist[i].linear.x;
                    person.velocity.y = modelStates.twist[i].linear.y;
                    person.velocity.z = modelStates.twist[i].linear.z;
                    people.people.push_back(person);
                }
                ROS_DEBUG("adding person");
                //ROS_INFO("adding person %s, %d", modelStates.name[i].substr(0,15).c_str(), modelStates.name[i].substr(0, 15).compare("person_standing_"));
            }

            people.header.seq = count;
            people.header.frame_id = "map";
            people.header.stamp = ros::Time::now();

            people_publisher.publish(people);
            ros::spinOnce();

            ++count;
        }
    }

private:
    ros::NodeHandle n;
    ros::Publisher people_publisher;
    ros::Subscriber people_subscriber;
    int count;

};

int main(int argc, char** argv){
    ros::init(argc, argv, "pedestrian_streamer");
    SubscribeAndPublish SAPObject;
    ros::spin();
    return 0;
}