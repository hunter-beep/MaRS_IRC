#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "sensor_msgs/Joy.h"

int speed = 100;  // Initial speed
int direction = 0;  // Default direction

class JoyControl {
public:
    JoyControl() {
        // Initialize ROS node handle
        ros::NodeHandle nh;

        // Subscribe to joystick inputs
        joy_subscriber_ = nh.subscribe("/joy", 10, &JoyControl::joyCallback, this);

        // Publishers for rover control
        pwm_publisher_ = nh.advertise<std_msgs::Int16>("/rover/pwm", 10, true);
        direction_publisher_ = nh.advertise<std_msgs::Int16>("/rover/choice", 10, true);
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
        std_msgs::Int16 pwm_msg;
        std_msgs::Int16 direction_msg;

        // Button mappings
        if (msg->buttons[0] == 1) {  // Button 0: Backward
            direction = 2;
            ROS_INFO("Backward");
        } else if (msg->buttons[4] == 1) {  // Button 4: Forward
            direction = 1;
            ROS_INFO("Forward");
        } else if (msg->buttons[1] == 1) {  // Button 1: Spot Right
            direction = 4;
            ROS_INFO("Right");
        } else if (msg->buttons[3] == 1) {  // Button 3: Spot Left
            direction = 3;
            ROS_INFO("Left");
        } else if (msg->buttons[10] == 1) {  // Button 10: Decrease speed
            speed = std::max(speed - 10, 0);  // Ensure speed does not go below 0
            ROS_INFO("Decrease Speed: %d", speed);
        } else if (msg->buttons[11] == 1) {  // Button 11: Increase speed
            speed = std::min(speed + 10, 255);  // Ensure speed does not exceed 255
            ROS_INFO("Increase Speed: %d", speed);
        }

        // Publish direction and speed
        pwm_msg.data = speed;
        direction_msg.data = direction;
        pwm_publisher_.publish(pwm_msg);
        direction_publisher_.publish(direction_msg);
    }

private:
    ros::Subscriber joy_subscriber_;
    ros::Publisher pwm_publisher_;
    ros::Publisher direction_publisher_;
};

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "joy_control_node");
    ROS_INFO("Joystick rover control node initialized.");

    // Create an instance of JoyControl
    JoyControl joy_control;

    // Spin to keep the node alive and process incoming messages
    ros::spin();

    return 0;
}
