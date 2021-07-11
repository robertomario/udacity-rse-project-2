#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;
const float DEFAULT_LINEAR_SPEED = 0.5f;
const float MAX_ANGULAR_SPEED = 2.0f;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    int count = 0;
    int max = 0;
    std::cout << img.step << std::endl;
    for (int i=0; i<img.height; i++){
        for (int j=0; j<3*img.width; j+=3){
            if((img.data[j+i*3*img.width]==white_pixel) && (img.data[j+i*3*img.width+1]==white_pixel) && (img.data[j+i*3*img.width+2]==white_pixel)){
                sum_x += j/3;
                sum_y += i;
                count++;
            }
            if(img.data[j+i*3*img.width]>max){
                max = img.data[j+i*3*img.width];
            }
        }
    }
    ROS_INFO("The number of white pixels is %d", count);
    ROS_INFO("The maximum pixel value is %d", max);
    if(count == 0){
        drive_robot(0.0f, 0.0f);
    }
    else{
        float mean_x = sum_x / ((float) count);
        float mean_y = sum_y / ((float) count);
        float half_width = (float) img.width / 2.0f;
        float offset_ratio = (half_width - mean_x)/half_width;
        float lin_x = DEFAULT_LINEAR_SPEED;
        float ang_z;
        if (abs(offset_ratio)>0.17){
            //lin_x = 0.0f;
            ang_z = offset_ratio*MAX_ANGULAR_SPEED;
        } else {
            //lin_x = DEFAULT_LINEAR_SPEED;
            ang_z = 0.0f;
        }       
        drive_robot(lin_x, ang_z);
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}