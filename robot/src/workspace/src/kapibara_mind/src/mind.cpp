#include <experimental/simd>
#include <iostream>
#include <string_view>
#include <cmath>
#include <chrono>
#include <cstddef>
#include <iomanip>
#include <numeric>
#include <fstream>
#include <sys/stat.h>
#include <fcntl.h>
#include <algorithm>
#include <csignal> 
#include <random>
#include <signal.h>



#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>

// quanterion
#include <sensor_msgs/msg/imu.hpp>
// 2d depth points
#include <sensor_msgs/msg/point_cloud2.hpp>
// mean face embedded
#include <kapibara_interfaces/msg/face_embed.hpp>
// emotions, network will be triggered by emotions message
#include <kapibara_interfaces/msg/emotions.hpp>

#include <kapibara_interfaces/srv/stop_mind.hpp>


// encoders speed
#include <nav_msgs/msg/odometry.hpp>
// spectogram
#include <sensor_msgs/msg/image.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// twist message used for ros2 control
#include <geometry_msgs/msg/twist.hpp>

#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/image_encodings.hpp"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include "config.hpp"

#include "simd_vector.hpp"

#include "layer_kac.hpp"
 
#include "initializers/gauss.hpp"
#include "initializers/constant.hpp"
#include "initializers/uniform.hpp"
#include "initializers/hu.hpp"


#include "activation/sigmoid.hpp"
#include "activation/relu.hpp"
#include "activation/softmax.hpp"
#include "activation/silu.hpp"

#include "simd_vector_lite.hpp"

#include "layer_counter.hpp"

#include "arbiter.hpp"

#include "kapibara_sublayer.hpp"

#include "attention.hpp"


#include "block_kac.hpp"

size_t snn::BlockCounter::BlockID = 0;

size_t snn::LayerCounter::LayerIDCounter = 0;

/*

    KapiBara input variables:

    quanterion - 4 values
    linear and angular speed - 6 values
    face embeddings - 160 values when more than two faces are spotted average thier embeddings
    spectogram 16x16 - 256 values
    2d points array from camera, compressed to 16x16 - 256 values

    Total 686 values


*/

using std::placeholders::_1;

using namespace std::chrono_literals;


#define MAX_LINEAR_SPEED (4.f)

#define MAX_ANGULAR_SPEED (400.f)



// Behavioral map size

#define MAP_WIDTH (8192)

#define MAP_HEIGHT (8192)

#define MAP_SIZE MAP_WIDTH*MAP_HEIGHT

#define STEP_SIZE (10.f)

class KapiBaraMind : public rclcpp::Node
{
   
    number map[MAP_WIDTH*MAP_HEIGHT];

    number position[3];

    number orientation[4];

    float yaw;

    number yaw_integral;


    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr orientation_subscription;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription;

    rclcpp::Subscription<kapibara_interfaces::msg::FaceEmbed>::SharedPtr face_subscription;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_subscription;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr spectogram_subscription;

    rclcpp::Subscription<kapibara_interfaces::msg::Emotions>::SharedPtr emotions_subscription;

    rclcpp::Service<kapibara_interfaces::srv::StopMind>::SharedPtr stop_mind_service;


    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher;

    rclcpp::TimerBase::SharedPtr network_timer;


    long double last_reward;

    void set_map_at(int32_t x,int32_t y,number value)
    {
        // center the coordinates

        x += (MAP_WIDTH/2);
        y += (MAP_HEIGHT/2);

        size_t offset = MAP_WIDTH*y + x;

        if( offset >= MAP_SIZE )
        {
            return;
        }

        this->map[ offset ] = value;
    }

    number get_map_at(int32_t x,int32_t y)
    {
        // center the coordinates

        x += (MAP_WIDTH/2);
        y += (MAP_HEIGHT/2);

        size_t offset = MAP_WIDTH*y + x;

        if( offset >= MAP_SIZE )
        {
            return std::nan("1");
        }

        return this->map[offset];
    }

    void stop_mind_handle(const std::shared_ptr<kapibara_interfaces::srv::StopMind::Request> request,
        std::shared_ptr<kapibara_interfaces::srv::StopMind::Response> response)
    {
        bool stop = request->stop;
        RCLCPP_INFO(this->get_logger(),"Stop mind request: %s",stop?"true":"false");

        if(stop)
        {
            this->stop_motors();
            this->network_timer->cancel();
        }
        else
        {
            this->network_timer->reset();
        }

        response->ok = true;
    }

    const std::string checkpoint_filename() const
    {
        return this->get_parameter("checkpoint_dir").as_string();
    }

    void orientation_callback(const sensor_msgs::msg::Imu::SharedPtr imu)
    {
        RCLCPP_DEBUG(this->get_logger(),"Got orientaion message!");

        // append orientaion data

        // this->inputs[0] = static_cast<number>(imu->orientation.x);
        // this->inputs[1] = static_cast<number>(imu->orientation.y);
        // this->inputs[2] = static_cast<number>(imu->orientation.z);
        // this->inputs[3] = static_cast<number>(imu->orientation.w);

    }

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
        RCLCPP_DEBUG(this->get_logger(),"Got odometry message!");

        // const auto& twist = odom->twist.twist;

        const auto& pose = odom->pose.pose;

        this->position[0] = static_cast<number>(pose.position.x)*STEP_SIZE;
        this->position[1] = static_cast<number>(pose.position.y)*STEP_SIZE;
        this->position[2] = static_cast<number>(pose.position.z)*STEP_SIZE;


        tf2::Quaternion quat;

        tf2::fromMsg(pose.orientation, quat);

        // raw, pitch, yaw
        double r,p,y;

        tf2::Matrix3x3 m(quat);

        m.getRPY(r,p,y);

        this->yaw = y;

        RCLCPP_DEBUG(this->get_logger(),"Robot coordinates: %f, %f,%f yaw: %f",
        pose.position.x,pose.position.y,pose.position.z,this->yaw);

        // append orientaion data

        // this->inputs[4] = static_cast<number>(twist.linear.x/MAX_LINEAR_SPEED);
        // this->inputs[5] = static_cast<number>(twist.linear.y/MAX_LINEAR_SPEED);
        // this->inputs[6] = static_cast<number>(twist.linear.z/MAX_LINEAR_SPEED);

        // this->inputs[7] = static_cast<number>(twist.angular.x/MAX_ANGULAR_SPEED);
        // this->inputs[8] = static_cast<number>(twist.angular.y/MAX_ANGULAR_SPEED);
        // this->inputs[9] = static_cast<number>(twist.angular.z/MAX_ANGULAR_SPEED);

    }

    void face_callback(const kapibara_interfaces::msg::FaceEmbed::SharedPtr face)
    {
        RCLCPP_DEBUG(this->get_logger(),"Got face embedding message!");

        // const std::vector<float>& embeddings = face->embedding;

        // size_t iter = 10;

        // float max_value = *std::max_element(embeddings.begin(), embeddings.end());

        // float min_value = *std::min_element(embeddings.begin(), embeddings.end());

        // for(const float elem : embeddings)
        // {

        //     this->inputs[iter] = static_cast<number>((elem - min_value)/(max_value - min_value));

        //     iter++;
        // }
    }

    void points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr points)
    {
        RCLCPP_DEBUG(this->get_logger(),"Got points message!");

        // if( points->width*points->height != 256 )
        // {
        //     RCLCPP_ERROR(this->get_logger(),"Got invalid points message!");
        //     return;
        // }

        // bool found_z = false;

        // for( const auto& field : points->fields )
        // {
        //     if( field.name == "z" )
        //     {
        //         found_z = true;
        //     }
        // }

        // if( !found_z || points->fields.size()!= 4 )
        // {
        //     RCLCPP_ERROR(this->get_logger(),"No z field found or not enough fields present!");
        //     return;
        // }

        // sensor_msgs::PointCloud2Iterator<float> iter_z(*points, "z");

        // float num = 0.f;

        // snn::SIMDVectorLite<256> z_points;

        // size_t iter = 0;

        // float min = 0;
        // float max = 0;

        // while( iter_z != iter_z.end() )
        // {
        //     num = *iter_z;

        //     num = std::min(num,10.f);
        //     num = std::max(num,-10.f);

        //     z_points[iter] = num;

        //     min = min == 0 ? num : std::min(min,num);

        //     max = max == 0 ? num : std::max(max,num);

        //     ++iter_z;
        // }

        // z_points = (z_points-min)/(max-min);

        // for(size_t i=0;i<256;i++)
        // {
        //     this->inputs[i+170] = z_points[i];
        // }
        
    }

    void spectogram_callback(const sensor_msgs::msg::Image::SharedPtr img)
    {
        RCLCPP_DEBUG(this->get_logger(),"Got spectogram message!");

        cv_bridge::CvImagePtr cv_ptr;

        try
        {
            cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot convert image exception: %s", e.what());
            return;
        }

        cv::Mat& image = cv_ptr->image;

        cv::Mat spectogram;

        resize(image, spectogram, cv::Size(16, 16), cv::INTER_LINEAR); 

        size_t iter = 426;

        // for(size_t y=0;y<16;y++)
        // {
        //     for(size_t x=0;x<16;x++)
        //     {
        //         this->inputs[iter++] = static_cast<float>(spectogram.at<uint8_t>(x,y))/255.f;
        //     }
        // }
    }

    void emotions_callback(const kapibara_interfaces::msg::Emotions::SharedPtr emotions)
    {
        RCLCPP_DEBUG(this->get_logger(),"Got emotions message!");

        long double reward = emotions->happiness*320.f + emotions->fear*-120.f + emotions->uncertainty*-40.f + emotions->angry*-60.f + emotions->boredom*-20.f;

        this->last_reward = reward;

        this->set_map_at(this->position[0],this->position[1],reward);
    }

    void network_callback()
    {
        RCLCPP_INFO(this->get_logger(),"Network fired!");

        // get values of neighbourhood blocks
        number blocks[9];

        int32_t x = this->position[0];
        int32_t y = this->position[1];

        number yaw = this->yaw;

        // up
        blocks[0] = this->get_map_at(x,y+1);

        // up,right
        blocks[1] = this->get_map_at(x-1,y+1);

        // right
        blocks[2] = this->get_map_at(x-1,y);

        // down, right
        blocks[3] = this->get_map_at(x-1,y-1);

        // down
        blocks[4] = this->get_map_at(x,y-1);

        // down, left
        blocks[5] = this->get_map_at(x+1,y-1);

        // left
        blocks[6] = this->get_map_at(x+1,y);

        // up,left
        blocks[7] = this->get_map_at(x+1,y+1);

        // center
        blocks[8] = this->get_map_at(x,y);

        number max_blocks = blocks[0];

        size_t max_i = 0;

        for(size_t i=1;i<9;++i)
        {
            if( blocks[i] > max_blocks )
            {
                max_blocks = blocks[i];

                max_i = i;
            }
        }

        // robot stay in place

        RCLCPP_INFO(this->get_logger(),"Current block: %f",blocks[8]);

        if( max_i == 8 )
        {
            this->yaw_integral = 0.f;
            RCLCPP_INFO(this->get_logger(),"Robot stay in place!");
            return;
        }

        RCLCPP_INFO(this->get_logger(),"Next block: %f",blocks[max_i]);

        number target_angle = static_cast<number>(max_i) * (M_PI/4.f) - (M_PI);

        number angle_error = target_angle - yaw;

        if( abs(angle_error) > 0.1f )
        {
            RCLCPP_INFO(this->get_logger(),"Target angle %f, current angle %f",target_angle,yaw);

            this->yaw_integral += 0.001f*angle_error;

            number angular_velocity = 2.f*angle_error + 0.5f*this->yaw_integral;

            this->send_twist(0.f,angular_velocity);
        }
        else
        {
            this->yaw_integral = 0.f;

            RCLCPP_INFO(this->get_logger(),"Moving forward, position: %i %i , current block: %f",x,y,blocks[8]);

            this->send_twist(1.f,0.f);
        }

    }

    void init_map()
    {
        // we will load map from file but by default we will initialize it with random data

        snn::GaussInit<0.f,1.f> gauss;

        for(size_t i=0;i<MAP_SIZE;++i)
        {
            this->map[i] = gauss.init();
        }
    }

    public:

    KapiBaraMind()
    : Node("kapibara_mind")
    {

        this->declare_parameter("checkpoint_dir", "/app/src/mind.kac");

        this->declare_parameter("max_linear_speed", 0.1f);

        this->declare_parameter("max_angular_speed", 2.f);

        this->last_reward = 0.f;

        this->init_map();

        this->yaw_integral = 0.f;

        // add all required subscriptions

        this->orientation_subscription = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", 10, std::bind(&KapiBaraMind::orientation_callback, this, _1));

        this->odometry_subscription = this->create_subscription<nav_msgs::msg::Odometry>(
      "motors/odom", 10, std::bind(&KapiBaraMind::odometry_callback, this, _1));

        this->face_subscription = this->create_subscription<kapibara_interfaces::msg::FaceEmbed>(
      "spoted_faces", 10, std::bind(&KapiBaraMind::face_callback, this, _1));

        this->points_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "midas/points", 10, std::bind(&KapiBaraMind::points_callback, this, _1));

        this->spectogram_subscription = this->create_subscription<sensor_msgs::msg::Image>(
      "spectogram", 10, std::bind(&KapiBaraMind::spectogram_callback, this, _1));

        this->emotions_subscription = this->create_subscription<kapibara_interfaces::msg::Emotions>(
      "emotions", 10, std::bind(&KapiBaraMind::emotions_callback, this, _1));

        this->stop_mind_service = this->create_service<kapibara_interfaces::srv::StopMind>(
      "stop_mind", std::bind(&KapiBaraMind::stop_mind_handle, this, std::placeholders::_1, std::placeholders::_2));


        // one publisher for ros2 control cmd

        this->twist_publisher = this->create_publisher<geometry_msgs::msg::Twist>("motors/cmd_vel_unstamped", 10);

        this->network_timer = this->create_wall_timer(100ms, std::bind(&KapiBaraMind::network_callback, this));
        
    }

    void send_twist(float linear,float angular)
    {
        geometry_msgs::msg::Twist twist;

        twist.angular.z = angular;

        twist.linear.x = linear;

        this->twist_publisher->publish(twist);
    }

    void stop_motors()
    {
        this->send_twist(0.f,0.f);
    }


    void shutdown()
    {
        this->stop_motors();
    }

    ~KapiBaraMind()
    {
        
    }

};


std::function<void(int)> shutdown_handler;
void signal_handler(int signal) { shutdown_handler(signal); }


int main(int argc,char** argv)
{   
    rclcpp::init(argc, argv);


    auto node = std::make_shared<KapiBaraMind>();

    shutdown_handler = [node](int signal)->void    {
        
        rclcpp::shutdown();

        node->shutdown();

        exit(0);

    };

    std::signal(SIGINT,signal_handler);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;

}

