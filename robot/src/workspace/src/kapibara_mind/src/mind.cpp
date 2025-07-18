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

#define MAP_WIDTH (1024)

#define MAP_HEIGHT (1024)

#define MAP_SIZE MAP_WIDTH*MAP_HEIGHT

class KapiBaraMind : public rclcpp::Node
{
   
    snn::SIMDVectorLite<MAP_WIDTH*MAP_HEIGHT> map;

    snn::SIMDVectorLite<3> position;

    snn::SIMDVectorLite<4> orientation;


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

    void set_map_at(size_t x,size_t y,number value)
    {
        size_t offset = MAP_WIDTH*y + x;

        if( offset >= MAP_SIZE )
        {
            return;
        }

        this->map[ offset ] = value;
    }

    number get_map_at(size_t x,size_t y)
    {
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

        this->position[0] = static_cast<number>(pose.position.x);
        this->position[1] = static_cast<number>(pose.position.y);
        this->position[2] = static_cast<number>(pose.position.z);

        this->orientation[0] = static_cast<number>(pose.orientation.x);
        this->orientation[1] = static_cast<number>(pose.orientation.y);
        this->orientation[2] = static_cast<number>(pose.orientation.z);
        this->orientation[3] = static_cast<number>(pose.orientation.w);

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
        RCLCPP_DEBUG(this->get_logger(),"Network fired!");


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

    void stop_motors()
    {
        geometry_msgs::msg::Twist twist;

        twist.angular.z = 0.f;

        twist.linear.x = 0.f;

        this->twist_publisher->publish(twist);
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

