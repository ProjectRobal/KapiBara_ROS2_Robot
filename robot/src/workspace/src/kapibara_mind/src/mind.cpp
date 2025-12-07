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
#include <filesystem>


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

#include "head.hpp"


#define MAX_LINEAR_SPEED (4.f)

#define MAX_ANGULAR_SPEED (400.f)

#define POPULATION_SIZE (80)

#define ACTION_COUNT (20)

#define IMAGE_WIDTH (224)

#define SPECTOGRAM_WIDTH (224)

void _thread(KapiBaraHead<224,8>& net,uint8_t x[],uint8_t y[])
{
    net.fire(x,y);
}

class KapiBaraMind : public rclcpp::Node
{
    std::random_device rd;
    std::mt19937 gen; // Standard mersenne_twister_engine seeded with rd()
    

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr orientation_subscription;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription;

    rclcpp::Subscription<kapibara_interfaces::msg::FaceEmbed>::SharedPtr face_subscription;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_subscription;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr spectogram_subscription;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription;

    rclcpp::Subscription<kapibara_interfaces::msg::Emotions>::SharedPtr emotions_subscription;

    rclcpp::Service<kapibara_interfaces::srv::StopMind>::SharedPtr stop_mind_service;

    uint8_t last_image[IMAGE_WIDTH*IMAGE_WIDTH];

    uint8_t last_image_during_reasoning[IMAGE_WIDTH*IMAGE_WIDTH];

    uint8_t spectogram_image[SPECTOGRAM_WIDTH*SPECTOGRAM_WIDTH];

    uint8_t spectogram_image_during_reasoning[SPECTOGRAM_WIDTH*SPECTOGRAM_WIDTH];

    uint8_t y[16*16];

    KapiBaraHead<IMAGE_WIDTH,8> image_head;

    KapiBaraHead<SPECTOGRAM_WIDTH,8> spectogram_head;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher;

    rclcpp::TimerBase::SharedPtr network_timer;

    rclcpp::TimerBase::SharedPtr network_save_timer;

    long double last_reward;
    long double delta_reward;

    uint8_t x_pool;
    uint8_t y_pool;


    enum Stages
    {
        REASONING, // generate costmap
        MOVING, // move towards generated costmap
    };

    Stages stage;


    void stop_mind_handle(const std::shared_ptr<kapibara_interfaces::srv::StopMind::Request> request,
        std::shared_ptr<kapibara_interfaces::srv::StopMind::Response> response)
    {
        bool stop = request->stop;
        RCLCPP_INFO(this->get_logger(),"Stop mind request: %s",stop?"true":"false");

        if(stop)
        {
            this->stage = REASONING;
            this->stop_motors();
            this->network_timer->cancel();
        }
        else
        {
            this->network_timer->reset();
        }

        response->ok = true;
    }


    void init_network()
    {
        std::fstream file;

        if( !std::filesystem::exists("mind.qkac") )
        {
            RCLCPP_ERROR(this->get_logger(),"Failed to load network!");

            this->image_head.init();
            this->spectogram_head.init();
            return;
        }

        file.open("mind.qkac",std::ios::in|std::ios::binary);
        
        this->image_head.load(file);
        this->spectogram_head.load(file);

        if(!file.good())
        {
            RCLCPP_ERROR(this->get_logger(),"Failed to load network, error during network load!");

            this->image_head.init();
            this->spectogram_head.init();   

            file.close();
            return;
        }

        file.close();

        RCLCPP_INFO(this->get_logger(),"Network loaded succesfully!");
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
    }

    void points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr points)
    {
        RCLCPP_DEBUG(this->get_logger(),"Got points message!");
        
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

        resize(image, spectogram, cv::Size(224, 224), cv::INTER_LINEAR); 

        for(size_t y=0;y<224;y++)
        {
            for(size_t x=0;x<224;x++)
            {
                this->spectogram_image[224*y + x] = static_cast<float>(spectogram.at<uint8_t>(x,y));
            }
        }
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr img)
    {
        RCLCPP_DEBUG(this->get_logger(),"Got image message!");

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

        resize(image, spectogram, cv::Size(224, 224), cv::INTER_LINEAR); 

        for(size_t y=0;y<224;y++)
        {
            for(size_t x=0;x<224;x++)
            {
                this->last_image[224*y + x] = spectogram.at<uint8_t>(x,y,0)/3 + (2*spectogram.at<uint8_t>(x,y,1))/3 + spectogram.at<uint8_t>(x,y,2)/9 ;
            }
        }
    }


    void emotions_callback(const kapibara_interfaces::msg::Emotions::SharedPtr emotions)
    {
        RCLCPP_DEBUG(this->get_logger(),"Got emotions message!");

        long double reward = emotions->happiness*320.f + emotions->fear*-120.f + emotions->uncertainty*-40.f + emotions->angry*-60.f + emotions->boredom*-20.f;

        this->last_reward = reward;

        this->delta_reward = reward - this->last_reward;

    }


    void network_callback()
    {
        RCLCPP_DEBUG(this->get_logger(),"Network fired!");

        // there is no need to do anything when robot is happy
        if( this->last_reward >= 0.f )
        {

            this->stop_motors();

            this->stage = REASONING;
            
            return;
        }

        switch(this->stage)
        {
            case REASONING:
            {
                RCLCPP_INFO(this->get_logger(),"Generating steps!");

                uint8_t y1[16*16](0);
                uint8_t y2[16*16](0);

                std::thread image_thread(_thread,std::ref(this->image_head),this->last_image,y1);
                std::thread spectogram_thread(_thread,std::ref(this->spectogram_head),this->spectogram_image,y2);

                if( image_thread.joinable() )
                {
                    image_thread.join();
                } 

                if( spectogram_thread.joinable() )
                {
                    spectogram_thread.join();
                }

                for(size_t i=0;i<(16*16);++i)
                {
                    this->y[i] = ( y1[i] + y2[i] ) / 2;
                }

                std::memcpy(this->last_image_during_reasoning,this->last_image,sizeof(this->last_image));
                std::memcpy(this->spectogram_image_during_reasoning,this->spectogram_image,sizeof(this->spectogram_image));

                // center of the costmap
                this->x_pool = 0;

                this->stage = MOVING;
            }
            break;

            case MOVING:
            {

                // look at reward change, not its value per say
                if( this->delta_reward < 0.f )
                {
                    this->stop_motors();

                    RCLCPP_INFO(this->get_logger(),"Performing mutations!");

                    this->stage = REASONING;

                    // perform mutations

                    uint8_t _y[16*16];

                    {

                        auto _the_runtime_scope = Runtime::scope();

                        Runtime::set_mutation_mode(true);
                        Runtime::set_global_reward(-10.0f);

                        this->image_head.fire(this->last_image_during_reasoning,_y);

                    }

                    {

                        auto _the_runtime_scope = Runtime::scope();

                        Runtime::set_mutation_mode(true);
                        Runtime::set_global_reward(-10.0f);

                        this->spectogram_head.fire(this->spectogram_image_during_reasoning,_y);

                    }

                    return;

                }

                if( this->x_pool == 16*16 )
                {
                    this->stop_motors();

                    this->stage = REASONING;

                    return;
                }

                // move a robot

                uint8_t cmd = this->y[this->x_pool];

                if( cmd < 11 )
                {
                    this->stop_motors();

                    this->stage = REASONING;
                    return;
                }
                
                cmd -= 11;

                const double max_linear_speed = this->get_parameter("max_linear_speed").as_double();

                const double max_angular_speed = this->get_parameter("max_angular_speed").as_double();

                double linear_speed = 0;

                double angular_speed = 0;

                // move backward
                if( cmd <= 61 )
                {
                    linear_speed = -(static_cast<double>(cmd)/61.0)*max_linear_speed;
                }
                // move left
                else if( cmd > 61 && cmd <= 122 )
                {
                    cmd -= 61;

                    angular_speed = -(static_cast<double>(cmd)/61.0)*max_angular_speed;
                }
                // move forward
                else if( cmd > 122 && cmd < 183 )
                {
                    cmd -= 122;

                    linear_speed = (static_cast<double>(cmd)/61.0)*max_linear_speed;
                }
                // move right
                else
                {
                    cmd -= 183;

                    angular_speed = (static_cast<double>(cmd)/61.0)*max_angular_speed;
                }


                
                this->x_pool++;

                // move towards max_iter

                geometry_msgs::msg::Twist twist;

                twist.angular.z = angular_speed;

                twist.linear.x = -linear_speed;

                this->twist_publisher->publish(twist);
            }
            break;

        }
        // decode x and y value

        int8_t x = 0 % 16;
        int8_t y = 0 / 16;

        x -= 8;
        y -= 8;

        const double max_linear_speed = this->get_parameter("max_linear_speed").as_double();

        const double max_angular_speed = this->get_parameter("max_angular_speed").as_double();

        if(abs(x)<=3)
        {
            x = 0;
        }

        if(abs(y)<=3)
        {
            y = 0;
        }        

        double linear_speed = (static_cast<double>(y)/8.f)*max_linear_speed;

        double angular_speed = (static_cast<double>(x)/8.f)*max_angular_speed;

        geometry_msgs::msg::Twist twist;

        twist.angular.z = angular_speed;

        twist.linear.x = -linear_speed;

        this->twist_publisher->publish(twist);

    }

    void save_network_callback()
    {
        this->stop_motors();

        int8_t ret = this->save_network();


        if( ret != 0 )
        {
            RCLCPP_ERROR(this->get_logger(),"Got error id during network save: %i",(int32_t)ret);
        }
    }

    public:

    int8_t save_network()
    {
        RCLCPP_INFO(this->get_logger(),"Saving network weights!");

        std::fstream file;

        file.open("mind.qkac",std::ios::out|std::ios::binary);
        
        this->image_head.save(file);
        this->spectogram_head.save(file);

        file.close();

        return 0;
    }

    KapiBaraMind()
    : Node("kapibara_mind"),
    gen(rd())
    {

        this->declare_parameter("checkpoint_dir", "/app/src/mind.kac");

        this->declare_parameter("max_linear_speed", 0.1f);

        this->declare_parameter("max_angular_speed", 2.f);

        this->last_reward = 0.f;
        this->delta_reward = 0.f;

        this->init_network();

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

        // save each 60min
        this->network_save_timer= this->create_wall_timer(60min, std::bind(&KapiBaraMind::save_network_callback, this));
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

        this->save_network();
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

