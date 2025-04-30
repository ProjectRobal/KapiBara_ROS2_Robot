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

#define POPULATION_SIZE (80)

class KapiBaraMind : public rclcpp::Node
{
    std::random_device rd;
    std::mt19937 gen; // Standard mersenne_twister_engine seeded with rd()
    

    snn::Arbiter arbiter;

    size_t max_iter;

    std::shared_ptr<snn::Attention<686,256,64,POPULATION_SIZE>> attention;

    std::shared_ptr<snn::LayerKAC<256,4096,POPULATION_SIZE>> layer1;

    std::shared_ptr<snn::LayerKAC<4096,2048,POPULATION_SIZE,snn::ReLu>> layer2;

    std::shared_ptr<snn::LayerKAC<2048,512,POPULATION_SIZE,snn::ReLu>> layer3;

    std::shared_ptr<snn::LayerKAC<512,256,POPULATION_SIZE,snn::ReLu>> layer4;

    std::shared_ptr<snn::LayerKAC<256,64,POPULATION_SIZE>> layer5;


    snn::SIMDVectorLite<686> inputs;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr orientation_subscription;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription;

    rclcpp::Subscription<kapibara_interfaces::msg::FaceEmbed>::SharedPtr face_subscription;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_subscription;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr spectogram_subscription;

    rclcpp::Subscription<kapibara_interfaces::msg::Emotions>::SharedPtr emotions_subscription;

    rclcpp::Service<kapibara_interfaces::srv::StopMind>::SharedPtr stop_mind_service;


    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher;

    rclcpp::TimerBase::SharedPtr network_timer;

    rclcpp::TimerBase::SharedPtr network_save_timer;

    long double last_reward;

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


    void init_network()
    {
        this->attention = std::make_shared<snn::Attention<686,256,64,20>>();

        this->layer1 = std::make_shared<snn::LayerKAC<256,4096,20>>();
        this->layer2 = std::make_shared<snn::LayerKAC<4096,2048,20,snn::ReLu>>();
        this->layer3 = std::make_shared<snn::LayerKAC<2048,512,20,snn::ReLu>>();
        this->layer4 = std::make_shared<snn::LayerKAC<512,256,20,snn::ReLu>>();
        this->layer5 = std::make_shared<snn::LayerKAC<256,64,20>>();

        this->arbiter.addLayer(this->attention);

        this->arbiter.addLayer(this->layer1);
        this->arbiter.addLayer(this->layer2);
        this->arbiter.addLayer(this->layer3);
        this->arbiter.addLayer(this->layer4);
        this->arbiter.addLayer(this->layer5);

        int ret = this->arbiter.load(this->checkpoint_filename());

        if(ret!=0)
        {
            RCLCPP_ERROR(this->get_logger(),"Failed to load network! with code: %i trying to load backup!",ret);

            ret = this->arbiter.load_backup(this->checkpoint_filename());
        }

        if(ret!=0)
        {
            RCLCPP_ERROR(this->get_logger(),"Failed to load network! with code: %i !",ret);
            this->arbiter.setup();

            ret = arbiter.save(this->checkpoint_filename());

            if(ret!=0)
            {
                RCLCPP_ERROR(this->get_logger(),"Failed to save network, error code: %i to %s",ret,this->checkpoint_filename().c_str());
            }
            else
            {
                RCLCPP_INFO(this->get_logger(),"Network has been saved!");
            }

        }
        else 
        {
            RCLCPP_INFO(this->get_logger(),"Networkd loaded from file: %s",this->checkpoint_filename().c_str());
        }

    }

    const std::string checkpoint_filename() const
    {
        return this->get_parameter("checkpoint_dir").as_string();
    }

    void orientation_callback(const sensor_msgs::msg::Imu::SharedPtr imu)
    {
        RCLCPP_DEBUG(this->get_logger(),"Got orientaion message!");

        // append orientaion data

        this->inputs[0] = static_cast<number>(imu->orientation.x);
        this->inputs[1] = static_cast<number>(imu->orientation.y);
        this->inputs[2] = static_cast<number>(imu->orientation.z);
        this->inputs[3] = static_cast<number>(imu->orientation.w);

    }

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
        RCLCPP_DEBUG(this->get_logger(),"Got odometry message!");

        const auto& twist = odom->twist.twist;

        // append orientaion data

        this->inputs[4] = static_cast<number>(twist.linear.x/MAX_LINEAR_SPEED);
        this->inputs[5] = static_cast<number>(twist.linear.y/MAX_LINEAR_SPEED);
        this->inputs[6] = static_cast<number>(twist.linear.z/MAX_LINEAR_SPEED);

        this->inputs[7] = static_cast<number>(twist.angular.x/MAX_ANGULAR_SPEED);
        this->inputs[8] = static_cast<number>(twist.angular.y/MAX_ANGULAR_SPEED);
        this->inputs[9] = static_cast<number>(twist.angular.z/MAX_ANGULAR_SPEED);

    }

    void face_callback(const kapibara_interfaces::msg::FaceEmbed::SharedPtr face)
    {
        RCLCPP_DEBUG(this->get_logger(),"Got face embedding message!");

        const std::vector<float>& embeddings = face->embedding;

        size_t iter = 10;

        float max_value = *std::max_element(embeddings.begin(), embeddings.end());

        float min_value = *std::min_element(embeddings.begin(), embeddings.end());

        for(const float elem : embeddings)
        {

            this->inputs[iter] = static_cast<number>((elem - min_value)/(max_value - min_value));

            iter++;
        }
    }

    void points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr points)
    {
        RCLCPP_DEBUG(this->get_logger(),"Got points message!");

        if( points->width*points->height != 256 )
        {
            RCLCPP_ERROR(this->get_logger(),"Got invalid points message!");
            return;
        }

        bool found_z = false;

        for( const auto& field : points->fields )
        {
            if( field.name == "z" )
            {
                found_z = true;
            }
        }

        if( !found_z || points->fields.size()!= 4 )
        {
            RCLCPP_ERROR(this->get_logger(),"No z field found or not enough fields present!");
            return;
        }

        sensor_msgs::PointCloud2Iterator<float> iter_z(*points, "z");

        float num = 0.f;

        snn::SIMDVectorLite<256> z_points;

        size_t iter = 0;

        float min = 0;
        float max = 0;

        while( iter_z != iter_z.end() )
        {
            num = *iter_z;

            num = std::min(num,10.f);
            num = std::max(num,-10.f);

            z_points[iter] = num;

            min = min == 0 ? num : std::min(min,num);

            max = max == 0 ? num : std::max(max,num);

            ++iter_z;
        }

        z_points = (z_points-min)/(max-min);

        for(size_t i=0;i<256;i++)
        {
            this->inputs[i+170] = z_points[i];
        }
        
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

        for(size_t y=0;y<16;y++)
        {
            for(size_t x=0;x<16;x++)
            {
                this->inputs[iter++] = static_cast<float>(spectogram.at<uint8_t>(x,y))/255.f;
            }
        }
    }

    void emotions_callback(const kapibara_interfaces::msg::Emotions::SharedPtr emotions)
    {
        RCLCPP_DEBUG(this->get_logger(),"Got emotions message!");

        long double reward = emotions->happiness*320.f + emotions->fear*120.f + emotions->uncertainty*-40.f + emotions->angry*-60.f + emotions->boredom*-20.f;

        this->arbiter.applyReward(reward);

        this->arbiter.shuttle();

        this->last_reward = reward;

    }

    void network_callback()
    {
        RCLCPP_DEBUG(this->get_logger(),"Network fired!");

        // there is no need to do anything when robot is happy
        if( this->last_reward >= 0.f )
        {

            geometry_msgs::msg::Twist twist;

            twist.angular.z = 0.f;

            twist.linear.x = 0.f;

            this->twist_publisher->publish(twist);
            
            return;
        }

        snn::SIMDVectorLite<64> output = this->fire_network(this->inputs);

        for(size_t i=0;i<64;++i)
        {
            output[i] = std::exp(output[i]);
        }

        output = output / output.reduce();

        std::uniform_real_distribution<float> dis(0.0, 1.0);

        size_t max_iter = 0;

        for(size_t i=1;i<64;++i)
        {
            if( output[i] > output[max_iter] )
            {
                max_iter = i;
            }
        }

        // float random_value = dis(this->gen);

        // float action_cumulator = 0.f;

        // size_t max_iter = 0;

        // for(size_t i=0;i<64;i++)
        // {
        //     action_cumulator += output[i];

        //     if(action_cumulator >= random_value)
        //     {
        //         max_iter = i;
        //         break;
        //     }
        // }

        // decode x and y value

        int8_t x = max_iter % 16;
        int8_t y = max_iter / 16;

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

        return this->arbiter.save(this->checkpoint_filename());
    }

    KapiBaraMind()
    : Node("kapibara_mind"),
    gen(rd())
    {

        this->declare_parameter("checkpoint_dir", "/app/src/mind.kac");

        this->declare_parameter("max_linear_speed", 0.1f);

        this->declare_parameter("max_angular_speed", 2.f);

        this->last_reward = 0.f;

        this->max_iter = 0;

        this->inputs = snn::SIMDVectorLite<686>(0);

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

    snn::SIMDVectorLite<64> fire_network(const snn::SIMDVectorLite<686>& input)
    {

        auto output = this->attention->process(input);

        auto output2 = this->layer1->fire(output);
        auto output3 = this->layer2->fire(output2);
        auto output4 = this->layer3->fire(output3);
        auto output5 = this->layer4->fire(output4);
        auto output6 = this->layer5->fire(output5);

        return output6;
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

