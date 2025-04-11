#include <rclcpp/rclcpp.hpp>
#include <mqtt/async_client.h>


#include <kapibara_interfaces/msg/emotions.hpp>

#include <kapibara_interfaces/srv/stop_mind.hpp>

#include <kapibara_interfaces/msg/microphone.hpp>


#include <map>
#include <functional>

#include <nlohmann/json.hpp>

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;
using json = nlohmann::json;

class MQTTROSBridge : public rclcpp::Node,virtual mqtt::callback {
public:
    MQTTROSBridge() : Node("mqtt_ros_bridge") {
        // Declare ROS 2 parameters
        this->declare_parameter("mqtt_broker", "tcp://0.0.0.0:1883");
        this->declare_parameter("stop_mind_enable", true);
        

        // Get parameters
        std::string broker = this->get_parameter("mqtt_broker").as_string();
        bool stop_mind_enable = this->get_parameter("stop_mind_enable").as_bool();

        // ROS 2 Publisher & Subscriber
        // publisher_ = this->create_publisher<std_msgs::msg::String>(ros_topic_, 10);

        this->emotion_publisher = this->create_publisher<kapibara_interfaces::msg::Emotions>("/KapiBara/emotions", 10);
        this->emotion_subscriber = this->create_subscription<kapibara_interfaces::msg::Emotions>("/KapiBara/emotions", 10,
            std::bind(&MQTTROSBridge::emotion_callback, this, std::placeholders::_1));

        this->microphone_subscriber = this->create_subscription<kapibara_interfaces::msg::Microphone>("microphone", 10,
            std::bind(&MQTTROSBridge::microphone_callback, this, std::placeholders::_1));


        this->mqtt_client_ = new mqtt::async_client(broker,"0","./persit");

        this->mqtt_to_ros["/emotion_state"] = std::bind(&MQTTROSBridge::emotion_state_from_mqtt,this,std::placeholders::_1);

        if(stop_mind_enable)
        {
            this->mqtt_to_ros["/stop"] = std::bind(&MQTTROSBridge::stop_mind_from_mqtt,this,std::placeholders::_1);
        }

        // MQTT Setup
        this->mqtt_client_->set_callback(*this);

        RCLCPP_INFO(this->get_logger(), "Connecting to MQTT server at %s",broker.c_str());

        if(stop_mind_enable)
        {

            this->stop_mind_client = this->create_client<kapibara_interfaces::srv::StopMind>("/KapiBara/stop_mind");

            while (!this->stop_mind_client->wait_for_service(20*60s)) {
                if (!rclcpp::ok()) {
                throw std::runtime_error("Stop Mind service is not avaliable!");
                }
                RCLCPP_INFO(this->get_logger(), "Stop Mind service not available, waiting again...");
            }

        }

        // wait 1 minute for connection
        this->mqtt_client_->connect()->wait_for(60*1000);

        if( this->mqtt_client_->is_connected() )
        {
            RCLCPP_INFO(this->get_logger(), "Connected to MQTT server at %s",broker.c_str());

            for( const auto& item : this->mqtt_to_ros )
            {
                this->mqtt_client_->subscribe(item.first, 1)->wait();
            }
        }
        else
        {
            throw std::runtime_error("Cannot connect to MQTT server!");
        }

    }

    void stop_mind_from_mqtt(const std::string& msg)
    {
        json emotions = json::parse(msg);

        if( !emotions.contains("stop") || !emotions["stop"].is_boolean())
        {
            return;
        }
        RCLCPP_DEBUG(this->get_logger(), "Got stop mind request");

        auto stop_mind_request = std::make_shared<kapibara_interfaces::srv::StopMind::Request>();

        bool stop = emotions["stop"];

        stop_mind_request->stop = stop;

        auto result = this->stop_mind_client->async_send_request(stop_mind_request);

        json response;

        // // Wait for the result.
        // if (rclcpp::spin_until_future_complete(this->shared_from_this(), result) ==
        //     rclcpp::FutureReturnCode::SUCCESS)
        // {
        //     response["ok"] = true;

        //     RCLCPP_INFO(this->get_logger(), "Stop mind request sent successfully");
        // } else {

        //     response["ok"] = false;

        //     RCLCPP_ERROR(this->get_logger(), "Failed to call service Stop Mind");
        // }

        response["ok"] = true;

        const std::string data = response.dump();

        this->mqtt_client_->publish(mqtt::make_message("/stop_mind_callback", data));
    }

    void emotion_state_from_mqtt(const std::string& msg)
    {

        json emotions = json::parse(msg);

        if( (!emotions.contains("angry")) || (!emotions.contains("fear")) ||
            (!emotions.contains("happiness")) || (!emotions.contains("uncertainty")) ||
            (!emotions.contains("boredom")) )
        {
            return;
        }

        RCLCPP_DEBUG(this->get_logger(), "Got emotion state");

        auto emotion_msg = kapibara_interfaces::msg::Emotions();     
        
        emotion_msg.angry = emotions["angry"];
        emotion_msg.fear = emotions["fear"];
        emotion_msg.happiness = emotions["happiness"];
        emotion_msg.uncertainty = emotions["uncertainty"];
        emotion_msg.boredom = emotions["boredom"];

        this->emotion_publisher->publish(emotion_msg);
    }

    // MQTT message callback
    void message_arrived(mqtt::const_message_ptr msg) override {
        RCLCPP_DEBUG(this->get_logger(), "Received MQTT at topic: %s", msg->get_topic().c_str());

        const std::string& topic = msg->get_topic();

        if( this->mqtt_to_ros.count(topic) > 0 )
        {
            this->mqtt_to_ros[topic](msg->to_string());
        }

    }

    void microphone_callback(const kapibara_interfaces::msg::Microphone::SharedPtr msg)
    {
        RCLCPP_DEBUG(this->get_logger(), "Publishing microphone to MQTT");

        auto& channel1 = msg->channel1;

        auto& channel2 = msg->channel2;

        uint32_t buffer_size = msg->buffor_size;

        uint8_t* buffer = new uint8_t[buffer_size*sizeof(int32_t)*2];

        for( uint32_t i = 0; i < buffer_size*2; i+= 2 )
        {
            uint32_t offset = i*sizeof(int32_t);

            uint8_t* serialized = reinterpret_cast<uint8_t*>(&channel1[i/2]);

            for( uint32_t j =0 ; j < sizeof(int32_t); j++)
            {
                buffer[offset+j] = serialized[j];
            }

        }

        for( uint32_t i = 1; i < buffer_size*2; i+= 2 )
        {
            uint32_t offset = i*sizeof(int32_t);

            uint8_t* serialized = reinterpret_cast<uint8_t*>(&channel2[i/2]);

            for( uint32_t j =0 ; j < sizeof(int32_t); j++)
            {
                buffer[offset+j] = serialized[j];
            }

        }
        
        this->mqtt_client_->publish("/microphone",buffer,buffer_size*sizeof(int32_t)*2);

        delete [] buffer;
    }

    // ROS to MQTT callback
    void emotion_callback(const kapibara_interfaces::msg::Emotions::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "Publishing emotion state to MQTT");
        
        
        json emotion;

        emotion["angry"] = msg->angry;
        emotion["fear"] = msg->fear;
        emotion["happiness"] = msg->happiness;
        emotion["uncertainty"] = msg->uncertainty;
        emotion["boredom"] = msg->boredom;

        const std::string data = emotion.dump();

        this->mqtt_client_->publish(mqtt::make_message("/emotion_state_callback", data));
    }

    void shutdown()
    {
        this->mqtt_client_->disconnect();
    }

    ~MQTTROSBridge()
    {
        delete this->mqtt_client_;
    }

private:
    mqtt::async_client* mqtt_client_;
    std::string mqtt_topic_;

    std::map<std::string,std::function<void(const std::string&)>> mqtt_to_ros;
    rclcpp::Publisher<kapibara_interfaces::msg::Emotions>::SharedPtr emotion_publisher;
    rclcpp::Subscription<kapibara_interfaces::msg::Emotions>::SharedPtr emotion_subscriber;

    rclcpp::Subscription<kapibara_interfaces::msg::Microphone>::SharedPtr microphone_subscriber;


    rclcpp::Client<kapibara_interfaces::srv::StopMind>::SharedPtr stop_mind_client;
};

std::function<void(int)> shutdown_handler;
void signal_handler(int signal) { shutdown_handler(signal); }

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MQTTROSBridge>();

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
