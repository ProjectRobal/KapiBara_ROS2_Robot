#include <rclcpp/rclcpp.hpp>
#include <mqtt/async_client.h>


#include <kapibara_interfaces/msg/emotions.hpp>

#include <map>
#include <functional>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

class MQTTROSBridge : public rclcpp::Node,virtual mqtt::callback {
public:
    MQTTROSBridge() : Node("mqtt_ros_bridge") {
        // Declare ROS 2 parameters
        this->declare_parameter("mqtt_broker", "tcp://0.0.0.0:1883");
        this->declare_parameter("ros_topic", "ros_to_mqtt");

        // Get parameters
        std::string broker = this->get_parameter("mqtt_broker").as_string();
        ros_topic_ = this->get_parameter("ros_topic").as_string();

        // ROS 2 Publisher & Subscriber
        // publisher_ = this->create_publisher<std_msgs::msg::String>(ros_topic_, 10);

        this->emotion_publisher = this->create_publisher<kapibara_interfaces::msg::Emotions>("emotions", 10);
        this->emotion_subscriber = this->create_subscription<kapibara_interfaces::msg::Emotions>("emotions", 10,
            std::bind(&MQTTROSBridge::emotion_callback, this, std::placeholders::_1));


        this->mqtt_client_ = new mqtt::async_client(broker,"0","./persit");

        this->mqtt_to_ros["/emotion_state"] = std::bind(&MQTTROSBridge::emotion_state_from_mqtt,this,std::placeholders::_1);

        // MQTT Setup
        this->mqtt_client_->set_callback(*this);

        RCLCPP_INFO(this->get_logger(), "Connecting to MQTT server at %s",broker.c_str());

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

        mqtt_client_->publish(mqtt::make_message("/emotion_state_callback", data));
    }

    ~MQTTROSBridge()
    {
        this->mqtt_client_->disconnect();

        delete this->mqtt_client_;
    }

private:
    mqtt::async_client* mqtt_client_;
    std::string mqtt_topic_;
    std::string ros_topic_;

    std::map<std::string,std::function<void(const std::string&)>> mqtt_to_ros;
    rclcpp::Publisher<kapibara_interfaces::msg::Emotions>::SharedPtr emotion_publisher;
    rclcpp::Subscription<kapibara_interfaces::msg::Emotions>::SharedPtr emotion_subscriber;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MQTTROSBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
