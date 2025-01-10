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


#include <rclcpp/rclcpp.hpp>

// quanterion
#include <sensor_msgs/msg/imu.hpp>
// 2d depth points
#include <sensor_msgs/msg/point_cloud2.hpp>
// mean face embedded
#include <kapibara_interfaces/msg/face_embed.hpp>
// emotions, network will be triggered by emotions message
#include <kapibara_interfaces/msg/emotions.hpp>
// spectogram
#include <sensor_msgs/msg/image.hpp>

// twist message used for ros2 control
#include <geometry_msgs/msg/twist.hpp>

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

#include "RResNet.hpp"


/*

 To save on memory we can store weights on disk and then load it to ram as a buffer.

  load 32 numbers from file.
  when execution is made load another 32 numbers in background.

  Each kac block will have line with N weights , each representing each population, plus one id with indicate what weight is choosen. In file we store
  weight with coresponding reward.
  When executing operation we will load each weight from each population.

 Each weights is going to have it's own population. We choose weight from population.
 Active weight gets reward, the lower reward is the higher probability of replacing weight,
 between 0.01 to 0.5 . 
 
 Sometimes the weights in population are going to be replace, the worst half of poplulation.
Some weights will be random some are going to be generated from collection of best weights, plus mutations.
The wieghts that achived positive rewards are collected and then used as replacment with some mutations
maybe. 
 Or to save on space we can generate new weights just using random distribution.

*/

#include "block_kac.hpp"

size_t snn::BlockCounter::BlockID = 0;

size_t snn::LayerCounter::LayerIDCounter = 0;

/*

    KapiBara input variables:

    quanterion - 4 values
    speed from encoders - 2 values
    spectogram 16x16 - 256 values
    2d points array from camera, compressed to 16x16 - 256 values
    face embeddings - 64 values when more than two faces are spotted average thier embeddings

    Total 518 values


*/

#define MEMBERS_COUNT (64u)

class KapiBaraMind : public rclcpp::Node
{

    snn::Arbiter arbiter;


    std::shared_ptr<snn::LayerKAC<582,64,20>> encoder;

    std::shared_ptr<snn::RResNet<64,256,20>> recurrent1;

    std::shared_ptr<snn::LayerKAC<64,32,20,snn::ReLu>> layer1;

    std::shared_ptr<snn::RResNet<32,128,20>> recurrent2;

    std::shared_ptr<snn::LayerKAC<32,MEMBERS_COUNT,20,snn::SoftMax>> decision;

    std::vector<std::shared_ptr<KapiBara_SubLayer>> layers;

    snn::SIMDVectorLite<582> inputs;


    void init_network()
    {

        this->encoder = std::make_shared<snn::LayerKAC<582,64,20>>();

        this->recurrent1 = std::make_shared<snn::RResNet<64,256,20>>();

        this->layer1 = std::make_shared<snn::LayerKAC<64,32,20,snn::ReLu>>();

        this->recurrent2 = std::make_shared<snn::RResNet<32,128,20>>();

        this->decision = std::make_shared<snn::LayerKAC<32,MEMBERS_COUNT,20,snn::SoftMax>>();   

        this->arbiter.addLayer(encoder);
        this->arbiter.addLayer(recurrent1);
        this->arbiter.addLayer(layer1);
        this->arbiter.addLayer(recurrent2);
        this->arbiter.addLayer(decision);

        this->layers.reserve(MEMBERS_COUNT);

        for( size_t i = 0; i < MEMBERS_COUNT; ++i )
        {

            auto layer = std::make_shared<KapiBara_SubLayer>();
    
            // auto sub_layer2 = std::make_shared<snn::LayerKAC<128,64,20,snn::ReLu>>();

            this->arbiter.addLayer(layer);
            this->layers.push_back(layer);
            
        }

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
                RCLCPP_DEBUG(this->get_logger(),"Network has been saved!");
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

    public:

    int8_t save_network()
    {
        return this->arbiter.save(this->checkpoint_filename());
    }

    KapiBaraMind()
    : Node("kapibara_mind")
    {

        this->declare_parameter("checkpoint_dir", "/app/src/mind.kac");

        this->inputs = snn::SIMDVectorLite<582>(0);

        this->init_network();

        // add all required subscriptions




        // one publisher for ros2 control cmd
    }

    snn::SIMDVectorLite<64> fire_network(const snn::SIMDVectorLite<582>& input)
    {

        auto output = this->encoder->fire(input);

        // output = output / output.size();

        auto output1 = this->recurrent1->fire(output);

        auto output2 = this->layer1->fire(output1);

        // output2 = output2 / output2.size();

        auto output3 = this->recurrent2->fire(output2);

        auto picker = this->decision->fire(output3);

        // select layer based on softmax output of picker

        auto out = this->layers[0]->fire(output3);

        // arbiter.applyReward(x);

        arbiter.shuttle();

    }

};



int main(int argc,char** argv)
{   
    rclcpp::init(argc, argv);

    auto node = std::make_shared<KapiBaraMind>();

    rclcpp::spin(node);

    node->save_network();

    rclcpp::shutdown();

    return 0;

}

