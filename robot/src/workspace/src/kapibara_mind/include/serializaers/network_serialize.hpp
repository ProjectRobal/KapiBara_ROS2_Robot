#pragma once

/*
    A class used for saving network.

*/

#include <filesystem>
#include <iostream>

#include "network.hpp"
#include "neuron.hpp"
#include "layer_proto.hpp"

#include "nlohmann/json.hpp"

namespace snn
{

    class NetworkSerializer
    {
        public:

        static int save(const Network& network,std::string output_path)
        {
            if(!std::filesystem::exists(output_path))
            {
                if(!std::filesystem::create_directory(output_path))
                {
                    std::cerr<<"Cannot create directory "<<output_path<<std::endl;
                    return -9;
                }
            }
            std::fstream meta_file;

            meta_file.open(output_path+"/metadata.json",std::ios::out);

            if(!meta_file.good())
            {
                std::cerr<<"Cannot create network metadata!"<<std::endl;
                return -1;
            }

            nlohmann::json metadata;

            size_t indent=3;

            metadata["version"]="1.0.0";

            metadata["number_size"]=sizeof(number);

            nlohmann::json layers = nlohmann::json::array();

            for(auto layer : network.getLayers())
            {
                nlohmann::json j;

                j["type"]=layer->getTypeID();

                layer->generate_metadata(j);

                layers.push_back(j);

                indent++;
            }

            metadata["layers"]=layers;

            meta_file<<metadata.dump(indent);

            meta_file.close();

            size_t i=0;

            for(auto layer : network.getLayers())
            {
                std::ofstream layers_file;

                layers_file.open(output_path+"/layer_"+std::to_string(i)+".ln",std::ios::out|std::ios::binary);

                if(layer->save(layers_file)!=0)
                {
                    std::cerr<<"Error saving layer number: "<<i<<std::endl;
                    return -2*i;
                }

                layers_file.close();

                ++i;
            }

            return 0;
        }

        static int load(Network& network,std::string input_path)
        {
            // we don't really care about metadata file.

            size_t i=0;

            for(auto layer : network.getLayers())
            {
                std::ifstream layers_file;

                layers_file.open(input_path+"/layer_"+std::to_string(i)+".ln",std::ios::in|std::ios::binary);

                if(layer->load(layers_file)!=0)
                {
                    std::cerr<<"Error loading layer number: "<<i<<std::endl;
                    return -2*i;
                }

                layers_file.close();

                ++i;
            }

            return 0;
        }
    
    };
};