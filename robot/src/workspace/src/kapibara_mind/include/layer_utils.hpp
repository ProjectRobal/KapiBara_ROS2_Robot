#pragma once

#include <cstddef>
#include <fstream>
#include <cstring>

namespace snn
{
    struct LayerHeader
        {
            char header[4]="KAC";
            size_t id=0;
            size_t input_size;
            size_t output_size;
        }; 

    
    LayerHeader readLayerHeader(std::fstream& file)
    {
        LayerHeader header={0};

        size_t fpos=file.tellg();

        file.read((char*)&header,sizeof(header));

        file.seekg(fpos);

        return header;
    }

    bool validateHeader(const LayerHeader& header)
    {
        return strcmp("KAC",header.header) == 0;
    }
    
};