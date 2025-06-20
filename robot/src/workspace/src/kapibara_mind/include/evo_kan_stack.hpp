#pragma once

#include "evo_kan_layer.hpp"
#include "simd_vector_lite.hpp"

#include <vector>
#include <memory>

#include "config.hpp"

namespace snn
{

    typedef std::shared_ptr<EvoKanLayerProto> EvoKanLayerPtr;

    class EvoKanStack
    {
        protected:

        std::vector<EvoKanLayerPtr> layers;

        public:

        EvoKanStack()
        {}

        
    };

}