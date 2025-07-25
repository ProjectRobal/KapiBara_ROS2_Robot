#pragma once

namespace snn
{
    class Linear
    {
        public:

        static inline void activate(SIMDVector& vec)
        {
            // do nothing, placeholder activation
        }

        template<size_t Size>
        static inline void activate(SIMDVectorLite<Size>& vec)
        {
            // do nothing, placeholder activation
        }

        static inline void inverse(SIMDVector& vec)
        {
            // do nothing, placeholder activation
        }
    };
}