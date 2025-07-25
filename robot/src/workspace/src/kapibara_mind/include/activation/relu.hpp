#pragma once

namespace snn
{
    class ReLu
    {
        public:

        static inline void activate(SIMDVector& vec)
        {
            SIMDVector filtered=vec>0;

            vec=vec*filtered;
        }

        template<size_t Size>
        static inline void activate(SIMDVectorLite<Size>& vec)
        {
            SIMDVectorLite<Size> filtered=vec>0;

            vec=vec*filtered;
        }

        static inline void inverse(SIMDVector& vec)
        {
            // clear values that are less than zero
            SIMDVector filtered=vec>0;

            vec=vec*filtered;
        }
    };
}