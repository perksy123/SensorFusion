#ifndef __GENERATOR_LINES_H__
#define __GENERATOR_LINES_H__

#include "CloudPlane.h"

template<typename PointT>
class Generator
{
    public:
    
        explicit Generator(int maxIndex)
        : MaxIndex(maxIndex)
        {
        }

        ~Generator() = default;

        bool CreatePlane(CloudPlane<PointT> &plane)
        {
            plane.SetP1(GenerateIndex());

            int pt = GenerateIndex();
            while (pt == plane.GetP1())
            {
                pt = GenerateIndex();
            }
            plane.SetP2(pt);

            pt = GenerateIndex();
            while (pt == plane.GetP1() || pt == plane.GetP2())
            {
                pt = GenerateIndex();
            }
            plane.SetP3(pt);

            return true;
        }


    private:

        int GenerateIndex()
        {
            return std::round(static_cast<double>(rand()) / RAND_MAX * (MaxIndex - 1));
        }

        int MaxIndex;
 
};

#endif