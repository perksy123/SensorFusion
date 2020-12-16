#ifndef __GENERATOR_LINES_H__
#define __GENERATOR_LINES_H__

#include "CloudLine.h"
#include "CloudPlane.h"
class Generator
{
    public:
    
        explicit Generator(int maxIndex);
        ~Generator() = default;

        bool CreateLine(CloudLine &line);
        bool CreatePlane(CloudPlane &plane);

    private:

        int GenerateIndex();

        int MaxIndex;
 
};

#endif