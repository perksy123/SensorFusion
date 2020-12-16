#include "Generator.h"
#include <stdlib.h>

Generator::Generator(int maxIndex)
: MaxIndex(maxIndex)
{
}

bool Generator::CreateLine(CloudLine &line)
{
    line.SetStart(GenerateIndex());

    int endIndex = GenerateIndex();
    while (endIndex == line.GetStart())
    {
        endIndex = GenerateIndex();
    }
    line.SetEnd(endIndex);

    return true;
}

bool Generator::CreatePlane(CloudPlane &plane)
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


int Generator::GenerateIndex()
{
    return std::round(static_cast<double>(rand()) / RAND_MAX * (MaxIndex - 1));
}

