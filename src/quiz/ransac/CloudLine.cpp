#include "CloudLine.h"

CloudLine::CloudLine()
: StartIndex(0),
  EndIndex(0),
  ACoeff(0.0),
  BCoeff(0.0),
  CCoeff(0.0)
{              
}

CloudLine::CloudLine(int startIndex, int endIndex)
: StartIndex(startIndex),
  EndIndex(endIndex),
  ACoeff(0.0),
  BCoeff(0.0),
  CCoeff(0.0)
{              
}

CloudLine::CloudLine(const CloudLine & other)
: StartIndex(other.StartIndex),
  EndIndex(other.EndIndex),
  ACoeff(other.ACoeff),
  BCoeff(other.BCoeff),
  CCoeff(other.CCoeff)
{
}

bool CloudLine::operator == (const CloudLine &other)
{
    if ( (StartIndex == other.StartIndex && EndIndex == other.EndIndex) ||
            (StartIndex == other.EndIndex && EndIndex == other.StartIndex) )
    {
        return true;
    }

    return false;
}

CloudLine &CloudLine::operator = (const CloudLine &other)
{
    StartIndex = other.StartIndex;
    EndIndex = other.EndIndex;
    ACoeff = other.ACoeff;
    BCoeff = other.BCoeff;
    CCoeff = other.CCoeff;
    return *this;
}

void CloudLine::SetLineCoefficients(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointXYZ &start = cloud->at(StartIndex);
    pcl::PointXYZ &end = cloud->at(EndIndex);

    std::cout << "Start x,y = " << start.x << "," << start.y << std::endl;
    std::cout << "End x,y = " << end.x << "," << end.y << std::endl;

    ACoeff = start.y - end.y;
    BCoeff = end.x - start.x;
    CCoeff = start.x * end.y - end.x * start.y;

    std::cout << "Coeffs A = " << ACoeff << " B = " << BCoeff << " C = " << CCoeff << std::endl;
}

bool CloudLine::IsInlier(const pcl::PointXYZ &candidate, double tolerance) const
{
    // distance = |Ax + By + C| / sqrt(A*A + B*B)
    double pointDistance = std::abs(ACoeff * candidate.x + BCoeff * candidate.y + CCoeff) / std::sqrt(ACoeff * ACoeff + BCoeff * BCoeff);
    std::cout << "Candidate x,y " <<  candidate.x << "," << candidate.y << " distance = " << pointDistance << std::endl;
    return pointDistance < tolerance;
}

bool CloudLine::Contains(int index) const
{
    return (index == StartIndex || index == EndIndex);
}

