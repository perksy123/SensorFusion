#include "CloudPlane.h"

CloudPlane::CloudPlane()
: P1(0),
  P2(0),
  P3(0),
  ACoeff(0.0),
  BCoeff(0.0),
  CCoeff(0.0),
  DCoeff(0.0)
{              
}

CloudPlane::CloudPlane(int p1, int p2, int p3)
: P1(p1),
  P2(p2),
  P3(p3),
  ACoeff(0.0),
  BCoeff(0.0),
  CCoeff(0.0),
  DCoeff(0.0)
{              
}

CloudPlane::CloudPlane(const CloudPlane & other)
: P1(other.P1),
  P2(other.P2),
  P3(other.P3),
  ACoeff(other.ACoeff),
  BCoeff(other.BCoeff),
  CCoeff(other.CCoeff),
  DCoeff(other.DCoeff)
{
}

CloudPlane &CloudPlane::operator = (const CloudPlane &other)
{
    P1 = other.P1;
    P2 = other.P2;
    P3 = other.P3;
    ACoeff = other.ACoeff;
    BCoeff = other.BCoeff;
    CCoeff = other.CCoeff;
    DCoeff = other.DCoeff;
    return *this;
}

void CloudPlane::SetPlaneCoefficients(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointXYZ &p1 = cloud->at(P1);
    pcl::PointXYZ &p2 = cloud->at(P2);
    pcl::PointXYZ &p3 = cloud->at(P3);

    ACoeff = (p2.y - p1.y) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.y - p1.y);
    BCoeff = (p2.z - p1.z) * (p3.x - p1.x) - (p2.x - p1.x) * (p3.z - p1.z);
    CCoeff = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
    DCoeff = -(ACoeff * p1.x + BCoeff * p1.y + CCoeff * p1.z);
 }

bool CloudPlane::IsInlier(const pcl::PointXYZ &candidate, double tolerance) const
{
    // distance = |Ax + By + Cz + D| / sqrt(A*A + B*B + C*C)
    double pointDistance = std::abs(ACoeff * candidate.x + BCoeff * candidate.y + CCoeff * candidate.z + DCoeff) / std::sqrt(ACoeff * ACoeff + BCoeff * BCoeff + CCoeff * CCoeff);
//    std::cout << "Candidate x,y,z " <<  candidate.x << "," << candidate.y << "," << candidate.z << " distance = " << pointDistance << std::endl;
    return pointDistance < tolerance;
}

bool CloudPlane::Contains(int index) const
{
    return (index == P1 || index == P2 || index == P3);
}

