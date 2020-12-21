
#ifndef __CLOUD_PLANE_H_
#define __CLOUD_PLANE_H_

#include "processPointClouds.h"

template<typename PointT>
class CloudPlane
{
    public:
        CloudPlane()
        : P1(0),
        P2(0),
        P3(0),
        ACoeff(0.0),
        BCoeff(0.0),
        CCoeff(0.0),
        DCoeff(0.0)
        {              
        }

        explicit CloudPlane(int p1, int p2, int p3)
        : P1(p1),
        P2(p2),
        P3(p3),
        ACoeff(0.0),
        BCoeff(0.0),
        CCoeff(0.0),
        DCoeff(0.0)
        {              
        }

        explicit CloudPlane(const CloudPlane<PointT> & other)
        : P1(other.P1),
        P2(other.P2),
        P3(other.P3),
        ACoeff(other.ACoeff),
        BCoeff(other.BCoeff),
        CCoeff(other.CCoeff),
        DCoeff(other.DCoeff)
        {
        }

        ~CloudPlane() = default;

        CloudPlane<PointT> &operator = (const CloudPlane<PointT> &other)
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

        void SetPlaneCoefficients(const typename pcl::PointCloud<PointT>::Ptr cloud)
        {
            PointT &p1 = cloud->at(P1);
            PointT &p2 = cloud->at(P2);
            PointT &p3 = cloud->at(P3);

            ACoeff = (p2.y - p1.y) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.y - p1.y);
            BCoeff = (p2.z - p1.z) * (p3.x - p1.x) - (p2.x - p1.x) * (p3.z - p1.z);
            CCoeff = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
            DCoeff = -(ACoeff * p1.x + BCoeff * p1.y + CCoeff * p1.z);
        }

        bool IsInlier(const PointT &candidate, double tolerance) const
        {
            // distance = |Ax + By + Cz + D| / sqrt(A*A + B*B + C*C)
            double pointDistance = std::abs(ACoeff * candidate.x + BCoeff * candidate.y + CCoeff * candidate.z + DCoeff) / std::sqrt(ACoeff * ACoeff + BCoeff * BCoeff + CCoeff * CCoeff);
            return pointDistance < tolerance;
        }

        bool Contains(int index) const
        {
            return (index == P1 || index == P2 || index == P3);
        }

        void SetP1(int p1) { P1 = p1; }
        void SetP2(int p2) { P2 = p2; }
        void SetP3(int p3) { P3 = p3; }
        int GetP1() const { return P1; }
        int GetP2() const { return P2; }
        int GetP3() const { return P3; }

    private:

        int P1;
        int P2;
        int P3;

        double ACoeff;
        double BCoeff;
        double CCoeff;
        double DCoeff;


};

#endif