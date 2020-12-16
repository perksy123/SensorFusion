
#ifndef __CLOUD_PLANE_H_
#define __CLOUD_PLANE_H_

#include "../../processPointClouds.h"

class CloudPlane
{
    public:
        CloudPlane();

        explicit CloudPlane(int p1, int p2, int p3);

        explicit CloudPlane(const CloudPlane & other);

        ~CloudPlane() = default;

        CloudPlane &operator = (const CloudPlane &other);

        void SetPlaneCoefficients(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

        bool IsInlier(const pcl::PointXYZ &candidate, double tolerance) const;

        bool Contains(int index) const;

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