
#ifndef __CLOUD_LINE_H_
#define __CLOUD_LINE_H_

#include "../../processPointClouds.h"

class CloudLine
{
    public:
        CloudLine();

        explicit CloudLine(int startIndex, int endIndex);

        explicit CloudLine(const CloudLine & other);

        ~CloudLine() = default;

        bool operator == (const CloudLine &other);

        CloudLine &operator = (const CloudLine &other);

        void SetLineCoefficients(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

        bool IsInlier(const pcl::PointXYZ &candidate, double tolerance) const;

        bool Contains(int index) const;

        void SetStart(int start) { StartIndex = start; }
        void SetEnd(int end) { EndIndex = end; }
        int GetStart() const { return StartIndex; }
        int GetEnd() const { return EndIndex; }

    private:

        int StartIndex;
        int EndIndex;

        double ACoeff;
        double BCoeff;
        double CCoeff;


};

#endif