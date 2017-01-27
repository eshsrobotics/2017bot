#ifndef PAPAS_VISION_H__
#define PAPAS_VISION_H__

#include <opencv2/opencv.hpp>

class PapasVision {
    public:
        PapasVision(double goalRejectionThresholdInches, bool writeIntermediateFilesToDisk);
    private:

        // Private stuff goes here

        cv::VideoCapture camera;

        // For debug purposes, we will have the ability to write files to the disk,
        // but in production when we are on the robot, we don't have a disk to write
        // to.
        bool writeIntermediateFilesToDisk;

        // Any target goal that is greater than this distance will be rejected by
        // findgoal(). This is important so we will not get false positives.
        double goalRejectionThresholdInches;
};

#endif // (#ifndef PAPAS_VISION_H__)
