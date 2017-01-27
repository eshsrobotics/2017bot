#ifndef PAPAS_VISION_H__
#define PAPAS_VISION_H__

#include "Config.h"
#include <vector>
#include <opencv2/opencv.hpp>

namespace robot {

class PapasVision {
    public:
        PapasVision(const Config& config, double goalRejectionThresholdInches, bool writeIntermediateFilesToDisk);

        void findGoal(int pictureFile);

    private:

        // Private stuff goes here
        Config config;
        cv::VideoCapture camera;

        //It is set to false in the beginning and then if their is a goal it is set true. We use getSolutionFound to return the value to the public.
        bool solutionFound;

        // For debug purposes, we will have the ability to write files to the disk,
        // but in production when we are on the robot, we don't have a disk to write
        // to.
        bool writeIntermediateFilesToDisk;

        // Any target goal that is greater than this distance will be rejected by
        // findgoal(). This is important so we will not get false positives.
        double goalRejectionThresholdInches;

        void getGreenResidual(const cv::Mat& rgbFrame, cv::Mat& greenResidual) const;
        void convertImage(const cv::Mat& input, cv::Mat& output) const;
        void cancelColorsTape(const cv::Mat& input, cv::Mat& output) const;
        std::vector<std::vector<cv::Point> > findContours(const cv::Mat& image) const;
        std::vector<std::vector<cv::Point> > filterContours(const std::vector<std::vector<cv::Point> >& contours);
        std::vector<cv::Point2f> approxPoly(const std::vector<cv::Point>& contour) const;
        std::vector<cv::Point> findBottomPts(const std::vector<cv::Point2f>& points, cv::Rect rect) const;
        std::vector<cv::Point> findTopPts(const std::vector<cv::Point2f>& points, cv::Rect rect) const;
};

} // end (namespace robot)

#endif // (#ifndef PAPAS_VISION_H__)
