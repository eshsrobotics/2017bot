#ifndef PAPAS_VISION_H__
#define PAPAS_VISION_H__

#include "Config.h"
#include <opencv2/opencv.hpp>
#include <vector>


namespace robot {

class PapasVision {
public:
  PapasVision(const Config &config, double goalRejectionThresholdInches,
              bool writeIntermediateFilesToDisk);

  void findPeg(int pictureFile); // Find the peg reflectors
  void
  findBoiler(int pictureFile); // Find the upper part of the boiler reflectors

  bool getSolutionFound()
      const; // Did the last call to findGoal() obtain a solution?
  double getAzimuthGoalDeg() const; // How far should we turns to face it?
  double getDistToGoalInch() const; // How far away is it?

private:
  const Config &config;

  // Use the given camera to find the goals.
  //
  // Some of our designs call for one camera and some call for two.  But
  // the camera employed does not affect the vision algorithms.
  void findPeg(int pictureFile, cv::VideoCapture &camera);
  void findBoiler(int pictureFile, cv::VideoCapture &camera);

  // It's one camera _right now_.
  cv::VideoCapture camera;

  // Ultimately, the findFoo() functions return their results in these
  // variables.
  // They each have their own accessor methods.
  double distToGoalInch;
  double azimuthGoalDeg;
  bool solutionFound;

  // For debug purposes, we will have the ability to write files to the disk,
  // but in production when we are on the robot, we don't have a disk to write
  // to.
  bool writeIntermediateFilesToDisk;

  // Any target goal that is greater than this distance will be rejected by
  // findgoal(). This is important so we will not get false positives.
  double goalRejectionThresholdInches;

  enum ThresholdingAlgorithm {
    STANDARD,           // Otsu's algorithm by itself
    WITH_BLUR,          // Otsu + Gaussian blurring with a 5x5 kernel
    CONSTANT_THRESHOLD, // Binary filter no Otsu cut off is determined by a
                        // seperate constant, THRESHOLD_GRAYSCALE_CUTOFF
  };

  void getGreenResidual(const cv::Mat &rgbFrame, cv::Mat &greenResidual) const;
  void convertImage(const cv::Mat &input, cv::Mat &output) const;
  void cancelColorsTape(const cv::Mat &input, cv::Mat &output,
                        ThresholdingAlgorithm algorithm = STANDARD) const;
  std::vector<std::vector<cv::Point>> findContours(const cv::Mat &image) const;
  std::vector<std::vector<cv::Point>>
  filterContours(const std::vector<std::vector<cv::Point>> &contours);
  std::vector<std::vector<cv::Point>>
  findBestContourPair(const std::vector<std::vector<cv::Point>> &contours);
  std::vector<cv::Point2f>
  approxPoly(const std::vector<cv::Point> &contour) const;
  std::vector<cv::Point> findBottomPts(const std::vector<cv::Point2f> &points,
                                       cv::Rect rect) const;
  std::vector<cv::Point> findTopPts(const std::vector<cv::Point2f> &points,
                                    cv::Rect rect) const;
  std::vector<cv::Point>
  findGoalContour(const std::vector<std::vector<cv::Point>> &contours) const;
  double findDistToGoal(const std::vector<cv::Point> &topPoints,
                        const std::vector<cv::Point> &bottomPoints) const;
  double findAzimuthGoal(const std::vector<cv::Point> &topPoints,
                         const std::vector<cv::Point> &bottomPoints) const;
};

} // end (namespace robot)

#endif // (#ifndef PAPAS_VISION_H__)
