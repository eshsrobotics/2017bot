#ifndef PAPAS_VISION_H__
#define PAPAS_VISION_H__

#include "Config.h"
#include <opencv2/opencv.hpp>
#include <vector>


namespace robot {

class PapasVision {
    public:
        enum SolutionType {
            Boiler, // The target we want to shoot balls into.
            Peg     // The target we want to hang gears on.
        };
    public:
        PapasVision(const Config &config, double goalRejectionThresholdInches,
                    bool writeIntermediateFilesToDisk);

        /// Tries to find the PapasVision peg solution for the given image
        /// file.
        ///
        /// The peg solution finds two green, parallel, vertical bars in the
        /// image which have a separation and proportion roughly corresponding
        /// to the pieces of reflective tape that surround the peg targets in
        /// the 2017 SteamWorks competition.
        ///
        /// WARNING: For the actual code running on the robot, you want live
        /// targets, so only pass an empty string into findPeg().  Otherwise,
        /// the robot will be processing sample images and essentially aiming
        /// at invisible targets.
        ///
        /// @param pictureFile The path to a JPG or PNG sample image to
        ///                    analyze.  If the path is not absolute, it will
        ///                    be considered to be relative to the Config's
        ///                    "cameraFolder" entry.  An empty string will
        ///                    cause the image to be read from the appropriate
        ///                    camera instead.
        ///
        /// @post If a solution is found, getSolutionFound() will return true,
        ///       getAzimuthGoalDeg() will return the azimuth needed to rotate
        ///       the *camera* to face the center of the bars, and
        ///       getDistToGoalInch() will return the distance, in inches, to
        ///       that point at the center of the bars.
        ///
        /// @post If no solution is found, getSolutionFound() will return
        ///       false and the values of getAzimuthGoalDeg() and
        ///       getDistToGoalInch() will be irrelevant.
        void findPeg(std::string pictureFile = "");

        /// For a given imageIndex N, calls findPeg("N.png") or
        /// findPeg("N.jpg"), depending on which one exists in the samples
        /// folder.
        ///
        /// If the file doesn't exist, the camera will be used.
        void findPeg(int imageIndex);


        /// Tries to find the PapasVision boiler solution for the given image
        /// file.
        ///
        /// The peg solution finds two green, parallel, horizontal bars in the
        /// image which have a separation and relative width roughly
        /// corresponding to the pieces of reflective tape near the top of the
        /// boiler in the 2017 SteamWorks competition.
        ///
        /// WARNING: For the actual code running on the robot, you want live
        /// targets, so only pass an empty string into findBoiler().
        /// Otherwise, the robot will be processing sample images and
        /// essentially aiming at invisible targets.
        ///
        /// @param pictureFile The path to a JPG or PNG sample image to
        ///                    analyze.  If the path is not absolute, it will
        ///                    be considered to be relative to the Config's
        ///                    "cameraFolder" entry.  An empty string will
        ///                    cause the image to be read from the appropriate
        ///                    camera instead.
        ///
        /// @post If a solution is found, getSolutionFound() will return true,
        ///       getAzimuthGoalDeg() will return the azimuth needed to rotate
        ///       the *camera* to face the center of the bars, and
        ///       getDistToGoalInch() will return the distance, in inches, to
        ///       that point at the center of the bars.
        ///
        /// @post If no solution is found, getSolutionFound() will return
        ///       false and the values of getAzimuthGoalDeg() and
        ///       getDistToGoalInch() will be irrelevant.
        void findBoiler(std::string pictureFile = "");

        /// For a given imageIndex N, calls findBoiler("N.png") or
        /// findBoiler("N.jpg"), depending on which one exists in the samples
        /// folder.
        ///
        /// If the file doesn't exist, the camera will be used.
        void findBoiler(int imageIndex);

        bool getSolutionFound() const;    // Did the last call to findPeg() or findBoiler() obtain a solution?
        double getAzimuthGoalDeg() const; // How far should we turn to face it?
        double getDistToGoalInch() const; // How far away is it?
        double getCalculationTimeMilliseconds() const; // How long did the call take?


    private:
        const Config &config;

        // Use the given camera (unless the samplePictureFile exists) to find
        // the goals.
        //
        // Some of our designs call for one camera and some call for two.  But
        // the camera employed does not affect the vision algorithms.
        void findSolutionCommon(const std::string& samplePictureFile,
                                cv::VideoCapture &camera,
                                SolutionType solutionType);
        void findPeg(const std::string& samplePictureFile, cv::VideoCapture &camera);
        void findBoiler(const std::string& samplePictureFile, cv::VideoCapture &camera);

        // Ultimately, the findFoo() functions return their results in these
        // variables.
        // They each have their own accessor methods.
        double distToGoalInch;
        double azimuthGoalDeg;
        bool solutionFound;
        double calculationTimeMilliseconds;

        // For debug purposes, we will have the ability to write files to the disk,
        // but in production when we are on the robot, we don't have a disk to write
        // to.
        bool writeIntermediateFilesToDisk;

        // Any target goal that is greater than this distance will be rejected by
        // findgoal(). This is important so we will not get false positives.
        double goalRejectionThresholdInches;

        /////////////////////////
        // Declare our camera. //
        /////////////////////////
        // The configuration below is for using a single camera for both the
        // boiler and the peg solutions.
        //
        // If we want to use two cameras instead, we can do so by declaring a
        // second cv::VideoCapture here, making sure they are assigned
        // different device numbers in the PapasVision constructor, and
        // assigning one to the boilerCamera reference and the other to the
        // pegCamera reference.
        //
        // The tricky bit will be figuring out which device number corresponds
        // to which camera on your system's USB hub.
        cv::VideoCapture camera;
        cv::VideoCapture& boilerCamera;
        cv::VideoCapture& pegCamera;

        enum ThresholdingAlgorithm {
            STANDARD,           // Otsu's algorithm by itself
            WITH_BLUR,          // Otsu + Gaussian blurring with a 5x5 kernel
            CONSTANT_THRESHOLD, // Binary filter no Otsu cut off is determined by a
                                // seperate constant, THRESHOLD_GRAYSCALE_CUTOFF
        };

        std::string getFullPath(const std::string& path) const;
        std::string getFullPath(int imageIndex) const;
        void getGreenResidual(const cv::Mat &rgbFrame, cv::Mat &greenResidual) const;
        void convertImage(const cv::Mat &input, cv::Mat &output) const;
        void cancelColorsTape(const cv::Mat &input, cv::Mat &output,
                              ThresholdingAlgorithm algorithm = STANDARD) const;
        std::vector<std::vector<cv::Point>> findContours(const cv::Mat &image) const;
        std::vector<std::vector<cv::Point>> filterContours(const std::vector<std::vector<cv::Point>> &contours);
        std::vector<std::vector<cv::Point>> findBestContourPair(const std::vector<std::vector<cv::Point>> &contours, SolutionType solutionType);
        std::vector<cv::Point2f> approxPoly(const std::vector<cv::Point> &contour) const;
        std::vector<cv::Point> findBottomPts(const std::vector<cv::Point2f> &points,
                                             cv::Rect rect) const;
        std::vector<cv::Point> findTopPts(const std::vector<cv::Point2f> &points,
                                          cv::Rect rect) const;
        std::vector<cv::Point> findGoalContour(const std::vector<std::vector<cv::Point>> &contours) const;
        double findDistToGoal(const std::vector<cv::Point> &topPoints,
                              const std::vector<cv::Point> &bottomPoints,
                              double realTapeHeight,
                              double elevationAngleDegrees,
                              int imgWidth,
                              int imgHeight) const;
        double findDistToCenterOfImage(const cv::Point& leftmostPoint,
                                       const cv::Point& rightmostPoint,
                                       double knownWidthInches,
                                       int imgWidthPixels) const;
        double findAzimuthGoal(const std::vector<cv::Point> &topPoints,
                               const std::vector<cv::Point> &bottomPoints, int imgWidth) const;
};

} // end (namespace robot)

#endif // (#ifndef PAPAS_VISION_H__)
