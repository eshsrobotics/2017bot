#include "PapasVision.h"

#include <array>
#include <cmath>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <sstream>
#include <tuple>

using namespace cv;
using namespace std;
using namespace std::chrono;

namespace robot {

//////////////////////////////////////////////
// Critical constants for findDistToGoal(). //
//////////////////////////////////////////////

const double DEGREES_TO_RADIANS = 3.1415926 / 180.0;

// From the official C270 camera specs.
const double HORIZ_FOV_DEG = 60;
const double HORIZ_FOV_RAD = HORIZ_FOV_DEG * DEGREES_TO_RADIANS;

// The height of the top boiler band in this year's competition, in inches.
const double BOILER_REAL_TAPE_TOP_HEIGHT_INCHES = 4;

// The height of the bottom boiler band in this year's competition, in inches.
const double BOILER_REAL_TAPE_BOTTOM_HEIGHT_INCHES = 2;

// The distance from the top of the bottom boiler tape edge to the bottom of the
// top boiler tape edge.
const double BOILER_REAL_TAPE_SEPARATION_INCHES = 4;

// The height of both pieces of reflective tape that surround the peg.
const double PEG_REAL_TAPE_HEIGHT_INCHES = 5;

// The width of both pieces of reflective tape that surround the peg.
const double PEG_REAL_TAPE_WIDTH_INCHES = 2;

// The distance from the left edge of the tape on the left side of the peg to
// the right edge of the tape on the peg's right side.
const double PEG_REAL_TAPE_OUTER_WIDTH_INCHES = 10.25;

// The inner width -- the distance from the right edge of the left peg tape to
// the left edge of the right peg tape.
const double PEG_REAL_TAPE_SEPARATION_INCHES = PEG_REAL_TAPE_OUTER_WIDTH_INCHES - 2 * PEG_REAL_TAPE_WIDTH_INCHES;

// The camera elevation angles, in degrees, from the horizontal plane.  If you
// adjust any of the cameras, be sure to update this.
//
// If it turns out that the system only has one camera in it, then
// that just means that these constants will be set to the same value.
const double BOILER_CAM_EL_DEG	= 20;
const double BOILER_CAM_EL_RAD	= BOILER_CAM_EL_DEG * DEGREES_TO_RADIANS;
const double PEG_CAM_EL_DEG	= BOILER_CAM_EL_DEG;                   // For now, Peg and Boiler are the same cam.
const double PEG_CAM_EL_RAD	= PEG_CAM_EL_DEG * DEGREES_TO_RADIANS;

// We used The GIMP's Colors->Threshold tool on the green residual
// image to determine this empirically.
const double THRESHOLD_GRAYSCALE_CUTOFF = 25;

// This is a percentage (i.e., a number between 0.0 and 1.0.)  If the smaller
// of any two contours being compared by findBestContourPair() has an area
// that is within this percentage of the larger contour's area, then we
// consider the areas to be close enough.
//
// For example, assume this tolerance was 0.15, contour c1 had an area of 50,
// and c2 had an area of 38.  Since 38 > 42.5 [that is, 38 > 50 * (1.0-0.15)],
// we'd consider the c1,c2 pair to close enough in area to not reject.
const double CONTOUR_PAIR_AREA_DEVIATION_TOLERANCE = 0.15;

// Similar to the above, these are percentages that we use to reject contour
// pairs when comparing them by the width and height of their bounding boxes.
// One dimension or the other is expected to be similar for contours
// representing the parallel bands we want.
const double CONTOUR_PAIR_BOUNDING_BOX_WIDTH_DEVIATION_TOLERANCE = 0.18;
const double CONTOUR_PAIR_BOUNDING_BOX_HEIGHT_DEVIATION_TOLERANCE = CONTOUR_PAIR_BOUNDING_BOX_WIDTH_DEVIATION_TOLERANCE;

// The Peg solution relies on finding contours that are as
// 'rectangular' as possible, which we quantify by dividing the area
// of the contour by the area of its minAreaRect().  The ratio must be
// greater than this number or else the contour will be considered too
// irregular and will be rejected.
const double MINIMUM_RECTANGLE_AREA_TO_TRUE_AREA_RATIO = 0.80;

/////////////////////////////
// Global utility methods. //
/////////////////////////////

// This template function works for Points, Point2fs, and anything that has
// public members called x and y.
template <typename Point>
double distance(const Point& p1, const Point& p2) {
    return sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
}

// Saves the given image to a file on disk that uses the following naming scheme:
//
//   prefix + "_" + index + "_" + suffix
void save(const string &pathPrefix, int index, const string &suffix,
          const Mat &imageToWrite) {

    stringstream stream;
    stream << pathPrefix << "_" << index << "_" << suffix;
    imwrite(stream.str(), imageToWrite);
}

// Calculate the vertical field of view of a camera by taking its horizontal
// field of view and multiplying this by its aspect ratio.
//
// This assumes square pixels.
double verticalFOV(double horizontalFOV, int width, int height) {
    return (horizontalFOV * height) / width;
}

/////////////////////////////////////
// Constructor and public methods. //
/////////////////////////////////////

PapasVision::PapasVision(const Config &config_,
                         double goalRejectionThresholdInches_,
                         bool writeIntermediateFilesToDisk_)
    : config(config_), distToGoalInch(0), azimuthGoalDeg(0),
      solutionFound(false), calculationTimeMilliseconds(0),
      writeIntermediateFilesToDisk(writeIntermediateFilesToDisk_),
      goalRejectionThresholdInches(goalRejectionThresholdInches_),
      camera(0), boilerCamera(camera), pegCamera(camera) {

    cout << "Welcome to OpenCV " << CV_VERSION << "\n";
}

// Converts a relative path (to a sample image) to an absolute path.
string PapasVision::getFullPath(const string& path) const {

    if (path.size() == 0) { // Not a path--use the camera.
        return path;
    }

    if (path.front() == '/') { // Already an absolute path.
        return path;
    }

    string prefix;
    if (config.cameraFolder() == "") {
        // No folder to get samples from?  Use the current directory.
        prefix = "./";
    } else {
        prefix = config.cameraFolder();
        if (prefix.back() != '/') {
            prefix += "/";
        }
    }

    return prefix + path;
}

// Converts an integer N to the full path to N.png or N.jpg, if either file
// exists in the samples folder (PNG files are tried first.)
string PapasVision::getFullPath(int imageIndex) const {

    string prefix;
    if (config.cameraFolder() == "") {
        // No folder to get samples from?  Use the current directory.
        prefix = "./";
    } else {
        // Use whatever the config file told us to use.
        prefix = config.cameraFolder();
        if (prefix.back() != '/') {
            prefix += "/";
        }
    }

    stringstream stream;
    stream << prefix << imageIndex;

    // Is this N.jpg or N.png?
    vector<string> extensionsToTry = { "png", "jpg" };
    for (string extension : extensionsToTry) {
        string filename = stream.str() + "." + extension;
        ifstream infile(filename);
        if (infile) {
            return filename;
        }
    }
    return "";
}


void PapasVision::findBoiler(const string& samplePictureFile, VideoCapture& camera) {
    findSolutionCommon(samplePictureFile, camera, Boiler);
}

void PapasVision::findPeg(const string& samplePictureFile, VideoCapture &camera) {
    findSolutionCommon(samplePictureFile, camera, Peg);
}


// Public interface.
bool PapasVision::getSolutionFound() const { return solutionFound; }
double PapasVision::getAzimuthGoalDeg() const { return azimuthGoalDeg; }
double PapasVision::getDistToGoalInch() const { return distToGoalInch; }
double PapasVision::getCalculationTimeMilliseconds() const { return calculationTimeMilliseconds; }

void PapasVision::findPeg(string samplePictureFile)    { findPeg(getFullPath(samplePictureFile),    pegCamera);    }
void PapasVision::findPeg(int imageIndex)              { findPeg(getFullPath(imageIndex),           pegCamera);    }
void PapasVision::findBoiler(string samplePictureFile) { findBoiler(getFullPath(samplePictureFile), boilerCamera); }
void PapasVision::findBoiler(int imageIndex)           { findBoiler(getFullPath(imageIndex),        boilerCamera); }

//////////////////////////////////////////////////////////////////////////////
// Our most important public functions.  Ultimately, their purpose is to use
// OpenCV's computer vision analysis functions to calculate three values from
// the latest camera image: SolutionFound, distToGoalInch, and azimuthGoalDeg.
//////////////////////////////////////////////////////////////////////////////

// This function does the leg-work for finding the vision solutions.  The only
// input it needs (either than input image sources) is the type of solution it
// should find.

// @param pictureFile  The absolute path to an image file.  An empty string
//                     will cause the image to be read from the appropriate
//                     camera instead.
//
// @param camera       The camera to use if pictureFile is an empty string.
//
// @param solutionType Either PapasVision::Boiler or PapasVision::Peg.

void PapasVision::findSolutionCommon(const string& samplePictureFile, VideoCapture &camera,
                                     SolutionType solutionType) {

    auto startTime = high_resolution_clock::now();

    // Determine whether or not the camera is present.  If not, we'll use the fake
    // images in 2017bot/samples.
    bool useCamera = (samplePictureFile == "");

    // A prefix intended to give the intermediate images written to disk
    // unique filenames.
    string pathPrefix;

    Mat output;

    // A sort of "screenshot" of the initial camera image or initial sample
    // image.
    Mat frame;

    // The index of the current debugging image we're writing to.
    int index = 1;

    if (!useCamera) {

        // If the file does not exist make the caller very much aware of this
        // fact.
        //
        // This won't affect production because useCamera will always be true
        // in that case.
        {
            ifstream infile(samplePictureFile);
            if (!infile) {
                throw runtime_error("The image path '" + samplePictureFile + "' cannot be opened for reading.");
            }
        }

        // Determine the prefix we'll use for writing future intermediate
        // images.
        string fileWithoutExtension;
        if (samplePictureFile.rfind(".") != string::npos) {
            fileWithoutExtension = samplePictureFile.substr(0, samplePictureFile.rfind("."));
        } else {
            fileWithoutExtension = samplePictureFile;
        }
        pathPrefix = fileWithoutExtension;

        // Read from the fake sample image.
        frame = imread(samplePictureFile);

    } else {

        // Determine the prefix we'll use for writing future intermediate
        // images.
        //
        // To make the camera images globally unique, prefix them with the
        // current date and time.
        array<char, 100> buffer;
        time_t now;
        time(&now);
        strftime(&buffer[0], buffer.size(), "%Y-%m-%dT%H-%M-%S", localtime(&now));

        pathPrefix = (config.cameraFolder() == "" ? "." : config.cameraFolder());
        if (pathPrefix.back() != '/') {
            pathPrefix += "/";
        }
        pathPrefix += &buffer[0];

        // Read from the real camera.
        bool cameraOpenedSuccessfully = camera.isOpened();
        bool cameraReadSuccessfully = false;
        if (cameraOpenedSuccessfully) {
            cameraReadSuccessfully = camera.read(frame);
        }

        if (!cameraOpenedSuccessfully) {
            // TODO: we should probably tell the driver station that we lost
            // our camera, but that might cause a lot of spam.
            cerr << "Camera for " << (solutionType == Boiler ? "Boiler" : "Peg") << " not ready (could not be opened.)\n";
            return;
        } else if (!cameraReadSuccessfully) {
            // TODO: we should probably tell the driver station that we lost
            // our camera, but that might cause a lot of spam.
            cerr << "Camera for " << (solutionType == Boiler ? "Boiler" : "Peg") << " not ready (read failed.)\n";
            return;
        }


        if (writeIntermediateFilesToDisk) {
            save(pathPrefix, index++, "original.png", frame);
        }
    }

    Mat greenFrameRes;
    getGreenResidual(frame, greenFrameRes);
    if (writeIntermediateFilesToDisk) {
        save(pathPrefix, index++, "green_residual.png", greenFrameRes);
    }

    Mat greenFrameResFilt;

    // This function that is in opencv removes noise and removes texture from
    // the image.
    //
    // See
    // http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/MANDUCHI1/Bilateral_Filtering.html
    // for more information.
    const double sigmaColor = 280.0;
    const double sigmaSpace = 280.0;
    bilateralFilter(greenFrameRes, greenFrameResFilt, 9, sigmaColor, sigmaSpace);
    if (writeIntermediateFilesToDisk) {
        save(pathPrefix, index++, "green_residual_filt.png", greenFrameResFilt);
    }

    cancelColorsTape(greenFrameResFilt, output, CONSTANT_THRESHOLD);
    if (writeIntermediateFilesToDisk) {
        save(pathPrefix, index++, "cancelcolors.png", output);
    }

    // A serious of calls to reduce the number of insignificant contours in the
    // already-thresholded monochrome cancelcolors image.
    erode(output, output, getStructuringElement(MORPH_OPEN, Size(5, 5)));
    dilate(output, output, getStructuringElement(MORPH_OPEN, Size(5, 5)));
    dilate(output, output, getStructuringElement(MORPH_OPEN, Size(5, 5)));
    erode(output, output, getStructuringElement(MORPH_OPEN, Size(5, 5)));

    if (writeIntermediateFilesToDisk) {
        save(pathPrefix, index++, "cancelcolors_morphfilt.png", output);
    }

    // VERY important: turns the black-and-white image into a series of
    // interesting contours.  (We then render these in red for the aid of
    // you, my dear programmer.)
    vector<vector<Point>> contours = findContours(output);

    Mat frameContours = frame.clone();
    for (unsigned int i = 0; i < contours.size(); i++) {
        drawContours(frameContours, contours, i, Scalar(0, 0, 255));
    }
    if (writeIntermediateFilesToDisk) {
        save(pathPrefix, index++, "frame_contours.png", frameContours);
    }


    // Since the intermediate images from this point on are specific to the
    // solution type, it makes sense to alter the prefix.
    string solutionPrefix = (solutionType == Boiler ? "boiler_" : "peg_");

    // This year's vision target consists of two parallel bands of reflective
    // tape.  We want to find the contours in our contour list that best
    // represent those bands, and there's no better way to do that than to
    // compare each pair of contours one by one.
    //
    // The first two contours in the array after the call are the two
    // best-scoring contours.  The other are just there so we can draw them.

    contours = findBestContourPair(contours, solutionType);

    Mat frameFiltContoursImage = frame.clone();
    array<Scalar, 3> sortedContourPairColors = { // These are B, G, R, *not* R, G ,B.
        Scalar(0, 255, 0),                       // Green light, highest score
        Scalar(0, 255, 255),                     // Yellow light, take warning
        Scalar(0, 0, 255),                       // Red light, not very good
    };
    unsigned int n = contours.size();
    if (n > 6) {
        n = 6;
    }
    for (unsigned int i = 0; i < n; ++i) {

        // Draw the oriented bounding box surrounding the contour.
        array<Point2f, 4> floatVertices;
        minAreaRect(contours[i]).points(floatVertices.data());
        array<Point, 4> vertices;
        copy(floatVertices.begin(), floatVertices.end(), vertices.begin());
        fillConvexPoly(frameFiltContoursImage, vertices.data(), vertices.size(), Scalar(255, 16, 255));

        // Draw the contour itself.
        drawContours(frameFiltContoursImage, contours, i, sortedContourPairColors.at(i / 2));
    }
    if (writeIntermediateFilesToDisk) {
        save(pathPrefix, index++, solutionPrefix + "frame_filtcontours.png", frameFiltContoursImage);
    }


    // If a pair of contours won, we assume that it represents the two parallel
    // bands we were looking for.
    solutionFound = false;
    if (contours.size() > 0) {

        const vector<Point>& contour1_ = contours[0];
        const vector<Point>& contour2_ = contours[1];

        vector<Point2f> contour1;
        vector<Point2f> contour2;

        // Convert the Points into Point2fs.
        copy(contour1_.begin(), contour1_.end(), back_inserter(contour1));
        copy(contour2_.begin(), contour2_.end(), back_inserter(contour2));

        // Both vision solutions (i.e., Peg and Boiler) call for finding the
        // corners of the winning contours closes to the bottom left and
        // bottom right.  BottomPoints1 has to be above BottomPoints2.

        vector<Point> bottomPoints1 = findBottomPts(contour1, boundingRect(contour1));
        vector<Point> bottomPoints2 = findBottomPts(contour2, boundingRect(contour2));

        if (bottomPoints1[0].y > bottomPoints2[0].y) {
            swap(bottomPoints1, bottomPoints2);
        }

        Mat framePoints = frame.clone();
        const int radius = 5, thickness = 2;
        circle(framePoints, bottomPoints1.at(0), radius, Scalar(0, 192, 255), thickness);
        circle(framePoints, bottomPoints1.at(1), radius, Scalar(0, 192, 255), thickness);
        circle(framePoints, bottomPoints2.at(0), radius, Scalar(255, 64, 255), thickness);
        circle(framePoints, bottomPoints2.at(1), radius, Scalar(255, 64, 255), thickness);
        if (writeIntermediateFilesToDisk) {
            save(pathPrefix, index++, solutionPrefix + "frame_points.png", framePoints);
        }

        // Alright, we have everything we need.  Trig time!
        if (solutionType == Boiler) {

            distToGoalInch = findDistToGoal(bottomPoints1,
                                            bottomPoints2,
                                            BOILER_REAL_TAPE_BOTTOM_HEIGHT_INCHES + BOILER_REAL_TAPE_SEPARATION_INCHES,
                                            BOILER_CAM_EL_DEG,
                                            frame.size().width,
                                            frame.size().height);
            azimuthGoalDeg = findAzimuthGoal(bottomPoints1, bottomPoints2, frame.size().width);

        } else {

            Point leftmostBottomPoint1 = (bottomPoints1.at(0).x < bottomPoints1.at(1).x ? bottomPoints1.at(0) : bottomPoints1.at(1));
            Point leftmostBottomPoint2 = (bottomPoints2.at(0).x < bottomPoints2.at(1).x ? bottomPoints2.at(0) : bottomPoints2.at(1));
            Point leftmostPoint = (leftmostBottomPoint1.x < leftmostBottomPoint2.x ? leftmostBottomPoint1 : leftmostBottomPoint2);

            Point rightmostBottomPoint1 = (bottomPoints1.at(0).x > bottomPoints1.at(1).x ? bottomPoints1.at(0) : bottomPoints1.at(1));
            Point rightmostBottomPoint2 = (bottomPoints2.at(0).x > bottomPoints2.at(1).x ? bottomPoints2.at(0) : bottomPoints2.at(1));
            Point rightmostPoint = (rightmostBottomPoint1.x > rightmostBottomPoint2.x ? rightmostBottomPoint1 : rightmostBottomPoint2);

            distToGoalInch = findDistToCenterOfImage(leftmostPoint, rightmostPoint, PEG_REAL_TAPE_OUTER_WIDTH_INCHES, frame.size().width);
            azimuthGoalDeg = findAzimuthGoal(bottomPoints1, bottomPoints2, frame.size().width);
        }

        if (distToGoalInch > goalRejectionThresholdInches) {
            if (writeIntermediateFilesToDisk) {
                cout << "Sorry, integrity check failed (distance to goal was found to "
                    "be "
                     << setprecision(4) << distToGoalInch
                     << " inches, but we were told to reject anything greater than"
                     << goalRejectionThresholdInches
                     << " inches.)  Image file: " << samplePictureFile << "\n";
            }
        } else {
            solutionFound = true;
        }
    }

    calculationTimeMilliseconds = duration<double, std::milli>(high_resolution_clock::now() - startTime).count();

    // If control made it here, no solution was found for the given solutionType.
}

///////////////////////////////////////////
// Computer vision processing functions. //
///////////////////////////////////////////

void PapasVision::getGreenResidual(const cv::Mat &rgbFrame,
                                   cv::Mat &greenResidual) const {

    array<Mat, 3> listRGB;
    split(rgbFrame, &listRGB[0]);
    greenResidual = listRGB.at(1);
    scaleAdd(listRGB.at(0), -0.5, greenResidual, greenResidual);
    scaleAdd(listRGB.at(2), -0.5, greenResidual, greenResidual);
}


// scalar params: H(0-180), S(0-255), V(0-255)
// This function takes a grey scale image through input. The output is a black
// and white image which is modified
// by Otsu's thresholding algorithm.
//
// @param input Grayscale input image.
// @param output The appropriately-thresholded result image.
// @param algorithm STANDARD to just use straight-up Otsu, WITH_BLUR to do a
//                  Gaussian blur before the Otsu filter, and
//                  CONSTANT_THRESHOLD to use a direct binary threshold
//                  with THRESHOLD_GRAYSCALE_CUTOFF as the cutoff point.
void PapasVision::cancelColorsTape(const Mat &input, Mat &output,
                                   ThresholdingAlgorithm algorithm) const {
    switch (algorithm) {
        case STANDARD:
            threshold(input, output, 0, 255, THRESH_BINARY + THRESH_OTSU);
            break;
        case WITH_BLUR: {
            Mat blur;
            GaussianBlur(input, blur, Size(5, 5), 0);
            threshold(blur, output, 0, 255, THRESH_BINARY + THRESH_OTSU);
            break;
        }
        case CONSTANT_THRESHOLD:
            threshold(input, output, THRESHOLD_GRAYSCALE_CUTOFF, 255, THRESH_BINARY);
            break;
    }
}

vector<vector<Point>> PapasVision::findContours(const Mat &image) const {
    // From
    // http://docs.opencv.org/master/d3/dc0/group__imgproc__shape.html#ga17ed9f5d79ae97bd4c7cf18403e1689a:
    //
    // contours
    //   Detected contours. Each contour is stored as a vector of points
    //   (e.g. std::vector<std::vector<cv::Point> >).
    // hierarchy
    //   Optional output vector (e.g. std::vector<cv::Vec4i>), containing
    //   information about the image topology.

    vector<vector<Point>> contours; // Former List<MatOfPoint> in Java-Land
    vector<Vec4i> hierarchy;        // Formerly Mat() in Java-Land

    cv::findContours(image, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    return contours;
}


// =========================================================================
// This function is used to find out of the list of contours two parallel
// contours that are close together.
//
// As it turns out, the peg and the boiler will both have reflective tape in
// those arrangements.
//
// @param contours     An array of contours -- essentially, polygons that
//                     surround interesting green targets in the camera image.
// @param solutionType Either Boiler or Peg.
//
// @return An even _more_ interesting array of the contours that we really
//         like, because they kind of look like the sort of parallel bands
//         that we want to shoot at.
//
//         Either 0, 2, 4, or 6 contours will be returned.  The only contours
//         that matter for vision calculations are the first and second
//         contour, since those will represent the highest-scoring pair.  The
//         other pairs that are returned (if any) are the second- and
//         third-highest scoring pairs, and should not be considered for
//         vision calculations.
//
//         If no contours are returned, then all of the pairs were rejected by
//         our internal heuristics, meaning there's probably no PapasVision
//         solution at the moment.
// =========================================================================
vector<vector<Point>> PapasVision::findBestContourPair(const vector<vector<Point>>& contours,
                                                       SolutionType solutionType) {

    typedef vector<Point> Contour;
    typedef tuple<double, int, int> ScoredContourPair;

    // The final pair of two contours that, in our opinion, best resemble the
    // boiler and peg targets.
    vector<Contour> results;

    // As we find non-rejected contour pairs, we score them by how closely
    // they resemble two parallel bands.  In the end, the highest scoring pair
    // is what we return in results.
    vector<ScoredContourPair> scoredPairsList;

    // What do you mean, there were no contours?
    if (contours.size() == 0) {
        return results;
    }


    /////////////////////////////////////////////////////////////
    // Here is our toolbox of rejection and scoring functions. //
    /////////////////////////////////////////////////////////////

    auto boundingBoxesAreTooDisjoint = [] (const Rect& rect1, const Rect& rect2) -> bool {
        // ---------------------------------------
        // Universal quick rejection heuristic #1.
        //
        // If two contours' bounding boxes don't overlap in the X or Y
        // directions, then they are too disjoint to represent the parallel
        // bands we're looking for.  (And before you say "but mah diagonal
        // bands," the sort of diagonally-aligned bands _we_ care about will
        // have bounding boxes that overlap.)

        bool xOverlap = false;
        bool yOverlap = false;

        if (rect1.x + rect1.width > rect2.x && rect1.x < rect2.x + rect2.width) {
            xOverlap = true;
        }

        if (rect1.y + rect1.height > rect2.y && rect1.y < rect2.y + rect2.height) {
            yOverlap = true;
        }

        if (!xOverlap && !yOverlap) {
            // Reject these!
            return true;
        } else {
            return false;
        }
    };

    auto boundingBoxWidthsAreTooDissimilar = [] (const RotatedRect& r1, const RotatedRect& r2) {
        // ------------------------------------
        // Boiler quick rejection heuristic #1.
        //
        // If two contours have very dissimilar widths, then we can safely
        // reject them.

        array<Point2f, 4> corners1, corners2;
        r1.points(corners1.data());
        r2.points(corners2.data());
        double width1 = max(distance(corners1[0], corners1[1]), distance(corners1[1], corners1[2]));
        double width2 = max(distance(corners2[0], corners2[1]), distance(corners2[1], corners2[2]));

        double minWidth = min(width1, width2);
        double maxWidth = max(width1, width2);
        double low = 1 - CONTOUR_PAIR_BOUNDING_BOX_WIDTH_DEVIATION_TOLERANCE;
        double high = 1 + CONTOUR_PAIR_BOUNDING_BOX_WIDTH_DEVIATION_TOLERANCE;

        if (minWidth < low * maxWidth || maxWidth > high * minWidth) {
            // cout.precision(3);
            // if (minWidth < low * maxWidth) {
            //     cout << "  Width " << minWidth << " < "
            //          << low * maxWidth << " (" << low  << "*" << maxWidth << "); ";
            // } else {
            //     cout << "  Width " << maxWidth << " > "
            //          << high * minWidth << " (" << high  << "*" << minWidth << "); ";
            // }

            // Reject these!
            return true;
        } else {
            return false;
        }
    };

    auto alignedBoundingBoxesAreNotBothHorizontal = [] (const RotatedRect& r1, const RotatedRect& r2) -> bool {
        // ------------------------------------
        // Boiler quick rejection heuristic #2.
        //
        // If either of the two contours have oriented bounding boxes whose
        // longest side's vector has a smaller x-component than the shortest
        // side's vector, then the oriented bounding box is taller than it is
        // wide and the pair can safely be rejected.

        array<array<Point2f, 4>, 2> corners;
        r1.points(corners[0].data());
        r2.points(corners[1].data());

        for (unsigned i = 0; i < 2; ++i) {
            double length01 = distance(corners[i][0], corners[i][1]);
            double xSeparation01 = abs(corners[i][0].x - corners[i][1].x);

            double length12 = distance(corners[i][1], corners[i][2]);
            double xSeparation12 = abs(corners[i][1].x - corners[i][2].x);

            if ((length01 > length12 && xSeparation01 < xSeparation12) ||
                (length12 > length01 && xSeparation12 < xSeparation01)) {
                // Reject these!
                return true;
            }
        }
        return false;
    };

    auto boundingBoxesAreNotVerticallyAligned = [] (const Rect& rect1, const Rect& rect2) -> bool {
        // ------------------------------------
        // Boiler quick rejection heuristic #3.
        //
        // If the bounding boxes don't overlap in the x-direction, we can't
        // consider them to be vertical.  (Again, for the one guy who keeps
        // saying "my diagonal bands MUST be considered, sir," keep quiet back
        // there.)
        //
        // Note that we don't perform any y-overlap rejection here.  The
        // boiler bands are too close to each other from certain distances and
        // angles for that to be a safe test.

        bool xOverlap = false;

        if (rect1.x + rect1.width > rect2.x && rect1.x < rect2.x + rect2.width) {
            xOverlap = true;
        }

        if (!xOverlap) {
            // Reject these!
            return true;
        } else {
            return false;
        }
    };

    auto boundingBoxesAreNotHorizontallyAligned = [] (const Rect& rect1, const Rect& rect2) -> bool {
        // ---------------------------------
        // Peg quick rejection heuristic #1.
        //
        // If the bounding boxes don't overlap in the y-direction, we can't
        // consider them to be horizontal.  And yes, we do also perform an
        // x-overlap rejection here (the peg tape is far apart, so unlike the
        // boiler, there's no chance of false positives.)

        bool xOverlap = false;
        bool yOverlap = false;

        if (rect1.x + rect1.width > rect2.x && rect1.x < rect2.x + rect2.width) {
            xOverlap = true;
        }

        if (rect1.y + rect1.height > rect2.y && rect1.y < rect2.y + rect2.height) {
            yOverlap = true;
        }

        if (!yOverlap || xOverlap) {
            // Reject these!
            return true;
        } else {
            return false;
        }
    };

    auto contoursAreTooNonRectangular = [] (const Contour& c1, const Contour& c2, const RotatedRect& r1, const RotatedRect& r2) -> bool {
        // ----------------------------------------
        // Universal? quick rejection heuristic #2.
        //
        // Comparing the area of the aligned bounding box a contour to the
        // area of the contour itself produces a ratio, rA.  For
        // rectangular-ish contours, rA is closer to 1.  When rA is too
        // low for _either_ contour, we reject the whole pair.
        //
        // Right now we're doing this for the Peg, but we may extend
        // it to the Boiler if we get good results.
        Contour convexHull1;
        Contour convexHull2;

        array<Point2f, 4> corners1, corners2;
        r1.points(corners1.data());
        r2.points(corners2.data());
        double area1 = distance(corners1[0], corners1[1]) * distance(corners1[1], corners1[2]); // Length * Width.
        double area2 = distance(corners2[0], corners2[1]) * distance(corners2[1], corners2[2]); // Length * Width.

        double areaRatio1 = contourArea(c1) / area1;
        double areaRatio2 = contourArea(c2) / area2;
        const double MINIMUM_RECTANGLE_AREA_TO_TRUE_AREA_RATIO = 0.75;

        cout.precision(3);
        cout << "  Area ratios are " << areaRatio1 << " and " << areaRatio2 << ", respectively ";

        if (areaRatio1 < MINIMUM_RECTANGLE_AREA_TO_TRUE_AREA_RATIO ||
            areaRatio2 < MINIMUM_RECTANGLE_AREA_TO_TRUE_AREA_RATIO) {
            // Reject these!
            cout << "(rejected.)\n";
            return true;
        } else {
            cout << "(accepted.)\n";
            return false;
        }
    };

    auto alignedBoundingBoxesAreNotBothVertical = [] (const RotatedRect& r1, const RotatedRect& r2) -> bool {
        // ------------------------------------
        // Peg quick rejection heuristic #2.
        //
        // If either of the two contours have oriented bounding boxes whose
        // longest side's vector has a smaller y-component than the shortest
        // side's vector, then the oriented bounding box is wider than it is
        // tall and the pair can safely be rejected.
        //
        // And yes, this is just the opposite of the
        // alignedBoundingBoxesAreNotBothHorizontal() test.

        array<array<Point2f, 4>, 2> corners;
        r1.points(corners[0].data());
        r2.points(corners[1].data());

        for (unsigned i = 0; i < 2; ++i) {
            double length01 = distance(corners[i][0], corners[i][1]);
            double ySeparation01 = abs(corners[i][0].y - corners[i][1].y);

            double length12 = distance(corners[i][1], corners[i][2]);
            double ySeparation12 = abs(corners[i][1].y - corners[i][2].y);

            if ((length01 > length12 && ySeparation01 < ySeparation12) ||
                (length12 > length01 && ySeparation12 < ySeparation01)) {
                // Reject these!
                return true;
            }
        }
        return false;
    };

    /////////////////////////////////////////////////
    // Scoring heuristics for the boiler solution. //
    /////////////////////////////////////////////////////////////////////////////
    // Let's measure 1ftH7ftD3Angle0Brightness.jpg (a boiler image) using the  //
    // GIMP's measuring tool.  The bounding boxes are only approximate -- JPEG //
    // distortion means it's not exactly clear where the contour boundaries    //
    // are.                                                                    //
    //                                                                         //
    // - Top rect dimensions: 64x31 pixels (2.06:1)                            //
    // - Bottom rect dimensions: 61x25 pixels (2.44:1)                         //
    // - Distance between centers of the bounding boxes: 27.1 pixels           //
    //                                                                         //
    // It looks like there are several interesting attacks we can make here    //
    // for boiler images:                                                      //
    //                                                                         //
    // - The bounding boxes seem to have aspect ratios somewhere between 2:1   //
    //   and 3:1, and not too much wider than that;                            //
    // - The centers of the bounding boxes seem to be separated by a distance  //
    //   which is similar to the height of the bounding boxes.  In other       //
    //   words, the bounding boxes almost always graze one another.            //
    /////////////////////////////////////////////////////////////////////////////

    auto scoreBoundingBoxesUsingBoilerDistance = [] (const Rect& rect1, const Rect& rect2) -> double {
        // ----------------------------
        // Boiler scoring heuristic #1.
        //
        // Award a better score to two contours whose bounding box centers are
        // separated by a distance close to the average height of the bounding
        // boxes.
        //
        // Note that by the time we make it here, we've already rejected pairs
        // where there is no X overlap between bounding boxes _regardless_ of
        // the distance that separates them.  This is just a bit of finesse on
        // top of that.

        double averageHeight = (rect1.height + rect2.height) / 2.0;
        double minimumExpectedDistance = averageHeight;
        double maximumExpectedDistance = 2 * minimumExpectedDistance;

        double xCenter1 = rect1.x + rect1.width / 2;
        double yCenter1 = rect1.y + rect1.height / 2;
        double xCenter2 = rect2.x + rect2.width / 2;
        double yCenter2 = rect2.y + rect2.height / 2;
        double actualDistance = sqrt((xCenter2 - xCenter1) * (xCenter2 - xCenter1) +
                                     (yCenter2 - yCenter1) * (yCenter2 - yCenter1));

        // Standard linear interpolation formula: u = (current - min)/(max - min).
        // That way, u is 0 when current == min and u is 1 when current == max,
        // varying smoothly between the two extremes.
        double u = (actualDistance - minimumExpectedDistance)/(maximumExpectedDistance - minimumExpectedDistance);

        // But being closer to minimumExpectedDistance is better.
        u = 1.0 - u;

        if (u > 1.0) {
            // Don't award a score too high to bounding boxes that are closer
            // than we expect.
            //
            // TODO: This will award a score of 1.0 to bounding boxes that
            // overlap.  Is that bad?
            u = 1.0;
        } else if (u < 0.0) {
            // And if you're beyond the maximumExpectedDistance, no score for
            // you.
            u = 0.0;
        }
        return u;
    };

    auto scoreBoundingBoxesUsingBoilerAspectRatio = [] (const Rect& rect1, const Rect& rect2) -> double {
        // ----------------------------
        // Boiler scoring heuristic #2.
        //
        // Award a better score to pairs of bounding boxes that are both
        // within the expected aspect ratio range.

        double aspectRatio1 = rect1.width / rect1.height;
        double aspectRatio2 = rect2.width / rect2.height;
        double minAspectRatio = 1.8/1;   // TODO: This should be a named, top-level const.
        double maxAspectRatio = 3.25/1;  // TODO: This should be a named, top-level const.
        double score = 0;

        double u1 = (aspectRatio1 - minAspectRatio) / (maxAspectRatio - minAspectRatio);
        u1 = min(1.0, max(u1, 0.0)); // Clamp between 0 and 1.
        u1 = 1 - u1;                 // Being closer to the minAspectRatio is better.
        score += u1 / 2;

        double u2 = (aspectRatio2 - minAspectRatio) / (maxAspectRatio - minAspectRatio);
        u2 = min(1.0, max(u2, 0.0));
        u2 = 1 - u2;
        score += u2 / 2;

        return score; // 0.0 <= score <= 1.0
    };

    //////////////////////////////////////////////
    // Scoring heuristics for the peg solution. //
    //////////////////////////////////////////////
    // We expect the contours to be vertical rectangles -- thus they need to
    // be taller than they are wide.  That is a difficult property to measure
    // for a regular contour (which is a polygon with arbitrary numbers of
    // vertices.)  What's easier to measure is:
    //
    // (1) The alignment of the oriented bounding boxes.  They should be
    //     close to parallel, even if something is obscuring one of the pieces
    //     of tape (leaving two contours where there was one before.)
    //
    // (2) In any given pair of contours, we shoot for at least one to be
    //     "unobscured" (specifically meaning that the ratio of its oriented
    //     bounding box's longer dimension to the shorter dimension approaches
    //     5in/2in.)
    //
    // (3) The aspect ratio of the bounding boxes.  They should both be
    //     taller than their widths.  Note that due to gears obscuring some of
    //     the tape, we don't necessarily know how /much/ taller a contour is
    //     than it is wide unless the contour is unobscured.

    auto scorePegContoursUsingOrientedBoundingBoxAngles = [] (const RotatedRect& rotatedRect1, const RotatedRect& rotatedRect2) -> double {
        // ----------------------------
        // Peg scoring heuristic #1.
        //
        // Award a better score to pairs of contours whose oriented bounding
        // boxes are closer to being parallel.  This should work even if an
        // obstacle (like a gear) is obscuring one of the pieces of tape,
        // forcing it to split into two contours.
        //
        // The closer the two oriented rects are to having the same angle, the
        // higher the score.  A perpendicular orientation has the lowest
        // score.  We're assuming here that the angles are between -180 and
        // 180 degrees, and that they are measured consistently (so that you
        // can't have two identical rects whose angles are 180 degrees out of
        // phase.)

        double deltaAngleDegrees = rotatedRect1.angle - rotatedRect2.angle;  // Is this between =-180 and 180?
        if (deltaAngleDegrees < 0) {
            deltaAngleDegrees += 180;
        }
        const double minDeltaAngleDegrees = 0;
        const double maxDeltaAngleDegrees = 90;
        double u = (deltaAngleDegrees - minDeltaAngleDegrees) / (maxDeltaAngleDegrees - minDeltaAngleDegrees);
        u = min(u, 1.0); // Clamp down if delta is greater than 90 degrees.
        u = 1 - u;       // But smaller deltas are better.
        return u;
    };

    auto scorePegContoursUsingOrientedBoundingBoxAspectRatios = [] (const RotatedRect& rotatedRect1, const RotatedRect& rotatedRect2) -> double {
        // ----------------------------
        // Peg scoring heuristic #2.
        //
        // Award a better score to pairs of contours when one of the two is
        // "unobscured" (i.e., it has an aspect ratio close to the expected
        // aspect ratio of 5:2.)
        //
        // Since any number of contours can pair up with an unobscured
        // contour, we accept only a narrow range for the unobscured contour's
        // aspect ratio.

        array<array<Point2f, 4>, 2> cornerPoints;
        rotatedRect1.points(cornerPoints[0].data());
        rotatedRect2.points(cornerPoints[1].data());

        array<double, 2> aspectRatioScores;

        for (unsigned int i = 0; i < 2; ++i) {
            // Which side is longest?  We only need to test three of the
            // four corners to find out.
            const Point2f& p0 = cornerPoints[i][0];
            const Point2f& p1 = cornerPoints[i][1];
            const Point2f& p2 = cornerPoints[i][2];

            double distance01 = sqrt((p0.x - p1.x) * (p0.x - p1.x) + (p0.y - p1.y) * (p0.y - p1.y));
            double distance12 = sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));

            if (distance01 < 1.0 || distance12 < 1.0) { // Box too short; no score for this pair.
                return 0.0;
            }

            double aspectRatio = max(distance01, distance12) / min(distance01, distance12);

            const double minAspectRatio = PEG_REAL_TAPE_HEIGHT_INCHES / PEG_REAL_TAPE_WIDTH_INCHES;
            const double maxAspectRatio = 2 * minAspectRatio;

            double u = abs(aspectRatio - minAspectRatio) / (maxAspectRatio - minAspectRatio);
            u = min(1.0, u); // Clamp down aspect ratios that are too high.
            u = 1 - u;       // Being closer to minAspectRatio is better.

            aspectRatioScores[i] = u;
        }

        return max(aspectRatioScores[0], aspectRatioScores[1]);
    };


    //////////////////////////////////////////////////////////////////////////
    // And finally, here's the actual findBestContourPair algorithm itself. //
    //                                                                      //
    // Yeah, this is O(N^2), so it's not efficient for large numbers of     //
    // contours.  We try to keep the runtime down with quick rejection      //
    // heuristics.                                                          //
    //////////////////////////////////////////////////////////////////////////

    for (unsigned int i = 0; i < contours.size() - 1; i++) {
        for (unsigned int j = i + 1; j < contours.size(); j++) {

            const Contour &c1 = contours.at(i);
            const Contour &c2 = contours.at(j);
            Rect rect1 = boundingRect(c1);
            Rect rect2 = boundingRect(c2);
            RotatedRect rotatedRect1 = minAreaRect(c1);
            RotatedRect rotatedRect2 = minAreaRect(c2);

            // ----------------------------------------------------------------
            // Throw out the contour pairs that fail our quick rejection
            // tests.

            if (boundingBoxesAreTooDisjoint(rect1, rect2)) {
                continue;
            }

            if (solutionType == Boiler) {
                if (boundingBoxWidthsAreTooDissimilar(rotatedRect1, rotatedRect2)) {
                    // cout << "  [Boiler] Rejecting (" << i << ", " << j << ") since their widths are too dissimilar.\n";
                    continue;
                }
                if (alignedBoundingBoxesAreNotBothHorizontal(rotatedRect1, rotatedRect2)) {
                    // cout << "  [Boiler] Rejecting (" << i << ", " << j << ") since at least one is vertical.\n";
                    continue;
                }
                if (boundingBoxesAreNotVerticallyAligned(rect1, rect2)) {
                    // cout << "  [Boiler] Rejecting (" << i << ", " << j << ") because their bounding boxes' x-ranges do not overlap or because their bounding boxes' y-ranges do overlap.\n";
                    continue;
                }
            } else {
                if (boundingBoxesAreNotHorizontallyAligned(rect1, rect2)) {
                    // cout << "  [Peg] Rejecting (" << i << ", " << j << ") because their bounding boxes' y-ranges do not overlap or because their bounding boxes' x-ranges do overlap.\n";
                    continue;
                }
                if (alignedBoundingBoxesAreNotBothVertical(rotatedRect1, rotatedRect2)) {
                    // cout << "  [Boiler] Rejecting (" << i << ", " << j << ") since at least one is horizontal.\n";
                    continue;
                }
                cout << "  [Peg] Considering (" << i << ", " << j << ") for concavity analysis: ";
                if (contoursAreTooNonRectangular(c1, c2, rotatedRect1, rotatedRect2)) {
                    continue;
                }
            }


            // -------------------------------------------------------------
            // We've rejected what we can.  It's time to score what remains.
            //
            // The higher the score, the more we like this pair.  The scoring
            // heuristics themselves often use linear interpolation to vary the
            // score between a min and a max.

            ScoredContourPair currentContourPair;
            double& score = get<0>(currentContourPair);
            get<1>(currentContourPair) = i;
            get<2>(currentContourPair) = j;

            // The boiler scoring heuristics right now are looking pretty
            // good.
            if (solutionType == Boiler) {
                score += scoreBoundingBoxesUsingBoilerDistance(rect1, rect2);
                score += scoreBoundingBoxesUsingBoilerAspectRatio(rect1, rect2);
            } else {
                score += scorePegContoursUsingOrientedBoundingBoxAngles(rotatedRect1, rotatedRect2);
                score += scorePegContoursUsingOrientedBoundingBoxAspectRatios(rotatedRect1, rotatedRect2); // Iffy.
            }

            // If the score is still too low, this pair sucks.
            // "Too low" here is pretty arbitrary.
            if (score < 0.1) { // TODO: This should be a named, top-level const.
                cout << "  Score " << score << " too low.  Rejecting (" << i << ", " << j << ").\n";
                continue;
            }

            // Accepted!
            cout << "  Pair (" << i << ", " << j << ") accepted with a score of " << score << "\n";
            scoredPairsList.push_back(currentContourPair);
        }
    }


    // Sort the scoredPairsList by score, descending.  Thus scoredPairsList[0]
    // will have the highest score.
    sort(scoredPairsList.begin(),
         scoredPairsList.end(),
         [] (const ScoredContourPair& p1, const ScoredContourPair& p2) -> bool {
             double score1 = get<0>(p1);
             double score2 = get<0>(p2);
             return (score1 > score2 ? true : false);
         });

    // Only the highest-scoring pair matters for the results, but return the
    // top 3 pairs so that we can draw them.
    unsigned int n = scoredPairsList.size();
    if (n > 3) {
        n = 3;
    }
    for (unsigned int i = 0; i < n; ++i) {
        const ScoredContourPair& highScoringPair = scoredPairsList[i];
        int j = get<1>(highScoringPair);
        int k = get<2>(highScoringPair);
        results.push_back(contours[j]);
        results.push_back(contours[k]);
    }

    return results;
}


// Finds the two "bottom vertices" of the given contour with the given
// bounding rectangle.
//
// The "bottom vertices" are the vertices which are closest, in
// pixels, to the bottom left and bottom right corners of the given
// bounding box.
//
// @param points The contour to examine.  Note that this is a
//               std::vector<cv::Point2f> rather than the usual
//               std::vector<cv::Point>.
// @param rect   The bounding rectangle for the contour.  You can use
//               boundingRect() to obtain this.
// @return       An array of two points: the one closest to the bottom left
//               corner of the rect, and the one closes to the bottom right
//               corner of the rect.

vector<Point> PapasVision::findBottomPts(const vector<Point2f> &points,
                                         Rect rect) const {

    Point rectBottomRight = rect.br();
    Point rectBottomLeft(rect.br().x - (rect.width - 1), rect.br().y);
    Point bottomRight;
    Point bottomLeft;

    double lowestDist = 0;
    for (unsigned int i = 0; i < points.size(); i++) {
        double dist = sqrt(
            (points[i].x - rectBottomLeft.x) * (points[i].x - rectBottomLeft.x) +
            (points[i].y - rectBottomLeft.y) * (points[i].y - rectBottomLeft.y));

        if (i == 0) {
            bottomLeft = points.at(i);
            lowestDist = dist;
        } else if (dist < lowestDist) {
            bottomLeft = points.at(i);
            lowestDist = dist;
        }
    }

    for (unsigned int i = 0; i < points.size(); i++) {
        double dist = sqrt(
            (points[i].x - rectBottomRight.x) * (points[i].x - rectBottomRight.x) +
            (points[i].y - rectBottomRight.y) * (points[i].y - rectBottomRight.y));

        if (i == 0) {
            bottomRight = points.at(i);
            lowestDist = dist;
        } else if (dist < lowestDist) {
            bottomRight = points.at(i);
            lowestDist = dist;
        }
    }

    vector<Point> bottomPts = {bottomLeft, bottomRight};
    return bottomPts;
}

// Exactly like findBottomPts(), but for the top left and top right corners of
// the bounding rect.

vector<Point> PapasVision::findTopPts(const vector<Point2f> &points,
                                      Rect rect) const {
    Point rectTopRight(rect.tl().x + (rect.width - 1), rect.tl().y);
    Point rectTopLeft = rect.tl();
    Point topRight;
    Point topLeft;

    double lowestDist = 0;
    for (unsigned int i = 0; i < points.size(); i++) {
        double dist =
            sqrt((points[i].x - rectTopLeft.x) * (points[i].x - rectTopLeft.x) +
                 (points[i].y - rectTopLeft.y) * (points[i].y - rectTopLeft.y));

        if (i == 0) {
            topLeft = points.at(i);
            lowestDist = dist;
        } else if (dist < lowestDist) {
            topLeft = points.at(i);
            lowestDist = dist;
        }
    }

    for (unsigned int i = 0; i < points.size(); i++) {
        double dist =
            sqrt((points[i].x - rectTopRight.x) * (points[i].x - rectTopRight.x) +
                 (points[i].y - rectTopRight.y) * (points[i].y - rectTopRight.y));

        if (i == 0) {
            topRight = points.at(i);
            lowestDist = dist;
        } else if (dist < lowestDist) {
            topRight = points.at(i);
            lowestDist = dist;
        }
    }

    vector<Point> topPts = {topLeft, topRight};
    return topPts;
}

///////////////////////////////////////////////////////////////////////////////////
// Trigonometric functions that deal with the computer vision contours we found.
///////////////////////////////////////////////////////////////////////////////////

// The Boiler distance-finding algorithm.
//
// Given a four-point quadrilateral that definitively surrounds our goal and
// has a known real-world height, determines the distance, in real-world
// inches, to the center of that quadrilateral using trigonometry.
//
// @param topPoints      The two top points of of the quadrilateral.
// @param bottomPoints   The two bottom points of of the quadrilateral.
// @param realTapeHeight The known height of the quadrileral (i.e., the distance
//                       between the top and bottom midpoints), as measured in
//                       real-world inches.
// @param elevationAngleDegrees The elevation, in degrees of the
//                              boiler camera from the horizontal plane.
// @return Distance to the center of the quadrilateral, in inches.
double PapasVision::findDistToGoal(const vector<Point> &topPoints,
                                   const vector<Point> &bottomPoints,
                                   double realTapeHeight,
                                   double elevationAngleDegrees,
                                   int imgWidth,
                                   int imgHeight) const {
    double topMidPointY = (topPoints[0].y + topPoints[1].y) / 2.0;
    double bottomMidPointY = (bottomPoints[0].y + bottomPoints[1].y) / 2.0;
    double degPerPixelVert = verticalFOV(HORIZ_FOV_DEG, imgWidth, imgHeight) / imgHeight;
    double theta_b = degPerPixelVert * (((imgHeight - 1) / 2.0) - bottomMidPointY);
    double theta_t = degPerPixelVert * (((imgHeight - 1) / 2.0) - topMidPointY);
    double theta_w = elevationAngleDegrees + theta_t;
    double theta_rb = 90.0 - theta_w;
    double theta_hg = theta_t - theta_b;
    double theta_cb = elevationAngleDegrees + theta_b;
    double rb =
        (realTapeHeight * sin(theta_rb * DEGREES_TO_RADIANS)) /
        sin(theta_hg * DEGREES_TO_RADIANS);
    double distance = rb * cos(theta_cb * DEGREES_TO_RADIANS);
    return distance;
}

// Utility function for findSolutionCommon().
//
// This is the equivalent of findDistToGoal(), but for the Peg solution and
// its known quantities.  The calculations are based on simple trigonometry,
// with the camera and its field of view forming an isosceles triangle
// pointing toward the center of the image.  What we're trying to calculate is
// the _height_ of that isosceles triangle.
//
// @param leftmostPoint The leftmost of the bottom points for both of
//                      the reflective tape contours.
// @param rightmostPoint The rightmost point, concomitantly.
// @param knownWidthInches The width from the left edge of the left
//                         tape to the right edge of the right tape,
//                         in real world inches.
// @param imgWidthPixels The horizontal width of the image.
//
// @return The distance to the center of the image.  For this to be
//         the true distance to the target, the PapasAngle must be
//         minimized to zero through rotation of the camera.
double PapasVision::findDistToCenterOfImage(const Point& leftmostPoint, const Point& rightmostPoint, double knownWidthInches, int imgWidthPixels) const {

    // Width between the two points in pixels    Width of the image in pixels
    // --------------------------------------- = ----------------------------
    // Known width in inches                     X (width of image in inches)

    double widthOfImageInInches = (imgWidthPixels * knownWidthInches) / distance(leftmostPoint, rightmostPoint);

    // Tan theta (where theta is half of the FOV) = (0.5 * Width of image in inches) / D (the PapasDistance)
    // Ergo, D = 0.5 * Width of image in inches / tan (0.5 * FOV).

    return (0.5 * widthOfImageInInches) / tan(HORIZ_FOV_RAD/2);
}


// Utility function for findSolutionCommon().
//
// Determines how far we need to turn in order to face our goal rectangle head
// on.
double PapasVision::findAzimuthGoal(const vector<Point> &topPoints,
                                    const vector<Point> &bottomPoints, int imgWidth) const {
    double topMidPointX = (topPoints[0].x + topPoints[1].x) / 2.0;
    double bottomMidPointX = (bottomPoints[0].x + bottomPoints[1].x) / 2.0;
    double goalCenterX = (topMidPointX + bottomMidPointX) / 2.0;
    double degPerPixelHoriz = HORIZ_FOV_DEG / imgWidth;
    double imageCenterX = (imgWidth - 1) / 2.0;
    double azimuthGoalDeg = (goalCenterX - imageCenterX) * degPerPixelHoriz;

    return azimuthGoalDeg;
}

} // end (namespace robot)
