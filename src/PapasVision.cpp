#include "PapasVision.h"

#include <array>
#include <cmath>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <sstream>
#include <tuple>

using namespace cv;
using namespace std;

namespace robot {

//////////////////////////////////////////////
// Critical constants for findDistToGoal(). //
//////////////////////////////////////////////

const double DEGREES_TO_RADIANS = 3.1415926 / 180.0;

const double HORIZ_FOV_DEG = 59.253; // 59.703;
const double HORIZ_FOV_RAD = HORIZ_FOV_DEG * DEGREES_TO_RADIANS;
const double VERT_FOV_DEG = 44.44; // 33.583;
const double VERT_FOV_RAD = VERT_FOV_DEG * DEGREES_TO_RADIANS;

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

const double IMG_HEIGHT = 480;            // pixels of image resolution
const double IMG_WIDTH = 640;             // pixels of image resolution
const double CAM_EL_DEG = 45;
const double CAM_EL_RAD = CAM_EL_DEG * DEGREES_TO_RADIANS;
const double THRESHOLD_GRAYSCALE_CUTOFF =
    25; // Used The GIMP's Colors->Threshold tool on the green residual image to
// determine this empirically.

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
const double CONTOUR_PAIR_BOUNDING_BOX_WIDTH_DEVIATION_TOLERANCE = 0.15;
const double CONTOUR_PAIR_BOUNDING_BOX_HEIGHT_DEVIATION_TOLERANCE = CONTOUR_PAIR_BOUNDING_BOX_WIDTH_DEVIATION_TOLERANCE;

/////////////////////////////
// Global utility methods. //
/////////////////////////////

// Saves the given image to a file on disk that uses the following naming scheme:
//
//   prefix + "_" + index + "_" + suffix
void save(const string &pathPrefix, int index, const string &suffix,
          const Mat &imageToWrite) {

    stringstream stream;
    stream << pathPrefix << "_" << index << "_" << suffix;
    imwrite(stream.str(), imageToWrite);
}

/////////////////////////////////////
// Constructor and public methods. //
/////////////////////////////////////

PapasVision::PapasVision(const Config &config_,
                         double goalRejectionThresholdInches_,
                         bool writeIntermediateFilesToDisk_)
    : config(config_), distToGoalInch(0), azimuthGoalDeg(0),
      solutionFound(false),
      writeIntermediateFilesToDisk(writeIntermediateFilesToDisk_),
      goalRejectionThresholdInches(goalRejectionThresholdInches_) {

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

void PapasVision::findPeg(const string& samplePictureFile, VideoCapture &camera2) {
    findSolutionCommon(samplePictureFile, camera, Peg);
}


// Public interface.
bool PapasVision::getSolutionFound() const { return solutionFound; }
double PapasVision::getAzimuthGoalDeg() const { return azimuthGoalDeg; }
double PapasVision::getDistToGoalInch() const { return distToGoalInch; }

void PapasVision::findPeg(string samplePictureFile)    { VideoCapture camera(1); findPeg(getFullPath(samplePictureFile),    camera); }
void PapasVision::findPeg(int imageIndex)              { VideoCapture camera(1); findPeg(getFullPath(imageIndex),           camera); }
void PapasVision::findBoiler(string samplePictureFile) { VideoCapture camera(0); findBoiler(getFullPath(samplePictureFile), camera); }
void PapasVision::findBoiler(int imageIndex)           { VideoCapture camera(0); findBoiler(getFullPath(imageIndex),        camera); }

/////////////////////////////////////////////////////////////////////////////////
// Our most important public functions.  Ultimately, their purpose is to use
// OpenCV's computer vision analysis functions to calculate three numbers from
// the latest camera image: distToGoalInch, azimuthGoalDeg, and
// elevationGoalDeg.
/////////////////////////////////////////////////////////////////////////////////

// This function does the leg-work for finding the vision solutions.  The only
// input it needs (either than input image sources) is the type of solution it
// should find.

// @param pictureFile  The path to a JPG or PNG sample image to
//                     analyze.  If the path is not absolute, it will
//                     be considered to be relative to the Config's
//                     "cameraFolder" entry.  An empty string will
//                     cause the image to be read from the appropriate
//                     camera instead.
//
// @param camera       The camera to use if pictureFile is an empty string.
//
// @param solutionType Either PapasVision::Boiler or PapasVision::Peg.

void PapasVision::findSolutionCommon(const string& samplePictureFile, VideoCapture &camera,
                                     SolutionType solutionType) {
    // clock_t startTime = clock();


    // Determine whether or not the camera is present.  If not, we'll use the fake
    // images in 2017bot/samples.

    string pathPrefix;
    bool useCamera = (samplePictureFile == "");

    Mat output;

    // A sort of "screenshot" of the initial camera image or initial sample
    // image.
    Mat frame;

    // The index of the current debugging image we're writing to.
    int index = 1;

    if (!useCamera) {

        // Determine the prefix we'll use for writing future intermediate
        // images.
        string fileWithoutExtension;
        if (samplePictureFile.rfind(".") >= 0) {
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
        camera.read(frame);
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

        drawContours(frameFiltContoursImage, contours, i, sortedContourPairColors.at(i / 2));
    }
    if (writeIntermediateFilesToDisk) {
        save(pathPrefix, index++, "frame_filtcontours.png", frameFiltContoursImage);
    }


    // If a pair of contours won, we assume that it represents the two parallel
    // bands we were looking for.
    if (contours.size() > 0) {

        const vector<Point>& contour1_ = contours[0];
        const vector<Point>& contour2_ = contours[1];

        vector<Point2f> contour1;
        vector<Point2f> contour2;

        // Convert the Points into Point2fs.
        copy(contour1_.begin(), contour1_.end(), back_inserter(contour1));
        copy(contour2_.begin(), contour2_.end(), back_inserter(contour2));

        // TODO: We need to find the two bottom points of each reflective
        // tape. The distance in real life from the bottom of the bottom
        // reflective tape to the bottom of the top reflective tape is 6
        // inches.

        vector<Point> bottomPoints1 = findBottomPts(contour1, boundingRect(contour1));
        vector<Point> bottomPoints2 = findBottomPts(contour2, boundingRect(contour2));

        Mat framePoints = frame.clone();
        const int radius = 5, thickness = 3;
        circle(framePoints, bottomPoints1.at(0), radius, Scalar(0, 192, 255), thickness);
        circle(framePoints, bottomPoints1.at(1), radius, Scalar(0, 192, 255), thickness);
        circle(framePoints, bottomPoints2.at(0), radius, Scalar(255, 64, 255), thickness);
        circle(framePoints, bottomPoints2.at(1), radius, Scalar(255, 64, 255), thickness);
        if (writeIntermediateFilesToDisk) {
            save(pathPrefix, index++, "frame_points.png", framePoints);
        }

        // TODO: We can do PapasDistance and PapasAngle calculations
        // unchanged with that set of four bottom points.

        solutionFound = false;
        if (solutionType == Boiler) {
            distToGoalInch = findDistToGoal(bottomPoints1, bottomPoints2, BOILER_REAL_TAPE_BOTTOM_HEIGHT_INCHES + BOILER_REAL_TAPE_SEPARATION_INCHES);
            azimuthGoalDeg = findAzimuthGoal(bottomPoints1, bottomPoints2);
        } else {
            distToGoalInch = findDistToGoal(bottomPoints1, bottomPoints2, PEG_REAL_TAPE_HEIGHT_INCHES);
            azimuthGoalDeg = findAzimuthGoal(bottomPoints1, bottomPoints2);
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
    } else {
        if (writeIntermediateFilesToDisk) {
            cout << "Solution not found";
        }
    }
    // if (writeIntermediateFilesToDisk)
    // {
    //     double processingTimeMs = 1000.0 * (clock() - startTime) /
    //     CLOCKS_PER_SEC;
    //     cout << "Processing time: " << setprecision(4) << processingTimeMs << "
    //     ms\n";
    // }
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
// by otsu's thresholding algorithm.
//
// @param input Grayscale input image.
// @param output The appropriately-thresholded result image.
// @param algorithm STANDARD to just use straight-up Otsu, and WITH_BLUR to do a
// Gaussian blur before the Otsu filter.
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


    /////////////////////////////////////////////////////////////
    // Here is our toolbox of rejection and scoring functions. //
    /////////////////////////////////////////////////////////////

    auto boundingBoxesAreTooDisjoint = [] (const Rect& rect1, const Rect& rect2) -> bool {
        // -----------------------------
        // Quick rejection heuristic #1.
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

    auto boundingBoxesAreNotBothVertical = [] (const Rect& rect1, const Rect& rect2) -> bool {
        // -----------------------------
        // Quick rejection heuristic #2.
        //
        // For the PEG, if two contours do not both have vertical bounding boxes, we
        // can safely reject them.

        if (rect1.width > rect1.height || rect2.width > rect2.height) {
            // Reject these!
            return true;
        } else {
            return false;
        }
    };

    auto boundingBoxWidthsAreTooDissimilar = [] (const Rect& rect1, const Rect& rect2) {
        // -----------------------------
        // Quick rejection heuristic #3.
        //
        // For the BOILER, if two contours have very dissimilar widths, then
        // we can safely reject them.
        //
        // This is pretty similar to the test above.

        double minWidth = min(rect1.width, rect2.width);
        double maxWidth = max(rect1.width, rect2.width);
        double low = 1 - CONTOUR_PAIR_BOUNDING_BOX_WIDTH_DEVIATION_TOLERANCE;
        double high = 1 + CONTOUR_PAIR_BOUNDING_BOX_WIDTH_DEVIATION_TOLERANCE;

        if (minWidth < low * maxWidth || maxWidth > high * minWidth) {
            // cout.precision(3);
            // if (minWidth < low * maxWidth) {
            //     cout << "Width " << minWidth << " < "
            //          << low * maxWidth << " (" << low  << "*" << maxWidth << "); ";
            // } else {
            //     cout << "Width " << maxWidth << " > "
            //          << high * minWidth << " (" << high  << "*" << minWidth << "); ";
            // }

            // Reject these!
            return true;
        } else {
            return false;
        }
    };

    auto boundingBoxesAreNotHorizontallyAligned = [] (const Rect& rect1, const Rect& rect2) -> bool {
        // -----------------------------
        // Quick rejection heuristic #4.
        //
        // For the PEG, if the bounding boxes don;t overlap in the
        // y-direction, we can't consider them to be horizontal.  (Again, for
        // the one guy who keeps saying "my diagonal bands MUST be considered,
        // sir," keep quiet back there.)

        bool yOverlap = false;

        if (rect1.y + rect1.height > rect2.y && rect1.y < rect2.y + rect2.height) {
            yOverlap = true;
        }

        if (!yOverlap) {
            // Reject these!
            return true;
        } else {
            return false;
        }
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
        // where there is no X or Y overlap between bounding boxes
        // _regardless_ of the distance that separates them.  This is just a
        // bit of finesse on top of that.

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
    ///////////////////////////////////////////////////////////////////////////
    // In the real world, a distance of 8.25 inches separates the centers of //
    // the reflect tape pieces that surround the Peg target.                 //
    //                                                                       //
    // In the real world, the peg targets have widths of two inches.         //
    //                                                                       //
    // Ergo, both the real world and in the sample image, we expect contours //
    // to have a width-to-separation-distance ratio close to 8.25/2.         //
    ///////////////////////////////////////////////////////////////////////////
    // Going back to old-fashioned pixel measurements on a sample image
    // (green-peg1.png in this case), we have:
    //
    // - Right bounding box: 65x146@773,1206
    // - Left bounding box: 75x171@@533,1231
    // - Distance between their (eyeballed) centers: about 239 pixels, give or
    //   take
    //
    // Note that the gear partially obscures the right contour, reducing its
    // height.
    // In spite of the tilts, the bounding box widths are similar, which can
    // be used as a quick rejection criterion.
    //
    // However, the empirically-observed ratio of the distance from the
    // centers to the widths of the bounding boxes are 3.677 and 3.187, not
    // quite the same as our real-world measurement of (10.25 - 1 - 1)/2 =
    // 4.125.
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // auto scoreBoundingBoxesUsingPegWidthToDistanceRatio = [] (const Rect& rect1, const Rect& rect2) -> double { //
    //     // ----------------------------                                                                         //
    //     // Peg scoring heuristic #2.                                                                            //
    //     //                                                                                                      //
    //     // Take the distance of one contour's center to the other contour's                                     //
    //     // center.  Then measure the contour's width.  The closer the ratio of                                  //
    //     // these two numbers is to 8.25/2, the higher the score.                                                //
    //     //                                                                                                      //
    //     // Presently, we use the width and centers of the bounding boxes as a                                   //
    //     // substitute for the actual numbers -- this could perhaps be improved                                  //
    //     // through empirical observation of the images using the GIMP's                                         //
    //     // measuring tool.                                                                                      //
    //                                                                                                             //
    //     const double distance = PEG_REAL_TAPE_OUTER_WIDTH_INCHES - 2 * (PEG_REAL_TAPE_WIDTH_INCHES / 2);        //
    //     double ratio1 = distance / rect1.width;                                                                 //
    //     double ratio2 = distance / rect2.width;                                                                 //
    //                                                                                                             //
    //     double minRatio = distance / 2;                                                                         //
    //     double maxRatio = 2 * minRatio;  // I dunno--arbitrary.                                                 //
    //                                                                                                             //
    //     double u1 =  abs(ratio1 - minRatio) / (maxRatio - minRatio);                                            //
    //                                                                                                             //
    //     u1 = min(1.0, u1);               // Clamp to be no greater than 1.                                      //
    //     u1 = 1 - u1;                     // Being closer to the minAspectRatio is better.                       //
    //     double score1 = u1 / 2;                                                                                 //
    //                                                                                                             //
    //     double u2 = abs(ratio2 - minRatio) / (maxRatio - minRatio);                                             //
    //     u2 = min(1.0, u2);                                                                                      //
    //     u2 = 1 - u2;                                                                                            //
    //     double score2 = u2 / 2;                                                                                 //
    //                                                                                                             //
    //     return score1 + score2;          // 0.0 <= score <= 1.0                                                 //
    // };                                                                                                          //
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // We expect the contours to be vertical rectangles -- thus they need to
    // be taller than they are wide.  That is a difficult property to measure
    // for a regular contour (which is a polygon with arbitrary numbers of
    // vertices.)  What's easier to measure is:
    //
    // (1) The aspect ratio of the bounding boxes.  They should both be
    //     taller than their widths.  Note that due to gears obscuring some of
    //     the tape, we don't necessarily know how /much/ taller a contour is
    //     than it is wide unless the contour is unobscured.
    //
    // (2) The alignment of the oriented bounding boxes.  They should be
    //     close to parallel, even if something is obscuring one of the pieces
    //     of tape (leaving two contours where there was one before.)
    //
    // (3) In any given pair of contours, we shoot for at least one to be
    //     "unobscured" (specifically meaning that the ratio of its oriented
    //     bounding box's longer dimension to the shorter dimension approaches
    //     5in/2in.)

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

            // ----------------------------------------------------------------
            // Throw out the contour pairs that fail our quick rejection
            // tests.

            if (boundingBoxesAreTooDisjoint(rect1, rect2)) {
                continue;
            }

            if (solutionType == Boiler) {
                if (boundingBoxWidthsAreTooDissimilar(rect1, rect2)) {
                    // cout << "  Rejecting (" << i << ", " << j << ").\n";
                    continue;
                }
            } else {
                if (boundingBoxesAreNotBothVertical(rect1, rect2) ||
                    boundingBoxesAreNotHorizontallyAligned(rect1, rect2)) {
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
                RotatedRect rotatedRect1 = minAreaRect(c1);
                RotatedRect rotatedRect2 = minAreaRect(c2);

                score += scorePegContoursUsingOrientedBoundingBoxAngles(rotatedRect1, rotatedRect2);
                //score += scorePegContoursUsingOrientedBoundingBoxAspectRatios(rotatedRect1, rotatedRect2);
            }

            // If the score is still too low, this pair sucks.
            // "Too low" here is pretty arbitrary.
            if (score < 0.1) { // TODO: This should be a named, top-level const.
                cout << "Score " << score << " too low.  Rejecting (" << i << ", " << j << ").\n";
                continue;
            }

            // Accepted!
            cout << "Pair (" << i << ", " << j << ") accepted with a score of " << score << "\n";
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

// -THE- distance-finding algorithm!
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
// @return Distance to the center of the quadrilateral, in inches.
//
// TODO: The camera image dimensions are not necessarily IMG_WIDTH x IMG_HEIGHT.
// We either have to scale the images down to size or pass the frame's width
// and height as arguments to this function.
double PapasVision::findDistToGoal(const vector<Point> &topPoints,
                                   const vector<Point> &bottomPoints,
                                   double realTapeHeight) const {
    double topMidPointY = (topPoints[0].y + topPoints[1].y) / 2.0;
    double bottomMidPointY = (bottomPoints[0].y + bottomPoints[1].y) / 2.0;
    double degPerPixelVert = VERT_FOV_DEG / IMG_HEIGHT;
    double theta_b = degPerPixelVert * (((IMG_HEIGHT - 1) / 2.0) - bottomMidPointY);
    double theta_t = degPerPixelVert * (((IMG_HEIGHT - 1) / 2.0) - topMidPointY);
    double theta_w = CAM_EL_DEG + theta_t;
    double theta_rb = 90.0 - theta_w;
    double theta_hg = theta_t - theta_b;
    double theta_cb = CAM_EL_DEG + theta_b;
    double rb =
        (realTapeHeight * sin(theta_rb * DEGREES_TO_RADIANS)) /
        sin(theta_hg * DEGREES_TO_RADIANS);
    double distance = rb * cos(theta_cb * DEGREES_TO_RADIANS);
    return distance;
}

// Utility function for filterContours().
//
// Determines how far we need to turn in order to face our goal rectangle head
// on.
double PapasVision::findAzimuthGoal(const vector<Point> &topPoints,
                                    const vector<Point> &bottomPoints) const {
    double topMidPointX = (topPoints[0].x + topPoints[1].x) / 2.0;
    double bottomMidPointX = (bottomPoints[0].x + bottomPoints[1].x) / 2.0;
    double goalCenterX = (topMidPointX + bottomMidPointX) / 2.0;
    double degPerPixelHoriz = HORIZ_FOV_DEG / IMG_WIDTH;
    double imageCenterX = (IMG_WIDTH - 1) / 2.0;
    double azimuthGoalDeg = (goalCenterX - imageCenterX) * degPerPixelHoriz;

    return azimuthGoalDeg;
}

} // end (namespace robot)
