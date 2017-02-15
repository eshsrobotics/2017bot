#include "PapasVision.h"

#include <array>
#include <cmath>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <iostream>
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
const double BOILER_REAL_TAPE_BOTTOM_HEIGHT =
    2; // inches of real boiler tape bottom height
const double BOILER_REAL_TAPE_TOP_HEIGHT =
    4; // inches of real boiler tape top height
const double BOILER_REAL_TAPE_WIDTH = 15; // inches of real boiler tape width
const double PEG_REAL_TAPE_HEIGHT = 5;    // inches of real peg tape height
const double PEG_REAL_TAPE_WIDTH = 2;     // inches of real peg tape width
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

void PapasVision::findPeg(int pictureFile) {
  VideoCapture camera2(1);
  findPeg(pictureFile, camera2);
}

void PapasVision::findBoiler(int pictureFile) {
  VideoCapture camera1(0);
  findBoiler(pictureFile, camera1);
}

bool PapasVision::getSolutionFound() const { return solutionFound; }

double PapasVision::getAzimuthGoalDeg() const { return azimuthGoalDeg; }

double PapasVision::getDistToGoalInch() const { return distToGoalInch; }

/////////////////////////////////////////////////////////////////////////////////
// Our most important public functions.  Ultimately, their purpose is to use //
// OpenCV's computer vision analysis functions to calculate three numbers from
// //
// the latest camera image: distToGoalInch, azimuthGoalDeg, and //
// elevationGoalDeg. //
/////////////////////////////////////////////////////////////////////////////////

void PapasVision::findBoiler(int pictureFile, VideoCapture &camera1) {
  // clock_t startTime = clock();

  // Determine whether or not the camera is present.  If not, we'll use the fake
  // images in 2017bot/samples.
  bool cameraPresent = (config.cameraFolder() != "");
  string cameraFolder = "./samples";
  stringstream stream;
  stream << cameraFolder << "/" << pictureFile;
  string pathPrefix = stream.str();

  if (cameraPresent) {
    cameraFolder = config.cameraFolder();
  }

  solutionFound = false;
  Mat frame;
  Mat frame1;
  Mat output;
  int index = 1;

  if (cameraPresent == false) {
    // Read from the fake sample image.
    frame = imread(pathPrefix + ".png");

    // none flipped images
    frame1 = frame;

    // for flipped images
    //  transpose(frame, frame1);
    //  flip(frame1, frame1, 1);
  } else {
    // Read from the real camera.
    camera1.read(frame1);

    if (writeIntermediateFilesToDisk) {
      imwrite(pathPrefix + ".png", frame);
    }
  }

  Mat greenFrameRes;
  getGreenResidual(frame1, greenFrameRes);
  if (writeIntermediateFilesToDisk) {
    save(pathPrefix, index++, "green_residual.png", greenFrameRes);
  }

  Mat greenFrameResFilt;
  // This function that is in opencv removes noise and removes texture from the
  // image.
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

  Mat frameContours = frame1.clone();
  for (unsigned int i = 0; i < contours.size(); i++) {
    drawContours(frameContours, contours, i, Scalar(0, 0, 255));
  }
  if (writeIntermediateFilesToDisk) {
    save(pathPrefix, index++, "frame_contours.png", frameContours);
  }

  // contours = filterContours(contours);
  contours = findBestContourPair(contours);


  Mat frameFiltContours = frame1.clone();
  for (unsigned int i = 0; i < contours.size(); i++) {
    drawContours(frameFiltContours, contours, i, Scalar(0, 0, 255));
  }
  if (writeIntermediateFilesToDisk) {
    save(pathPrefix, index++, "frame_filtcontours.png", frameFiltContours);
  }

  if (contours.size() > 0) {
    vector<Point> goalContour = findGoalContour(contours);
    Rect goalRect = boundingRect(goalContour);
    vector<Point2f> points2f = approxPoly(goalContour);

    vector<Point> bottomPts = findBottomPts(points2f, goalRect);
    vector<Point> topPts = findTopPts(points2f, goalRect);

    Mat framePoints = frame1.clone();
    circle(framePoints, topPts.at(0), 5, Scalar(0, 255, 0));
    circle(framePoints, topPts.at(1), 5, Scalar(0, 255, 0));
    circle(framePoints, bottomPts.at(0), 5, Scalar(0, 0, 255));
    circle(framePoints, bottomPts.at(1), 5, Scalar(0, 0, 255));
    if (writeIntermediateFilesToDisk) {
      save(pathPrefix, index++, "frame_points.png", framePoints);
    }

    distToGoalInch = findDistToGoal(topPts, bottomPts);
    azimuthGoalDeg = findAzimuthGoal(topPts, bottomPts);

    if (distToGoalInch > goalRejectionThresholdInches) {
      if (writeIntermediateFilesToDisk) {
        cout << "Sorry, integrity check failed (distance to goal was found to "
                "be "
             << setprecision(4) << distToGoalInch
             << " inches, but we were told to reject anything greater than"
             << goalRejectionThresholdInches
             << " inches.)  PictureFile number: " << pictureFile << "\n";
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

void PapasVision::findPeg(int pictureFile, VideoCapture &camera2) {
  // clock_t startTime = clock();

  // Determine whether or not the camera is present.  If not, we'll use the fake
  // images in 2017bot/samples.
  bool cameraPresent = (config.cameraFolder() != "");
  string cameraFolder = "./samples";
  stringstream stream;
  stream << cameraFolder << "/" << pictureFile;
  string pathPrefix = stream.str();

  if (cameraPresent) {
    cameraFolder = config.cameraFolder();
  }

  solutionFound = false;
  Mat frame;
  Mat frame1;
  Mat output;

  if (cameraPresent == false) {
    // Read from the fake sample image.
    frame = imread(pathPrefix + ".png");

    // none flipped images
    frame1 = frame;

    // for flipped images
    //  transpose(frame, frame1);
    //  flip(frame1, frame1, 1);
  } else {
    // Read from the real camera.
    camera2.read(frame1);

    if (writeIntermediateFilesToDisk) {
      imwrite(pathPrefix + ".png", frame);
    }
  }

  Mat greenFrameRes;
  getGreenResidual(frame1, greenFrameRes);
  if (writeIntermediateFilesToDisk) {
    imwrite(pathPrefix + "_1_green_residual.png", greenFrameRes);
  }

  Mat greenFrameResFilt;
  bilateralFilter(greenFrameRes, greenFrameResFilt, 9, 280, 280);
  if (writeIntermediateFilesToDisk) {
    imwrite(pathPrefix + "_2_green_residual_filt.png", greenFrameResFilt);
  }

  cancelColorsTape(greenFrameResFilt, output);
  if (writeIntermediateFilesToDisk) {
    imwrite(pathPrefix + "_3_cancelcolors.png", output);
  }

  erode(output, output, getStructuringElement(MORPH_OPEN, Size(5, 5)));
  dilate(output, output, getStructuringElement(MORPH_OPEN, Size(5, 5)));
  dilate(output, output, getStructuringElement(MORPH_OPEN, Size(5, 5)));
  erode(output, output, getStructuringElement(MORPH_OPEN, Size(5, 5)));

  if (writeIntermediateFilesToDisk) {
    imwrite(pathPrefix + "_4_cancelcolors_morphfilt.png", output);
  }

  vector<vector<Point>> contours = findContours(output);

  Mat frameContours = frame1.clone();
  for (unsigned int i = 0; i < contours.size(); i++) {
    drawContours(frameContours, contours, i, Scalar(0, 0, 255));
  }
  if (writeIntermediateFilesToDisk) {
    imwrite(pathPrefix + "_5_frame_contours.png", frameContours);
  }

  contours = filterContours(contours);

  Mat frameFiltContours = frame1.clone();
  for (unsigned int i = 0; i < contours.size(); i++) {
    drawContours(contours, frameFiltContours, i, Scalar(0, 0, 255));
  }
  if (writeIntermediateFilesToDisk) {
    imwrite(pathPrefix + "_6_frame_filtcontours.png", frameFiltContours);
  }

  if (contours.size() > 0) {
    vector<Point> goalContour = findGoalContour(contours);
    Rect goalRect = boundingRect(goalContour);
    vector<Point2f> points2f = approxPoly(goalContour);

    vector<Point> bottomPts = findBottomPts(points2f, goalRect);
    vector<Point> topPts = findTopPts(points2f, goalRect);

    Mat framePoints = frame.clone();
    circle(framePoints, topPts.at(0), 5, Scalar(0, 255, 0));
    circle(framePoints, topPts.at(1), 5, Scalar(0, 255, 0));
    circle(framePoints, bottomPts.at(0), 5, Scalar(0, 0, 255));
    circle(framePoints, bottomPts.at(1), 5, Scalar(0, 0, 255));
    if (writeIntermediateFilesToDisk) {
      imwrite(pathPrefix + "_7_frame_points.png", framePoints);
    }

    distToGoalInch = findDistToGoal(topPts, bottomPts);
    azimuthGoalDeg = findAzimuthGoal(topPts, bottomPts);

    if (distToGoalInch > goalRejectionThresholdInches) {
      if (writeIntermediateFilesToDisk) {
        cout << "Sorry, integrity check failed (distance to goal was found to "
                "be "
             << setprecision(4) << distToGoalInch
             << " inches, but we were told to reject anything greater than"
             << goalRejectionThresholdInches
             << " inches.)  PictureFile number: " << pictureFile << "\n";
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

void PapasVision::convertImage(const Mat &input, Mat &output) const {
  // cvtColor(input, output, COLOR_RGB2GRAY);

  blur(input, output, Size(5, 5));
  // cvtColor(output, output, COLOR_RGB2GRAY);
  cvtColor(output, output, COLOR_BGR2HSV);
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

// OLD: Filtered the contour list for last year's robot.
vector<vector<Point>>
PapasVision::filterContours(const vector<vector<Point>> &contours) {
  vector<vector<Point>> newContours;
  vector<vector<Point>> convexHulls;

  for (unsigned int i = 0; i < contours.size(); i++) {
    vector<int> convexHullMatOfInt;
    vector<Point> convexHullMatOfPoint;

    convexHull(contours.at(i), convexHullMatOfInt);

    for (unsigned int j = 0; j < convexHullMatOfInt.size(); j++) {
      convexHullMatOfPoint.push_back(
          contours.at(i).at(convexHullMatOfInt.at(j)));
    }

    double contourArea = cv::contourArea(contours.at(i));
    double convexHullArea = cv::contourArea(convexHullMatOfPoint);
    double contourToConvexHullRatio = contourArea / convexHullArea;

    Rect rect = boundingRect(contours.at(i));

    vector<Point2f> points2f = approxPoly(convexHullMatOfPoint);

    vector<Point> bottomPts = findBottomPts(points2f, rect);
    vector<Point> topPts = findTopPts(points2f, rect);

    double topWidth = abs(topPts[1].x - topPts[0].x);
    double bottomWidth = abs(bottomPts[1].x - bottomPts[0].x);
    double leftHeight = abs(bottomPts[0].y - topPts[0].y);
    double rightHeight = abs(bottomPts[1].y - topPts[1].y);
    double widthPercentDiff =
        abs(topWidth - bottomWidth) / ((topWidth + bottomWidth) / 2.0) * 100.0;
    double heightPercentDiff = abs(leftHeight - rightHeight) /
                               ((leftHeight + rightHeight) / 2.0) * 100.0;

    double topLen =
        sqrt((topPts[1].x - topPts[0].x) * (topPts[1].x - topPts[0].x) +
             (topPts[1].y - topPts[0].y) * (topPts[1].y - topPts[0].y));
    double bottomLen = sqrt(
        (bottomPts[1].x - bottomPts[0].x) * (bottomPts[1].x - bottomPts[0].x) +
        (bottomPts[1].y - bottomPts[0].y) * (bottomPts[1].y - bottomPts[0].y));
    double leftLen =
        sqrt((topPts[0].x - bottomPts[0].x) * (topPts[0].x - bottomPts[0].x) +
             (topPts[0].y - bottomPts[0].y) * (topPts[0].y - bottomPts[0].y));
    double rightLen =
        sqrt((topPts[1].x - bottomPts[1].x) * (topPts[1].x - bottomPts[1].x) +
             (topPts[1].y - bottomPts[1].y) * (topPts[1].y - bottomPts[1].y));

    double equivalentAspectRatio =
        ((topLen + bottomLen) / 2.0) / ((leftLen + rightLen) / 2.0);

    if (contourToConvexHullRatio < 0.6 && rect.width > 40 && rect.height > 40 &&
        widthPercentDiff < 10.0 && heightPercentDiff < 10.0 &&
        equivalentAspectRatio > 1.17 && equivalentAspectRatio < 2.17 &&
        points2f.size() == 4) {
      // avgAspectRatio > 1.0 && avgAspectRatio < 4.0) {

      newContours.push_back(convexHullMatOfPoint);
      // newContours.add(contours.at(i));
    }
  }
  return newContours;
}

// =========================================================================
// This function is used to find out of the list of contours two parallel
// contours that are close together.
//
// As it turns out, the peg and the boiler will both have reflective tape in
// those arrangements.
//
// @param contours An array of contours -- essentially, polygons that surround
//                 interesting green targets in the camera image.
//
// @return An even _more_ interesting array of the contours that we really
//         like, because they kind of look like the sort of parallel bands
//         that we want to shoot at.
// =========================================================================
vector<vector<Point>>
PapasVision::findBestContourPair(const vector<vector<Point>> &contours) {

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

    auto areasAreTooDissimilar = [] (const Contour& c1, const Contour& c2) -> bool {
        // -----------------------------
        // Quick rejection heuristic #2.
        //
        // If two contours have very dissimilar areas, then we can safely
        // reject them.

        double area1 = contourArea(c1);
        double area2 = contourArea(c2);
        double maxArea = max(area1, area2);
        double minArea = min(area1, area2);

        // Let's say the areas are max=50 and min=36, with a tolerance of 15%.
        //
        //   Is 36 < (1-0.15)*50 ? Yes (36 < 42.5).  That's too little.
        //
        // But let's look at it from the other direction.
        //
        //   Is 50 > (1+0.15)*36 ? Yes (50 > 41.4).  That's too much.
        //
        // So areas of 36 and 50 are out of the tolerance range for each other
        // either way you interpret it.

        if (minArea < (1 - CONTOUR_PAIR_AREA_DEVIATION_TOLERANCE) * maxArea ||
            maxArea > (1 + CONTOUR_PAIR_AREA_DEVIATION_TOLERANCE) * minArea) {

            cout << minArea << " and " << maxArea << " are too far apart.";
            // Reject these!
            return true;
        } else {
            return false;
        }
    };

    auto boundingBoxWidthsAreTooDissimilar = [] (const Rect& rect1, const Rect& rect2) {
        // -------------------------------------
        // Quick rejection heuristic #3, part 1.
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

    auto boundingBoxHeightsAreTooDissimilar = [] (const Rect& rect1, const Rect& rect2) {
        // -------------------------------------
        // Quick rejection heuristic #3, part 2.
        //
        // For the PEG, if two contours have very dissimilar heights, then we
        // can safely reject them.
        //
        // This is pretty similar to the test above.

        double minHeight = min(rect1.height, rect2.height);
        double maxHeight = max(rect1.height, rect2.height);
        double low = 1 - CONTOUR_PAIR_BOUNDING_BOX_HEIGHT_DEVIATION_TOLERANCE;
        double high = 1 + CONTOUR_PAIR_BOUNDING_BOX_HEIGHT_DEVIATION_TOLERANCE;

        if (minHeight < low * maxHeight ||  maxHeight > high * minHeight) {
            // cout.precision(3);
            // if (minHeight < low * maxHeight) {
            //     cout << "Height " << minHeight << " < "
            //          << low * maxHeight << " (" << low  << "*" << maxHeight << "); ";
            // } else {
            //     cout << "Height " << maxHeight << " > "
            //          << high * minHeight << " (" << high  << "*" << minHeight << "); ";
            // }

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
        // Award a better score to two contours whose centers are separated by
        // a distance close to the average height of the bounding boxes.
        //
        // Note that by the time we make it here, we've already rejected pairs
        // where there is no X or Y overlap between bounding boxes
        // _regardless_ of the distance that separates them.  This is just a
        // bit of finesse on top of that.

        double averageHeight = (rect1.height + rect2.height) / 2.0;
        double minimumExpectedDistance = averageHeight;
        double maximumExpectedDistance = 2 * minimumExpectedDistance;

        double xCenter1 = rect1.x + rect1.width/2;
        double yCenter1 = rect1.y + rect1.height/2;
        double xCenter2 = rect2.x + rect2.width/2;
        double yCenter2 = rect2.y + rect2.height/2;
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

        double u1 = (aspectRatio1 - minAspectRatio)/(maxAspectRatio - minAspectRatio);
        u1 = min(1.0, max(u1, 0.0)); // Clamp between 0 and 1.
        score += u1 / 2;

        double u2 = (aspectRatio2 - minAspectRatio)/(maxAspectRatio - minAspectRatio);
        u2 = min(1.0, max(u2, 0.0));
        score += u2 / 2;

        return score; // 0.0 <= score <= 1.0
    };

    //////////////////////////////////////////////////////////////////////////
    // And finally, here's the actual findBestContourPair algorithm itself. //
    //                                                                      //
    // Yeah, this is O(N^2), so it's not efficient for large numbers of     //
    // contours.  We try to keep the runtime down with quick rejection      //
    // heuristics.                                                          //
    //////////////////////////////////////////////////////////////////////////

    for (unsigned int i = 0; i < contours.size(); i++) {
        for (unsigned int j = i + 1; j < contours.size() - 1; j++) {

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

            /*
            if (areasAreTooDissimilar(c1, c2)) {
                // This is rejecting too much right now--it's providing false
                // negatives by throwing out the actual boiler tape contours.
                cout << "  Rejecting (" << i << ", " << j << ").\n";
                continue;
            }
            */

            // Use boundingBoxHeightsAreTooDissimilar() for the peg solution.
            if (boundingBoxWidthsAreTooDissimilar(rect1, rect2)) {
                // cout << "  Rejecting (" << i << ", " << j << ").\n";
                continue;
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
            score += scoreBoundingBoxesUsingBoilerDistance(rect1, rect2);
            score += scoreBoundingBoxesUsingBoilerAspectRatio(rect1, rect2);

            // If the score is still too low, this pair sucks.
            // "Too low" here is pretty arbitrary.
            if (score < 0.1) { // TODO: This should be a named, top-level const.
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

    // TODO: Draw the first three contours in the sorted list.

    // Only the highest-scoring pair matters for the results.
    if (scoredPairsList.size() > 0) {
        int i = get<1>(scoredPairsList[0]);
        int j = get<2>(scoredPairsList[0]);
        results.push_back(contours[i]);
        results.push_back(contours[j]);
    }

    return results;
}

// Utility function for filterContours().
//
// Finds approximate point vertices of contoured goal tape.
vector<Point2f> PapasVision::approxPoly(const vector<Point> &contour) const {
  vector<Point2f> point2f;
  Mat(contour).copyTo(point2f);
  approxPolyDP(point2f, point2f, 5.0, true); // third parameter:
  // smaller->more
  // points
  return point2f;
}

// Utility function for filterContours().
//
// Finds bottom vertices of goal tape, left to right.
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

// Utility function for filterContours().
//
// Finds top vertices of goal tape, left to right.
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

// Utility function for filterContours().
//
// Identifies the largest contour in the input list -- that's where we assume
// our taped goal is.
vector<Point>
PapasVision::findGoalContour(const vector<vector<Point>> &contours) const {
  vector<Rect> rects;
  rects.push_back(boundingRect(contours.at(0)));
  int lrgstRectIndx = 0;
  for (unsigned int i = 1; i < contours.size(); i++) {
    Rect rect = boundingRect(contours.at(i));
    rects.push_back(rect);
    if (rect.width > rects.at(lrgstRectIndx).width) {
      lrgstRectIndx = i;
    }
  }
  return contours.at(lrgstRectIndx);
}

///////////////////////////////////////////////////////////////////////////////////
// Trigonometric functions that deal with the computer vision contours we found.
// //
///////////////////////////////////////////////////////////////////////////////////

// Utility function for filterContours().
//
// Uses trigonometry to deduce the distance to our goal rectangle.
double PapasVision::findDistToGoal(const vector<Point> &topPoints,
                                   const vector<Point> &bottomPoints) const {
  double topMidPointY = (topPoints[0].y + topPoints[1].y) / 2.0;
  double bottomMidPointY = (bottomPoints[0].y + bottomPoints[1].y) / 2.0;
  double degPerPixelVert = VERT_FOV_DEG / IMG_HEIGHT;
  double theta_b =
      degPerPixelVert * (((IMG_HEIGHT - 1) / 2.0) - bottomMidPointY);
  double theta_t = degPerPixelVert * (((IMG_HEIGHT - 1) / 2.0) - topMidPointY);
  double theta_w = CAM_EL_DEG + theta_t;
  double theta_rb = 90.0 - theta_w;
  double theta_hg = theta_t - theta_b;
  double theta_cb = CAM_EL_DEG + theta_b;
  double rb =
      (BOILER_REAL_TAPE_TOP_HEIGHT * sin(theta_rb * DEGREES_TO_RADIANS)) /
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
