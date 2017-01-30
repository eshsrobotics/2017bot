#include "PapasVision.h"

#include <array>
#include <iostream>
#include <sstream>
#include <ctime>
#include <cmath>
#include <iomanip>

using namespace cv;
using namespace std;

namespace robot
{

// Critical constants for findDistToGoal().

const double DEGREES_TO_RADIANS = 3.1415926 / 180.0;

const double HORIZ_FOV_DEG = 59.253; // 59.703;
const double HORIZ_FOV_RAD = HORIZ_FOV_DEG * DEGREES_TO_RADIANS;
const double VERT_FOV_DEG = 44.44; // 33.583;
const double VERT_FOV_RAD = VERT_FOV_DEG * DEGREES_TO_RADIANS;
const double REAL_TAPE_HEIGHT = 12; // inches of real tape height
const double IMG_HEIGHT = 480;      // pixels of image resolution
const double IMG_WIDTH = 640;       // pixels of image resolution
const double CAM_EL_DEG = 45;
const double CAM_EL_RAD = CAM_EL_DEG * DEGREES_TO_RADIANS;

PapasVision::PapasVision(const Config &config_, double goalRejectionThresholdInches_, bool writeIntermediateFilesToDisk_)
    : config(config_),
      camera(VideoCapture()),
      distToGoalInch(0),
      azimuthGoalDeg(0),
      solutionFound(false),
      writeIntermediateFilesToDisk(writeIntermediateFilesToDisk_),
      goalRejectionThresholdInches(goalRejectionThresholdInches_)
{

    cout << "Welcome to OpenCV " << CV_VERSION << "\n";
}

bool PapasVision::getSolutionFound() const
{
    return solutionFound;
}

double PapasVision::getAzimuthGoalDeg() const
{
    return azimuthGoalDeg;
}

double PapasVision::getDistToGoalInch() const
{
    return distToGoalInch;
}

// Our most important public function.  Ultimately, the purpose of this
// function is to use OpenCV's computer vision analysis functions to calculate
// three numbers from the latest camera image: distToGoalInch, azimuthGoalDeg,
// and elevationGoalDeg.
void PapasVision::findGoal(int pictureFile)
{

    clock_t startTime = clock();

    // Determine whether or not the camera is present.  If not, we'll use the fake
    // images in 2017bot/samples.
    bool cameraPresent = (config.cameraFolder() != "");
    string cameraFolder = "./samples";
    stringstream stream;
    stream << cameraFolder << "/" << pictureFile;
    string pathPrefix = stream.str();

    if (cameraPresent)
    {
        cameraFolder = config.cameraFolder();
    }

    solutionFound = false;
    Mat frame;
    Mat output;

    if (cameraPresent == false)
    {
        // Read from the fake sample image.
        frame = imread(pathPrefix + ".png");
    }
    else
    {
        // Read from the real camera.
        camera.read(frame);

        if (writeIntermediateFilesToDisk)
        {
            imwrite(pathPrefix + ".png", frame);
        }
    }

    Mat greenFrameRes;
    getGreenResidual(frame, greenFrameRes);
    if (writeIntermediateFilesToDisk)
    {
        imwrite(pathPrefix + "_1_green_residual.png", greenFrameRes);
    }

    Mat greenFrameResFilt;
    bilateralFilter(greenFrameRes, greenFrameResFilt, 9, 75, 75);
    if (writeIntermediateFilesToDisk)
    {
        imwrite(pathPrefix + "_2_green_residual_filt.png", greenFrameResFilt);
    }

    cancelColorsTape(greenFrameResFilt, output);
    if (writeIntermediateFilesToDisk)
    {
        imwrite(pathPrefix + "_3_cancelcolors.png", output);
    }

    erode(output, output, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    dilate(output, output, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    dilate(output, output, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    erode(output, output, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    if (writeIntermediateFilesToDisk)
    {
        imwrite(pathPrefix + "_4_cancelcolors_morphfilt.png", output);
    }

    vector<vector<Point>> contours = findContours(output);

    Mat frameContours = frame.clone();
    for (unsigned int i = 0; i < contours.size(); i++)
    {
        drawContours(frameContours, contours, i, Scalar(0, 0, 255));
    }
    if (writeIntermediateFilesToDisk)
    {
        imwrite(pathPrefix + "_5_frame_contours.png", frameContours);
    }

    contours = filterContours(contours);

    Mat frameFiltContours = frame.clone();
    for (unsigned int i = 0; i < contours.size(); i++)
    {
        drawContours(frameFiltContours, contours, i, Scalar(0, 0, 255));
    }
    if (writeIntermediateFilesToDisk)
    {
        imwrite(pathPrefix + "_6_frame_filtcontours.png", frameFiltContours);
    }

    if (contours.size() > 0)
    {
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
        if (writeIntermediateFilesToDisk)
        {
            imwrite(pathPrefix + "_7_frame_points.png", framePoints);
        }

        distToGoalInch = findDistToGoal(topPts, bottomPts);
        azimuthGoalDeg = findAzimuthGoal(topPts, bottomPts);

        if (distToGoalInch > goalRejectionThresholdInches)
        {
            if (writeIntermediateFilesToDisk)
            {
                cout << "Sorry, integrity check failed (distance to goal was found to be "
                     << setprecision(4) << distToGoalInch
                     << " inches, but we were told to reject anything greater than"
                     << goalRejectionThresholdInches
                     << " inches.)  PictureFile number: " << pictureFile
                     << "\n";
            }
        }
        else
        {
            solutionFound = true;
        }
    }
    else
    {
        if (writeIntermediateFilesToDisk)
        {
            cout << "Solution not found";
        }
    }
    // if (writeIntermediateFilesToDisk)
    // {
    //     double processingTimeMs = 1000.0 * (clock() - startTime) / CLOCKS_PER_SEC;
    //     cout << "Processing time: " << setprecision(4) << processingTimeMs << " ms\n";
    // }
}

void PapasVision::getGreenResidual(const cv::Mat &rgbFrame, cv::Mat &greenResidual) const
{

    array<Mat, 3> listRGB;
    split(rgbFrame, &listRGB[0]);
    greenResidual = listRGB.at(1);
    scaleAdd(listRGB.at(0), -0.5, greenResidual, greenResidual);
    scaleAdd(listRGB.at(2), -0.5, greenResidual, greenResidual);
}

void PapasVision::convertImage(const Mat &input, Mat &output) const
{
    // cvtColor(input, output, COLOR_RGB2GRAY);

    blur(input, output, Size(5, 5));
    // cvtColor(output, output, COLOR_RGB2GRAY);
    cvtColor(output, output, COLOR_BGR2HSV);
}

// scalar params: H(0-180), S(0-255), V(0-255)
void PapasVision::cancelColorsTape(const Mat &input, Mat &output) const
{

    threshold(input, output, 0, 255, THRESH_BINARY + THRESH_OTSU);
}

vector<vector<Point>> PapasVision::findContours(const Mat &image) const
{
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

vector<vector<Point>> PapasVision::filterContours(const vector<vector<Point>> &contours)
{
    vector<vector<Point>> newContours;
    vector<vector<Point>> convexHulls;

    for (unsigned int i = 0; i < contours.size(); i++)
    {
        vector<int> convexHullMatOfInt;
        vector<Point> convexHullMatOfPoint;

        convexHull(contours.at(i), convexHullMatOfInt);

        for (unsigned int j = 0; j < convexHullMatOfInt.size(); j++)
        {
            convexHullMatOfPoint.push_back(contours.at(i).at(convexHullMatOfInt.at(j)));
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
        double widthPercentDiff = abs(topWidth - bottomWidth) / ((topWidth + bottomWidth) / 2.0) * 100.0;
        double heightPercentDiff = abs(leftHeight - rightHeight) / ((leftHeight + rightHeight) / 2.0) * 100.0;

        double topLen = sqrt((topPts[1].x - topPts[0].x) * (topPts[1].x - topPts[0].x) + (topPts[1].y - topPts[0].y) * (topPts[1].y - topPts[0].y));
        double bottomLen = sqrt((bottomPts[1].x - bottomPts[0].x) * (bottomPts[1].x - bottomPts[0].x) + (bottomPts[1].y - bottomPts[0].y) * (bottomPts[1].y - bottomPts[0].y));
        double leftLen = sqrt((topPts[0].x - bottomPts[0].x) * (topPts[0].x - bottomPts[0].x) + (topPts[0].y - bottomPts[0].y) * (topPts[0].y - bottomPts[0].y));
        double rightLen = sqrt((topPts[1].x - bottomPts[1].x) * (topPts[1].x - bottomPts[1].x) + (topPts[1].y - bottomPts[1].y) * (topPts[1].y - bottomPts[1].y));

        double equivalentAspectRatio = ((topLen + bottomLen) / 2.0) / ((leftLen + rightLen) / 2.0);

        if (contourToConvexHullRatio < 0.6 &&
            rect.width > 40 &&
            rect.height > 40 &&
            widthPercentDiff < 10.0 &&
            heightPercentDiff < 10.0 &&
            equivalentAspectRatio > 1.17 &&
            equivalentAspectRatio < 2.17 &&
            points2f.size() == 4)
        {
            // avgAspectRatio > 1.0 && avgAspectRatio < 4.0) {

            newContours.push_back(convexHullMatOfPoint);
            // newContours.add(contours.at(i));
        }
    }
    return newContours;
}

// Utility function for filterContours().
//
// Finds approximate point vertices of contoured goal tape.
vector<Point2f> PapasVision::approxPoly(const vector<Point> &contour) const
{
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
vector<Point> PapasVision::findBottomPts(const vector<Point2f> &points, Rect rect) const
{

    Point rectBottomRight = rect.br();
    Point rectBottomLeft(rect.br().x - (rect.width - 1), rect.br().y);
    Point bottomRight;
    Point bottomLeft;

    double lowestDist = 0;
    for (unsigned int i = 0; i < points.size(); i++)
    {
        double dist = sqrt((points[i].x - rectBottomLeft.x) * (points[i].x - rectBottomLeft.x) + (points[i].y - rectBottomLeft.y) * (points[i].y - rectBottomLeft.y));

        if (i == 0)
        {
            bottomLeft = points.at(i);
            lowestDist = dist;
        }
        else if (dist < lowestDist)
        {
            bottomLeft = points.at(i);
            lowestDist = dist;
        }
    }

    for (unsigned int i = 0; i < points.size(); i++)
    {
        double dist = sqrt((points[i].x - rectBottomRight.x) * (points[i].x - rectBottomRight.x) + (points[i].y - rectBottomRight.y) * (points[i].y - rectBottomRight.y));

        if (i == 0)
        {
            bottomRight = points.at(i);
            lowestDist = dist;
        }
        else if (dist < lowestDist)
        {
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
vector<Point> PapasVision::findTopPts(const vector<Point2f> &points, Rect rect) const
{
    Point rectTopRight(rect.tl().x + (rect.width - 1), rect.tl().y);
    Point rectTopLeft = rect.tl();
    Point topRight;
    Point topLeft;

    double lowestDist = 0;
    for (unsigned int i = 0; i < points.size(); i++)
    {
        double dist = sqrt((points[i].x - rectTopLeft.x) * (points[i].x - rectTopLeft.x) + (points[i].y - rectTopLeft.y) * (points[i].y - rectTopLeft.y));

        if (i == 0)
        {
            topLeft = points.at(i);
            lowestDist = dist;
        }
        else if (dist < lowestDist)
        {
            topLeft = points.at(i);
            lowestDist = dist;
        }
    }

    for (unsigned int i = 0; i < points.size(); i++)
    {
        double dist = sqrt((points[i].x - rectTopRight.x) * (points[i].x - rectTopRight.x) + (points[i].y - rectTopRight.y) * (points[i].y - rectTopRight.y));

        if (i == 0)
        {
            topRight = points.at(i);
            lowestDist = dist;
        }
        else if (dist < lowestDist)
        {
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
vector<Point> PapasVision::findGoalContour(const vector<vector<Point>> &contours) const
{
    vector<Rect> rects;
    rects.push_back(boundingRect(contours.at(0)));
    int lrgstRectIndx = 0;
    for (unsigned int i = 1; i < contours.size(); i++)
    {
        Rect rect = boundingRect(contours.at(i));
        rects.push_back(rect);
        if (rect.width > rects.at(lrgstRectIndx).width)
        {
            lrgstRectIndx = i;
        }
    }
    return contours.at(lrgstRectIndx);
}

// Utility function for filterContours().
//
// Uses trigonometry to deduce the distance to our goal rectangle.
double PapasVision::findDistToGoal(const vector<Point> &topPoints, const vector<Point> &bottomPoints) const
{
    double topMidPointY = (topPoints[0].y + topPoints[1].y) / 2.0;
    double bottomMidPointY = (bottomPoints[0].y + bottomPoints[1].y) / 2.0;
    double degPerPixelVert = VERT_FOV_DEG / IMG_HEIGHT;
    double theta_b = degPerPixelVert * (((IMG_HEIGHT - 1) / 2.0) - bottomMidPointY);
    double theta_t = degPerPixelVert * (((IMG_HEIGHT - 1) / 2.0) - topMidPointY);
    double theta_w = CAM_EL_DEG + theta_t;
    double theta_rb = 90.0 - theta_w;
    double theta_hg = theta_t - theta_b;
    double theta_cb = CAM_EL_DEG + theta_b;
    double rb = (REAL_TAPE_HEIGHT * sin(theta_rb * DEGREES_TO_RADIANS)) / sin(theta_hg * DEGREES_TO_RADIANS);
    double distance = rb * cos(theta_cb * DEGREES_TO_RADIANS);
    return distance;
}

// Utility function for filterContours().
//
// Determines how far we need to turn in order to face our goal rectangle head
// on.
double PapasVision::findAzimuthGoal(const vector<Point> &topPoints, const vector<Point> &bottomPoints) const
{
    double topMidPointX = (topPoints[0].x + topPoints[1].x) / 2.0;
    double bottomMidPointX = (bottomPoints[0].x + bottomPoints[1].x) / 2.0;
    double goalCenterX = (topMidPointX + bottomMidPointX) / 2.0;
    double degPerPixelHoriz = HORIZ_FOV_DEG / IMG_WIDTH;
    double imageCenterX = (IMG_WIDTH - 1) / 2.0;
    double azimuthGoalDeg = (goalCenterX - imageCenterX) * degPerPixelHoriz;

    return azimuthGoalDeg;
}
} // end (namespace robot)
