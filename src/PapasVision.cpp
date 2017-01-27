#include "PapasVision.h"

#include <array>
#include <iostream>
#include <sstream>
#include <ctime>
#include <cmath>
#include <iomanip>

// #include <opencv2/modules/core/include/opencv2/core/version.hpp> // CV_VERSION

using namespace cv;
using namespace std;

namespace robot {

// /**
//  * @author Ari Berkowicz
//  */
//  PapasVision{
//      final static double HORIZ_FOV_DEG = 59.253;
//      // final static double HORIZ_FOV_DEG = 59.703;
//      final static double HORIZ_FOV_RAD = Math.toRadians(HORIZ_FOV_DEG);
//      final static double VERT_FOV_DEG = 44.44;
//      // final static double VERT_FOV_DEG = 33.583;
//      final static double VERT_FOV_RAD = Math.toRadians(VERT_FOV_DEG);
//      final static double REAL_TAPE_HEIGHT = 12; // inches of real tape height
//      final static double IMG_HEIGHT = 480; // pixels of image resolution
//      final static double IMG_WIDTH = 640; // pixels of image resolution
//      final static double CAM_EL_DEG = 45;
//      final static double CAM_EL_RAD = Math.toRadians(CAM_EL_DEG);
//
//      VideoCapture camera;
//
//      double distToGoalInch;
//      double azimuthGoalDeg;
//      double elevationGoalDeg;

//      public PapasVision(double goalRejectionThresholdInches, boolean writeIntermediateFilesToDisk) {
//              System.load("/usr/local/share/OpenCV/java/libopencv_java310.so");
//              System.out.println("Welcome to OpenCV " + Core.VERSION);
//              camera = new VideoCapture(0);
//
//              this.goalRejectionThresholdInches = goalRejectionThresholdInches;
//              this.writeIntermediateFilesToDisk = writeIntermediateFilesToDisk;
//      }


PapasVision::PapasVision(const Config& config_, double goalRejectionThresholdInches_, bool writeIntermediateFilesToDisk_)
    : config(config_),
      camera(VideoCapture()),
      solutionFound(false),
      writeIntermediateFilesToDisk(writeIntermediateFilesToDisk_),
      goalRejectionThresholdInches(goalRejectionThresholdInches_) {

    cout << "Welcome to OpenCV " << CV_VERSION << "\n";
}

void PapasVision::findGoal(int pictureFile) {

    clock_t startTime = clock();

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

    if(writeIntermediateFilesToDisk)
    {
            cout << "Image number: " << pictureFile << "\n";
    }

    solutionFound = false;
    Mat frame;
    Mat output;

    if (cameraPresent == false) {
        // Read from the fake sample image.
        frame = imread(pathPrefix + ".png");
    } else {
        // Read from the real camera.
        camera.read(frame);

        if (writeIntermediateFilesToDisk) {
            imwrite(pathPrefix + ".png", frame);
        }
    }

    Mat greenFrameRes;
    getGreenResidual(frame, greenFrameRes);
    if (writeIntermediateFilesToDisk) {
        imwrite(pathPrefix + "_1_green_residual.png", greenFrameRes);
    }

    // convertImage(frame, output);
    // if (writeIntermediateFilesToDisk) {
    //     imwrite(pictureFile + "_converted.png", output);
    // }

    Mat greenFrameResFilt;
    bilateralFilter(greenFrameRes, greenFrameResFilt, 9, 75, 75);
    if (writeIntermediateFilesToDisk) {
        imwrite(pathPrefix + "_2_green_residual_filt.png", greenFrameResFilt);
    }

    // cancelColorsTape(output, output);
    cancelColorsTape(greenFrameResFilt, output);
    if (writeIntermediateFilesToDisk) {
        imwrite(pathPrefix + "_3_cancelcolors.png", output);
    }

    erode(output,  output, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    dilate(output, output, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    dilate(output, output, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    erode(output,  output, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    if (writeIntermediateFilesToDisk) {
        imwrite(pathPrefix + "_4_cancelcolors_morphfilt.png", output);
    }

    vector<vector<Point> > contours = findContours(output);

    Mat frameContours = frame.clone();
    for (unsigned int i = 0; i < contours.size(); i++) {
        drawContours(frameContours, contours, i, Scalar(0, 0, 255));
    }
    if (writeIntermediateFilesToDisk) {
        imwrite(pathPrefix + "_5_frame_contours.png", frameContours);
    }

    contours = filterContours(contours);

    Mat frameFiltContours = frame.clone();
    for (unsigned int i = 0; i < contours.size(); i++) {
        drawContours(frameFiltContours, contours, i, Scalar(0, 0, 255));
    }
    if (writeIntermediateFilesToDisk) {
        imwrite(pathPrefix + "_6_frame_filtcontours.png", frameFiltContours);
    }
//
//              if (contours.size() > 0) {
//                      MatOfPoint goalContour = findGoalContour(contours);
//                      Rect goalRect = Imgproc.boundingRect(goalContour);
//                      MatOfPoint2f points2f = approxPoly(goalContour);
//
//                      Point[] bottomPts = findBottomPts(points2f.toArray(), goalRect);
//                      Point[] topPts = findTopPts(points2f.toArray(), goalRect);
//                      // Point[] bottomPts = findBottomPts(points2f.toArray());
//                      // Point[] topPts = findTopPts(points2f.toArray());
//
//                      Mat framePoints = new Mat();
//                      framePoints = frame.clone();
//                      Imgproc.circle(framePoints, topPts[0], 5, new Scalar(0, 255, 0));
//                      Imgproc.circle(framePoints, topPts[1], 5, new Scalar(0, 255, 0));
//                      Imgproc.circle(framePoints, bottomPts[0], 5, new Scalar(0, 0, 255));
//                      Imgproc.circle(framePoints, bottomPts[1], 5, new Scalar(0, 0, 255));
//                      if (this.writeIntermediateFilesToDisk) {
//                              Imgcodecs.imwrite(pictureFile + "_7_frame_points.png", framePoints);
//                      }
//                      // double distToGoal = findDistToGoal(goalRect.width, 31);
//                      distToGoalInch = findDistToGoal(topPts, bottomPts);
//                      azimuthGoalDeg = findAzimuthGoal(topPts, bottomPts);
//
//                      System.out.println("Solution Found, PapasDistance: " + distToGoalInch + " inches, PapasAngle: " + azimuthGoalDeg + " degrees");
//
//                      if (distToGoalInch > goalRejectionThresholdInches) {
//                              if (this.writeIntermediateFilesToDisk) {
//                                      System.out.println("Sorry integrity check failed");
//                                      System.out.println("PictureFile number: " + pictureFile);
//                              }
//                      } else {
//                              solutionFound = true;
//                      }
//
//              } else {
//                      if (this.writeIntermediateFilesToDisk) {
//                              System.out.println("Solution not found");
//                      }
//              }
    if (writeIntermediateFilesToDisk) {
            double processingTimeMs = 1000.0 * (clock() - startTime) / CLOCKS_PER_SEC;
            cout << "Processing time: " << setprecision(4) << processingTimeMs << " ms\n";
    }
}

void PapasVision::getGreenResidual(const cv::Mat& rgbFrame, cv::Mat& greenResidual) const {

    array<Mat, 3> listRGB;
    split(rgbFrame, &listRGB[0]);
    greenResidual = listRGB.at(1);
    scaleAdd(listRGB.at(0), -0.5, greenResidual, greenResidual);
    scaleAdd(listRGB.at(2), -0.5, greenResidual, greenResidual);
}

void PapasVision::convertImage(const Mat& input, Mat& output) const {
        // cvtColor(input, output, COLOR_RGB2GRAY);

        blur(input, output, Size(5, 5));
        // cvtColor(output, output, COLOR_RGB2GRAY);
        cvtColor(output, output, COLOR_BGR2HSV);
}


// scalar params: H(0-180), S(0-255), V(0-255)
void PapasVision::cancelColorsTape(const Mat& input, Mat& output) const {

    threshold(input, output, 0, 255, THRESH_BINARY + THRESH_OTSU);
}

vector<vector<Point> > PapasVision::findContours(const Mat& image) const {
    // From
    // http://docs.opencv.org/master/d3/dc0/group__imgproc__shape.html#ga17ed9f5d79ae97bd4c7cf18403e1689a:
    //
    // contours
    //   Detected contours. Each contour is stored as a vector of points
    //   (e.g. std::vector<std::vector<cv::Point> >).
    // hierarchy
    //   Optional output vector (e.g. std::vector<cv::Vec4i>), containing
    //   information about the image topology.

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    cv::findContours(image, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    return contours;
}


vector<vector<Point> > PapasVision::filterContours(const vector<vector<Point> >& contours) {
    vector<vector<Point> > newContours;
    vector<vector<Point> > convexHulls;

    for (unsigned int i = 0; i < contours.size(); i++) {
        vector<int> convexHullMatOfInt;
        // vector<Point> convexHullPointArrayList;
        vector<Point> convexHullMatOfPoint;
        // ArrayList convexHullMatOfPointArrayList = new
        // ArrayList<MatOfPoint>();

        convexHull(contours.at(i), convexHullMatOfInt);

        for (unsigned int j = 0; j < convexHullMatOfInt.size(); j++) {
            // convexHullPointArrayList.push_back(contours.at(i).at(convexHullMatOfInt.at(j)));
            convexHullMatOfPoint.push_back(contours.at(i).at(convexHullMatOfInt.at(j)));
        }
        // convexHullMatOfPoint.fromList(convexHullPointArrayList);

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

        double topLen = sqrt((topPts[1].x - topPts[0].x) * (topPts[1].x - topPts[0].x)
                             + (topPts[1].y - topPts[0].y) * (topPts[1].y - topPts[0].y));
        double bottomLen = sqrt((bottomPts[1].x - bottomPts[0].x) * (bottomPts[1].x - bottomPts[0].x)
                                + (bottomPts[1].y - bottomPts[0].y) * (bottomPts[1].y - bottomPts[0].y));
        double leftLen = sqrt((topPts[0].x - bottomPts[0].x) * (topPts[0].x - bottomPts[0].x)
                              + (topPts[0].y - bottomPts[0].y) * (topPts[0].y - bottomPts[0].y));
        double rightLen = sqrt((topPts[1].x - bottomPts[1].x) * (topPts[1].x - bottomPts[1].x)
                               + (topPts[1].y - bottomPts[1].y) * (topPts[1].y - bottomPts[1].y));

        double equivalentAspectRatio = ((topLen + bottomLen) / 2.0) / ((leftLen + rightLen) / 2.0);

        if (contourToConvexHullRatio < 0.6 &&
            rect.width > 40 &&
            rect.height > 40 &&
            widthPercentDiff < 10.0 &&
            heightPercentDiff < 10.0 &&
            equivalentAspectRatio > 1.17 &&
            equivalentAspectRatio < 2.17 &&
            points2f.size()  == 4) {
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
vector<Point2f> PapasVision::approxPoly(const vector<Point>& contour) const {
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
vector<Point> PapasVision::findBottomPts(const vector<Point2f>& points, Rect rect) const {

    Point rectBottomRight = rect.br();
    Point rectBottomLeft(rect.br().x - (rect.width - 1), rect.br().y);
    Point bottomRight;
    Point bottomLeft;

    double lowestDist = 0;
    for (unsigned int i = 0; i < points.size(); i++) {
        double dist = sqrt((points[i].x - rectBottomLeft.x) * (points[i].x - rectBottomLeft.x)
                           + (points[i].y - rectBottomLeft.y) * (points[i].y - rectBottomLeft.y));

        if (i == 0) {
            bottomLeft = points.at(i);
            lowestDist = dist;
        } else if (dist < lowestDist) {
            bottomLeft = points.at(i);
            lowestDist = dist;
        }
    }

    for (unsigned int i = 0; i < points.size(); i++) {
        double dist = sqrt((points[i].x - rectBottomRight.x) * (points[i].x - rectBottomRight.x)
                           + (points[i].y - rectBottomRight.y) * (points[i].y - rectBottomRight.y));

        if (i == 0) {
            bottomRight = points.at(i);
            lowestDist = dist;
        } else if (dist < lowestDist) {
            bottomRight = points.at(i);
            lowestDist = dist;
        }
    }

    vector<Point> bottomPts = { bottomLeft, bottomRight };
    return bottomPts;
}

// Utility function for filterContours().
//
// Finds top vertices of goal tape, left to right.
vector<Point> PapasVision::findTopPts(const vector<Point2f>& points, Rect rect) const {
    Point rectTopRight(rect.tl().x + (rect.width - 1), rect.tl().y);
    Point rectTopLeft = rect.tl();
    Point topRight;
    Point topLeft;

    double lowestDist = 0;
    for (unsigned int i = 0; i < points.size(); i++) {
        double dist = sqrt((points[i].x - rectTopLeft.x) * (points[i].x - rectTopLeft.x)
                           + (points[i].y - rectTopLeft.y) * (points[i].y - rectTopLeft.y));

        if (i == 0) {
            topLeft = points.at(i);
            lowestDist = dist;
        } else if (dist < lowestDist) {
            topLeft = points.at(i);
            lowestDist = dist;
        }
    }

    for (unsigned int i = 0; i < points.size(); i++) {
        double dist = sqrt((points[i].x - rectTopRight.x) * (points[i].x - rectTopRight.x)
                           + (points[i].y - rectTopRight.y) * (points[i].y - rectTopRight.y));

        if (i == 0) {
            topRight = points.at(i);
            lowestDist = dist;
        } else if (dist < lowestDist) {
            topRight = points.at(i);
            lowestDist = dist;
        }
    }

    vector<Point> topPts = { topLeft, topRight };
    return topPts;
}


//      static MatOfPoint findGoalContour(List<MatOfPoint> contours) {
//              std::list<Rect> rects = new ArrayList<Rect>();
//              rects.add(Imgproc.boundingRect(contours.get(0)));
//              int lrgstRectIndx = 0;
//              for (int i = 1; i < contours.size(); i++) {
//                      Rect rect = Imgproc.boundingRect(contours.get(i));
//                      rects.add(rect);
//                      if (rect.width > rects.get(lrgstRectIndx).width) {
//                              lrgstRectIndx = i;
//                      }
//              }
//              return contours.get(lrgstRectIndx);
//      }
//
//
//
//
//      static double pointDist(Point[] points) {
//              double dist = Math.sqrt((points[0].x - points[1].x) * (points[0].x - points[1].x)
//                              + (points[0].y - points[1].y) * (points[0].y - points[1].y));
//              return dist;
//      }
//
//      static double findDistToGoal(Point topPoints[], Point bottomPoints[]) {
//              double topMidPointY = (topPoints[0].y + topPoints[1].y) / 2.0;
//              double bottomMidPointY = (bottomPoints[0].y + bottomPoints[1].y) / 2.0;
//              double degPerPixelVert = VERT_FOV_DEG / IMG_HEIGHT;
//              double theta_b = degPerPixelVert * (((IMG_HEIGHT - 1) / 2.0) - bottomMidPointY);
//              double theta_t = degPerPixelVert * (((IMG_HEIGHT - 1) / 2.0) - topMidPointY);
//              double theta_w = CAM_EL_DEG + theta_t;
//              double theta_rb = 90.0 - theta_w;
//              double theta_hg = theta_t - theta_b;
//              double theta_cb = CAM_EL_DEG + theta_b;
//              double rb = (REAL_TAPE_HEIGHT * Math.sin(Math.toRadians(theta_rb))) / Math.sin(Math.toRadians(theta_hg));
//              double distance = rb * (Math.cos(Math.toRadians(theta_cb)));
//              return distance;
//      }
//
//      static double findAzimuthGoal(Point topPoints[], Point bottomPoints[]) {
//              double topMidPointX = (topPoints[0].x + topPoints[1].x) / 2.0;
//              double bottomMidPointX = (bottomPoints[0].x + bottomPoints[1].x) / 2.0;
//              double goalCenterX = (topMidPointX + bottomMidPointX) / 2.0;
//              double degPerPixelHoriz = HORIZ_FOV_DEG / IMG_WIDTH;
//              double imageCenterX = (IMG_WIDTH - 1) / 2.0;
//              double azimuthGoalDeg = (goalCenterX - imageCenterX) * degPerPixelHoriz;
//
//              return azimuthGoalDeg;
//      }
//
//      // facing forward relative to closest goal
//      static boolean isFacingForward(Point[] points) {
//              return Math.abs(points[0].y - points[1].y) < 5;
//      }
//
//      // if true then robot turns right to face goal straight on
//      static boolean isFacingLeft(Point[] points) {
//              if (points[0].y > points[1].y) {
//                      return points[0].x < points[1].x;
//              }
//              return points[0].x > points[1].x;
//      }
//
//      // finds angle at which camera is facing the goal
//      static double findLateralAngle(Point[] points) {
//              double angle = Math.toDegrees(Math.asin(Math.abs(points[0].y - points[1].y) / pointDist(points)));
//              return angle;
//      }
//
//      // midline of the closest goal
//      static boolean isOnMidline(Rect rect) {
//              return Math.abs(IMG_WIDTH / 2 - (rect.x + rect.width / 2)) < 10;
//      }
//
//      // if true then robot is on right of midline
//      static boolean isOnRight(Rect rect) {
//              return (rect.x + rect.width / 2) < IMG_WIDTH / 2;
//      }
//
//      static double distFromMidline(Point[] bottomPts, Point[] topPts, double dist) {
//              Point bottomLeft = bottomPts[0];
//              if (bottomPts[1].x < bottomPts[0].x) {
//                      bottomLeft = bottomPts[1];
//              }
//              Point topLeft = topPts[0];
//              if (topPts[1].x < topPts[0].x) {
//                      topLeft = topPts[1];
//              }
//              return dist * (Math.abs(bottomLeft.x - topLeft.x) / Math.abs(bottomLeft.y - topLeft.y));
//      }
//
//      Boolean getSolutionFound() {
//              return solutionFound;
//      }
//
//      public double getAzimuthGoalDeg() {
//              return azimuthGoalDeg;
//      }
//
//      public double getDistToGoalInch() {
//              return distToGoalInch;
//      }
// }

} // end (namespace robot)
