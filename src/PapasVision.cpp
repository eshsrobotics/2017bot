#include "PapasVision.h"

#include <iostream>
// #include <opencv2/modules/core/include/opencv2/core/version.hpp> // CV_VERSION

using namespace cv;
using namespace std;

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
//      Boolean solutionFound;
//      long processingTimeMs;



//      public PapasVision(double goalRejectionThresholdInches, boolean writeIntermediateFilesToDisk) {
//              System.load("/usr/local/share/OpenCV/java/libopencv_java310.so");
//              System.out.println("Welcome to OpenCV " + Core.VERSION);
//              camera = new VideoCapture(0);
//
//              this.goalRejectionThresholdInches = goalRejectionThresholdInches;
//              this.writeIntermediateFilesToDisk = writeIntermediateFilesToDisk;
//      }


PapasVision::PapasVision(double goalRejectionThresholdInches_, bool writeIntermediateFilesToDisk_)
    : camera(VideoCapture()),
      writeIntermediateFilesToDisk(writeIntermediateFilesToDisk_),
      goalRejectionThresholdInches(goalRejectionThresholdInches_) {

    cout << "Welcome to OpenCV " << CV_VERSION << "\n";
}

//      public void findGoal(int pictureFile, boolean useCam) {
//              long time = System.currentTimeMillis();
//              if(this.writeIntermediateFilesToDisk)
//              {
//                      System.out.println("Image number: " + pictureFile);
//              }
//
//              solutionFound = false;
//              Mat frame = new Mat();
//              Mat output = new Mat();
//
//              if (useCam == false) {
//                      frame = Imgcodecs.imread(pictureFile + ".png");
//              } else {
//                      camera.read(frame);
//
//                      if (this.writeIntermediateFilesToDisk) {
//                              Imgcodecs.imwrite(pictureFile + ".png", frame);
//                      }
//              }
//
//              Mat greenFrameRes = new Mat();
//              getGreenResidual(frame, greenFrameRes);
//              if (this.writeIntermediateFilesToDisk) {
//                      Imgcodecs.imwrite(pictureFile + "_1_green_residual.png", greenFrameRes);
//              }
//
//              // convertImage(frame, output);
//              // if (this.writeIntermediateFilesToDisk) {
//              // Imgcodecs.imwrite(pictureFile + "_converted.png", output);}
//
//              Mat greenFrameResFilt = new Mat();
//              Imgproc.bilateralFilter(greenFrameRes, greenFrameResFilt, 9, 75, 75);
//              if (this.writeIntermediateFilesToDisk) {
//                      Imgcodecs.imwrite(pictureFile + "_2_green_residual_filt.png", greenFrameResFilt);
//              }
//
//              // cancelColorsTape(output, output);
//              cancelColorsTape(greenFrameResFilt, output);
//              if (this.writeIntermediateFilesToDisk) {
//                      Imgcodecs.imwrite(pictureFile + "_3_cancelcolors.png", output);
//              }
//
//              Imgproc.erode(output, output, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5)));
//              Imgproc.dilate(output, output, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5)));
//              Imgproc.dilate(output, output, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5)));
//              Imgproc.erode(output, output, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5)));
//              if (this.writeIntermediateFilesToDisk) {
//                      Imgcodecs.imwrite(pictureFile + "_4_cancelcolors_morphfilt.png", output);
//              }
//
//              std::ist<MatOfPoint> contours = findContours(output);
//
//              Mat frameContours = new Mat();
//              frameContours = frame.clone();
//              for (int i = 0; i < contours.size(); i++) {
//                      Imgproc.drawContours(frameContours, contours, i, new Scalar(0, 0, 255));
//              }
//              if (this.writeIntermediateFilesToDisk) {
//                      Imgcodecs.imwrite(pictureFile + "_5_frame_contours.png", frameContours);
//              }
//
//              contours = filterContours(contours);
//
//              Mat frameFiltContours = new Mat();
//              frameFiltContours = frame.clone();
//              for (int i = 0; i < contours.size(); i++) {
//                      Imgproc.drawContours(frameFiltContours, contours, i, new Scalar(0, 0, 255));
//              }
//              if (this.writeIntermediateFilesToDisk) {
//                      Imgcodecs.imwrite(pictureFile + "_6_frame_filtcontours.png", frameFiltContours);
//              }
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
//              if (this.writeIntermediateFilesToDisk) {
//                      processingTimeMs = System.currentTimeMillis() - time;
//                      System.out.println("Processing time: " + processingTimeMs + " ms");
//              }
//      }
//
//      static void getGreenResidual(Mat rgbFrame, Mat greenResidual) {
//              std::list<Mat> listRGB = new ArrayList<Mat>(3);
//              Core.split(rgbFrame, listRGB);
//              listRGB.get(1).copyTo(greenResidual);
//              Core.scaleAdd(listRGB.get(0), -0.5, greenResidual, greenResidual);
//              Core.scaleAdd(listRGB.get(2), -0.5, greenResidual, greenResidual);
//      }
//
//      static void convertImage(Mat input, Mat output) {
//              // Imgproc.cvtColor(input, output, Imgproc.COLOR_RGB2GRAY);
//
//              Imgproc.blur(input, output, new Size(5, 5));
//              // Imgproc.cvtColor(output, output, Imgproc.COLOR_RGB2GRAY);
//              Imgproc.cvtColor(output, output, Imgproc.COLOR_BGR2HSV);
//      }
//
//      // scalar params: H(0-180), S(0-255), V(0-255)
//      static void cancelColorsTape(Mat input, Mat output) {
//
//              Imgproc.threshold(input, output, 0, 255, Imgproc.THRESH_BINARY + Imgproc.THRESH_OTSU);
//      }
//
//      static std::list<MatOfPoint> findContours(Mat image) {
//              std::list<MatOfPoint> contours = new ArrayList<>();
//              Imgproc.findContours(image, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
//              return contours;
//      }
//
//      static std::list<MatOfPoint> filterContours(List<MatOfPoint> contours) {
//              std::list<MatOfPoint> newContours = new ArrayList<MatOfPoint>();
//              std::list<MatOfPoint> convexHulls = new ArrayList<MatOfPoint>();
//
//              for (int i = 0; i < contours.size(); i++) {
//                      MatOfInt convexHullMatOfInt = new MatOfInt();
//                      ArrayList convexHullPointArrayList = new ArrayList<Point>();
//                      MatOfPoint convexHullMatOfPoint = new MatOfPoint();
//                      // ArrayList convexHullMatOfPointArrayList = new
//                      // ArrayList<MatOfPoint>();
//
//                      Imgproc.convexHull(contours.get(i), convexHullMatOfInt);
//
//                      for (int j = 0; j < convexHullMatOfInt.toList().size(); j++) {
//                              convexHullPointArrayList.add(contours.get(i).toList().get(convexHullMatOfInt.toList().get(j)));
//                      }
//                      convexHullMatOfPoint.fromList(convexHullPointArrayList);
//                      // convexHullMatOfPointArrayList.add(convexHullMatOfPoint);
//
//                      double contourArea = Imgproc.contourArea(contours.get(i));
//                      double convexHullArea = Imgproc.contourArea(convexHullMatOfPoint);
//                      double contourToConvexHullRatio = contourArea / convexHullArea;
//
//                      Rect rect = Imgproc.boundingRect(contours.get(i));
//
//                      MatOfPoint2f points2f = approxPoly(convexHullMatOfPoint);
//
//                      Point[] bottomPts = findBottomPts(points2f.toArray(), rect);
//                      Point[] topPts = findTopPts(points2f.toArray(), rect);
//
//                      double topWidth = Math.abs(topPts[1].x - topPts[0].x);
//                      double bottomWidth = Math.abs(bottomPts[1].x - bottomPts[0].x);
//                      double leftHeight = Math.abs(bottomPts[0].y - topPts[0].y);
//                      double rightHeight = Math.abs(bottomPts[1].y - topPts[1].y);
//                      double widthPercentDiff = Math.abs(topWidth - bottomWidth) / ((topWidth + bottomWidth) / 2.0) * 100.0;
//                      double heightPercentDiff = Math.abs(leftHeight - rightHeight) / ((leftHeight + rightHeight) / 2.0) * 100.0;
//
//                      double topLen = Math.sqrt((topPts[1].x - topPts[0].x) * (topPts[1].x - topPts[0].x)
//                                      + (topPts[1].y - topPts[0].y) * (topPts[1].y - topPts[0].y));
//                      double bottomLen = Math.sqrt((bottomPts[1].x - bottomPts[0].x) * (bottomPts[1].x - bottomPts[0].x)
//                                      + (bottomPts[1].y - bottomPts[0].y) * (bottomPts[1].y - bottomPts[0].y));
//                      double leftLen = Math.sqrt((topPts[0].x - bottomPts[0].x) * (topPts[0].x - bottomPts[0].x)
//                                      + (topPts[0].y - bottomPts[0].y) * (topPts[0].y - bottomPts[0].y));
//                      double rightLen = Math.sqrt((topPts[1].x - bottomPts[1].x) * (topPts[1].x - bottomPts[1].x)
//                                      + (topPts[1].y - bottomPts[1].y) * (topPts[1].y - bottomPts[1].y));
//
//                      double equivalentAspectRatio = ((topLen + bottomLen) / 2.0) / ((leftLen + rightLen) / 2.0);
//
//                      if (contourToConvexHullRatio < 0.6 && rect.width > 40 && rect.height > 40 && widthPercentDiff < 10.0
//                                      && heightPercentDiff < 10.0 && equivalentAspectRatio > 1.17 && equivalentAspectRatio < 2.17
//                                      && points2f.toArray().length == 4) {
//                              // avgAspectRatio > 1.0 && avgAspectRatio < 4.0) {
//                              newContours.add(convexHullMatOfPoint);
//                              // newContours.add(contours.get(i));
//                      }
//
//                      convexHullMatOfInt = null;
//                      convexHullPointArrayList = null;
//                      convexHullMatOfPoint = null;
//              }
//              return newContours;
//      }
//
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
//      // find approximate point vertices of contoured goal tape
//      static MatOfPoint2f approxPoly(MatOfPoint contour) {
//              MatOfPoint2f point2f = new MatOfPoint2f();
//              std::list<Point> points = contour.toList();
//              point2f.fromList(points);
//              Imgproc.approxPolyDP(point2f, point2f, 5.0, true); // third parameter:
//                                                                                                                      // smaller->more
//                                                                                                                      // points
//              return point2f;
//      }
//
//      // finds bottom vertices of goal tape, left to right
//      static Point[] findBottomPts(Point[] points, Rect rect) {
//              Point rectBottomRight = new Point();
//              rectBottomRight = rect.br().clone();
//              Point rectBottomLeft = new Point(rect.br().x - (rect.width - 1), rect.br().y);
//              Point bottomRight = new Point();
//              Point bottomLeft = new Point();
//
//              double lowestDist = 0;
//              for (int i = 0; i < points.length; i++) {
//                      double dist = Math.sqrt((points[i].x - rectBottomLeft.x) * (points[i].x - rectBottomLeft.x)
//                                      + (points[i].y - rectBottomLeft.y) * (points[i].y - rectBottomLeft.y));
//
//                      if (i == 0) {
//                              bottomLeft = points[i];
//                              lowestDist = dist;
//                      } else if (dist < lowestDist) {
//                              bottomLeft = points[i];
//                              lowestDist = dist;
//                      }
//              }
//
//              for (int i = 0; i < points.length; i++) {
//                      double dist = Math.sqrt((points[i].x - rectBottomRight.x) * (points[i].x - rectBottomRight.x)
//                                      + (points[i].y - rectBottomRight.y) * (points[i].y - rectBottomRight.y));
//
//                      if (i == 0) {
//                              bottomRight = points[i];
//                              lowestDist = dist;
//                      } else if (dist < lowestDist) {
//                              bottomRight = points[i];
//                              lowestDist = dist;
//                      }
//              }
//
//              Point[] bottomPts = { bottomLeft, bottomRight };
//              return bottomPts;
//      }
//
//      // finds top vertices of goal tape, left to right
//      static Point[] findTopPts(Point[] points, Rect rect) {
//              Point rectTopRight = new Point(rect.tl().x + (rect.width - 1), rect.tl().y);
//              Point rectTopLeft = new Point();
//              rectTopLeft = rect.tl().clone();
//              Point topRight = new Point();
//              Point topLeft = new Point();
//
//              double lowestDist = 0;
//              for (int i = 0; i < points.length; i++) {
//                      double dist = Math.sqrt((points[i].x - rectTopLeft.x) * (points[i].x - rectTopLeft.x)
//                                      + (points[i].y - rectTopLeft.y) * (points[i].y - rectTopLeft.y));
//
//                      if (i == 0) {
//                              topLeft = points[i];
//                              lowestDist = dist;
//                      } else if (dist < lowestDist) {
//                              topLeft = points[i];
//                              lowestDist = dist;
//                      }
//              }
//
//              for (int i = 0; i < points.length; i++) {
//                      double dist = Math.sqrt((points[i].x - rectTopRight.x) * (points[i].x - rectTopRight.x)
//                                      + (points[i].y - rectTopRight.y) * (points[i].y - rectTopRight.y));
//
//                      if (i == 0) {
//                              topRight = points[i];
//                              lowestDist = dist;
//                      } else if (dist < lowestDist) {
//                              topRight = points[i];
//                              lowestDist = dist;
//                      }
//              }
//
//              Point[] topPts = { topLeft, topRight };
//              return topPts;
//      }
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
