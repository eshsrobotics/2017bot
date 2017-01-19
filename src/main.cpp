// MAIN.CPP
//
// Entry point for the program that the Nvidia Jetson TK1 will be excuting at
// boot time.

#include <iostream>
#include <opencv2/core/core.hpp>

int main() {
    cv::Mat matrix;
    std::cout << "OpenCV version " << CV_VERSION << " ready!\n";
}
