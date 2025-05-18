#include <iostream>
#include <opencv2/opencv.hpp>
#include "ximea_odometry/lbp.hpp"

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>
#include <string>

using namespace std;
using namespace cv;

int main() {
    string file_path = "/home/biorobotics/Documents/varun/boeing/ximea_ws/src/ximea_feature_detection/frame10.png";
    LBP lbp(file_path);
    lbp.compute_lbp();
    lbp.display_image(lbp.lbp_image);

  return 0;
}