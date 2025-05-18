#ifndef XIMEA_ODOMETRY__LBP_HPP_
#define XIMEA_ODOMETRY__LBP_HPP_

#include <opencv2/opencv.hpp>
#include <vector>

// height: 512
// width: 640

class LBP {
 public:
  struct Grid {
    std::vector<std::vector<cv::Mat>> grid;
    int numRows, numCols, cellHeight, cellWidth;
    Grid();
    void process_image(cv::Mat image);
    cv::Mat& at_cell(int row, int col);
  };

  LBP(std::string& image_filepath);

  enum Channel { BLUE = 0, GREEN = 1, RED = 2 };

  Grid grid;
  cv::Mat image;
  cv::Mat lbp_image;
  std::vector<cv::Mat> color_channels;

  // Main LBP
  void compute_lbp(cv::Mat &lbp_image_, cv::Mat image_, int radius, int points);

  // Utility
  void divide_color_channels(cv::Mat& image_, std::vector<cv::Mat>& color_channels_);

  // Testing/Debugging
  cv::Mat read_png_to_mat(const std::string& filePath);
  void display_image(cv::Mat displayed_image);
  void visualize_points(cv::Mat image_, std::vector<std::pair<int, int>> coordinates);

 private:
  int height;
  int width;
};

#endif  // XIMEA_ODOMETRY__LBP_HPP_