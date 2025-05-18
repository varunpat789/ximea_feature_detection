#include "ximea_odometry/lbp.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <utility>
#include <vector>

// Multi-scale rotation-invariant LBP

LBP::LBP(std::string& image_filepath) {
  image = read_png_to_mat(image_filepath);
  width = image.cols;
  height = image.rows;
  grid = Grid();
  grid.process_image(image);
}

// Main LBP
void compute_lbp(cv::Mat& lbp_image_, const cv::Mat& image_, int radius) {
  // GOAL: Scalability for potential MLPB implementation.
  // FOR TESTING LBP IDEA THIS ONLY WORKS FOR RADIUS = 1 (surrounding pixels)
  // NOTE: perhaps have lookup table for scale 1.0x, 1.5x, 2.0x etc. for speed 
  // instead of calc points and having to perform some kind of interpolation
  for (int y = radius; y < image_.rows - radius; ++y) {
    for (int x = radius; x < image_.cols - radius; ++x) {
      uchar center = image_.at<uchar>(y, x);
      unsigned char lbp_code = 0;

      // clockwise
      if (image_.at<uchar>(y - 1, x - 1) >= center) lbp_code |= (1 << 0);  // Top-left
      if (image_.at<uchar>(y - 1, x) >= center) lbp_code |= (1 << 1);      // Top
      if (image_.at<uchar>(y - 1, x + 1) >= center) lbp_code |= (1 << 2);  // Top-right
      if (image_.at<uchar>(y, x + 1) >= center) lbp_code |= (1 << 3);      // Right
      if (image_.at<uchar>(y + 1, x + 1) >= center) lbp_code |= (1 << 4);  // Bottom-right
      if (image_.at<uchar>(y + 1, x) >= center) lbp_code |= (1 << 5);      // Bottom
      if (image_.at<uchar>(y + 1, x - 1) >= center) lbp_code |= (1 << 6);  // Bottom-left
      if (image_.at<uchar>(y, x - 1) >= center) lbp_code |= (1 << 7);      // Left

      lbp_image_.at<uchar>(y, x) = lbp_code;
    }
  }
}

// Utility
void divide_color_channels(cv::Mat& image_, std::vector<cv::Mat>& color_channels_) {
  cv::split(image_, color_channels_);
}

// Testing/Debugging
cv::Mat LBP::read_png_to_mat(const std::string& image_filepath) {
  image = cv::imread(image_filepath, cv::IMREAD_COLOR);

  if (image.empty()) {
    std::cerr << "ERROR: couldn't read image from " << image_filepath << std::endl;
    return cv::Mat();
  }

  std::cout << "loaded image: " << image_filepath << " (" << image.cols << "x" << image.rows << ")"
            << std::endl;
  return image;
}

void LBP::display_image(cv::Mat displayed_image) {
  if (!displayed_image.empty()) {
    cv::namedWindow("Current Image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Current Image", displayed_image);
    cv::waitKey(0);
    cv::destroyWindow("Current Image");
  }
}

void LBP::visualize_points(cv::Mat image_, std::vector<std::pair<int, int>> coordinates) {
  // Convert to BGR if greyscale
  if (image_.channels() == 1) {
    cv::cvtColor(image_, image_, cv::COLOR_GRAY2BGR);
  }

  cv::Vec3b green(0, 255, 0);

  for (int i = 0; i < coordinates.size(); ++i) {
    int row = coordinates[i].first;
    int col = coordinates[i].second;

    if (row >= 0 && row < image_.rows && col >= 0 && col < image_.cols) {
      image_.at<cv::Vec3b>(row, col) = green;
    }
  }

  // Display the image
  display_image(image_);
}

// Grid Structure
LBP::Grid::Grid() : numRows(6), numCols(16) {}

void LBP::Grid::process_image(cv::Mat image_) {
  cellHeight = image_.rows / numRows;
  cellWidth = image_.cols / numCols;
  grid.resize(numRows, std::vector<cv::Mat>(numCols));

  for (int i = 0; i < numRows; ++i) {
    for (int j = 0; j < numCols; ++j) {
      int x = j * cellWidth;
      int y = i * cellHeight;

      grid[i][j] = image_(cv::Rect(x, y, cellWidth, cellHeight)).clone();
    }
  }
}

cv::Mat& LBP::Grid::at_cell(int row, int col) { return grid.at(row).at(col); }
