/*!
* Jason Hughes
* August 2024
*
* Wrapper around cv::Mat to handle more than 3 channels
* of different types, dubbed Multi-Channel Different-Type Matrix.

* DTC PRONTO 2024
*/

#ifndef MATRIX_HPP
#define MATRIX_HPP

#include <vector>
#include <opencv2/opencv.hpp>

class MCTDMatrix
{
    public:
        MCTDMatrix() = default;
        MCTDMatrix(cv::Mat starting_img, int num_channels = 5);

        template <typename T>
        void set(T val, int r, int c, int ch);

        template <typename T>
        T get(int row, int col, int channel);

        int rows;
        int cols;
        int channels;

    private:
        cv::Mat original_img_;
        std::vector<cv::Mat> channels_;
};
#endif
