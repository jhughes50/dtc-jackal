/*!
 * Jason Hughes
 * August 2024
 *
 * Matrix logic
 *
 * DTC PRONTO 2024
 */

#include "amm/matrix.hpp"

MCTDMatrix::MCTDMatrix(cv::Mat starting_img, int num_channels)
{
    rows = starting_img.rows;
    cols = starting_img.cols;
    channels = num_channels;

    cv::split(starting_img, channels_);
    
    for (int i=0; i< channels-3; ++i)
    {
        channels_.push_back(cv::Mat(rows, cols, CV_32F));
    }
}	

template <typename T>
void MCTDMatrix::set(T val, int r, int c, int ch)
{
    channels_[ch].at<T>(r,c) = val;
}

template <typename T>
T MCTDMatrix::get(int r, int c, int ch)
{
    return channels_[ch].at<T>(r,c);
}

template void MCTDMatrix::set<float>(float, int, int, int);
template float MCTDMatrix::get<float>(int, int, int);

//template void MCTDMatrix::set<uint8_t>(float, int, int, int);
template uint8_t MCTDMatrix::get<uint8_t>(int, int, int);
