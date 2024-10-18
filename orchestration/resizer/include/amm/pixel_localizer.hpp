/*!
* Jason Hughes
* August 2024
*
* Build a 2D grid map for map visualization
*
* DTC PRONTO 2024
*/

#ifndef PIXEL_LOCALIZER_HPP
#define PIXEL_LOCALIZER_HPP

#include <tbb/blocked_range2d.h>
#include <tbb/parallel_reduce.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include "matrix.hpp"

struct PixelLocalizer
{
    MCTDMatrix& mat;
    Eigen::Matrix3d K_inv;
    Eigen::Matrix3d R_t;
    Eigen::Vector3d t;
    double alt;
    double res;

    PixelLocalizer(MCTDMatrix& m, Eigen::Matrix3d k, Eigen::Matrix3d r, Eigen::Vector3d t, double a, double res);
    PixelLocalizer(PixelLocalizer& pl, tbb::split);

    double getDepth(int i, int j, int rows, int cols);

    void operator()(const tbb::blocked_range2d<int>& range);
    void join(const PixelLocalizer& other);
};
#endif
