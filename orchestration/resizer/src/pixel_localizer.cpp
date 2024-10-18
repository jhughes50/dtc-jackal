/*!
* Jason Hughes
* August 2024
*
* Aerial Mapping Module
* This class uses TBB to parallelize the 
* building of the grid map, but using camera extrinsics 
* and extrinsics to calculate the x, y position of each 
* pixel in the local frame.
*
* DTC PRONTO 2024
*/

#include "amm/pixel_localizer.hpp"

PixelLocalizer::PixelLocalizer(MCTDMatrix& m, Eigen::Matrix3d k, Eigen::Matrix3d r, Eigen::Vector3d t, double a, double res) : mat(m), K_inv(k), R_t(r), t(t), alt(a), res(res) { }

PixelLocalizer::PixelLocalizer(PixelLocalizer& pl, tbb::split) : mat(pl.mat), K_inv(pl.K_inv), R_t(pl.R_t), t(pl.t), alt(pl.alt), res(pl.res)  { }

void PixelLocalizer::operator()(const tbb::blocked_range2d<int>& r)
{
    for (int i = r.rows().begin(); i != r.rows().end(); ++i)
    {
        for (int j = r.cols().begin(); j != r.cols().end(); ++j)
        {
	    Eigen::Vector3d norm = K_inv * Eigen::Vector3d(i, j, 1.0);
	    Eigen::Vector3d ray_cam = norm.normalized();

	    Eigen::Vector3d ray_world = R_t * ray_cam;
	    Eigen::Vector3d cam_center_world = -R_t * t;

	    double depth = getDepth(i, j, mat.rows, mat.cols);

	    double lambda = depth / ray_cam.z();

	    Eigen::Vector3d world = cam_center_world + lambda * ray_world;

            mat.set<float>(world(0), i, j, 3);
            mat.set<float>(world(1), i, j, 4);
        }
    }
}

double PixelLocalizer::getDepth(int i, int j, int rows, int cols)
{
    // convert i, j to center coordinates
    int mr = rows / 2;
    int mc = cols / 2;

    int u = std::abs(mr - i);
    int v = std::abs(mc - j);

    double row_w = u *(double) res;
    double col_w = v *(double) res;

    double ground_hypotenuse = std::sqrt(std::pow(row_w, 2) + std::pow(col_w, 2));

    double depth = std::sqrt(std::pow(ground_hypotenuse,2) + std::pow(alt,2));

    return depth;
}

void PixelLocalizer::join(const PixelLocalizer& other) { }
