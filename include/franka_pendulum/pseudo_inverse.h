#pragma once

#include <Eigen/Dense>

namespace franka_pendulum
{
    ///Computes Moore-Penrose inverse of a matrix
    ///@param matrix Matrix to invert
    ///@param damped `true` for non-zero damping term, `false` for zero
    ///@return Moore-Penrose inverse of the matrix
    template <int H, int W> Eigen::Matrix<double, W, H> pseudo_inverse(const Eigen::Matrix<double, H, W> &matrix, const Eigen::Matrix<double, W, 1> &weights, double mu)
    {
        Eigen::DiagonalMatrix<double, W> weights_inverse(weights.array().inverse().matrix());
        return weights_inverse * matrix.transpose() * (matrix * weights_inverse * matrix.transpose() + mu * mu * Eigen::Matrix<double, H, H>::Identity()).inverse();
    }
}