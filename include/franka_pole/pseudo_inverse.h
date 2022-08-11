#pragma once

#include <Eigen/Dense>

namespace franka_pole
{
    ///Computes Moore-Penrose inverse of a matrix
    ///@param matrix Matrix to invert
    ///@param damped `true` for non-zero damping term, `false` for zero
    ///@return Moore-Penrose inverse of the matrix
    template <int H, int W> Eigen::Matrix<double, W, H> pseudo_inverse(const Eigen::Matrix<double, H, W> &matrix, bool damped)
    {
        double lambda = damped ? 0.2 : 0.0;
        Eigen::JacobiSVD<Eigen::Matrix<double, H, W>> svd(matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
        typename Eigen::JacobiSVD<Eigen::Matrix<double, W, H>>::SingularValuesType singular_values = svd.singularValues();
        Eigen::Matrix<double, H, W> s = Eigen::Matrix<double, H, W>::Zero();
        for (unsigned int i = 0; i < singular_values.size(); i++) s(i, i) = (singular_values(i)) / (singular_values(i) * singular_values(i) + lambda * lambda);
        return svd.matrixV() * s.transpose() * svd.matrixU().transpose();
    }
}