#include "ergodic_soaring/fourier_coefficient.h"

FourierCoefficient::FourierCoefficient(int num_coefficients) : K_(num_coefficients) {
  coefficients_ = Eigen::ArrayXXd(K_, K_);
  normalization_ = Eigen::ArrayXXd(K_, K_);
}

void FourierCoefficient::FourierTransform(grid_map::GridMap &distribution_map) {
  grid_map_ = distribution_map;
  double L_1 = distribution_map.getLength()[0];
  double L_2 = distribution_map.getLength()[1];
  double cell_area = std::pow(distribution_map.getResolution(), 2);

  for (size_t i = 0; i < K_; i++) {
    for (size_t j = 0; j < K_; j++) {
      // Compute fourier normalization constants
      double h_k_sum{0.0};  // fourier coefficients
      for (grid_map::GridMapIterator iterator(distribution_map); !iterator.isPastEnd(); ++iterator) {
        const grid_map::Index gridMapIndex = *iterator;
        Eigen::Vector2d cell_pos;
        distribution_map.getPosition(gridMapIndex, cell_pos);
        h_k_sum += std::pow(BasisFunction(i, L_1, cell_pos(0)), 2) * std::pow(BasisFunction(j, L_2, cell_pos(1)), 2);
      }
      double h_k = std::sqrt(h_k_sum);
      normalization_(i, j) = h_k;

      // Compute fourier coefficients
      double phi_k{0.0};  // fourier coefficients
      for (grid_map::GridMapIterator iterator(distribution_map); !iterator.isPastEnd(); ++iterator) {
        const grid_map::Index gridMapIndex = *iterator;
        Eigen::Vector3d cell_pos;
        distribution_map.getPosition3("distribution", gridMapIndex, cell_pos);
        double F_k = (1 / h_k) * BasisFunction(i, L_1, cell_pos(0)) * BasisFunction(j, L_2, cell_pos(1));
        phi_k += cell_pos(2) * F_k;
      }
      coefficients_(i, j) = phi_k;
    }
  }
}

void FourierCoefficient::FourierTransform(const std::vector<State> &trajectory) {
  double N = trajectory.size();
  double L_1 = grid_map_.getLength()[0];
  double L_2 = grid_map_.getLength()[1];
  double cell_area = std::pow(grid_map_.getResolution(), 2);

  coefficients_ = Eigen::ArrayXXd(K_, K_);
  normalization_ = Eigen::ArrayXXd(K_, K_);

  for (size_t i = 0; i < K_; i++) {
    for (size_t j = 0; j < K_; j++) {
      double h_k_sum{0.0};  // fourier coefficients
      for (auto state : trajectory) {
        Eigen::Vector3d pos = state.position;
        h_k_sum += std::pow(BasisFunction(i, L_1, pos(0)), 2) * std::pow(BasisFunction(j, L_2, pos(1)), 2);
      }
      double h_k = std::sqrt(h_k_sum);
      normalization_(i, j) = h_k;

      double phi_k{0.0};  // fourier coefficients
      for (auto state : trajectory) {
        Eigen::Vector3d pos = state.position;
        double F_k = (1 / h_k) * BasisFunction(i, L_1, pos(0)) * BasisFunction(j, L_2, pos(1));
        phi_k += F_k;
      }
      // Since we are reconstructing the distribution with a discretized cell, we need to normalize the fourier
      // coefficients accordingly
      coefficients_(i, j) = phi_k * cell_area / (L_1 * L_2);
    }
  }
}

void FourierCoefficient::InverseFourierTransform(const std::string layer) {
  double L_1 = grid_map_.getLength()[0];
  double L_2 = grid_map_.getLength()[1];

  grid_map_.add(layer);
  grid_map_.add(layer + "_visualization");

  Eigen::MatrixXf &layer_reconstruction = grid_map_[layer];
  Eigen::MatrixXf &layer_visual = grid_map_[layer + "_visualization"];
  double sum{0.0};
  for (grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index gridMapIndex = *iterator;
    Eigen::Vector2d cell_pos;
    grid_map_.getPosition(gridMapIndex, cell_pos);

    double phi{0.0};
    for (size_t i = 0; i < K_; i++) {
      for (size_t j = 0; j < K_; j++) {
        double h_k = normalization_(i, j);
        phi +=
            coefficients_(i, j) * (1 / h_k) * BasisFunction(i, L_1, cell_pos(0)) * BasisFunction(j, L_2, cell_pos(1));
      }
    }
    layer_reconstruction(gridMapIndex(0), gridMapIndex(1)) = phi;
    layer_visual(gridMapIndex(0), gridMapIndex(1)) = 100.0 * phi;
    sum += phi;
  }
  // std::cout << "Reconstruction Fourier====================" << std::endl;
  // std::cout << "  - Sum of distribution: " << sum << std::endl;
}

double FourierCoefficient::getErgodicity(Eigen::ArrayXXd trajectory_coefficients) {
  double ergodicity{0.0};
  // std::cout << "Start sum: " << std::endl;
  for (int i = 0; i < K_; i++) {
    for (int j = 0; j < K_; j++) {
      double sigma_k = 1 / std::pow(1 + std::pow(Eigen::Vector2d(i, j).norm(), 2), 1.5);
      ergodicity += sigma_k * std::pow(trajectory_coefficients(i, j) - coefficients_(i, j), 2);
    }
  }
  return ergodicity;
}

Eigen::Matrix<double, 3, 1> FourierCoefficient::getErgodicGradient(const int N, const State &state,
                                                                   Eigen::ArrayXXd trajectory_coefficients) {
  double L_1 = grid_map_.getLength()[0];
  double L_2 = grid_map_.getLength()[1];

  Eigen::Vector2d pos(state.position(0), state.position(1));
  Eigen::Matrix<double, 3, 1> gradient = Eigen::Matrix<double, 3, 1>::Zero();
  // std::cout << "Start sum: " << std::endl;
  for (int i = 0; i < K_; i++) {
    for (int j = 0; j < K_; j++) {
      double sigma_k = 1 / std::pow(1 + std::pow(Eigen::Vector2d(i, j).norm(), 2), 1.5);
      Eigen::Matrix<double, 3, 1> delta_F_k = GradientBasisFunction(i, L_1, pos(0), j, L_2, pos(1));
      gradient += (2 / N + 1) * sigma_k * (trajectory_coefficients(i, j) - coefficients_(i, j)) * delta_F_k;
    }
  }
  return gradient;
}
