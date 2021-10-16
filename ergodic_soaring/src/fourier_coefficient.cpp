#include "ergodic_soaring/fourier_coefficient.h"

FourierCoefficient::FourierCoefficient() {
  grid_map_ = grid_map::GridMap({"distribution", "reconstruction"});
  // Set Gridmap properties
  Settings settings;
  settings.center_lat = 0.0;
  settings.center_lon = 0.0;
  settings.resolution = 0.1;
  settings.delta_easting = 10.0;
  settings.delta_northing = 10.0;
  grid_map_.setFrameId("world");
  grid_map_.setGeometry(grid_map::Length(settings.delta_easting, settings.delta_northing), settings.resolution,
                        grid_map::Position(settings.center_lat, settings.center_lon));
  printf(
      "Created map with size %f x %f m (%i x %i cells).\n The center of the "
      "map is located at (%f, %f) in the %s frame.",
      grid_map_.getLength().x(), grid_map_.getLength().y(), grid_map_.getSize()(0), grid_map_.getSize()(1),
      grid_map_.getPosition().x(), grid_map_.getPosition().y(), grid_map_.getFrameId().c_str());

  // Initialize gridmap layers
  grid_map_["distribution"].setConstant(0);
  grid_map::Matrix &layer_elevation = grid_map_["distribution"];

  double sum{0.0};
  double cell_area = std::pow(grid_map_.getResolution(), 2);
  for (grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index gridMapIndex = *iterator;
    grid_map::Position cell_pos;
    grid_map_.getPosition(gridMapIndex, cell_pos);

    double sigma = 5.0;
    grid_map::Position mean = Eigen::Vector2d::Zero();
    Eigen::Matrix2d variance = sigma * Eigen::Matrix2d::Identity();
    Eigen::Vector2d error_pos = cell_pos - mean;
    double point_distribution = 1 / (std::sqrt(std::pow(2 * M_PI, 2) * variance.norm())) *
                                std::exp(-0.5 * error_pos.transpose() * variance * error_pos);
    sum += point_distribution;
    layer_elevation(gridMapIndex(0), gridMapIndex(1)) = point_distribution;
  }
  v_c_ = 1 / sum;
  double normailized_sum{0.0};

  for (grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index gridMapIndex = *iterator;
    layer_elevation(gridMapIndex(0), gridMapIndex(1)) = layer_elevation(gridMapIndex(0), gridMapIndex(1)) * v_c_;
    normailized_sum += layer_elevation(gridMapIndex(0), gridMapIndex(1));
  }
  std::cout << std::endl;
  std::cout << "Genearating sample distribution=====" << std::endl;
  std::cout << "  - Sum of distribution: " << sum << std::endl;
  std::cout << "  - normalized_sum: " << normailized_sum << std::endl;
  std::cout << "  - V_c: :" << v_c_ << std::endl;
}

void FourierCoefficient::FourierTransform(grid_map::GridMap &distribution_map) {
  double L_1 = distribution_map.getLength()[0];
  double L_2 = distribution_map.getLength()[1];
  double cell_area = std::pow(distribution_map.getResolution(), 2);
  std::cout << "Fourier Transform==================" << std::endl;
  std::cout << "  - Cell area: " << cell_area << std::endl;
  std::cout << "  - resolution: " << distribution_map.getResolution() << std::endl;
  std::cout << "  - L1: " << L_1 << std::endl;
  std::cout << "  - L2: " << L_2 << std::endl;

  coefficients_ = Eigen::ArrayXXd(K_, K_);
  normalization_ = Eigen::ArrayXXd(K_, K_);

  for (size_t i = 0; i < K_; i++) {
    for (size_t j = 0; j < K_; j++) {
      // Compute fourier normalization constants
      double h_k_sum{0.0};  // fourier coefficients
      for (grid_map::GridMapIterator iterator(distribution_map); !iterator.isPastEnd(); ++iterator) {
        const grid_map::Index gridMapIndex = *iterator;
        Eigen::Vector2d cell_pos;
        distribution_map.getPosition(gridMapIndex, cell_pos);
        h_k_sum =
            h_k_sum + std::pow(BasisFunction(i, L_1, cell_pos(0)), 2) * std::pow(BasisFunction(j, L_2, cell_pos(1)), 2);
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

void FourierCoefficient::FourierTransform(std::vector<Eigen::Vector2d> trajectory) {
  /// WIP: Fourier transformation of trajectory
  FourierCoefficients fourier;
  double dt = 0.1;
  double T = trajectory.size() * dt;
  double L_1 = grid_map_.getLength()[0];
  double L_2 = grid_map_.getLength()[1];
  double cell_area = std::pow(grid_map_.getResolution(), 2);

  coefficients_ = Eigen::ArrayXXd(K_, K_);
  normalization_ = Eigen::ArrayXXd(K_, K_);

  for (size_t i = 0; i < K_; i++) {
    for (size_t j = 0; j < K_; j++) {
      double h_k_sum{0.0};  // fourier coefficients
      for (auto pos : trajectory) {
        h_k_sum += std::pow(BasisFunction(i, L_1, pos(0)), 2) * std::pow(BasisFunction(j, L_2, pos(1)), 2);
      }
      double h_k = std::sqrt(h_k_sum);

      double phi_k{0.0};  // fourier coefficients
      for (auto pos : trajectory) {
        double F_k = (1 / h_k) * BasisFunction(i, L_1, pos(0)) * BasisFunction(j, L_2, pos(1));
        phi_k += F_k * dt / T;
      }
      normalization_(i, j) = h_k;
      coefficients_(i, j) = phi_k;
    }
  }
}

void FourierCoefficient::InverseFourierTransform(const std::string layer) {
  double L_1 = grid_map_.getLength()[0];
  double L_2 = grid_map_.getLength()[1];

  double cell_area = std::pow(grid_map_.getResolution(), 2);
  grid_map_.add(layer);

  Eigen::MatrixXf &layer_reconstruction = grid_map_[layer];
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
    sum += phi;
  }
  std::cout << "Reconstruction Fourier====================" << std::endl;
  std::cout << "  - Sum of distribution: " << sum << std::endl;
}
