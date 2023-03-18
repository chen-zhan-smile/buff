#ifndef __FILTER__
#define __FILTER__

#pragma once
#include <iostream>

#include <ctime>
#include <random>
#include <vector>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

using namespace std;
using namespace Eigen;

template <typename T>
bool initMatrix( Eigen::MatrixXd &matrix, std::vector<T> &vector ) {
    int cnt = 0;
    for ( int row = 0; row < matrix.rows(); row++ ) {
        for ( int col = 0; col < matrix.cols(); col++ ) {
            matrix( row, col ) = vector[ cnt ];
            cnt++;
        }
    }
    return true;
}

const string pf_path =
    "../params/filter/filter_param.yaml"; //读取yaml参数文件路径

class ParticleFilter {
  public:
    ParticleFilter( YAML::Node &config, const string param_name );
    ParticleFilter();
    ~ParticleFilter();

    Eigen::VectorXd predict();

    bool initParam( YAML::Node &config, const string param_name );
    bool initParam( ParticleFilter parent );
    bool update( Eigen::VectorXd measure );
    bool is_ready;

  private:
    bool resample();

    int vector_len;
    int num_particle;

    Eigen::MatrixXd process_noise_cov; //过程噪声协方差
    Eigen::MatrixXd observe_noise_cov; //观测噪声协方差
    Eigen::MatrixXd weights;           //权重

    Eigen::MatrixXd matrix_estimate;
    Eigen::MatrixXd matrix_particle;
    Eigen::MatrixXd matrix_weights;
};

#endif