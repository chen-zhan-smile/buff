
/*
 * @Description:
 * @Version: 2.0
 * @Autor: chen-zhan-smile
 * @Date: 2023-03-07 22:51:17
 * @LastEditors: chen-zhan-smile
 * @LastEditTime: 2023-03-18 15:14:34
 */

#ifndef __Ceres_Solver__
#define __Ceres_Solver__

#include "filter.hpp"
#include <Eigen/Core>
#include <ceres/ceres.h>
#include <chrono>
#include <cstddef>
#include <ctime>
#include <deque>
#include <future>
#include <iostream>
#include <vector>

namespace BUFF {

enum class Mode { Small = 0, BIG };

class BuffCeresSolver {
  private:
    struct CURVE_FITTING_COST {
        CURVE_FITTING_COST( double x, double y )
            : _x( x )
            , _y( y ) {}
        // 残差的计算
        template <typename T>
        bool operator()( const T *params,    // 模型参数，有3维
                         T *      residual ) const // 残差
        {
            // f(x) = a * sin(ω * t + θ) + b

            // clang-format off
            residual[ 0 ] = T( _y ) -params[ 0 ] * ceres::sin( params[ 1 ] * T( _x ) +params[ 2 ] ) -params[ 3 ]; 
            // clang-format off
            return true;
        }
        const double _x, _y; // x,y数据
    };
    
    struct CURVE_FITTING_COST_PHASE {
        CURVE_FITTING_COST_PHASE( double x, double y, double a, double omega,
                                  double dc )
            : _x( x )
            , _y( y )
            , _a( a )
            , _omega( omega )
            , _dc( dc ) {}
        // 残差的计算
        template <typename T>
        bool operator()( const T *phase,     // 模型参数，有1维
                         T *      residual ) const // 残差
        {
            // f(x) = a * sin(ω * t + θ)

            // clang-format off
            residual[ 0 ] = T( _y ) -T( _a ) * ceres::sin( T( _omega ) * T( _x ) + phase[ 0 ] ) -T( _dc );
            return true;
            // clang-format on
        }
        const double _x, _y, _a, _omega, _dc; // x,y数据
    };
    struct TargetInfo {
        double speed;     //击打点运动速度
        double dist;      //解算出的距离
        int    timestamp; //记录目标时的时间戳
    };

  private:
    // clang-format off
    double params[ 4 ]; //存放ceres-solver拟合参数
    /**
     * @description: f(x) = a * sin(ω * t + θ) + b
     * @param params[0] -->a
     * @param params[1] -->ω
     * @param params[2] -->θ
     * @param params[3] -->b
     * @return {*}
     * @author: chen-zhan-smile
     */    
    double bullet_speed = 28;
    std::deque<TargetInfo> history_info; //目标队列，注意std::deque只允许前后插入，中间插入会出错
    const int max_timespan = 20000; //最大时间跨度，大于该时间重置预测器(ms)
    const double max_rmse = 0.4; // TODO:回归函数最大Cost
    const int max_v = 3;   //设置最大速度,单位rad/s
    const int max_a = 8;   //设置最大角加速度,单位rad/s^2
    const int history_deque_len_cos = 250; //大符全部参数拟合队列长度
    const int history_deque_len_phase = 100; //大符相位参数拟合队列长度
    const int history_deque_len_uniform = 100; //小符转速求解队列长度
    const int delay_small = 175; //小符发弹延迟
    const int delay_big = 100; //大符发弹延迟
    // clang-format on
  public:
    int mode;
    int last_mode; //上一次的模式

    TargetInfo     last_target; //最后的击打目标
    ParticleFilter pf;          //滤波器
    ParticleFilter pf_param_loader;

    bool is_params_confirmed; //参数确认

    explicit BuffCeresSolver( const enum Mode mode );
    bool   CurveFitting( double speed, double dist, int timestamp,
                         double &result );
    bool   Mode_1_fitting( double &mean_velocity, double &rotate_speed_sum,
                           int &rotate_sign );
    double calcAimingAngleOffset( double params[ 4 ], double t0, double t1,
                                  int mode );
    bool   setBulletSpeed( double speed ); //更新弹丸得速度，在计算击打得时候有用
    double evalRMSE( double params[ 4 ] );
    double evalMAPE( double params[ 4 ] );
};
} // namespace BUFF
#endif