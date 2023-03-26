#include "ceresSolver.hpp"
#include <cstddef>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <string>

BUFF::BuffCeresSolver::BuffCeresSolver() {

    is_params_confirmed = false;
    /**
     * @description: f(x) = a * sin(ω * t + θ) + b
     * @param params[0] -->a
     * @param params[1] -->ω
     * @param params[2] -->θ
     * @param params[3] -->b
     * @author: chen-zhan-smile
     */
    params[ 0 ] = 0;
    params[ 1 ] = 0;
    params[ 2 ] = 0;
    params[ 3 ] = 0;
    //读取滤波器yaml配置文件参数
    YAML::Node config = YAML::LoadFile( pf_path );
    //初始化滤波器类
    pf_param_loader.initParam( config, "buff" );
}

bool BUFF::BuffCeresSolver::setBulletSpeed( double speed ) {
    bullet_speed = speed;
    return true;
}

bool BUFF::BuffCeresSolver::CurveFitting( double speed, double dist,
                                          int timestamp, double &result ) {
    // auto t1 = std::chrono::steady_clock::now();
    TargetInfo target = { speed, dist, timestamp };
    //更新击打模式，切换大小符
    if ( this->last_mode != this->mode ) {
        this->last_mode = this->mode;
        history_info.clear();
        //重载滤波器
        pf.initParam( pf_param_loader );
        is_params_confirmed = false;
    }
    //当时间跨度过长视作目标已更新，需清空历史信息队列
    if ( history_info.empty() ||
         target.timestamp - history_info.front().timestamp >= max_timespan ) {
        history_info.clear();
        history_info.emplace_back( target );
        params[ 0 ] = 0;
        params[ 1 ] = 0;
        params[ 2 ] = 0;
        params[ 3 ] = 0;
        pf.initParam( pf_param_loader );
        last_target = target;

        is_params_confirmed = false;
        return false;
    }
    //滤波器的标志位
    auto is_ready = pf.is_ready;
    //传入speed的向量,是一个动态的向量
    Eigen::VectorXd measure( 1 );
    measure << speed;
    pf.update( measure );

    if ( is_ready ) {
        auto predict = pf.predict();
        //滤波出的速度
        target.speed = predict[ 0 ];
    }
    size_t deque_len = 0;
    //判断打符的模式，0为小符，1为大符
    if ( mode == 0 ) {
        deque_len = history_deque_len_uniform;
    } else if ( mode == 1 ) {
        if ( !is_params_confirmed )
            deque_len = history_deque_len_cos;
        else
            deque_len = history_deque_len_phase;
    }
    //判断长度，前面出后面进，实现历史数据的更新，契合deque的特性
    if ( history_info.size() < deque_len ) {
        history_info.push_back( target );
        last_target = target;
        return false;
    } else if ( history_info.size() == deque_len ) {
        history_info.pop_front();
        history_info.push_back( target );
    } else if ( history_info.size() > deque_len ) {
        while ( history_info.size() >= deque_len ) {
            history_info.pop_front();
        }
        history_info.push_back( target );
    }

    // 计算来判断旋转方向
    double rotate_speed_sum = 0; //
    int    rotate_sign;
    for ( auto target_info : history_info )
        rotate_speed_sum += target_info.speed;
    //速度和除以队列长度等于平均速度
    auto mean_velocity = rotate_speed_sum / history_info.size();
    //小符的速度恒定，不需要额外的计算
    if ( mode == 0 ) {
        params[ 3 ] = mean_velocity;
    } else if ( mode == 1 ) {
        bool flag =
            Mode_1_fitting( mean_velocity, rotate_speed_sum, rotate_sign );
        if ( !flag ) {
            return false;
        }
    }
    std::string coutstream = "";
    for ( auto param : params )
        coutstream + to_string( param ) + " ";
    RCLCPP_INFO( rclcpp::get_logger( "CeresSolver" ), "[params] : %s",
                 coutstream.c_str() );
    //设置发弹延迟
    int delay = ( mode == 1 ? delay_big : delay_small );
    //距离/弹丸速度+延迟发弹差
    float delta_time_estimate = ( (double) dist / bullet_speed ) * 1e3 + delay;
    //最后新的目标信息
    float timespan = history_info.back().timestamp;
    //时间戳+延迟时间=击打时刻目标时间点
    float time_estimate = delta_time_estimate + timespan;
    result = calcAimingAngleOffset( params, timespan / 1e3, time_estimate / 1e3,
                                    mode );
    last_target = target;
    return true;
}

bool BUFF::BuffCeresSolver::Mode_1_fitting( double &mean_velocity,
                                            double &rotate_speed_sum,
                                            int &   rotate_sign ) {
    //拟合函数: f(x) = a * sin(ω * t + θ) + b， 其中a， ω， θ需要拟合.
    //参数未确定时拟合a， ω， θ
    if ( !is_params_confirmed ) {
        ceres::Problem         problem;
        ceres::Solver::Options options;
        ceres::Solver::Summary summary; // 优化信息

        double params_fitting[ 4 ] = { 1, 1, 1, mean_velocity };

        //旋转方向，逆时针为正
        if ( rotate_speed_sum / fabs( rotate_speed_sum ) >= 0 )
            rotate_sign = 1;
        else
            rotate_sign = -1;

        for ( auto target_info : history_info ) {
            // clang-format off
            //向问题中添加误差项
            // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
            problem.AddResidualBlock( 
                    new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 4>(
                        new CURVE_FITTING_COST(
                            (float) ( target_info.timestamp ) / 1e3,
                            target_info.speed * rotate_sign ) 
                    ),
                    // CauchyLoss是Ceres Solver附带的损失函数之一。
                    // 参数0.5指定了损失函数的规模。
                    new ceres::CauchyLoss( 0.5 ),
                    params_fitting // 待估计参数
                );
            // clang-format on
        }
        /**
         * @description: f(x) = a * sin(ω * t + θ) + b
         * @param params[0] -->a
         * @param params[1] -->ω
         * @param params[2] -->θ
         * @param params[3] -->b
         * @description:
         * 这里要设置ceres-solver拟合的上下限，参数需根据场上大符实际调整
         * @author: chen-zhan-smile
         */
        problem.SetParameterLowerBound( params_fitting, 0, 0.7 );
        problem.SetParameterUpperBound( params_fitting, 0, 1.2 );
        problem.SetParameterLowerBound( params_fitting, 1, 1.6 );
        problem.SetParameterUpperBound( params_fitting, 1, 2.2 );
        problem.SetParameterLowerBound( params_fitting, 2, -CV_PI );
        problem.SetParameterUpperBound( params_fitting, 2, CV_PI );
        problem.SetParameterLowerBound( params_fitting, 3, 0.5 );
        problem.SetParameterUpperBound( params_fitting, 3, 2.5 );
        //传入参数到Solve
        ceres::Solve( options, &problem, &summary );
        // clang-format off
        //参数中间储存器
        double params_tmp[ 4 ] = { params_fitting[ 0 ] * rotate_sign,
                                   params_fitting[ 1 ], 
                                   params_fitting[ 2 ],
                                   params_fitting[ 3 ] * rotate_sign };
        // clang-format on
        //计算拟合的RMSE值，如果不小于设定的值，输出参数报告(可以注释)，继续拟合
        auto rmse = evalRMSE( params_tmp );
        if ( rmse > max_rmse ) {
            // cout << summary.BriefReport() << endl;
            RCLCPP_INFO( rclcpp::get_logger( "CeresSolver" ), "%s",
                         summary.BriefReport().c_str() );
            RCLCPP_WARN( rclcpp::get_logger( "CeresSolver" ),
                         "[BUFF_PREDICT]RMSE is too high, Fitting failed!" );
            return false;
        } else {
            //这里代表拟合成功，将 is_params_confirmed 置true
            params[ 0 ] = params_fitting[ 0 ] * rotate_sign;
            params[ 1 ] = params_fitting[ 1 ];
            params[ 2 ] = params_fitting[ 2 ];
            params[ 3 ] = params_fitting[ 3 ] * rotate_sign;

            is_params_confirmed = true;
        }
    } else {
        ceres::Problem         problem;
        ceres::Solver::Options options;
        ceres::Solver::Summary summary; // 优化信息

        double phase;

        for ( auto target_info : history_info ) {
            // clang-format off
            problem.AddResidualBlock( // 向问题中添加误差项
                // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
                new ceres::AutoDiffCostFunction<CURVE_FITTING_COST_PHASE, 1, 1>(
                    new CURVE_FITTING_COST_PHASE(
                        (float) ( target_info.timestamp ) / 1e3,
                        ( target_info.speed - params[ 3 ] ) * rotate_sign,
                        params[ 0 ], params[ 1 ], params[ 3 ] ) ),
                new ceres::CauchyLoss( 1e1 ),
                &phase // 待估计参数
            );
            // clang-format on
        }

        //设置上下限
        problem.SetParameterLowerBound( &phase, 0, -CV_PI );
        problem.SetParameterUpperBound( &phase, 0, CV_PI );
        ceres::Solve( options, &problem, &summary );
        // clang-format off
        double params_new[ 4 ] = { params[ 0 ], 
                                   params[ 1 ], 
                                   phase,
                                   params[ 3 ] };
        // clang-format on
        auto old_rmse = evalRMSE( params );
        auto new_rmse = evalRMSE( params_new );
        //如果新拟合的RMSE值比上一次的小，参数更新
        if ( new_rmse < old_rmse ) {
            // LOG( INFO ) << "[BUFF_PREDICT]Params Updated! RMSE: " <<;
            params[ 2 ] = phase;
        }
        RCLCPP_INFO( rclcpp::get_logger( "CereSolver" ), "RMSE: %lf",
                     new_rmse );
    }
    return true;
}

double BUFF::BuffCeresSolver::calcAimingAngleOffset( double params[ 4 ],
                                                     double t0, double t1,
                                                     int mode ) {
    /**
     * @description: f(x) = a * sin(ω * t + θ) + b
     * @param params[0] -->a
     * @param params[1] -->ω
     * @param params[2] -->θ
     * @param params[3] -->b
     * @return {*}
     * @author: chen-zhan-smile
     */
    auto   a     = params[ 0 ];
    auto   omega = params[ 1 ];
    auto   theta = params[ 2 ];
    auto   b     = params[ 3 ];
    double theta1;
    double theta0;
    //对目标函数进行积分
    // clang-format off
    /**
     * @description: 小能量机关的转速固定为 10RPM。
                     大能量机关转速按照三角函数呈周期性变化。
                     速度目标函数为：spd= a ∗ sin(𝜔 ∗ 𝑡) +𝑏，
                     其中 spd 的单位 为 rad/s，t 的单位为 s，
                     a 的取值范围为0.780~1.045，
                     ω的取值范围为 1.884~2.000，
                     b 始终满足 b=2.090-a。
                     每次大能量机关进入可激活状态时，所有参数重置，
                     其中 t重置为 0，a 和ω重置为取值范围内任意值。
     * @return {*}
     * @author: chen-zhan-smile
     */
    // clang-format on
    if ( mode == 0 ) //适用于小符模式
    {
        theta0 = b * t0;
        theta1 = b * t1;
    } else {
        // clang-format off

        //  ∫ a∗sin(𝜔∗𝑡)+𝑏 dt = b*t-(a/𝜔)*cos(𝜔*t+θ)
        theta0 = ( b * t0 - ( a / omega ) * cos( omega * t0 + theta ) );
        theta1 = ( b * t1 - ( a / omega ) * cos( omega * t1 + theta ) );
        // clang-format on
    }
    return theta1 - theta0;
}

/**
 * @brief 计算RMSE指标
 * @brief RMSE均方根误差（Root Mean SquareError）
          当预测值与真实值完全吻合时等于0，即完美模型；误差越大，该值越大。
 * @param params 参数首地址指针
 * @return RMSE值
 */
double BUFF::BuffCeresSolver::evalRMSE( double params[ 4 ] ) {
    double rmse_sum = 0;
    double rmse     = 0;
    for ( auto target_info : history_info ) {
        auto t = (float) ( target_info.timestamp ) / 1e3;
        auto pred =
            params[ 0 ] * sin( params[ 1 ] * t + params[ 2 ] ) + params[ 3 ];
        auto measure = target_info.speed;
        rmse_sum += pow( ( pred - measure ), 2 );
    }
    rmse = sqrt( rmse_sum / history_info.size() );
    return rmse;
}

/**
 * @brief 计算MAPE指标,平均绝对百分比误差（Mean Absolute Percentage Error）
 *
 * @param params 参数首地址指针
 * @return MAPE值
 */
double BUFF::BuffCeresSolver::evalMAPE( double params[ 4 ] ) {
    double mape_sum = 0;
    double mape     = 0;
    for ( auto target_info : history_info ) {
        auto t = (float) ( target_info.timestamp ) / 1e3;
        auto pred =
            params[ 0 ] * sin( params[ 1 ] * t + params[ 2 ] ) + params[ 3 ];
        auto measure = target_info.speed;

        mape_sum += abs( ( measure - pred ) / measure );
    }
    return mape;
}
