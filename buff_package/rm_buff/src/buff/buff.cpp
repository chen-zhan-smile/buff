#include "buff.hpp"
#include <image_transport/publisher.hpp>
#include <memory>
#include <opencv2/core/types.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>

using namespace cv;
using namespace std;

BUFF::Buff::Buff()
    : Node( "rm_buff" ) {
    //初始化图像接收客户端
    image_sub = std::make_shared<image_transport::Subscriber>(
        image_transport::create_subscription(
            this, "camera/image",
            std::bind( &Buff::image_callback, this, std::placeholders::_1 ),
            transport_, rmw_qos_profile_sensor_data ) );
    // clang-format off
    //初始化图像发送器
    binary_img_pub_  = image_transport::create_publisher( this, "/binary_img" );
    predict_img_pub_ = image_transport::create_publisher( this, "/predict_img" );
    predict_pub_ = this->create_publisher<msg_predict_pub>("buff_predict",10);
    serial_sub_ =this->create_subscription<msg_serial_sub>(
        "topic", 10, std::bind(&BUFF::Buff::serial_topic_callbak, this, std::placeholders::_1)
    );
    //初始化yolo-face网络模型或者detector类
    
    // clang-format off
    // detector.initModel( network_path );
    /**
     * @description: 这里要添加识别程序，坐标转换可能得单独写一个节点，service
     * @return {*}
     * @author: chen-zhan-smile
     */
    lost_cnt              = 0;
    is_last_target_exists = false;
    input_size            = { 640, 640 };
    last_bullet_speed     = 0;
    RCLCPP_INFO( this->get_logger(), "[BUFF] Buff init model success!" );
    //实例化预测器
    predictor_ = std::make_shared<BUFF::BuffCeresSolver>();
    //实例化坐标变换器
    coordSolver_ = std::make_unique<BUFF::CoordSolver>();
    /**
     * @param :camera_param_path , camera_name
     * @description: 相机参数路径 ，对应机器人参数
     * @return {*}
     * @author: chen-zhan-smile
     */
    coordSolver_->loadParam( camera_param_path, camera_name );
    RCLCPP_INFO( this->get_logger(), "[BUFF] predictor_ init success!" );
}

/**
 * @brief 根据上次装甲板位置截取ROI
 *
 * @param img 所需处理的图像
 * @return ** Point2i ROI中心点
 */

cv::Point2i BUFF::Buff::cropImageByROI( Mat &img ) {
    if ( !is_last_target_exists ) {
        //当丢失目标帧数过多或lost_cnt为初值
        if ( lost_cnt > max_lost_cnt || lost_cnt == 0 ) {
            return Point2i( 0, 0 );
        }
    }
    //若目标大小大于阈值
    // cout<<last_target_area / img.size().area()<<endl;
    if ( ( last_target_area / img.size().area() ) > no_crop_thres ) {
        return Point2i( 0, 0 );
    }
    //处理X越界
    if ( last_roi_center.x <= input_size.width / 2 )
        last_roi_center.x = input_size.width / 2;
    else if ( last_roi_center.x > ( img.size().width - input_size.width / 2 ) )
        last_roi_center.x = img.size().width - input_size.width / 2;
    //处理Y越界
    if ( last_roi_center.y <= input_size.height / 2 )
        last_roi_center.y = input_size.height / 2;
    else if ( last_roi_center.y >
              ( img.size().height - input_size.height / 2 ) )
        last_roi_center.y = img.size().height - input_size.height / 2;

    //左上角顶点
    auto offset = last_roi_center -
                  Point2i( input_size.width / 2, input_size.height / 2 );
    Rect roi_rect = Rect( offset, input_size );
    img( roi_rect ).copyTo( img );

    return offset;
}

/**
 * @description:
 * @brief 选择击打目标
 * @param {Fan} &target
 * @return {*}
 * @author: chen-zhan-smile
 */
bool BUFF::Buff::chooseFromTargets( vector<Fan> &fans, Fan &target ) {
    int target_fan_cnt = 0;
    for ( const auto &fan : fans ) {
        if ( fan.id == 1 ) {
            target = fan;
            target_fan_cnt++;
        }
    }
    return target_fan_cnt == 1;
}

/**
 * @description: 串口接收的回调，接收到串口主题的更新后，对visionData进行更新
 * @param {msg_serial_sub} &msg_serial
 * @return {*}
 * @author: chen-zhan-smile
 */
void BUFF::Buff::serial_topic_callbak(const msg_serial_sub &msg_serial){
    /**
     * @description: 这里是串口回调函数，当订阅的串口主题发布新数据时，产生回调
     * @description: 在回调中对 this->visionData_ 数据进行更新
     * @param :  this->visionData_.isSwitched=msg_serial.data;
     * @return {*}
     * @author: chen-zhan-smile
     */     
}

void BUFF::Buff::image_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr &msg ) {
    auto cv_ptr =
        cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8 );
    //ros2的时间戳管理,和C++中的std::chrono::system_clock是一样的，即系统时间。
    rclcpp::Time timestemp_ = msg->header.stamp;
    double time_temp=timestemp_.seconds();
    //向任务数据中传参数
    /**
     * @description: 订阅相机图像发布节点，当图像更新时，产生回调
     * @description: 在图像回调节点调用主程序，这里相当于main函数入口
     * @return {*}
     * @author: chen-zhan-smile
     */    
    taskData_.img = cv_ptr->image; // ros节点图片转换成cv::Mat形式
    taskData_.bullet_speed = 0;
    taskData_.mode         = 0;
    taskData_.timestamp    = time_temp;
    // taskData->quat = Eigen::Quaterniond ;

    //-------------------------------------------------------------//

    //电控从串口传来的数据
    VisionData visionData = this->visionData_;
    //调用能量机关主函数
    this->run( taskData_, visionData );
}

/**
 * @brief run 自瞄主函数
 * @param src 图像、时间戳、IMU数据
 * @param data 发送数据所用结构体
 * @return true 识别成功
 * @return false 识别失败
 */
bool BUFF::Buff::run( TaskData &src, VisionData &data ) {

    auto time_start = std::chrono::steady_clock::now();
    /*
    struct BuffObject {
        Point2f                  apex[ 5 ];//能量机关五点
        cv::Rect_<float>         rect;  //矩形框
        int                      cls; //class分类 0是已激活，1是未激活
        int                      color; //红蓝色
        float                    prob; //置信度
        std::vector<cv::Point2f> pts;
    };*/
    vector<BuffObject> objects;
    vector<Fan>        fans;
    auto               input = src.img;
    if ( src.bullet_speed > 10 ) {
        double bullet_speed;
        if ( abs( src.bullet_speed - last_bullet_speed ) < 0.5 ||
             abs( src.bullet_speed - last_bullet_speed ) > 1.5 ) {
            bullet_speed = src.bullet_speed;
            predictor_->setBulletSpeed( bullet_speed );
            // coordsolver.setBulletSpeed( bullet_speed );
            last_bullet_speed = bullet_speed;
        }
    }
    Eigen::Matrix3d rmat_imu;
    if ( USING_IMU ) {
        rmat_imu = src.quat.toRotationMatrix();
    } else {
        rmat_imu = Eigen::Matrix3d::Identity();
    } // USING_IMU
    // TODO:修复ROI
    if ( USING_ROI ) {
        roi_offset = cropImageByROI( input );
    }
    // USING_ROI
    auto time_crop = std::chrono::steady_clock::now();
    //若未检测到目标
    /**
     * @description: bool detector.detect(input, objects)
     * @param input --输入图像
     * @param objects --vector<BuffObject> objects;
     * @return {取地址，当识别程序运行完成后，objects里装载识别结果}
     * @return 识别成功为true
     * @author: chen-zhan-smile
     */
    //这里是未识别到得情况
    bool flag_detect = false;
    // flag_detect =detectnet();
    if ( flag_detect ) {
        lost_cnt++;
        is_last_target_exists = false;
        last_target_area      = 0;
        data = { { (float) 0 }, { (float) 0 }, { (float) 0 }, 0, 0, 0, 1 };
        RCLCPP_WARN( this->get_logger(), "[BUFF_Detect] No target detected!" );
        return false;
    }
    auto time_infer = std::chrono::steady_clock::now();
    ///------------------------生成扇叶对象----------------------------------------------///
    for ( auto object : objects ) {
        //筛选红蓝符，若不是己方要击打的符，直接进入下一次循环
        if ( color_mode ) {
            if ( object.color != 1 ) continue;
        }

        else {
            if ( object.color != 0 ) continue;
        }

        Fan fan;
        fan.id = object.cls; //能量机关的状态 0是激活状态 1是击打状态
        fan.color = object.color; //能量机关颜色
        fan.conf  = object.prob;  //置信度
        if ( object.color == 0 )
            fan.key = "B" + string( object.cls == 0 ? "Activated" : "Target" );
        if ( object.color == 1 )
            fan.key = "R" + string( object.cls == 0 ? "Activated" : "Target" );
        //将object中的5点拷贝到fan的5点中，该拷贝为内存拷贝
        memcpy( fan.apex2d, object.apex, 5 * sizeof( cv::Point2f ) );
        for ( auto &i : fan.apex2d ) {
            //若进行了roi操作，则点在原始帧上的点为，roi帧点加上roi左上的坐标
            i += Point2f( (float) roi_offset.x, (float) roi_offset.y );
        }
        std::vector<Point2f> points_pic( fan.apex2d, fan.apex2d + 5 );
        auto pnp_result = coordSolver_->pnp( points_pic, rmat_imu, 
                                           SOLVEPNP_ITERATIVE );
        //--------------------------------------------------------------------------//
        //以下的数据应该通过请求坐标变换节点求得，为service服务
        fan.armor3d_cam     = pnp_result.armor_cam;
        fan.armor3d_world   = pnp_result.armor_world;
        fan.centerR3d_cam   = pnp_result.R_cam;
        fan.centerR3d_world = pnp_result.R_world;
        fan.euler = pnp_result.euler;
        fan.rmat  = pnp_result.rmat;

        fans.push_back( fan );
    }
    ///------------------------生成/分配FanTracker----------------------------
    if ( !trackers.empty() ) {
        //维护Tracker队列，删除过旧的Tracker
        for ( auto iter = trackers.begin(); iter != trackers.end(); ) {
            //删除元素后迭代器会失效，需先行获取下一元素
            auto next = iter;
            if ( ( src.timestamp - ( *iter ).last_timestamp ) > max_delta_t )
                next = trackers.erase( iter );
            else
                ++next;
            iter = next;
        }
    }
    // TODO:增加防抖
    std::vector<FanTracker> trackers_tmp;
    //为扇叶分配或新建最佳FanTracker
    for ( auto &fan : fans ) {
        if ( trackers.empty() ) {
            FanTracker fan_tracker( fan, src.timestamp );
            trackers_tmp.push_back( fan_tracker );
        } else {
            // 1e9无实际意义，仅用于以非零初始化
            double min_v                   = 1e9;
            int    min_last_delta_t        = 1e9;
            bool   is_best_candidate_exist = false;

            std::vector<FanTracker>::iterator best_candidate;
            for ( auto iter = trackers.begin(); iter != trackers.end();
                  iter++ ) {
                double            delta_t;
                Eigen::AngleAxisd angle_axisd;
                double            rotate_speed;
                double            sign;
                //----------------------------计算角度,求解转速----------------------------
                //若该扇叶完成初始化,且隔一帧时间较短
                if ( ( *iter ).is_initialized &&
                     ( src.timestamp - ( *iter ).prev_timestamp ) <
                         max_delta_t ) {
                    delta_t = src.timestamp - ( *iter ).prev_timestamp;
                    //目前扇叶到上一次扇叶的旋转矩阵
                    auto relative_rmat =
                        ( *iter ).prev_fan.rmat.transpose() * fan.rmat;
                    angle_axisd = Eigen::AngleAxisd( relative_rmat );
                    auto rotate_axis_world =
                        ( *iter ).last_fan.rmat * angle_axisd.axis();
                    // clang-format off
                    sign = ( fan.centerR3d_world.dot( rotate_axis_world ) > 0 )? 1: -1;
                    // clang-format on
                } else {
                    delta_t = src.timestamp - ( *iter ).last_timestamp;
                    //目前扇叶到上一次扇叶的旋转矩阵
                    auto relative_rmat =
                        ( *iter ).last_fan.rmat.transpose() * fan.rmat;
                    // TODO:使用点乘判断旋转方向
                    angle_axisd            = Eigen::AngleAxisd( relative_rmat );
                    auto rotate_axis_world = fan.rmat * angle_axisd.axis();
                    // clang-format off
                    sign = ( fan.centerR3d_world.dot( rotate_axis_world ) > 0 )? 1: -1;
                    // clang-format on
                }
                rotate_speed = sign * ( angle_axisd.angle() ) / delta_t *
                               1e3; //计算角速度(rad/s)
                // clang-format off
                if (abs( rotate_speed ) <= max_v &&
                    abs( rotate_speed ) <= min_v &&
                    ( src.timestamp - ( *iter ).last_timestamp ) <=min_last_delta_t ){

                        min_last_delta_t = src.timestamp - ( *iter ).last_timestamp;
                        min_v            = rotate_speed;
                        best_candidate   = iter;
                        is_best_candidate_exist = true;

                }
                // clang-format on
            }
            if ( is_best_candidate_exist ) {
                ( *best_candidate ).update( fan, src.timestamp );
                ( *best_candidate ).rotate_speed = min_v;
            } else {
                FanTracker fan_tracker( fan, src.timestamp );
                trackers_tmp.push_back( fan_tracker );
            }
        }
    }
    for ( const auto &new_tracker : trackers_tmp ) {
        trackers.push_back( new_tracker );
    }
    ///------------------------检测待激活扇叶是否存在----------------------------
    Fan  target;
    bool is_target_exists = chooseFromTargets( fans, target );

    //若不存在待击打扇叶则返回false
    if ( !is_target_exists ) {
        if ( SHOW_ALL_FANS ) {
            for ( auto fan : fans ) {
                putText( src.img, fmt::format( "{:.2f}", fan.conf ),
                         fan.apex2d[ 4 ], FONT_HERSHEY_SIMPLEX, 1,
                         { 0, 255, 0 }, 2 );
                if ( fan.color == 0 )
                    putText( src.img, fmt::format( "{}", fan.key ),
                             fan.apex2d[ 0 ], FONT_HERSHEY_SIMPLEX, 1,
                             Scalar( 0, 255, 0 ), 2 );
                if ( fan.color == 1 )
                    putText( src.img, fmt::format( "{}", fan.key ),
                             fan.apex2d[ 0 ], FONT_HERSHEY_SIMPLEX, 1,
                             Scalar( 0, 255, 0 ), 2 );
                for ( int i = 0; i < 5; i++ )
                    line( src.img, fan.apex2d[ i % 5 ],
                          fan.apex2d[ ( i + 1 ) % 5 ], Scalar( 0, 255, 0 ), 1 );
                auto fan_armor_center =
                    coordSolver_->reproject( fan.armor3d_cam );
                circle( src.img, fan_armor_center, 4, { 0, 0, 255 }, 2 );
            }
        }

        lost_cnt++;
        is_last_target_exists = false;
        data = { { (float) 0 }, { (float) 0 }, { (float) 0 }, 0, 0, 0, 1 };
        RCLCPP_WARN( this->get_logger(),
                     "[BUFF] No available target fan or Multiple target \
                      fan detected!" );
        return false;
    }

    int             avail_tracker_cnt = 0;
    double          rotate_speed_sum  = 0;
    double          mean_rotate_speed = 0;
    Eigen::Vector3d r_center_sum      = { 0, 0, 0 };
    Eigen::Vector3d mean_r_center     = { 0, 0, 0 };

    //计算平均转速与平均R字中心坐标
    for ( const auto &tracker : trackers ) {
        if ( tracker.is_last_fan_exists &&
             tracker.last_timestamp == src.timestamp ) {
            rotate_speed_sum += tracker.rotate_speed;
            r_center_sum += tracker.last_fan.centerR3d_world;
            avail_tracker_cnt++;
        }
    }
    //若不存在可用的扇叶则返回false
    if ( avail_tracker_cnt == 0 ) {
        RCLCPP_WARN( this->get_logger(),
                     "[BUFF] No available fan tracker exist!" );
        lost_cnt++;
        data = { { (float) 0 }, { (float) 0 }, { (float) 0 }, 0, 0, 0, 1 };
        return false;
    }
    mean_rotate_speed   = rotate_speed_sum / avail_tracker_cnt;
    mean_r_center       = r_center_sum / avail_tracker_cnt;
    double theta_offset = 0;
    ///------------------------进行预测----------------------------
    predictor_->mode = 1;
    if ( src.mode == 3 )
        //进入小能量机关识别模式
        predictor_->mode = 0;
    else if ( src.mode == 4 )
        //进入大能量机关识别模式
        predictor_->mode = 1;

    if ( !predictor_->CurveFitting( mean_rotate_speed, mean_r_center.norm(),
                                    src.timestamp, theta_offset ) ) {
        if ( SHOW_ALL_FANS ) {
            for ( auto fan : fans ) {
                putText( src.img, fmt::format( "{:.2f}", fan.conf ),
                         fan.apex2d[ 4 ], FONT_HERSHEY_SIMPLEX, 1,
                         { 0, 255, 0 }, 2 );
                if ( fan.color == 0 )
                    putText( src.img, fmt::format( "{}", fan.key ),
                             fan.apex2d[ 0 ], FONT_HERSHEY_SIMPLEX, 1,
                             Scalar( 0, 255, 0 ), 2 );
                if ( fan.color == 1 )
                    putText( src.img, fmt::format( "{}", fan.key ),
                             fan.apex2d[ 0 ], FONT_HERSHEY_SIMPLEX, 1,
                             Scalar( 0, 255, 0 ), 2 );
                for ( int i = 0; i < 5; i++ )
                    line( src.img, fan.apex2d[ i % 5 ],
                          fan.apex2d[ ( i + 1 ) % 5 ], Scalar( 0, 255, 0 ), 1 );
                auto fan_armor_center =
                    coordSolver_->reproject( fan.armor3d_cam );
                circle( src.img, fan_armor_center, 4, { 0, 0, 255 }, 2 );
            }
        }

        RCLCPP_WARN( this->get_logger(),
                     "[BUFF] predictor_ is still progressing!" );
        data = { { (float) 0 }, { (float) 0 }, { (float) 0 }, 0, 0, 0, 1 };
        return false;
    }

    ///------------------------计算击打点----------------------------
    //将角度转化至[-PI,PI范围内]
    // cout<<theta_offset<<endl;
    // theta_offset = rangedAngleRad(theta_offset);
    // cout<<theta_offset<<endl;
    //由offset生成欧拉角和旋转矩阵
    Eigen::Vector3d hit_point_world = {
        sin( theta_offset ) * fan_length,
        ( cos( theta_offset ) - 1 ) * fan_length, 0 };
    // cout<<hit_point_world<<endl;
    Eigen::Vector3d hit_point_cam = { 0, 0, 0 };
    /**
     * @description:  Pc = R * Pw + T
     * @brief: 通过pnp解算出来的旋转和平移向量可以解算出世界坐标系
     * @author: chen-zhan-smile
     * @description: auto r_center_cam =
     *                     coordsolver.worldToCam(mean_r_center,rmat_imu);
     */
    // clang-format off
    hit_point_world   = ( target.rmat * hit_point_world ) + target.armor3d_world;
    hit_point_cam     = coordSolver_->worldToCam( hit_point_world, rmat_imu );
    auto r_center_cam = coordSolver_->worldToCam( target.centerR3d_world, rmat_imu );
    auto center2d_src = coordSolver_->reproject( r_center_cam );
    auto target2d     = coordSolver_->reproject( hit_point_cam );
    auto angle        = coordSolver_->getAngle( hit_point_cam, rmat_imu );
    // clang-format on
    //-----------------判断扇叶是否发生切换-------------------------
    bool is_switched   = false;
    auto delta_t       = src.timestamp - last_timestamp;
    auto relative_rmat = last_fan.rmat.transpose() * target.rmat;
    // TODO:使用点乘判断旋转方向
    auto angle_axisd = Eigen::AngleAxisd( relative_rmat );
    // sign = ((*fan).centerR3d_world.dot(angle_axisd.axis()) > 0 ) ? 1 : -1;
    auto rotate_speed =
        ( angle_axisd.angle() ) / delta_t * 1e3; //计算角速度(rad/s)
    if ( abs( rotate_speed ) > max_v ) is_switched = true;

    lost_cnt              = 0;
    last_roi_center       = center2d_src; // center2d_src; //R的相机坐标
    last_timestamp        = src.timestamp;
    last_fan              = target;
    is_last_target_exists = true;

    //若预测出错取消本次数据发送
    if ( isnan( angle[ 0 ] ) || isnan( angle[ 1 ] ) ) {
        RCLCPP_WARN( this->get_logger(), "[BUFF] NAN Detected!" );
        return false;
    }

    auto time_predict = std::chrono::steady_clock::now();
    // clang-format off
    double dr_full_ms =
        std::chrono::duration<double, std::milli>( time_predict - time_start ).count();
    double dr_crop_ms =
        std::chrono::duration<double, std::milli>( time_crop - time_start ).count();
    double dr_infer_ms =
        std::chrono::duration<double, std::milli>( time_infer - time_crop ).count();
    double dr_predict_ms =
        std::chrono::duration<double, std::milli>( time_predict - time_infer ).count();

if (SHOW_ALL_FANS){
    for ( auto fan : fans ) {
        putText( src.img, fmt::format( "{:.2f}", fan.conf ), fan.apex2d[ 4 ],
                 FONT_HERSHEY_SIMPLEX, 1, { 0, 255, 0 }, 2 );
        if ( fan.color == 0 )
            putText( src.img, fmt::format( "{}", fan.key ), fan.apex2d[ 0 ],
                     FONT_HERSHEY_SIMPLEX, 1, Scalar( 0, 255, 0 ), 2 );
        if ( fan.color == 1 )
            putText( src.img, fmt::format( "{}", fan.key ), fan.apex2d[ 0 ],
                     FONT_HERSHEY_SIMPLEX, 1, Scalar( 0, 255, 0 ), 2 );
        for ( int i = 0; i < 5; i++ )
            line( src.img, fan.apex2d[ i % 5 ], fan.apex2d[ ( i + 1 ) % 5 ],
                  Scalar( 0, 255, 0 ), 1 );
        auto fan_armor_center = coordSolver_->reproject( fan.armor3d_cam );
        circle( src.img, fan_armor_center, 4, { 0, 0, 255 }, 2 );
    }
}

if (SHOW_PREDICT){
    circle( src.img, center2d_src, 5, Scalar( 0, 0, 255 ), 2 );
    circle( src.img, target2d, 5, Scalar( 255, 255, 255 ), 2 );
    predict_img_pub_.publish(cv_bridge::CvImage(this->img_msg->header, "rgb8", src.img).toImageMsg());
} 
    
if(SHOW_FPS){
    putText( src.img, fmt::format( "FPS: {}", int( 1000 / dr_full_ms ) ),
             { 10, 25 }, FONT_HERSHEY_SIMPLEX, 1, { 0, 255, 0 } );
     predict_img_pub_.publish(cv_bridge::CvImage(this->img_msg->header, "rgb8", src.img).toImageMsg());
}


if (PRINT_LATENCY){
    //降低输出频率，避免影响帧率
    if (src.timestamp % 10 == 0 ) {
        RCLCPP_INFO( this->get_logger(),
                    "-----------TIME------------\n" );
        RCLCPP_INFO( this->get_logger(), "Crop: {%lf} ms\n",
                    dr_crop_ms );
        RCLCPP_INFO( this->get_logger(), "Infer: {%lf} ms\n",
                    dr_infer_ms );
        RCLCPP_INFO( this->get_logger(), "Predict: {%lf} ms\n",
                    dr_predict_ms );
        RCLCPP_INFO( this->get_logger(), "Total: {%lf} ms\n",
                    dr_full_ms );
    }
}

if (PRINT_TARGET_INFO){
    RCLCPP_INFO( this->get_logger(), "-----------INFO------------\n" );
    RCLCPP_INFO( this->get_logger(), "Yaw: {%lf} \n", angle[ 0 ] );
    RCLCPP_INFO( this->get_logger(), "Pitch: {%lf} \n", angle[ 1 ] );
    RCLCPP_INFO( this->get_logger(), "Dist: {%lf} m\n",
                (float) hit_point_cam.norm() );
    RCLCPP_INFO( this->get_logger(), "Is Switched: {%s} \n",
                is_switched?"True":"False" );
}

    data = { {(float) angle[ 1 ]},
             {(float) angle[ 0 ]},
             {(float) hit_point_cam.norm()},
             is_switched,
             1,
             1,
             1 };
    auto message = msg_serial_sub();
    message.data="hello";
    /**
     * @description: 这里将data的数据放进消息体msg中，利用发布器发布到串口节点
     * @description: 形同如下的格式，最后根据定义的msg形式来赋值
     * @param : message.data="hello";
     * @return {*}
     * @author: chen-zhan-smile
     */    
    predict_pub_->publish(message);
    return true;
}

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto node =std::make_shared<BUFF::Buff>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
