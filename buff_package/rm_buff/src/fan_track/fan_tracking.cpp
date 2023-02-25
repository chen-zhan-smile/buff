/*
 * @Description: 
 * @Version: 1.0
 * @Autor: chen zhan
 * @Date: 2023-02-21 21:46:40
 * @LastEditors: chen zhan
 * @LastEditTime: 2023-02-25 21:56:03
 */
#include "fan_tracking.hpp"

/*这里宏定义的是筛选可疑的轮廓的面积条件*/
#define MAX_AREA 6000.0
#define MIN_AREA 150.0

inline double Fan_tracking::distance(cv::Point a,cv::Point b)
{
    return sqrt((a.x -b.x)*(a.x -b.x) + (a.y -b.y)*(a.y -b.y));
}

void Fan_tracking::drawRotatedRect(cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness)
{
    cv::Point2f Vertex[4];
    rect.points(Vertex);
    for(int i = 0 ; i < 4 ; i++)
    {
        cv::line(img , Vertex[i] , Vertex[(i + 1) % 4] , color , thickness);
    }
}

void Fan_tracking::image_process()
{   
    
    /*尺寸变化，图片的resolution设置巍峨640*480*/
    cv::resize(this->image_,this->image_,cv::Size(640,480));
    vector<cv::Mat> bgr_images;
    /*图像分离通道*/
    cv::split(this->image_,bgr_images);
    /*这里是用理想的蓝色能量机关作为素材，故分离的蓝色通道作为预处理后的图
      且通道分离后图片成为了灰度图*/
    cv::Mat b_src_img = bgr_images[0];
    /*进行3x3核的滤波减小噪声*/
    cv::blur(b_src_img,b_src_img,cv::Size(3,3));
    
    cv::Mat threshold_img;
    /*阈值分割，使图像二值化*/
    cv::threshold(b_src_img,threshold_img,130,255,cv::THRESH_BINARY);
    /*储存可疑的所有的轮廓,在储存前先将轮廓容器清空*/
    this->contours_.clear();
    cv::findContours(threshold_img,this->contours_,cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
}

void Fan_tracking::suspect_profile_screen()
{
    for (unsigned int contour_index = 0; contour_index < this->contours_.size(); contour_index++) {
    //     //寻找最小旋转矩形,放进容器，顺便将面积过大的剔除,长宽比悬殊的剔除
        cv::RotatedRect minrect = minAreaRect(this->contours_[contour_index]);
        cv::Rect rect = boundingRect(this->contours_[contour_index]);
        if (minrect.size.area() <= 6000.0 && minrect.size.area()>150) {
            float width;
            float height;
            //判断长宽比限制条件
            if (minrect.size.width > minrect.size.height){
                width = minrect.size.width;
                height = minrect.size.height;
            } else {
                width = minrect.size.height;
                height = minrect.size.width;
            }
            if (width / height < 5) {
                if(minrect.size.area() > 200 && minrect.size.area() < 350 && minrect.center.y > 100) { // find R
                    if(height / width > 0.85){
                        // R_minRects.push_back(minrect);
                        this->r_center = minrect.center;
                        cv::circle(this->image_,minrect.center,15,cv::Scalar(5,255,100));   
                    }
                } else {
                    if(minrect.size.area() > 300 && minrect.size.area() < 4350 && (height / width) < 0.7) {
                        this->suspect_armor_mrect.emplace_back(minrect);
                    }     
                }
            }   
        }
    }
}

int Fan_tracking::matching_profile()
{   
    for (int i = 0;i<suspect_armor_mrect.size()-1;i++){
            for (int j = i+1;j<suspect_armor_mrect.size();j++){
                double dis = distance(suspect_armor_mrect[i].center,suspect_armor_mrect[j].center);
                if(dis<100){
                    target_min_rects.push_back(suspect_armor_mrect[i]);
                    target_min_rects.push_back(suspect_armor_mrect[j]);
                    this->find_ok = true;
                    break;
                }
                // std::cout<<dis<<std::endl;
            }   
            if (this->find_ok){
                break;
            }    
        }
    if(target_min_rects.size() != 2){
        /*上下轮廓拟合失败，轮廓超过2个，定位击打区域失败*/
           return Match_Error;
        } else {
            /*上下轮廓定位击打区域*/
            cv::RotatedRect rrect_in; //内侧拟合轮廓
            cv::RotatedRect rrect_out;// 外侧拟合轮廓
            /*上下轮廓到R中心的距离，距离大的为外侧轮廓，反之为内侧*/
            double dis1 = distance(r_center,target_min_rects[0].center);
            double dis2 = distance(r_center,target_min_rects[1].center);
            if (dis1 > dis2){
                rrect_in = target_min_rects[1];
                rrect_out = target_min_rects[0];
            } else {
                rrect_in = target_min_rects[0];
                rrect_out = target_min_rects[1];
            }
            /*通过上下轮廓中心坐标相加除以二,大概定位中心位置*/  
            cv::Point target_center = cv::Point((int)((rrect_in.center.x + rrect_out.center.x)/2),(int)((rrect_in.center.y + rrect_out.center.y)/2));
#ifdef _SHOW_
            cv::circle(this->image_,target_center,5,cv::Scalar(0,0,255),-1);
            /*画出内外侧轮廓*/
            drawRotatedRect(this->image_,rrect_in,cv::Scalar(0,250,0),1);
            drawRotatedRect(this->image_,rrect_out,cv::Scalar(0,0,255),1);
#endif
            cv::Point2f in_vet_points[4];
            cv::Point2f out_vet_points[4];
            rrect_in.points(in_vet_points);
            rrect_out.points(out_vet_points);
            vector<armor_point> armor_points;
#ifdef _SHOW_
            /*标出里击打目标点最近的几个交点，更加直观*/ 
            for(int i = 0;i<4;i++){
                armor_point point;
                point.point = in_vet_points[i];
                point.dis = distance(target_center,in_vet_points[i]);
                armor_points.emplace_back(point);
            }
            for(int i = 0;i<4;i++){
                armor_point point;
                point.point = out_vet_points[i];
                point.dis = distance(target_center,out_vet_points[i]);
                armor_points.emplace_back(point);
            }
            /*sort按离目标点距离进行自动排序，从小排到大*/
            sort(armor_points.begin(),armor_points.end(),[](armor_point a,armor_point b){return a.dis < b.dis;});
            for(int i = 0;i<4;i++)
            {
                cv::circle(this->image_,armor_points[i].point,3,cv::Scalar(255,0,255),-1);
            }
            int buff_run_radius = (int)distance(r_center,target_center);
            cv::circle(this->image_,r_center,buff_run_radius,cv::Scalar(55,110,255),1);
            cv::imshow("main",image_);
            cv::waitKey(200);
        }
#endif 
    return Match_Success;
}

void Fan_tracking::update_image(const cv::Mat img_update)
{
    this->image_=img_update.clone();
    this->image_process();
    this->suspect_profile_screen();
    this->matching_profile();
}

Fan_tracking::Fan_tracking(cv::Mat image)
{
    this->image_=image.clone();
    this->image_process();
    this->suspect_profile_screen();
    this->matching_profile();
}

// int main(int argc, char const *argv[])
// {   cv::namedWindow("main");
//     string video_path = "/home/chen/Robomaster/Buff/buff_test/buff_blue.mp4"; 
//     cv::VideoCapture cap;
//     cap.open(video_path);
//     while(true){
//         cv::Mat src_frame;
//         cap >> src_frame;
//         Fan_tracking tracker(src_frame);
//         }
//     return 0;
// }
