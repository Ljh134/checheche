#include<iostream>
#include "opencv4/opencv2/aruco.hpp"
#include"opencv4/opencv2/opencv.hpp"
#include "opencv4/opencv2/core/persistence.hpp"
#include "opencv4/opencv2/core/mat.hpp"
#include "opencv4/opencv2/calib3d.hpp"
#include <string>
#include <vector>
#include <chrono>
#include <unistd.h>
#include <cmath>

using namespace std;
using namespace cv;
using namespace std::chrono;

#define IPAD_SIZE 0.106f //m

class config
{
    public:

        //1920*1080

        const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
            982.45075722923072, 0., 1036.742344369498,  // 第 1 行：f_x, 0, c_x
            0., 987.51667190235230, 569.44939162861921,  // 第 2 行：0, f_y, c_y
            0., 0., 1.                                   // 第 3 行：0, 0, 1
        );

        const cv::Mat distortion_coeffs = (cv::Mat_<double>(1, 5) <<
            -0.022887155685407199, 0.041816419909876452,
            0.00074324526518525864, 0.0004643365718131034,
            -0.03678297685359066
        );


        // 640*480

        // const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
        //     982.45075722923072, 0., 1036.742344369498 * (640.0/1920.0),  // 新c_x = 1036.74 × (640/1920) ≈ 345.58
        //     0., 987.51667190235230, 569.44939162861921 * (480.0/1080.0),  // 新c_y = 569.45 × (480/1080) ≈ 253.09
        //     0., 0., 1.
        //     );  
        
        
        //800*600
        
        // const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
        //     982.45075722923072, 0., 1036.742344369498 * (800.0/1920.0),  // 新c_x ≈ 431.97
        //     0., 987.51667190235230, 569.44939162861921 * (600.0/1080.0),  // 新c_y ≈ 316.36
        //     0., 0., 1.
        //     );



        const double reprojection_error = 1.7534637048144051;
        const double square_size_cm = 3.1500000953674316;

        float markerLength = IPAD_SIZE; //标记实际边长(m)

        double frame = 4;      // fps
        double time = 1000000/frame;

        int view_width = 1920;
        int view_higth = 1080;
        double spin_react_range = 0.8;      // %
        double follow_react_range = 0.08;   // %
        double search_speed = 1.57;        // rad/s
        double follow_distance = 1;   // m
        double follow_distance_error_range = 0.1;     // m 
        std::vector<int> target_ids;
        

        config()
        {
            target_ids = {0,1,1,1,1,2,3,4,5,6};
        }


} config ;


class Operate
{
    private:
    VideoCapture video;
    int video_id = 4;
    public:
    int video_capture(Mat& img)
    {   
        if(!video.isOpened())
        {
            cout << "video_capture error: could not open video"
            <<endl;
            return -1;
        }
        video.set(CAP_PROP_FRAME_WIDTH, config.view_width);
        video.set(CAP_PROP_FRAME_HEIGHT, config.view_higth);
        video.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
        video >> img;
        if(img.empty())
        {
            cout << "video_capture error: video is empty "
             <<endl;
        }
        return 0;
    }


     // 将旋转向量转换为欧拉角（弧度/角度）
    // 旋转顺序：先绕Z轴旋转(偏航角Yaw)，再绕Y轴旋转(俯仰角Pitch)，最后绕X轴旋转(滚转角Roll)
    // 这是航空和机器人领域最常用的顺序之一。
    // 快速将单个旋转向量转换为欧拉角 (弧度)
// 返回值: cv::Vec3d(roll, pitch, yaw)
    void fastRvecToEuler(const vector<Vec3d>& rvecs , vector<double>& output) 
    {
        Mat rotMat;
        double x = 0, y = 0, z = 0;

        for(int i = 0; i<rvecs.size(); ++i)
        {
            Rodrigues(rvecs[i], rotMat);
            
            double sy = std::sqrt(rotMat.at<double>(0,0) * rotMat.at<double>(0,0) +
                                rotMat.at<double>(1,0) * rotMat.at<double>(1,0));
            bool singular = sy < 1e-6;
            
            
            if (!singular) 
            {
                x += std::atan2(rotMat.at<double>(2,1), rotMat.at<double>(2,2));
                y += std::atan2(-rotMat.at<double>(2,0), sy);
                z += std::atan2(rotMat.at<double>(1,0), rotMat.at<double>(0,0));
            } 
            else 
            {
                x += std::atan2(-rotMat.at<double>(1,2), rotMat.at<double>(1,1));
                y += std::atan2(-rotMat.at<double>(2,0), sy);
                z += 0;
            }
        }

        output[0] = x/rvecs.size();
        output[1] = y/rvecs.size();
        output[2] = z/rvecs.size();
    }

    Operate()
    {
        video.open(video_id, cv::CAP_V4L2); // 构造时初始化
        if(!video.isOpened()) {
            cerr << "摄像头打开失败！" << endl;
        }
    }
} operate ;




class Video_process
{
    private:
    
    enum preprocess_method
    {
        GAUSS = 1,
        BLUR = 2
    };
    
    public:
    void change(Mat& input , Mat& res)
    {
        if (input.channels() == 3) 
        {
           cvtColor(input , res , COLOR_BGR2GRAY);
        } 
        else 
        {
            res = input.clone();
        }

        
    }


    void preprocess(
        Mat& input, 
        Mat& res, 
        int method
    )
    {
        Mat res0;
        change(input,res0);
        if(method == GAUSS)
        {
            GaussianBlur(res0 , res, Size(3,3),1);
        }
        if(method == BLUR)
        {
            blur(res0 ,res ,Size(3,3));
        }
        if(method == 0)
        {
            res = res0.clone();
        }
    }
};


class Detect
{
    private:
    Ptr<aruco::Dictionary> dictionary 
    = aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    Ptr<aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();;

    public:

    Detect()
    {
        //一般视频设置:
        // 1. 自适应阈值调整（应对光照不均）
        detectorParams->adaptiveThreshWinSizeMin = 5;
        detectorParams->adaptiveThreshWinSizeMax = 31;
        detectorParams->adaptiveThreshWinSizeStep = 8;
        detectorParams->adaptiveThreshConstant = 7.0;
        // 2. 尺寸过滤（检测小标志，过滤噪声）
        detectorParams->minMarkerPerimeterRate = 0.01;  // 允许更小的标志
        detectorParams->maxMarkerPerimeterRate = 10.0;   // 限制过大的候选
        detectorParams->minMarkerDistanceRate = 0.02;   // 允许标志密集
        // 3. 编码验证（提升容错率，应对边框污染）
        detectorParams->maxErroneousBitsInBorderRate = 0.4;  // 边框错误容忍度提升
        detectorParams->perspectiveRemoveIgnoredMarginPerCell = 0.15;  // 忽略更多边框干扰
        // 4. 角点优化（提升姿态估计精度）
        detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;  // 亚像素角点
        detectorParams->cornerRefinementWinSize = 7;

        // //图片检测参数优化设置
        // detectorParams->adaptiveThreshWinSizeMin = 3;
        // detectorParams->adaptiveThreshWinSizeMax = 15;
        // detectorParams->adaptiveThreshConstant = 5.0; // 降低阈值，增强二值化敏感性
        // detectorParams->minMarkerPerimeterRate = 0.05; // 降低周长占比阈值，允许更小的标记
        // detectorParams->maxMarkerPerimeterRate = 3.0;  // 放宽最大周长限制
        // detectorParams->maxErroneousBitsInBorderRate = 0.5; // 提升边框错误容忍度
        // detectorParams->perspectiveRemoveIgnoredMarginPerCell = 0.13; // 恢复默认值，适配标准标记
        // detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE; // 临时关闭亚像素（减少计算干扰）



    }
    bool detect_AUcode(
        Mat& input,
        Mat& mat_show,
        vector<int>& ids,
        vector<int>& target_ids, 
        vector<vector<Point2f>>& corners,
        int& stright_command ,
        int show = 1
        )
        {
            aruco::detectMarkers(
                input,
                dictionary,
                corners,
                ids
            );

            //测试输出
            {
                cout << "detectMarkers检测到的ID数量:" << ids.size() << endl;
                if(!ids.empty()) {
                cout << "检测到的ID列表:";
                for(int id : ids) cout << id << " ";
                cout << endl;
                }
            }

            if(!ids.empty())
            {
                vector<Point2f> corners_temp;
                int ids_temp;
                stright_command = 1 ;
                for(int i = 0; i< ids.size(); ++i)
                {
                    if (ids[i] >= target_ids.size() || target_ids[ids[i]] == 0) 
                    {
                        corners.erase(corners.begin() + i);
                        ids.erase(ids.begin() + i);
                        i--;
                        continue;
                    }

                    if(target_ids[ids[i]] != 1 && target_ids[ids[i]] != 0)
                    {
                        corners_temp = corners[i];
                        ids_temp = ids[i];
                        corners.clear();
                        ids.clear();
                        corners.push_back(corners_temp);
                        ids.push_back(ids_temp);
                        stright_command = target_ids[ids[i]];
                        break;
                    }
                }

                
                
                if(show)
                {
                    aruco::drawDetectedMarkers(
                        mat_show,
                        corners,
                        ids,
                        Scalar(0,0,255)
                    );
                }
                return true;
            }
            else
            {
                return false;
            }  
        }



    void estimate_pose(
        Mat& image,
        Mat& draw_img,
        vector<vector<Point2f>>& corners,
        vector<Vec3d>& rvecs,
        vector<double>& rvecs_angle,
        vector<Vec3d>& tvecs,
        int show =0
    )
    {
        aruco::estimatePoseSingleMarkers(
            corners,
            config.markerLength,
            config.camera_matrix,
            config.distortion_coeffs,
            rvecs,
            tvecs
        );

        operate.fastRvecToEuler(rvecs,rvecs_angle);

        if(show)
        {
            for(int i = 0;i< rvecs.size(); ++i)
            {

            
            aruco::drawAxis(
                draw_img,
                config.camera_matrix,
                config.distortion_coeffs,
                rvecs[i],
                tvecs[i],
                2*config.markerLength
            );
            }
        }
    }

    
};


class Navigate
{
    private:
    double spin_react_range = 0.8;   
    double search_speed = 1.57;    // rad/s

    // 定义输出指令的索引（这是你给底盘发送速度的通道）
    enum CommandIndex
    {
        CMD_X = 0, // 左右横移速度
        CMD_Z = 1, // 前后移动速度
        CMD_W = 2  // 旋转角速度
    };
    
    // 定义OpenCV坐标系的索引（这是读取相机数据的标准）
    enum CVIndex
    {
        CV_X = 0, // 水平位移
        CV_Y = 1, // 垂直位移
        CV_Z = 2  // 深度距离（重点！）
    };

    void if_spin(vector<vector<Point2f>>& corners ,
                bool& right_spin,
                bool& left_spin
            )
    {
        right_spin = true;
        left_spin = true;
        for (int i = 0; i< corners.size(); ++i)
        {
            for(int j = 0; j< corners[i].size() ; ++j)
            {
                if(corners[i][j].x < (1 - spin_react_range)*config.view_width/2 )
                {
                    right_spin = false;
                    return ;
                }
                if ( corners[i][j].x > (1+ spin_react_range)*config.view_width/2)
                {
                    left_spin = false;
                    return ;
                }
            }
        }
    }

    public:

    double calculate_angle_speed(
        vector<double>& rvecs_angle,
        vector<double>& last
    )
    {
        double res = 0;
        // 修正：使用 index 1 (Yaw/Y轴旋转) 而不是 2 (Roll/Z轴旋转)
        // 假设 Y 轴旋转对应左右偏航
        res = (rvecs_angle[1] - last[1]) * config.frame;
        return res;
    }
 
    void calculate_pos_speed(
        vector<double>& tvec_center, // 这里传入的是物理坐标
        vector<double>& last_center,
        vector<double>& output
    )
    {
        // 计算速度变化
        output[CMD_X] = (tvec_center[CV_X] - last_center[CV_X]) * config.frame;
        
        // 修正：前后速度应该基于深度(CV_Z)的变化，而不是高度(CV_Y)
        output[CMD_Z] = (tvec_center[CV_Z] - last_center[CV_Z]) * config.frame;
    }

    // 修正：输入应该是深度值 z_distance
    int if_closer(double z_distance)
    {
        // 距离判断：如果当前深度 > 期望距离，则需要前进(0.5)，反之后退
        if( abs(z_distance - config.follow_distance) < config.follow_distance_error_range )
        {
            return 0;
        }
        if(z_distance > config.follow_distance)
        {
            return 0.5; // 太远了，向前
        }
        if(z_distance < config.follow_distance)
        {
            return -0.5; // 太近了，向后
        }
        return 0;
    }

    void translation(
        vector<double> tvecs_center,
        vector<double> tvecs_center_last,
        vector<double> output
    )
    {
        // 简单的 P 控制：如果目标在右边(X>0)，向右移
        if(tvecs_center[CV_X] > 0 )
        {
            output[CMD_X] += 0.5;
        }
        if(tvecs_center[CV_X] < 0 )
        {
            output[CMD_X] -= 0.5;
        }
    }

    void follow_(
        vector<vector<Point2f>>& corners,
        vector<double>& output
    )
    {
        Point2f center(0,0);
        int total_points = 0;
        for(int i = 0 ; i< corners.size(); ++i)
        {
            for(int j=0; j<4; ++j) {
                center.x += corners[i][j].x;
                center.y += corners[i][j].y;
            }
            total_points += 4;
        }

        if(total_points > 0) {
            center.x /= total_points;
            center.y /= total_points;
        }

        if(center.x > (1 + config.follow_react_range)*config.view_width/2)
        {
            // 目标在画面右侧，需要顺时针转/向右转
            output[CMD_W] -= 0.5; // 注意：这里正负号取决于你的电机转向定义，通常左转为正
        }
        else if(center.x < (1 - config.follow_react_range)*config.view_width/2)
        {
            output[CMD_W] += 0.5;
        }
    }

    void speed_cal(
        vector<double>& rvecs_angle,
        vector<double>& rvecs_last_angle,
        vector<Vec3d>& tvecs,
        vector<Vec3d>& tvecs_last,
        vector<vector<Point2f>>& corners,
        vector<double>& output
    )
    {
        // 临时变量存储当前的物理中心坐标（OpenCV坐标系）
        vector<double> tvecs_center(3, 0); 
        vector<double> tvecs_center_last(3, 0);
        
        bool right_spin = true, left_spin = true;

        // 1. 计算当前的中心坐标
        double sum_x = 0, sum_y = 0, sum_z = 0;
        for(int i = 0; i< tvecs.size(); ++i)
        {
            sum_x += tvecs[i][0]; // CV_X
            sum_y += tvecs[i][1]; // CV_Y (高度)
            sum_z += tvecs[i][2]; // CV_Z (深度) !!! 修正关键点
        }
        tvecs_center[0] = sum_x / tvecs.size();
        tvecs_center[1] = sum_y / tvecs.size();
        tvecs_center[2] = sum_z / tvecs.size();

        // 2. 计算上一帧的中心坐标 (简单处理，假设 tvecs_last 结构相同)
        // 注意：这里由于 tvecs_last 是 Vec3d，可以直接取
        if(tvecs_last.size() > 0) {
            tvecs_center_last[0] = tvecs_last[0][0]; 
            tvecs_center_last[1] = tvecs_last[0][1];
            tvecs_center_last[2] = tvecs_last[0][2];
        }

        // 3. 计算位移速度
        calculate_pos_speed(tvecs_center , tvecs_center_last, output);
       
        // 4. 深度保持逻辑 (Follow distance)
        // 修正：传入 tvecs_center[2] 即深度 Z
        output[CMD_Z] += if_closer(tvecs_center[2]); 

        // 5. 角速度计算
        output[CMD_W] = calculate_angle_speed(rvecs_angle , rvecs_last_angle);
        
        // 6. 旋转限制逻辑
        if_spin(corners, right_spin, left_spin);

        if(!(right_spin) && output[CMD_W] > 0) // 假设 >0 是向左转
        {
            output[CMD_W] = 0;
        }
        if(!(left_spin) && output[CMD_W] < 0)
        {
            output[CMD_W] = 0;
        }

        // 7. 横移对齐
        translation(tvecs_center, tvecs_center_last, output);
        
        // 8. 像素级辅助对齐 (可能会与 calculate_angle_speed 冲突，建议调试时先开启其中一个)
        follow_(corners, output);
    }

    void dock(vector<Vec3d>& tvecs, vector<vector<Point2f>>& corners, vector<double>& output)
    {
        follow_(corners, output);
        
        double tvec_x = 0;
        for(int i = 0; i< tvecs.size(); ++i)
        {
            tvec_x += tvecs[i][0]; // CV_X
        }
        double center_x = tvec_x / tvecs.size();

        if(center_x > 0 ) output[CMD_X] += 0.5;
        if(center_x < 0 ) output[CMD_X] -= 0.5;
    }
    

    void spin_search(vector<double>& output)
    {
        output[CMD_X] = 0;
        output[CMD_Z] = 0;
        output[CMD_W] = search_speed;
    }
};


class mode
{
    private:
    enum speed_angle
    {
        x = 0,
        z = 1,
        w = 2
    };

    enum preprocess_method
    {
        GAUSS = 1,
        BLUR = 2
    };

    enum if_show
    {
        SHOW_RESULT = 1,
        NOT_SHOW_RESULT =0
    };

    const int TARGET_CYCLE_US = (int) config.time;

    Video_process* video_process = new Video_process();
    Detect* detect = new Detect();
    Navigate* navigate = new Navigate();
    

    public:

    int photo_detect()
    {
        Mat img0 = imread("/home/lllljhhhhh/checheche/capture/AUmarker2.jpg");
        Mat img1;
        vector<int> ids;
        vector<vector<Point2f>> corners;
        vector<Vec3d> rvecs ;
        vector<Vec3d> tvecs ;
        vector<double> rvecs_angle(3,0);
        int stright_command = 1;
        cout << "加载的标记图像尺寸：" << img0.size() << endl;

        video_process ->preprocess(img0 , img1, 0);
        if( detect -> detect_AUcode(
            img1,
            img0,
            ids,
            config.target_ids,
            corners,
            stright_command,
            SHOW_RESULT
        ))
        {
            detect -> estimate_pose(
                img1,
                img0,
                corners,
                rvecs,
                rvecs_angle,
                tvecs,
                SHOW_RESULT
            );
            cout << "原始检测到的ID列表: ";
            for(int id : ids) cout << id << " ";
            cout << endl;
    
        }
        else
        {
            cout <<"No marker detected" <<endl;
        }
        imshow("GREY (press q to quit)",img1);
        imshow("RESULT (press q to quit)",img0);
        waitKey(0);
        destroyAllWindows();
        return 0;
    }


    int detect_and_lock()
    {
        namedWindow("GREY (press q to quit)" , WINDOW_NORMAL);
        resizeWindow("GREY (press q to quit)",
            config.view_width,config.view_higth);

        namedWindow("RESULT (press q to quit)" , WINDOW_NORMAL);
        resizeWindow("RESULT (press q to quit)",
            config.view_width,config.view_higth);

        Mat img0 ,img1 ;
        vector<int> ids; 
        vector<vector<Point2f>> corners;
        vector<Vec3d> rvecs ;
        vector<double> rvecs_angle(3,0);
        vector<Vec3d> tvecs ;
        
        int stright_command = 1 ;

        while(1)
        {
            cv::TickMeter tm;
            tm.start(); 
            //timer
            //auto start = high_resolution_clock::now();

        //  =================== code ================================
                        

            if( ! ( operate.video_capture(img0) ))
            {
                //testing
                //cout << "实际图像分辨率: " << img0.cols << " x " << img0.rows << endl;
                video_process -> preprocess(img0 , img1,GAUSS);

               if( detect -> detect_AUcode(
                    img1,
                    img0, 
                    ids,
                    config.target_ids,
                    corners,
                    stright_command,
                    SHOW_RESULT
                    )
                )
                {
                    detect -> estimate_pose(
                        img1,
                        img0,
                        corners,
                        rvecs,
                        rvecs_angle,
                        tvecs,
                        SHOW_RESULT
                    );

                    cout << "marker pos: x = " << tvecs[0][x] << "(m) , z = " << tvecs[0][z] << "(m)" << endl;
                }
                else
                {
                    cout <<"No marker detected" <<endl;
                }

                
                imshow("GREY (press q to quit)",img1);
                imshow("RESULT (press q to quit)",img0);
            }

            else
            {
                cout << "detect_and_lock error: could not open video" << endl;
                return -1;
            }
        //  =================== code end ================================

                //timer
            // auto end = high_resolution_clock::now();
            // auto logic_cost_us = duration_cast<microseconds>(end - start).count();

            // // 3. 动态补充延时（确保总周期 = 目标周期）
            // if (logic_cost_us < TARGET_CYCLE_US) {
            //     int delay_us = TARGET_CYCLE_US - logic_cost_us;
            //     usleep(delay_us);  // 补充差值延时
            // }

            tm.stop();
            double cost_ms = tm.getTimeMilli();
            double delay_ms = 1000/config.frame - cost_ms; // 目标帧间隔：30帧≈33ms
            if (delay_ms > 0) 
            {
                waitKey(static_cast<int>(delay_ms)); // 用waitKey替代usleep，不阻塞窗口
            } 
            else 
            {
                cout << "times out at least " << delay_ms << " ms\n" ;
                waitKey(1); // 至少等待1ms，避免CPU占满
            }
            tm.reset();

            // 4. 按键退出（避免 OpenCV 窗口卡死）
            if (waitKey(1) == 'q') 
            {
                destroyAllWindows();
                break;
            }

        }

        cout << "detect_and_lock is break" << endl ;
        return 0;
    }

    int detect_and_navigate(int if_visual = 1 )
    {
        namedWindow("GREY (press q to quit)" , WINDOW_NORMAL);
        resizeWindow("GREY (press q to quit)",
                    config.view_width,config.view_higth);

        namedWindow("RESULT (press q to quit)" , WINDOW_NORMAL);
        resizeWindow("RESULT (press q to quit)",
            config.view_width,config.view_higth);

        std::cout << "begin running" <<std::endl;
        Mat img0 ,img1 ;
        vector<int> ids; 
        vector<vector<Point2f>> corners;
        vector<Vec3d> rvecs = {0};
        vector<double> rvecs_angle(3,0);
        vector<double> rvecs_angle_last(3,0);
        vector<Vec3d> tvecs = {0};
        vector<Vec3d> tvecs_last = {0};
        vector<double> speed_output = {0};
        int stright_command = 1;

        while(1)
        {
            //timer
            auto start = high_resolution_clock::now();

            //  =================== code ================================
            
            

            if( ! ( operate.video_capture(img0) ))
            {
                video_process -> preprocess(img0 , img1, GAUSS);

                if(detect -> detect_AUcode(
                    img1,
                    img0, 
                    ids,
                    config.target_ids, 
                    corners,
                    stright_command,
                    SHOW_RESULT
                    ))
                {
                    detect -> estimate_pose(
                        img1,
                        img0,
                        corners,
                        rvecs,
                        rvecs_angle,
                        tvecs,
                        NOT_SHOW_RESULT
                    );

                    //testing
                    //std::cout << "detect over" << std::endl;
                    if(if_visual)
                    {
                        imshow("GREY (press q to quit)",img1);
                        imshow("RESULT (press q to quit)",img0);

                        //testing
                        //std::cout << "visual on" << std::endl;
                    }
                    
                    if(stright_command == 1)
                    {

                        navigate -> speed_cal(
                                rvecs_angle,
                                rvecs_angle_last,
                                tvecs,
                                tvecs_last,
                                corners,
                                speed_output
                                );  
                        
                    }
                    if(stright_command != 1)
                    {
                        if(tvecs[0][z] > config.follow_distance + config.follow_distance_error_range)
                        {
                           
                                speed_output[x] = 0;
                                speed_output[w] = 0;
                                navigate->dock(tvecs,corners,speed_output);
                                speed_output[z] = 0.5;
                        }
                        if(tvecs[0][z] < config.follow_distance - config.follow_distance_error_range)
                        {
                            speed_output[x] = 0;
                            speed_output[w] = 0;
                            navigate->dock(tvecs,corners,speed_output);
                            speed_output[z] = -0.5;
                        }
                        if(tvecs[0][z] > config.follow_distance - config.follow_distance_error_range
                        && tvecs[0][z] < config.follow_distance + config.follow_distance_error_range)
                        {
                            if(stright_command == 2)
                            {
                                speed_output[x] = 0;
                                speed_output[z] = 0;
                                speed_output[w] = 0.5;
                            }
                            if(stright_command == 3)
                            {
                                speed_output[x] = 0;
                                speed_output[z] = 0;
                                speed_output[w] = -0.5;
                            }
                            if(stright_command == 4)
                            {
                                speed_output[x] = 0;
                                speed_output[z] = 0;
                                speed_output[w] = 0;
                            }
                        }

                    }


                }
                else
                {
                    navigate-> spin_search(speed_output);
                }
            

                cout << "x = " << speed_output[x] << " (m/s)" << endl;
                cout << "z = " << speed_output[z] << " (m/s)" << endl;
                cout << "w = " << speed_output[w] << " (rad/s)" << endl;
                
                for(int i = 0; i<3; ++i)
                {
                    rvecs_angle_last[i] = rvecs_angle[i];
                }
                for(int i = 0;i<rvecs.size();++i)
                {
                    for(int j = 0 ;j <3 ;++j)
                    {
                        tvecs_last[i][j] = tvecs[i][j];
                    }
                }
            }
            else
            {
                cout << "detect_and_lock error: could not open video" << endl;
                return -1;
            }
                //  =================== code end ================================

                //timer
            auto end = high_resolution_clock::now();
            auto logic_cost_us = duration_cast<microseconds>(end - start).count();

            // 3. 动态补充延时（确保总周期 = 目标周期）
            if (logic_cost_us < TARGET_CYCLE_US) {
                int delay_us = TARGET_CYCLE_US - logic_cost_us;
                usleep(delay_us);  // 补充差值延时
            }

            // 4. 按键退出（避免 OpenCV 窗口卡死）
            if (waitKey(1) == 'q')
            {
                destroyAllWindows();
                break;
            }

        }
        return 0;
    }


    ~mode()
    {
        delete video_process;
        delete detect;
        delete navigate;
    }

};






int main()
{
    cv::setNumThreads(8);
    mode* Mode = new mode();

    ////视频测试:
    {
    if(Mode -> detect_and_lock())
    {
        cout << "detect_and_lock arrupted" << endl;
    }
    else cout << "detect_and_lock ended normally" << endl;
    }

    // if(Mode -> detect_and_navigate(1))
    // {
    //     cout << "detect_and_navigate arrupted" << endl;
    // }
    // else cout << "detect_and_navigate ended normally" << endl;

    //照片测试:
    //{
    // if(Mode ->photo_detect() != 0)
    // {
    //     cout << "photo_detect stopped" <<endl;
    //     return -1;
    // }
    //}

    delete Mode;
    return 0;
}