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


// enum MouseEventTypes {
//        EVENT_MOUSEMOVE      = 0, //!< 移动鼠标
//        EVENT_LBUTTONDOWN    = 1, //!< 左键按下
//        EVENT_RBUTTONDOWN    = 2, //!< 右键按下
//        EVENT_MBUTTONDOWN    = 3, //!< 中键按下
//        EVENT_LBUTTONUP      = 4, //!< 左键松开
//        EVENT_RBUTTONUP      = 5, //!< 右键松开
//        EVENT_MBUTTONUP      = 6, //!< 中键松开
//        EVENT_LBUTTONDBLCLK  = 7, //!< 左键按下
//        EVENT_RBUTTONDBLCLK  = 8, //!< 左键按下
//        EVENT_MBUTTONDBLCLK  = 9, //!< 左键按下
//        EVENT_MOUSEWHEEL     = 10,//!< 正值表示向前滚，负值后滚
//        EVENT_MOUSEHWHEEL    = 11 //!< 正值左滚，负值右滚
//      };

enum direction
{
    x= 0,
    z = 1,
    w = 2
};



class config
{
    public:
    int frame = 25;
    int window_width = 1920;
    int window_height = 1080;
}config;



class operate
{
    private:
    
    void show_process(Mat& img ,Point2f& p)
    {
        int x = p.x;
        int y = p.y;
        img.at<Vec3b>(y,x) = Vec3b(0,255,0);
        if (x - 1 >= 0) 
        {
            img.at<Vec3b>(y, x - 1) = Vec3b(255, 0, 0);
        }
        if (x + 1 < img.cols) 
        {
            img.at<Vec3b>(y, x + 1) = Vec3b(255, 0, 0);
        }
        if (y + 1 < img.rows) 
        {
            img.at<Vec3b>(y + 1, x) = Vec3b(255, 0, 0);
        }
        if (y - 1 >= 0) 
        {
            img.at<Vec3b>(y - 1, x) = Vec3b(255, 0, 0);
        }
        imshow("MAP", img);
    }

    public:
    void run(Point2f p , Point2f last_p ,Mat& img , vector<double>& output)
    {
        
            output[x] = (p.y - last_p.y ) * config.frame;
            output[z] = (p.x - last_p.x) *  config.frame;
            show_process(img , p);
        
    }

    
} operate;


class Mapping
{
    private:

    Point2f prePoint = Point2f(0, 0);
    vector<Point2f> points;

    Point2f pointl = Point2f(0, 0);

    bool leftbottom = false;

    


    public:

    struct MouseCallbackData
    {
        Mat* img;
        Mapping* thisPtr;
    };

    static void mouse(int event, int x, int y, int flags, void*user_data)
    {
        MouseCallbackData* data = (MouseCallbackData*)user_data;
        Mapping* self = data->thisPtr;
        Mat& img = *((Mat*)(data->img)); 
         
        
        if (event == EVENT_RBUTTONDOWN) //单击右键
        {
            cout << "按住左键开始绘制轨迹";
        }

        if (event == EVENT_LBUTTONDOWN) //单击左键，输出坐标
        {
            putText( img, "drawing", Point(0.05*config.window_height,0.05*config.window_width),
                         2, 0.1, Scalar(0,0,255));
            self->leftbottom = true;
            self->prePoint = Point2f(x,y);
        }

        if (event == EVENT_LBUTTONUP)
        {
            self->leftbottom = false;
        }

        if (event == EVENT_MOUSEMOVE && self->leftbottom) // 鼠标移动（先不判断按键）
        {
            if(x <= config.window_width && x>= 0 && y >= 0 && y <= config.window_height)
            {
                img.at<Vec3b>(y, x) = Vec3b(255, 0, 0);
                if (x - 1 >= 0) 
                {
                    img.at<Vec3b>(y, x - 1) = Vec3b(255, 0, 0);
                }
                if (x + 1 < img.cols) 
                {
                    img.at<Vec3b>(y, x + 1) = Vec3b(255, 0, 0);
                }
                if (y + 1 < img.rows) 
                {
                    img.at<Vec3b>(y + 1, x) = Vec3b(255, 0, 0);
                }
                if (y - 1 >= 0) 
                {
                    img.at<Vec3b>(y - 1, x) = Vec3b(255, 0, 0);
                }
                //imshow("图像窗口 2", img);
            }

                // 绘制直线
            //通过改变图像像素显示鼠标移动轨迹
            img.at<Vec3b>(y, x) = Vec3b(255, 0, 0);
            // img.at<Vec3b>(y, x - 1) = Vec3b(255, 0, 0);
            // img.at<Vec3b>(y, x + 1) = Vec3b(255, 0, 0);
            // img.at<Vec3b>(y + 1, x) = Vec3b(255, 0, 0);
            // img.at<Vec3b>(y + 1, x) = Vec3b(255, 0, 0);
            //imshow("图像跟随绘制点", img);
            
            
            //通过绘制直线显示鼠标移动轨迹
            Point2f pt(x, y);
            line(img, self->prePoint, pt, Scalar(0, 0, 255), 2, 5, 0);
            self->pointl = self->prePoint;
            self->prePoint = pt;
            self->points.push_back(pt);
            imshow("MAP", img);
        }
    }


    

    vector<Point2f> creating_still_map(Mat& img)
    {
        points.clear();
        namedWindow("MAP" , WINDOW_NORMAL);
        imshow("MAP" , img);

        MouseCallbackData callbackData = { &img, this};
        setMouseCallback("MAP" , mouse,(void*)& callbackData);
        while(waitKey(1) != 13)
        {}
        destroyAllWindows();
        return points;
    }

    vector<Point2f> creating_immediate_map(Mat& img)
    {
        points.clear();
        //imshow("Map",img);
        
        waitKey(10);
        vector<Point2f> res = {pointl, prePoint};
        
        return res;
    }


}mapping;





class mode
{
    private:

    public:
    

    int still_mapping_mode()
    {
        Mat map(config.window_height, config.window_width, CV_8UC3, Scalar(255,255,255));
        
        vector<double> speed_output(3,0);
        vector<Point2f> points;
        double cost_ms , delay_ms;

        while(1)
        {
            map.setTo(Scalar(255,255,255));
            
            points = mapping.creating_still_map(map);

            if (points.empty()) 
            {
                cout << "未绘制任何轨迹，跳过后续处理" << endl;
                continue;
            }

            Point2f last_point = points[0];
            
            
            for(Point2f point : points)
            {
                cv::TickMeter tm;
                tm.start(); 

                //============================ timer ========================================

                operate.run(point , last_point , map, speed_output);
                last_point = point;


                cout << "x : " << speed_output[x] <<
                " z : " << speed_output[z] << " w : " << speed_output[w] << endl; 
                //================================ timer =========================================


                tm.stop();
                cost_ms = tm.getTimeMilli();
                delay_ms = 1000/config.frame - cost_ms; // 目标帧间隔：30帧≈33ms
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
            }

            cout<<"process over" << endl;
            cout <<"press 1 to reset , others to exit\n";



            if(getchar() == '1')
            {
                cin.get();
            }
            else
            {
                cout <<"exit\n";
                break;
            }
        }

        return 0;
    }


    int immediate_mapping_mode()
    {
        Mat map(config.window_height, config.window_width, CV_8UC3, Scalar(0,0,0));

        vector<double> speed_output(3,0);
        Point2f last_point;
        bool first_point = true;
        vector<Point2f> pointoutput(2);
        namedWindow("MAP" , WINDOW_NORMAL);
        imshow("MAP", map);

        Mapping::MouseCallbackData callbackData = { &map, &mapping };
        setMouseCallback("MAP" , Mapping::mouse, (void*)& callbackData);


        double cost_ms,delay_ms;

        while(1)
        {
            //timer
            cv::TickMeter tm;
                tm.start(); 
            //  =================== code ================================
            pointoutput = mapping.creating_immediate_map(map);
            if(first_point)
            {
                first_point = false;
                continue;
            }
            operate.run(pointoutput[1], pointoutput[0], map, speed_output);

            cout << "x = " << speed_output[x] 
            << " z = " << speed_output[z] << " w = " <<speed_output[w] << endl;
            // ===================== code ==============================
            tm.stop();
            cost_ms = tm.getTimeMilli();
            delay_ms = 1000/config.frame - cost_ms;
            if(delay_ms > 0)
            {
               int key = waitKey(static_cast<int>(delay_ms));
               if (key == 27) 
               {
                    cout << "退出即时映射模式" << endl;
                    break; 
               } 
            }
            else
            {
                cout << "times out at least " << delay_ms << " ms\n" ;
                waitKey(1);
            }
        }

        destroyAllWindows();
        return 0;
    }
};



int main()
{
    mode m;
    int mode_select = 1;
    while(mode_select != 0)
    {
        cout << "=================== Mapping Mode ===================\n"
            << endl
            << "select mode :\n"
            << "1. still mapping mode\n"
            << "2. immediate mapping mode\n"
            << "0. exit\n\n"
            << "input : ";

            
        
        cin >> mode_select;
        cin.get();
        switch(mode_select)
        {
            case 1:
                m.still_mapping_mode();
                break;
            case 2 :
                m.immediate_mapping_mode();
                break;
            case 0 :
                cout << "exit\n";
                break;
            default:
                cout << "invalid input , please input again\n";
                break;
        }
    }

    return 0;
}

