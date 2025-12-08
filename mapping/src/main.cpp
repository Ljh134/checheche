#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include <cstring>
#include <vector>

using namespace std;
using namespace cv;


enum MouseEventTypes {
       EVENT_MOUSEMOVE      = 0, //!< 移动鼠标
       EVENT_LBUTTONDOWN    = 1, //!< 左键按下
       EVENT_RBUTTONDOWN    = 2, //!< 右键按下
       EVENT_MBUTTONDOWN    = 3, //!< 中键按下
       EVENT_LBUTTONUP      = 4, //!< 左键松开
       EVENT_RBUTTONUP      = 5, //!< 右键松开
       EVENT_MBUTTONUP      = 6, //!< 中键松开
       EVENT_LBUTTONDBLCLK  = 7, //!< 左键按下
       EVENT_RBUTTONDBLCLK  = 8, //!< 左键按下
       EVENT_MBUTTONDBLCLK  = 9, //!< 左键按下
       EVENT_MOUSEWHEEL     = 10,//!< 正值表示向前滚，负值后滚
       EVENT_MOUSEHWHEEL    = 11 //!< 正值左滚，负值右滚
     };

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



class Still_Mapping
{
    private:

    Point2f prePoint;
    vector<Point2f> points;

    void mouse(int event, int x, int y, int flags, void*user_data)
    {
        Mat img = (Mat*)user_data; 
        bool leftbottom = false;
        if (event == EVENT_RBUTTONDOWN) //单击右键
        {
            cout << "按住左键开始绘制轨迹";
        }

        if (event == EVENT_LBUTTONDOWN) //单击左键，输出坐标
        {
            putText( img, "drawing", Point(0.05*window_height,0.05*window_width),
                         2, 0.1, Scalar(0,0,255));
            leftbottom = true;
        }

        if (event == EVENT_LBUTTONUP)
        {
            leftbottom = false;
        }

        if (event == EVENT_MOUSEMOVE & leftbottom) // 鼠标移动（先不判断按键）
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
            img.at<Vec3b>(y, x - 1) = Vec3b(255, 0, 0);
            img.at<Vec3b>(y, x + 1) = Vec3b(255, 0, 0);
            img.at<Vec3b>(y + 1, x) = Vec3b(255, 0, 0);
            img.at<Vec3b>(y + 1, x) = Vec3b(255, 0, 0);
            //imshow("图像跟随绘制点", img);
            
            
            //通过绘制直线显示鼠标移动轨迹
            Point2f pt(x, y);
            line(img, prePoint, pt, Scalar(0, 0, 255), 2, 5, 0);
            prePoint = pt;
            points.push_back(pt);
            imshow("MAP", img);
        }
    }


    public:

    vector<Point2f> creating_still_map(Mat& img)
    {
        points.clear();
        setMouseCallback("MAP" , mouse,(void*)& img);
        return points;
    }

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
    }

}Still_Mapping;

class operate
{
    private:


    public:
    bool run(vector<Point2f> ps ,Mat& img , vector<double>& output)
    {
        Point2f prepoint = ps[0];
        for(Point2f point:ps)
        {
            output[x] = (point.y - prepoint.y ) * config.frame;
            output[z] = (point.x - prepoint.x) *  config.frame;
            Still_Mapping.show_process(img , point);
        }
    }

    void 
}

