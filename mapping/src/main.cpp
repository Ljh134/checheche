#include<iostream>
#include <fcntl.h>      // File control definitions
#include <termios.h>    // POSIX terminal control definitions
#include <unistd.h>     // UNIX standard function definitions
#include <cstring>
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
    double kp_speed = 0.005;
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
        //testing
        //cout << "p :" << p << "last_p " << last_p<<endl;
            output[x] = (p.y - last_p.y ) * config.frame *config.kp_speed;
            output[z] = (p.x - last_p.x) *  config.frame *config.kp_speed;
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

    vector<Point2f> creating_immediate_map(Mat& img,bool stop_signal)
    {
        points.clear();
        //imshow("Map",img);
        
        waitKey(10);
        vector<Point2f> res = {pointl, prePoint};
        
        //testing
        //cout << "pointl : " << pointl << " prePoint : " << prePoint << endl;
        if(stop_signal)
        {
            res[0] = res[1];
            pointl = prePoint;
        }

        //testing
        //cout << "output points : "<< res[0] << " , " << res[1] << endl;
        return res;
    }


}mapping;

//=================================================
//串口通信类
//==================================================
class SerialPort {
private:
    int serial_fd = -1;

    // 定义发送的数据包结构体 (取消字节对齐，确保紧凑)
    struct __attribute__((packed)) DataPacket {
        uint8_t header[2] = {0x55, 0xAA}; // 帧头
        float x;                          // 横移速度
        float z;                          // 前后速度
        float w;                          // 旋转速度
        uint8_t tail = 0xBB;              // 帧尾
    };

public:
    // 初始化串口
    // port_name: 例如 "/dev/ttyUSB0" 或 "/dev/ttyACM0"
    // baud_rate: 例如 115200
    bool init(const char* port_name, int baud_rate = 115200) {
        // 打开串口
        serial_fd = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd == -1) {
            std::cerr << "[Serial] Error opening port: " << port_name << std::endl;
            return false;
        }

        struct termios tty;
        if (tcgetattr(serial_fd, &tty) != 0) {
            std::cerr << "[Serial] Error getting attributes" << std::endl;
            return false;
        }

        // 设置波特率
        speed_t speed;
        switch(baud_rate) {
            case 9600: speed = B9600; break;
            case 115200: speed = B115200; break;
            default: speed = B115200; break;
        }
        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);

        // 设置 8N1 (8数据位, 无校验, 1停止位)
        tty.c_cflag &= ~PARENB; // 无校验
        tty.c_cflag &= ~CSTOPB; // 1停止位
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;     // 8数据位

        // 禁用流控制
        tty.c_cflag &= ~CRTSCTS; 
        
        // 启用读取和忽略控制行
        tty.c_cflag |= CREAD | CLOCAL; 

        // 禁用特殊字符处理 (raw mode)
        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;
        tty.c_lflag &= ~ECHOE;
        tty.c_lflag &= ~ISIG;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_oflag &= ~OPOST; // 原始输出

        // 应用设置
        if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
            std::cerr << "[Serial] Error setting attributes" << std::endl;
            return false;
        }

        std::cout << "[Serial] Port opened successfully: " << port_name << std::endl;
        return true;
    }

    // 发送速度指令
    void send_command(double x, double z, double w) {
        if (serial_fd == -1) return;

        DataPacket packet;
        // 将 double 转换为 float 以节省带宽 (下位机通常处理float更快)
        packet.x = static_cast<float>(x);
        packet.z = static_cast<float>(z);
        packet.w = static_cast<float>(w);

        // 写入串口
        int bytes_written = write(serial_fd, &packet, sizeof(packet));
        
        // 可选：调试打印
        // std::cout << "Sent " << bytes_written << " bytes." << std::endl;
    }

    // 关闭串口
    void close_port() {
        if (serial_fd != -1) {
            close(serial_fd);
            serial_fd = -1;
        }
    }

    ~SerialPort() {
        close_port();
    }
};




class mode
{
    private:
    std::unique_ptr<SerialPort> serial;

    public:
    mode()
    {
        serial = std::make_unique<SerialPort>();
        serial->init("/dev/ttyUSB0", 115200);
    }

    int still_mapping_mode(bool debug = true)
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

                if(debug)
                cout << "x : " << speed_output[x] <<
                " z : " << speed_output[z] << " w : " << speed_output[w] << endl; 

                else
                    serial->send_command(speed_output[x], speed_output[z], speed_output[w]);
                
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


    int immediate_mapping_mode(bool debug = true)
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
        bool stop_signal = true;
        cout << "按住左键绘制轨迹，松开停止绘制\n"
             << "按 b 键切换停止/继续运动\n"
             << "按 ESC 键退出即时映射模式\n";
        cout << "初始为停止状态，按 b 键开始运动\n";
        while(1)
        {
            //timer
            cv::TickMeter tm;
                tm.start(); 
            //  =================== code ================================
            pointoutput = mapping.creating_immediate_map(map,stop_signal);
            if(first_point)
            {
                first_point = false;
                continue;
            }
            if(stop_signal)
            {
                pointoutput[0] = pointoutput[1];
            }
            operate.run(pointoutput[1], pointoutput[0], map, speed_output);

           if(stop_signal)
           {
                speed_output[x] = 0;
                speed_output[z] = 0;
           }


            if(debug)
            cout << "x = " << speed_output[x] 
            << " z = " << speed_output[z] << " w = " <<speed_output[w] << endl;

            else
                serial->send_command(speed_output[x], speed_output[z], speed_output[w]);
                
            // ===================== code ==============================
            tm.stop();
            cost_ms = tm.getTimeMilli();
            delay_ms = 1000/config.frame - cost_ms;
            if(delay_ms > 0)
            {
               int key = waitKey(static_cast<int>(delay_ms));
               if((key == 'b'  && !stop_signal))
               {
                    stop_signal = true;
                    cout << "收到停止信号，停止运动" << endl;
               }
               else if(key == 'b' && stop_signal)
               {
                    stop_signal = false;
                    cout << "收到继续信号，继续运动" << endl;
               }
               
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




/*
下位机接收：
        // 定义和上位机完全一样的结构体
        typedef struct {
            uint8_t header[2];
            float x;
            float z;
            float w;
            uint8_t tail;
        } __attribute__((packed)) Packet;

        void on_uart_receive(uint8_t* buffer, int len) {
            Packet* pkt = (Packet*)buffer;

            // 校验帧头和帧尾
            if (pkt->header[0] == 0x55 && pkt->header[1] == 0xAA && pkt->tail == 0xBB) {
                // 接收成功，直接使用浮点数
                control_robot(pkt->x, pkt->z, pkt->w);
            }
        }
*/