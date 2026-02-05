#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <cmath>
#include <thread>
#include <memory> // For smart pointers
#include <fcntl.h>      // File control definitions
#include <termios.h>    // POSIX terminal control definitions
#include <unistd.h>     // UNIX standard function definitions
#include <cstring>

#include "opencv4/opencv2/aruco.hpp"
#include "opencv4/opencv2/opencv.hpp"
#include "opencv4/opencv2/core/persistence.hpp"
#include "opencv4/opencv2/calib3d.hpp"

using namespace std;
using namespace cv;
using namespace std::chrono;

// ==========================================
// 全局配置与常量定义
// ==========================================

// 机器人控制指令索引 (发送给下位机/底盘)
enum CommandIndex {
    CMD_LATERAL = 0, // X: 左右平移 (m/s)
    CMD_FORWARD = 1, // Z: 前后移动 (m/s)
    CMD_ROTATION = 2 // W: 自转角速度 (rad/s)
};

// OpenCV 相机坐标系索引 (读取视觉数据)
// 相机坐标系: X向右, Y向下, Z向前(深度)
enum CVIndex {
    CV_X = 0,
    CV_Y = 1,
    CV_Z = 2
};

struct Config {
    // 相机内参 (1920x1080)
    const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
        982.450757, 0., 1036.742344,
        0., 987.516672, 569.449392,
        0., 0., 1.);

    const cv::Mat distortion_coeffs = (cv::Mat_<double>(1, 5) <<
        -0.022887, 0.041816, 0.000743, 0.000464, -0.036783);

    // 物理参数
    const float markerLength = 0.106f; // m (IPAD_SIZE)
    
    // 性能参数
    const double target_fps = 30.0; // 提高帧率以获得更流畅的控制
    const double frame_time_ms = 1000.0 / target_fps;

    // 图像参数
    const int view_width = 1920;
    const int view_height = 1080;

    // 导航控制参数 (P-Controller 增益)
    // 误差越大速度越快
    const double kp_forward = 1.2;  // 前后移动比例系数
    const double kp_lateral = 1.0;  // 横移比例系数
    const double kp_rotate  = 2.5;  // 旋转比例系数

    const double follow_distance = 0.3;         // 目标距离 (m)
    const double follow_deadzone = 0.05;        // 距离死区 (m)，在此范围内不移动
    const double rotate_deadzone_pixel = 50.0;  // 旋转死区 (像素)，中心偏移在此范围内不旋转

    const double max_speed_linear = 1.0; // m/s 最大线速度限制
    const double max_speed_angular = 1.5; // rad/s 最大角速度限制
    const double search_speed = 0.8;        // rad/s 寻找速度
    // 状态映射表
    std::vector<int> target_id_map;

    // 串口参数
    const char* usb_name = "/dev/ttyUSB0"; // 根据实际情况

    Config() {
        // ID 映射: index是aruco ID, value是命令类型
        target_id_map.resize(100, 0);
        std::vector<int> init_ids = {0, 1, 1, 1, 1, 2, 3, 4, 5, 6};
        for(size_t i = 0; i < init_ids.size() && i < target_id_map.size(); ++i) {
            target_id_map[i] = init_ids[i];
        }
    }
} config;

// ==========================================
// 硬件操作类 (相机 IO)
// ==========================================
class Operate {
private:
    VideoCapture video;
    int video_id = 4; // 根据实际情况修改

public:
    Operate() {
        // 尝试打开相机
        open_camera();
    }

    void open_camera() {
        video.open(video_id, cv::CAP_V4L2);
        if (video.isOpened()) {
            video.set(CAP_PROP_FRAME_WIDTH, config.view_width);
            video.set(CAP_PROP_FRAME_HEIGHT, config.view_height);
            video.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
            // 尝试设置相机自身的FPS
            video.set(CAP_PROP_FPS, config.target_fps);
        } else {
            cerr << "[Error] Camera open failed on ID " << video_id << endl;
        }
    }

    // 返回 false 表示失败
    bool video_capture(Mat& img) {
        if (!video.isOpened()) {
            // 尝试重连
            // open_camera(); 
            return false;
        }
        if (!video.read(img)) {
            cerr << "[Warn] Frame empty" << endl;
            return false;
        }
        return true;
    }

    // 优化的欧拉角转换
    // output: [0]=Pitch(X), [1]=Yaw(Y), [2]=Roll(Z)
    // 机器人导航主要关注 Yaw (output[1])
    void fastRvecToEuler(const Vec3d& rvec, Vec3d& euler) {
        Mat rotMat;
        Rodrigues(rvec, rotMat);
        
        double* R = (double*)rotMat.data; // 直接访问数据指针加速
        // R 布局:
        // 0  1  2
        // 3  4  5
        // 6  7  8
        
        double sy = std::sqrt(R[0]*R[0] + R[3]*R[3]);
        bool singular = sy < 1e-6;

        if (!singular) {
            euler[0] = std::atan2(R[7], R[8]);      // Pitch (X轴旋转)
            euler[1] = std::atan2(-R[6], sy);       // Yaw (Y轴旋转 - 关键)
            euler[2] = std::atan2(R[3], R[0]);      // Roll (Z轴旋转)
        } else {
            euler[0] = std::atan2(-R[5], R[4]);
            euler[1] = std::atan2(-R[6], sy);
            euler[2] = 0;
        }
    }
};

// ==========================================
// 图像预处理类
// ==========================================
class VideoProcess {
public:
    enum Method { NONE = 0, GAUSS = 1, BLUR = 2 };

    // 复用内存，避免重复分配
    void preprocess(const Mat& input, Mat& output, int method) {
        if (input.channels() == 3) {
            cvtColor(input, output, COLOR_BGR2GRAY);
        } else {
            input.copyTo(output);
        }

        switch (method) {
            case GAUSS:
                GaussianBlur(output, output, Size(3, 3), 1);
                break;
            case BLUR:
                blur(output, output, Size(3, 3));
                break;
            default:
                break;
        }
    }
};

// ==========================================
// 检测算法类
// ==========================================
class Detect {
private:
    Ptr<aruco::Dictionary> dictionary;
    Ptr<aruco::DetectorParameters> detectorParams;

public:
    Detect() {
        dictionary = aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        detectorParams = cv::aruco::DetectorParameters::create();
        
        // 优化检测参数
        detectorParams->adaptiveThreshWinSizeMin = 3;
        detectorParams->adaptiveThreshWinSizeMax = 23;
        detectorParams->adaptiveThreshWinSizeStep = 10;
        detectorParams->minMarkerPerimeterRate = 0.03; 
        detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX; 
        detectorParams->cornerRefinementWinSize = 5;
    }

    // 返回是否检测到有效目标
    bool detect_and_filter(const Mat& input, vector<int>& ids, vector<vector<Point2f>>& corners, int& command_type) {
        // 清空容器
        ids.clear();
        corners.clear();
        
        vector<int> raw_ids;
        vector<vector<Point2f>> raw_corners;

        aruco::detectMarkers(input, dictionary, raw_corners, raw_ids, detectorParams);

        if (raw_ids.empty()) return false;

        // 筛选逻辑
        command_type = 1; // 默认为 1 (Follow)
        int best_idx = -1;
        
        // 优先级策略：优先找 ID 对应 target_ids != 0 的，如果有特殊指令(非1)优先执行
        for (size_t i = 0; i < raw_ids.size(); ++i) {
            int current_id = raw_ids[i];
            
            // 安全检查，防止 ID 越界
            if (current_id < 0 || current_id >= config.target_id_map.size()) continue;
            
            int type = config.target_id_map[current_id];
            
            if (type == 0) continue; // 忽略此 ID

            // 发现特殊指令 (比如停靠)，立即锁定
            if (type != 1) {
                best_idx = i;
                command_type = type;
                break; // 找到高优先级指令，跳出
            }
            
            // 如果还没找到特殊指令，先暂存这个普通的
            if (best_idx == -1) {
                best_idx = i;
                command_type = 1;
            }
        }

        if (best_idx != -1) {
            ids.push_back(raw_ids[best_idx]);
            corners.push_back(raw_corners[best_idx]);
            return true;
        }

        return false;
    }

    void solve_pose(const vector<vector<Point2f>>& corners, vector<Vec3d>& rvecs, vector<Vec3d>& tvecs, Mat& debug_img, bool draw = false) {
        aruco::estimatePoseSingleMarkers(corners, config.markerLength, config.camera_matrix, config.distortion_coeffs, rvecs, tvecs);
        
        if (draw) {
            for (size_t i = 0; i < rvecs.size(); ++i) {
                aruco::drawAxis(debug_img, config.camera_matrix, config.distortion_coeffs, rvecs[i], tvecs[i], config.markerLength * 1.5);
            }
            aruco::drawDetectedMarkers(debug_img, corners);
        }
    }
};

// ==========================================
// 导航控制类 (核心修复)
// ==========================================
class Navigate {
private:
    // 限幅函数
    double clamp(double val, double max_val) {
        if (val > max_val) return max_val;
        if (val < -max_val) return -max_val;
        return val;
    }

public:
    // 计算 P 控制速度
    // 输入: tvecs (相机坐标系), corners (像素坐标)
    // 输出: output (机器人坐标系 CMD_X, CMD_Z, CMD_W)
    void calculate_control(
        const vector<Vec3d>& tvecs, 
        const vector<vector<Point2f>>& corners,
        vector<double>& output,
        const int command_type
    ) {
        // 重置输出
        fill(output.begin(), output.end(), 0.0);

        if (tvecs.empty()) return;

        // 获取相机坐标系下的位姿
        // OpenCV: X(右), Y(下), Z(前/深)
        double cam_x = tvecs[0][CV_X];
        double cam_z = tvecs[0][CV_Z]; 
        
        // 1. 计算像素中心 
        Point2f center_pixel(0, 0);
        for(const auto& p : corners[0]) center_pixel += p;
        center_pixel *= 0.25f; // 除以4求平均
        
        double pixel_error_x = config.view_width / 2.0 - center_pixel.x; // 正值表示目标在左边(需左转)

        // ----------------------------------------------------
        // 核心控制逻辑 (P-Controller)
        // ----------------------------------------------------

        // A. 旋转控制 (Align Yaw)
        // 目标在图像左侧 -> pixel_error_x > 0 -> 需要向左旋转(正角速度)
        // 目标在图像右侧 -> pixel_error_x < 0 -> 需要向右旋转(负角速度)
        if (std::abs(pixel_error_x) > config.rotate_deadzone_pixel) {
            // 归一化误差到 -1~1 之间
            double norm_error = pixel_error_x / (config.view_width / 2.0);
            output[CMD_ROTATION] = clamp(norm_error * config.kp_rotate, config.max_speed_angular);
        }

        // B. 前后移动控制 (Distance Keep)
        // 距离误差: 实际深度 - 期望距离
        double dist_error = cam_z - config.follow_distance;
        
        if (std::abs(dist_error) > config.follow_deadzone) {
            // 误差 > 0 (太远) -> 需要正向速度 (前进)
            // 误差 < 0 (太近) -> 需要负向速度 (后退)
            output[CMD_FORWARD] = clamp(dist_error * config.kp_forward, config.max_speed_linear);
        }

        // C. 横向对齐控制 (Lateral Align) - 仅在 Docking 模式或偏差大时启用
        // cam_x > 0 (目标在相机右侧) -> 机器人需要向右平移 (CMD_LATERAL > 0)
        // 注意：这里假设底盘坐标系 X 正方向也是右
        if (command_type != 1) { // 特殊模式下加强横移对齐
             output[CMD_LATERAL] = clamp(cam_x * config.kp_lateral, config.max_speed_linear);
        } else {
             // 跟随模式主要靠旋转对齐，横移作为辅助
             // 只有当旋转也无法对齐时(比如纯平移)才启用，或者设为0
             output[CMD_LATERAL] = cam_x * 0.5 * config.kp_lateral; 
        }

        // ----------------------------------------------------
        // 特殊指令处理 (模拟原代码逻辑)
        // ----------------------------------------------------
        if (command_type == 2) {
             // 示例: 强制旋转
             output[CMD_ROTATION] = 0.5; 
             output[CMD_FORWARD] = 0;
             output[CMD_LATERAL] = 0;
        }
        else if (command_type == 3) {
             output[CMD_ROTATION] = -0.5;
             output[CMD_FORWARD] = 0;
             output[CMD_LATERAL] = 0;
        }
        else if (command_type == 4) {
             // 停止
             fill(output.begin(), output.end(), 0.0);
        }
    }

    void spin_search(vector<double>& output) {
        output[CMD_LATERAL] = 0;
        output[CMD_FORWARD] = 0;
        output[CMD_ROTATION] = config.search_speed;
    }
};


//==========================================
//串口通信类
//===========================================
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


// ==========================================
// 主工作模式类
// ==========================================
class Mode {
private:
    std::unique_ptr<Operate> operate;
    std::unique_ptr<VideoProcess> video_process;
    std::unique_ptr<Detect> detect;
    std::unique_ptr<Navigate> navigate;
    std::unique_ptr<SerialPort> serial;

public:
    Mode() {
        operate = std::make_unique<Operate>();
        video_process = std::make_unique<VideoProcess>();
        detect = std::make_unique<Detect>();
        navigate = std::make_unique<Navigate>();
        serial = std::make_unique<SerialPort>();
        serial->init(config.usb_name, 115200); // 根据实际情况修改串口号
    }

    // 核心任务循环
    int run_navigation(bool visual_debug = true) {
        if (visual_debug) {
            namedWindow("View", WINDOW_NORMAL);
            resizeWindow("View", 1920, 1080); // 缩小显示窗口以节省资源
        }

        Mat frame_raw, frame_gray;
        vector<int> ids;
        vector<vector<Point2f>> corners;
        vector<Vec3d> rvecs, tvecs;
        
        // 速度输出: [0]=X, [1]=Z, [2]=W
        vector<double> speed_cmd(3, 0.0); 
        int command_type = 1;

        cout << "System Start. Press 'q' to quit." << endl;

        while (true) {
            auto loop_start = high_resolution_clock::now();

            // 1. 获取图像
            if (!operate->video_capture(frame_raw)) {
                // 读取失败缓冲
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // 2. 预处理
            video_process->preprocess(frame_raw, frame_gray, VideoProcess::GAUSS);

            // 3. 检测
            bool detected = detect->detect_and_filter(frame_gray, ids, corners, command_type);

            if (detected) {
                // 4. 位姿解算
                detect->solve_pose(corners, rvecs, tvecs, frame_raw, visual_debug);

                // 5. 计算控制量 (修复后的逻辑)
                navigate->calculate_control(tvecs, corners, speed_cmd, command_type);

                // Debug print
                if (visual_debug) {
                    // 使用 OpenCV 坐标索引 CV_Z 来显示距离
                    printf("ID: %d | Type: %d | Dist: %.2fm | CMD -> X:%.2f Z:%.2f W:%.2f\n", 
                           ids[0], command_type, tvecs[0][CV_Z], 
                           speed_cmd[CMD_LATERAL], speed_cmd[CMD_FORWARD], speed_cmd[CMD_ROTATION]);
                }

                //正方向：x：水平向右，z：垂直向前，w：逆时针自转

            } 
            else {
                // 6. 丢失目标 -> 搜索模式
                navigate->spin_search(speed_cmd);
                if (visual_debug) cout << "Searching..." << endl;
            }

            // TODO: 在这里通过串口发送 speed_cmd 给底盘
            if(!visual_debug)
            serial->send_command(speed_cmd[0], speed_cmd[1], speed_cmd[2]);
            // send_serial(speed_cmd);

            // 7. 显示与延时控制
            if (visual_debug) {
                imshow("View", frame_raw);
                if (waitKey(1) == 'q') break;
            }

            // 8. 帧率控制
            auto loop_end = high_resolution_clock::now();
            double elapsed_ms = duration<double, std::milli>(loop_end - loop_start).count();
            double sleep_ms = config.frame_time_ms - elapsed_ms;
            
            if (sleep_ms > 0) {
                // 如果显示开启，waitKey已经充当了延时
                if (!visual_debug) {
                    std::this_thread::sleep_for(std::chrono::milliseconds((int)sleep_ms));
                }
            }
        }

        destroyAllWindows();
        return 0;
    }
};

int main() {
    cv::setNumThreads(4); // 限制线程数，避免占用过多 CPU 资源给控制逻辑
    
    try {
        Mode system_mode;
        system_mode.run_navigation(true); // true 开启可视化调试
    } catch (const std::exception& e) {
        cerr << "Fatal Error: " << e.what() << endl;
        return -1;
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