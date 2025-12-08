#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>

using namespace cv;
using namespace std;

// -------------------------- 配置参数（可根据需求修改）--------------------------
const int CHESSBOARD_COLS = 8;    // 棋盘格横向内角点数量（必须比实际格子数少1）
const int CHESSBOARD_ROWS = 5;    // 棋盘格纵向内角点数量（必须比实际格子数少1）
const float SQUARE_SIZE_CM = 3.15f; // 棋盘格单个方格的实际边长（单位：厘米，可自定义）
const int IMG_WIDTH = 1920;       // 生成棋盘格图像的宽度（像素）
const int IMG_HEIGHT = 1080;      // 生成棋盘格图像的高度（像素）
const string CHESSBOARD_SAVE_PATH = "../result/chessboard.png"; // 棋盘格保存路径
const int CALIB_IMAGE_COUNT = 20;  // 标定所需的有效棋盘格图像数量（建议15-30张）
// -----------------------------------------------------------------------------

/**
 * @brief 生成自定义尺寸的棋盘格图像（实际尺寸由 SQUARE_SIZE_CM 定义）
 * @return 成功返回true，失败返回false
 */
bool generateChessboard() {
    // 1. 计算棋盘格实际总尺寸（厘米）
    float total_height_cm = (CHESSBOARD_ROWS +1)* SQUARE_SIZE_CM;
    float total_width_cm = total_height_cm;
    cout << "[生成棋盘格] 单个方格尺寸：" << SQUARE_SIZE_CM << "cm | "
         << "总尺寸：" << total_width_cm << "cm × " << total_height_cm << "cm" << endl;

    // 2. 创建空白图像（白色背景）
    Mat chessboard = Mat::ones(IMG_HEIGHT, IMG_WIDTH, CV_8UC1) * 255;

    // 3. 计算每个方格在图像中的像素尺寸
    
    int square_pixel_h = IMG_HEIGHT / CHESSBOARD_ROWS;
    int square_pixel_w = square_pixel_h;

    // 4. 绘制棋盘格（黑白交替）
    for (int i = 0; i < CHESSBOARD_ROWS +1; ++i) {
        for (int j = 0; j < CHESSBOARD_COLS+1; ++j) {
            // 奇偶行交替上色（黑色：0，白色：255）
            if ((i + j) % 2 == 0) {
                Rect square_roi(j * square_pixel_w, i * square_pixel_h, square_pixel_w, square_pixel_h);
                chessboard(square_roi) = 0; // 黑色方格
            }
        }
    }

    // 5. 保存棋盘格图像
    if (!imwrite(CHESSBOARD_SAVE_PATH, chessboard)) {
        cerr << "[错误] 无法保存棋盘格图像到：" << CHESSBOARD_SAVE_PATH << endl;
        return false;
    }
    cout << "[成功] 棋盘格已保存到：" << CHESSBOARD_SAVE_PATH << endl;
    cout << "[提示] 请打印该图像（确保打印尺寸与 " << SQUARE_SIZE_CM << "cm 一致，无缩放）" << endl;

    // 显示棋盘格（可选）
    imshow("Generated Chessboard", chessboard);
    waitKey(3000); // 显示3秒
    destroyWindow("Generated Chessboard");
    return true;
}

/**
 * @brief 摄像头标定（基于打印的棋盘格，使用实际尺寸计算内参）
 * @return 成功返回true，失败返回false
 */
bool calibrateCamera() {
    // 1. 定义存储变量
    vector<vector<Point3f>> object_points; // 世界坐标系中的3D点（真实尺寸）
    vector<vector<Point2f>> image_points;  // 图像坐标系中的2D点（像素）
    vector<Point3f> chessboard_corners_3d; // 单个棋盘格的3D内角点
    vector<Point2f> chessboard_corners_2d; // 单个棋盘格的2D内角点

    // 2. 初始化世界坐标系3D点（Z轴为0，X/Y为实际尺寸，单位：厘米）
    for (int i = 0; i < CHESSBOARD_ROWS; ++i) {
        for (int j = 0; j < CHESSBOARD_COLS; ++j) {
            // (j*方格尺寸, i*方格尺寸, 0)：左上角为原点，向右为X轴，向下为Y轴
            chessboard_corners_3d.emplace_back(j * SQUARE_SIZE_CM, i * SQUARE_SIZE_CM, 0.0f);
        }
    }

    // 3. 打开摄像头（默认摄像头，索引0）
    VideoCapture cap(4);
    if (!cap.isOpened()) {
        cerr << "[错误] 无法打开摄像头！" << endl;
        return false;
    }
    int target_width = 1920;  // 目标宽度（如 640、1280、1920）
    int target_height = 1080;  // 目标高度（如 480、720、1080）
    cap.set(CAP_PROP_FRAME_WIDTH, target_width);   // 设置宽度
    cap.set(CAP_PROP_FRAME_HEIGHT, target_height); // 设置高度
    cout << "[提示] 摄像头已打开，请移动棋盘格到不同角度/位置（需要 " << CALIB_IMAGE_COUNT << " 张有效图像）" << endl;
    cout << "[操作] 按 's' 保存当前帧，按 'q' 退出标定" << endl;

    int valid_image_count = 0;
    Mat frame, gray;

    while (true) {
        cap >> frame;
        if (frame.empty()) {
            cerr << "[警告] 无法读取摄像头画面" << endl;
            break;
        }

        // 4. 转换为灰度图（角点检测需要）
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        // 5. 检测棋盘格内角点（CV_CALIB_CB_ADAPTIVE_THRESH：自适应阈值，提高检测鲁棒性）
        bool corners_found = findChessboardCorners(
            gray, Size(CHESSBOARD_COLS , CHESSBOARD_ROWS ),
            chessboard_corners_2d,
            CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK
        );

        // 6. 若检测到角点，亚像素优化并绘制
        if (corners_found) {
            // 亚像素级角点优化（提高标定精度）
            cornerSubPix(
                gray, chessboard_corners_2d, Size(11, 11), Size(-1, -1),
                TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.001)
            );
            // 在图像上绘制角点和棋盘格轮廓
            drawChessboardCorners(frame, Size(CHESSBOARD_COLS, CHESSBOARD_ROWS), chessboard_corners_2d, true);
        }

        // 显示画面
        imshow("Camera Calibration (Press 's' to save, 'q' to quit)", frame);

        // 键盘事件处理
        char key = waitKey(1);
        if (key == 'q') {
            cout << "[退出] 用户主动退出标定" << endl;
            break;
        } else if (key == 's' && corners_found) {
            // 保存有效帧的3D/2D点
            object_points.push_back(chessboard_corners_3d);
            image_points.push_back(chessboard_corners_2d);
            valid_image_count++;
            cout << "[进度] 已保存 " << valid_image_count << "/" << CALIB_IMAGE_COUNT << " 张有效图像" << endl;

            // 收集够足够图像后开始标定
            if (valid_image_count >= CALIB_IMAGE_COUNT) {
                cout << "[开始] 收集到足够图像，开始标定..." << endl;
                break;
            }
        }
    }

    // 释放摄像头和窗口
    cap.release();
    destroyAllWindows();

    // 若有效图像不足，退出
    if (valid_image_count < CALIB_IMAGE_COUNT) {
        cerr << "[错误] 有效图像数量不足（仅 " << valid_image_count << " 张），标定失败" << endl;
        return false;
    }

    // 7. 执行摄像头标定
    Mat camera_matrix;       // 内参矩阵 [fx, 0, cx; 0, fy, cy; 0, 0, 1]
    Mat distortion_coeffs;   // 畸变系数 [k1, k2, p1, p2, k3]
    vector<Mat> rvecs, tvecs; // 旋转向量、平移向量（每帧的外参）
    double reproj_error;     // 重投影误差（越小精度越高）

    reproj_error = calibrateCamera(
        object_points, image_points, gray.size(),
        camera_matrix, distortion_coeffs,
        rvecs, tvecs,
        CALIB_FIX_K4 | CALIB_FIX_K5 // 固定k4、k5畸变系数（默认不考虑高阶畸变）
    );

    // 8. 输出标定结果
    cout << "\n==================== 标定结果 ====================" << endl;
    cout << "重投影误差（越小越好）：" << reproj_error << " 像素" << endl;
    cout << "\n内参矩阵（Camera Matrix）：" << endl;
    cout << camera_matrix << endl;
    cout << "\n畸变系数（Distortion Coefficients）：" << endl;
    cout << distortion_coeffs << endl;
    cout << "==================================================" << endl;

    // 9. 保存标定结果到文件（便于后续使用）
    FileStorage fs("../result/camera_calib_result.yaml", FileStorage::WRITE);
    if (fs.isOpened()) {
        fs << "camera_matrix" << camera_matrix;
        fs << "distortion_coeffs" << distortion_coeffs;
        fs << "reprojection_error" << reproj_error;
        fs << "square_size_cm" << SQUARE_SIZE_CM;
        fs.release();
        cout << "\n[成功] 标定结果已保存到：camera_calib_result.yaml" << endl;
    } else {
        cerr << "[警告] 无法保存标定结果到文件" << endl;
    }

    return true;
}

int main(int argc, char** argv) {
    cout << "==================== 摄像头标定工具 ====================" << endl;
    cout << "1. 首先生成棋盘格（实际方格尺寸：" << SQUARE_SIZE_CM << "cm）" << endl;
    cout << "2. 打印棋盘格后，进行摄像头标定" << endl;
    cout << "=======================================================" << endl;

    cout << endl << endl <<endl;
    cout <<"选项： " 
    << endl 
    <<" 1、 生成棋盘格"
    << endl;
    cout <<" 2、 开始标定"
    <<endl;
    cout << "选择操作 (1或2)： " << endl<< endl;
    int choise;
    cin >> choise;
    
    switch(choise)
    {
        case 1:
        if (!generateChessboard())
        {
        return -1;
        }
        cout << "\n[提示] 请打印生成的棋盘格（确保无缩放，方格实际尺寸为 " << SQUARE_SIZE_CM << "cm）" << endl;
        cin.get(); // 等待用户按键

        break;

        case 2:
        if (!calibrateCamera()) 
        {
        return -1;
        }
        break;

        
        default:
        cout <<"无效选项： " << choise <<" 已终止"<<endl;
        return -1;
            
    }
   
    
   

    cout << "\n[完成] 标定流程结束！" << endl;
    return 0;
}
