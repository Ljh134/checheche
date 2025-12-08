#include <iostream>
#include<opencv2/opencv.hpp>
#include<filesystem>

using namespace std;
using namespace cv;
namespace fs = std::filesystem;

fs::path base_path = "../resource";
fs::path file_name = "robotvideo.mp4";
fs::path file_path = base_path / file_name;

int main ()
{
    VideoCapture video(4);
    if (video.isOpened())
    {
        // 2. 设置目标分辨率（根据摄像头支持的尺寸修改，推荐常用尺寸）
    int target_width = 1920;  // 目标宽度（如 640、1280、1920）
    int target_height = 1080;  // 目标高度（如 480、720、1080）
    video.set(CAP_PROP_FRAME_WIDTH, target_width);   // 设置宽度
    video.set(CAP_PROP_FRAME_HEIGHT, target_height); // 设置高度

        cout << "视频中图像的宽度=" << video.get(CAP_PROP_FRAME_WIDTH) << endl;
        cout << "视频中图像的高度=" << video.get(CAP_PROP_FRAME_HEIGHT) << endl;
        cout << "视频帧率=" << video.get(CAP_PROP_FPS) << endl;
        cout << "视频的总帧数=" << video.get(CAP_PROP_FRAME_COUNT);
    }
    else
    {
        cout << "请确认视频文件名称是否正确" << endl;
        return -1;
    }
     while (1)
    {
        Mat frame;
        video >> frame;
        if (frame.empty())
        {
            break;
        }
        imshow("video", frame);
        waitKey(1000 / video.get(CAP_PROP_FPS));
    }
    waitKey(1000);
    destroyAllWindows();
    return 0;
}
