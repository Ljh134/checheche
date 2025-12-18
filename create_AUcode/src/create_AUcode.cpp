#include"iostream"
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/aruco.hpp>
#include "vector"
#include "string"

using namespace std;
using namespace cv;
using namespace cv::aruco;


class Create_AUcode
{
    private:
    Ptr<Dictionary> dictionary_type 
    = getPredefinedDictionary(DICT_6X6_250);
    int sidePixels = 200;          // 原始标记尺寸（像素）
    int borderBits = 1;            // 标记内部的黑色边框（ArUco标准）
    int whiteBorderWidth = 50 ;     // 自定义的白色外边框宽度（像素，可调整）
    Scalar borderColor = Scalar(0);

    public:
    void create(string save_path , int l_id , int r_id)
    {
        Mat res, markerWithWhiteBorder;
        string save_path1 ;
        
        for(int i =l_id ;i<= r_id;i++)
        {
            // 1. 生成原始ArUco标记（黑框白底的标记图案）
            drawMarker(
                dictionary_type,
                i,
                sidePixels,
                res,
                borderBits
            );

            // 2. 创建带白色边框的画布：尺寸 = 原始尺寸 + 2×白色边框宽度
            int newSize = sidePixels + 2 * whiteBorderWidth;
            markerWithWhiteBorder = Mat::ones(newSize, newSize, CV_8UC1) * 255; // 全白画布

            // 3. 将原始标记居中复制到白色画布上
            res.copyTo(markerWithWhiteBorder(Rect(whiteBorderWidth, whiteBorderWidth, sidePixels, sidePixels)));

            // 4. 保存带白色边框的标记
            save_path1 = save_path + to_string(i) + ".jpg";
            if(imwrite(save_path1, markerWithWhiteBorder))
            {
                cout << "已保存带白色边框的标记 id=" << i <<endl;
            }
            else{
                cout << "error: 保存标记错误 id = " << i <<"请检查" <<endl;
            }
        }
    }
};

/*

class Create_AUcode
{
    private:
    Ptr<Dictionary> dictionary_type 
    = getPredefinedDictionary(DICT_6X6_250);
    int sidePixels = 900;
    int borderBits = 2;
    Scalar borderColor = Scalar(0);

    public:
    void create(string save_path , int l_id , int r_id)
    {
        Mat res;//=Mat::zeros(400, 400, CV_8UC3) 
        //res = 255;
        Mat show ; 
        int n = r_id - l_id +1;
        string save_path1 ;
        for(int i =l_id ;i<= r_id;i++)
        {
            drawMarker(
                dictionary_type,
                i,
                sidePixels,
                res,
                borderBits
                );
            save_path1 = save_path + to_string(i) + ".png";
            if(imwrite(save_path1,res))
            {
                cout << "已保存标记 id=" << i <<endl;
            }
            else{
                cout << "error: 保存标记错误 id = " << i
                <<"请检查" <<endl;
            }
        }
    }

    // bool show_marker(string save_path , int l_id , int r_id)
    // {
    //     imshow
    // }
};

*/

int main()
{
    string sp = "../result/AUmarker";
    int li =0,ri =0;
    cout << "输入创建开始ID,结束ID:" << endl;
    cin >>li;
    cin >>ri;
    cin.get();
    Create_AUcode create;
    create.create(sp, li , ri);
    return 0;
}