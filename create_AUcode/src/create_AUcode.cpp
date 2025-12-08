#include"iostream"
#include "opencv2/opencv.hpp"
#include <opencv2/aruco.hpp>
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