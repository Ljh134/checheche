#include <iostream>
#include <iomanip>

using namespace std;

int main()
{
    int i;
    cout << "cal? (y)";
    //i = getchar();
    while (getchar() == 'y')
    {
        cin.get();
        double a, b, c, n = 3;
        cin >> a;
        cin >> b;
        cin >> c;
        // 输出时明确设置小数位数：例如保留 6 位小数
        cout << fixed << setprecision(14) << (a + b + c) / n;
        cin.get();
        // i = getchar();
        // cout << "cal? (y)";
    }
    return 0;

}