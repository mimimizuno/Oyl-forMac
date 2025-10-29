#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP
#include <cmath>
// 定数をまとめておくファイル

constexpr double e = 0.1602;     // 電荷量
constexpr int leg0 = 0;          // 足の数の定義
constexpr int leg1 = 1;
constexpr int leg2 = 2;
constexpr int leg3 = 3;
constexpr int leg4 = 4;
constexpr int leg5 = 5;
constexpr int leg6 = 6;
constexpr int leg7 = 7;
constexpr int leg8 = 8;
constexpr int leg9 = 9;
constexpr int leg10 = 10;
constexpr double Cj_leg1 = 18.0; // 足1振動子のトンネル容量[aF]
constexpr double Cj_leg2 = 16.0; // 足2振動子のトンネル容量[aF]
constexpr double Cj_leg3 = 14.0; // 足3振動子のトンネル容量[aF]
constexpr double Cj_leg4 = 12.0; // 足4振動子のトンネル容量[aF]
constexpr double Cj_leg5 = 10.0; // 足5振動子のトンネル容量[aF]
constexpr double Cj_leg6 = 8.0;  // 足6振動子のトンネル容量[aF]
constexpr double multi_Cj = 390; // 20重振動子のトンネル容量[aF]

// 閾値電圧に合わせて、足の数、接合容量から振動子のCjの値を返す関数
inline int seo_junction_cj_calc(int leg, double C, double Vth){
    return static_cast<int>((e / (2 * Vth)) - leg * C);
}

// 閾値電圧に合わせて、多重数、足の数、接合容量から多重振動子のCjの値を返す関数(sample参照)
inline int multi_junction_cj_calc(int multi_junction_num, int leg, double C, double Vth){
    double Cj_first;
    double formula_item_a = 2 * Vth;
    double formula_item_b = 2 * Vth * leg * multi_junction_num * C - multi_junction_num * e;
    double formula_item_c = - multi_junction_num * e * leg * (multi_junction_num - 1) * C;
    double contens_in_root = pow(formula_item_b, 2.0) - 4 * formula_item_a * formula_item_c; 
    Cj_first = (-formula_item_b + pow(contens_in_root, 0.5)) / (2 * formula_item_a);
    return Cj_first;
}

// pulse波を生成する
inline double getRectangularV(double t, double V_amp, double t_on, double t_off) {
    double period = t_on + t_off;
    return (std::fmod(t, period) < t_on) ? V_amp : 0.0;
}

// 隣接振振動子のトンネルで変化する電圧
inline double tunnelV(double C, int myleg, int yourleg, double myCj, double yourCj)
{
    return ((C * e) / ((myleg * C + myCj) * (yourleg * C + yourCj)));
}

// 隣接振振動子のトンネルで変化する電圧
inline double multi_tunnelV(double C, int myleg, double myCj, double yourCj, int multi_num, double ratio=0.8)
{
    return ratio * ((C * multi_num * multi_num * e) / ((myleg * multi_num * C + myCj) * (myleg * multi_num * C + yourCj)));
}

#endif // CONSTANTS_HPP
