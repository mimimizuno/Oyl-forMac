#ifndef MEMORY_CLASS_HPP
#define MEMORY_CLASS_HPP

#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <cmath>
#include <stdexcept>
#include <random>
#include <memory>
#include "base_element.hpp"
#include "constants.hpp"

class MEMORY : public BaseElement {
private:
    double Vn;                                              // ノード電圧
    double Vd;                                              // バイアス電圧
    double Rj;                                              // トンネル抵抗
    double Cj;                                              // 接合容量
    double C;                                               // 接続容量
    double CL;                                              // 上部の容量
    int legs;                                               // 足の数
    int upper_electrons;                                    // 上側の電子数
    int lower_electrons;                                    // 下側の電子数
    double V_sum;                                           // 周囲のノード電圧の総和
    std::map<std::string, double> dE;                       // エネルギー変化量(up1,up2,down1,down2)
    std::map<std::string, double> wt;                       // トンネル待時間(up1,up2,down1,down2)
    std::vector<std::shared_ptr<BaseElement>> connections;  // 接続されている素子のポインタ

public:
    //-----------コンストラクタ---------// 
    // vectorの初期化用
    MEMORY();
    // 引数あり初期設定用
    MEMORY(double rj, double cj, double c, double cl, double vd, int legscounts);

    //-----------セッター------------//
    // パラメータセットアップ
    void setUp(double rj, double cj, double c, double cl, double vd, int legscounts);

    // バイアス電圧を設定
    void setVias(const double vd) override;

    // V_sumを設定
    void setVsum(double v) override;

    void setConnections(const std::vector<std::shared_ptr<BaseElement>>& conns) override;

    // 周囲の電圧を設定
    void setSurroundingVoltages() override;

    // 振動子のパラメータ計算
    void setPcalc() override;

    // 振動子のエネルギー計算
    void setdEcalc() override;

    // 電荷の更新
    void setNodeCharge(const double dt) override;

    // トンネル待ち時間計算(upまたはdownが正の時にwtを計算してtrueを返す)
    bool calculateTunnelWt() override;

    // 振動子のトンネル
    void setTunnel(const std::string& direction) override;

    //-----------ゲッター------------//    
    // ノード電圧を取得
    double getVn() const override;

    // 接続されてる振動子の電圧の総和を取得
    double getSurroundingVsum() const override;

    // dEの取得
    std::map<std::string, double> getdE() const override;

    // Qの取得
    double getQ() const override;

    // Rゲッター
    double getR() const override;
    
    // Rjゲッター
    double getRj() const override;
    
    // Cjゲッター
    double getCj() const override;
    
    // Cゲッター
    double getC() const override;

    // Vdゲッター
    double getVd() const override;
    
    // legsゲッター
    int getlegs() const override;

    // wtの取得
    std::map<std::string, double> getWT() const override;

    // oneway用の関数のため呼び出すとエラー発生
    std::shared_ptr<BaseElement> getInternalElement(int index) const override;

    //-------- 汎用処理 -------------//
    // 0から1の間の乱数を生成
    double Random();

    // テスト用dEセッター
    void setdE(const std::string& direction, double value);

    // テスト用Vnセッター
    void setVn(double vn);
};

#endif // SEO_HPP
