// seo particle computation test
// --------------------------------------------------------------------------------
#include <iostream>
#include <fstream>
#include <memory>
#include <vector>
#include "seo_class.hpp"
#include "oneway_unit.hpp"
#include "oneway_unit_6.hpp"
#include "oneway_and.hpp"
#include "grid_2dim.hpp"
#include "simulation_2d.hpp"
#include "oyl_video.hpp"
#include "constants.hpp"
#include "particle_computation_methods.hpp"

constexpr int particles = 2;
constexpr int size_x = 17;
constexpr int size_y = 11;
constexpr double Vd_seo = 0.0039;
constexpr double Vd_oneway = 0.0039;
constexpr double Vd_adjust = 0.0036;
constexpr double R = 1;
constexpr double R_detec = 0.5;
constexpr double R_small = 0.8;
constexpr double Rj = 0.0005;
constexpr double C = 2;
constexpr double C_detec = 1;
constexpr double Cj_detec = 12;
constexpr double dt = 0.1;
constexpr double endtime = 300;
constexpr double setVth = 0.004;

double cj_leg2 = seo_junction_cj_calc(leg2, C, setVth);
double cj_leg3 = seo_junction_cj_calc(leg3, C, setVth);
double cj_leg4 = seo_junction_cj_calc(leg4, C, setVth);
double cj_leg5 = seo_junction_cj_calc(leg5, C, setVth);
double cj_leg6 = seo_junction_cj_calc(leg6, C, setVth);
constexpr int repeat_count = 1;
constexpr int junction_num_max = 5;
constexpr double max_pulse_voltage = 0;

using Grid = Grid2D<BaseElement>;
using Sim = Simulation2D<BaseElement>;

int main()
{
    std::vector<std::vector<int>> maze = {
        {0,0,0,0,0,0,0,1,0,1,0,1,0,1,0},
        {0,0,1,0,0,0,0,1,0,1,0,1,0,1,0},
        {0,1,1,1,1,0,0,1,0,1,0,1,0,1,0},
        {0,1,1,1,1,1,1,1,1,1,1,1,1,1,0},
        {0,0,1,0,1,0,0,0,0,0,0,1,0,1,0},
        {0,0,1,0,1,0,0,1,0,0,0,1,0,1,0},
        {0,0,1,0,1,0,1,1,1,1,0,1,0,1,0},
        {0,0,1,0,1,0,1,1,1,1,1,1,1,1,0},
        {0,0,1,0,1,0,0,1,0,1,0,0,0,0,0},
    };

    for(double pulse_voltage = 0; pulse_voltage <= max_pulse_voltage; pulse_voltage+=0.001){

        double Vd_detec = Vd_seo - 0.64 * tunnelV(C,leg5,leg3,cj_leg5,cj_leg3);

        std::vector<std::vector<double>> allResults;
        for(int trial = 0; trial < repeat_count; trial++){
            std::cout << "multi_num = " << pulse_voltage << " trial = " << trial << std::endl;
            // === Gridを生成 ===
            Grid command_downA(size_y, size_x); // 命令方向回路A（下）
            Grid command_downB(size_y, size_x); // 命令方向回路B（下）
            Grid detection_down(size_y, size_x); // 衝突方向回路（下）
            Grid command_leftA(size_y, size_x); // 命令方向回路A（左）
            Grid command_leftB(size_y, size_x); // 命令方向回路A（左）
            Grid detection_left(size_y, size_x); // 衝突方向回路（左）
            Grid command_upA(size_y, size_x); // 命令方向回路A（上）
            Grid command_upB(size_y, size_x); // 命令方向回路B（上）
            Grid detection_up(size_y, size_x); // 衝突方向回路（上）  
            Grid command_rightA(size_y, size_x); // 命令方向回路A（右）
            Grid command_rightB(size_y, size_x); // 命令方向回路A（右）
            Grid detection_right(size_y, size_x); // 衝突方向回路（右）
            Grid oneway_command_downA(size_y, size_x, false);  // 命令方向回路（下）における一方通行回路A
            Grid oneway_command_downB(size_y, size_x, false);  // 命令方向回路（下）における一方通行回路B
            Grid oneway_CtoD_downA(size_y, size_x, false);     // 命令方向回路（下）から衝突判定回路（下）をつなぐ一方通行回路A
            Grid oneway_CtoD_downB(size_y, size_x, false);     // 命令方向回路（下）から衝突判定回路（下）をつなぐ一方通行回路B
            Grid oneway_DtoC_downtoleftA(size_y, size_x, false);   // 衝突判定回路（下）から命令方向回路（左）をつなぐ一方通行回路A
            Grid oneway_DtoC_downtoleftB(size_y, size_x, false);   // 衝突判定回路（下）から命令方向回路（左）をつなぐ一方通行回路B
            Grid oneway_command_leftA(size_y, size_x, false);  // 命令方向回路（左）における一方通行回路A
            Grid oneway_command_leftB(size_y, size_x, false);  // 命令方向回路（左）における一方通行回路B
            Grid oneway_CtoD_leftA(size_y, size_x, false);     // 命令方向回路（左）から衝突判定回路（左）をつなぐ一方通行回路A
            Grid oneway_CtoD_leftB(size_y, size_x, false);     // 命令方向回路（左）から衝突判定回路（左）をつなぐ一方通行回路B
            Grid oneway_DtoC_lefttoupA(size_y, size_x, false);   // 衝突判定回路（左）から命令方向回路（上）をつなぐ一方通行回路A
            Grid oneway_DtoC_lefttoupB(size_y, size_x, false);   // 衝突判定回路（左）から命令方向回路（上）をつなぐ一方通行回路B
            Grid oneway_command_upA(size_y, size_x, false);  // 命令方向回路（上）における一方通行回路A
            Grid oneway_command_upB(size_y, size_x, false);  // 命令方向回路（上）における一方通行回路B
            Grid oneway_CtoD_upA(size_y, size_x, false);     // 命令方向回路（上）から衝突判定回路（上）をつなぐ一方通行回路A
            Grid oneway_CtoD_upB(size_y, size_x, false);     // 命令方向回路（上）から衝突判定回路（上）をつなぐ一方通行回路B
            Grid oneway_DtoC_uptorightA(size_y, size_x, false);   // 衝突判定回路（上）から命令方向回路（右）をつなぐ一方通行回路A
            Grid oneway_DtoC_uptorightB(size_y, size_x, false);   // 衝突判定回路（上）から命令方向回路（右）をつなぐ一方通行回路B
            Grid oneway_command_rightA(size_y, size_x, false);  // 命令方向回路（右）における一方通行回路A
            Grid oneway_command_rightB(size_y, size_x, false);  // 命令方向回路（右）における一方通行回路B
            Grid oneway_CtoD_rightA(size_y, size_x, false);     // 命令方向回路（右）から衝突判定回路（右）をつなぐ一方通行回路A
            Grid oneway_CtoD_rightB(size_y, size_x, false);     // 命令方向回路（右）から衝突判定回路（右）をつなぐ一方通行回路B
            Grid oneway_DtoC_righttodownA(size_y, size_x, false);   // 衝突判定回路（右）から命令方向回路（下）をつなぐ一方通行回路A
            Grid oneway_DtoC_righttodownB(size_y, size_x, false);   // 衝突判定回路（右）から命令方向回路（下）をつなぐ一方通行回路B

            // 動画出力するgridに名前をつける
            command_downA.setOutputLabel("command_downA");
            command_downB.setOutputLabel("command_downB");
            detection_down.setOutputLabel("detection_down");
            command_leftA.setOutputLabel("command_leftA");
            command_leftB.setOutputLabel("command_leftB");
            detection_left.setOutputLabel("detection_left");
            command_upA.setOutputLabel("command_upA");
            command_upB.setOutputLabel("command_upB");
            detection_up.setOutputLabel("detection_up");
            command_rightA.setOutputLabel("command_rightA");
            command_rightB.setOutputLabel("command_rightB");
            detection_right.setOutputLabel("detection_right");


            // === 全グリッド初期化 ===
            // 命令方向回路・衝突判定回路
            for (int y = 0; y < command_downA.numRows(); ++y) {
                for (int x = 0; x < command_downA.numCols(); ++x) {
                    // ------------命令方向回路------------------------------------------------
                    // down_A
                    {
                        auto seo = std::make_shared<SEO>();
                        seo->setUp(R, Rj, cj_leg4, C, Vd_seo, leg4);
                        command_downA.setElement(y, x, seo);
                    }
                    // down_B
                    {
                        auto seo = std::make_shared<SEO>();
                        seo->setUp(R, Rj, cj_leg4, C, Vd_seo, leg4);
                        command_downB.setElement(y, x, seo);
                    }
                    // up_A
                    {
                        auto seo = std::make_shared<SEO>();
                        seo->setUp(R, Rj, cj_leg4, C, Vd_seo, leg4);
                        command_upA.setElement(y, x, seo);
                    }
                    // up_B
                    {
                        auto seo = std::make_shared<SEO>();
                        seo->setUp(R, Rj, cj_leg4, C, Vd_seo, leg4);
                        command_upB.setElement(y, x, seo);
                    }
                    // left_A
                    {
                        auto seo = std::make_shared<SEO>();
                        seo->setUp(R, Rj, cj_leg4, C, Vd_seo, leg4);
                        command_leftA.setElement(y, x, seo);
                    }
                    // left_B
                    {
                        auto seo = std::make_shared<SEO>();
                        seo->setUp(R, Rj, cj_leg4, C, Vd_seo, leg4);
                        command_leftB.setElement(y, x, seo);
                    }
                    // right_A
                    {
                        auto seo = std::make_shared<SEO>();
                        seo->setUp(R, Rj, cj_leg4, C, Vd_seo, leg4);
                        command_rightA.setElement(y, x, seo);
                    }
                    // right_B
                    {
                        auto seo = std::make_shared<SEO>();
                        seo->setUp(R, Rj, cj_leg4, C, Vd_seo, leg4);
                        command_rightB.setElement(y, x, seo);
                    }

                    // ----------- 衝突判定回路 --------------------------------------------------------------------
                    // down
                    {
                        auto seo = std::make_shared<SEO>();
                        seo->setUp(R_detec, Rj, Cj_detec, C_detec, Vd_detec, leg5);
                        detection_down.setElement(y, x, seo);
                    }
                    // left
                    {
                        auto seo = std::make_shared<SEO>();
                        seo->setUp(R_detec, Rj, Cj_detec, C_detec, Vd_detec, leg5);
                        detection_left.setElement(y, x, seo);
                    }                    
                    // up
                    {
                        auto seo = std::make_shared<SEO>();
                        seo->setUp(R_detec, Rj, Cj_detec, C_detec, Vd_detec, leg5);
                        detection_up.setElement(y, x, seo);
                    }                    
                    // right
                    {
                        auto seo = std::make_shared<SEO>();
                        seo->setUp(R_detec, Rj, Cj_detec, C_detec, Vd_detec, leg5);
                        detection_right.setElement(y, x, seo);
                    }
                }
            }

            for (int y = 0; y < oneway_command_downA.numRows(); ++y) {
                for (int x = 0; x < oneway_command_downA.numCols(); ++x) {
                    auto mk_6  = [] { std::array<std::shared_ptr<BaseElement>,6> a{}; for(int i=0;i<6;++i) a[i]=std::make_shared<SEO>(); return a; };
                    auto mk_7S = [] { std::array<std::shared_ptr<BaseElement>,7> a{}; for(int i=0;i<7;++i) a[i]=std::make_shared<SEO>(); return a; };
                    
                    // ---初期化---
                    // ----------- 命令方向回路間 --------------------------------------------------------------------
                    // downA,B
                    auto u_cmdDownA = std::make_shared<OnewayUnit6>();
                    u_cmdDownA->setInternalElements(mk_6());
                    oneway_command_downA.setElement(y, x, u_cmdDownA);

                    auto u_cmdDownB = std::make_shared<OnewayUnit6>();
                    u_cmdDownB->setInternalElements(mk_6());
                    oneway_command_downB.setElement(y, x, u_cmdDownB);
                    
                    // leftA,B
                    auto u_cmdLeftA = std::make_shared<OnewayUnit6>("reverse");
                    u_cmdLeftA->setInternalElements(mk_6());
                    oneway_command_leftA.setElement(y, x, u_cmdLeftA);

                    auto u_cmdLeftB = std::make_shared<OnewayUnit6>("reverse");
                    u_cmdLeftB->setInternalElements(mk_6());
                    oneway_command_leftB.setElement(y, x, u_cmdLeftB);

                    // upA,B
                    auto u_cmdUpA = std::make_shared<OnewayUnit6>("reverse");
                    u_cmdUpA->setInternalElements(mk_6());
                    oneway_command_upA.setElement(y, x, u_cmdUpA);

                    auto u_cmdUpB = std::make_shared<OnewayUnit6>("reverse");
                    u_cmdUpB->setInternalElements(mk_6());
                    oneway_command_upB.setElement(y, x, u_cmdUpB);

                    // rightA,B
                    auto u_cmdRightA = std::make_shared<OnewayUnit6>();
                    u_cmdRightA->setInternalElements(mk_6());
                    oneway_command_rightA.setElement(y, x, u_cmdRightA);

                    auto u_cmdRightB = std::make_shared<OnewayUnit6>();
                    u_cmdRightB->setInternalElements(mk_6());
                    oneway_command_rightB.setElement(y, x, u_cmdRightB);

                    // ----------- 命令方向から衝突判定 --------------------------------------------------------------------
                    // down
                    auto u_Downc2dA = std::make_shared<OnewayUnit6and>("default","end");
                    u_Downc2dA->setInternalElements(mk_7S());
                    oneway_CtoD_downA.setElement(y, x, u_Downc2dA);

                    auto u_Downc2dB = std::make_shared<OnewayUnit6and>("default","end");
                    u_Downc2dB->setInternalElements(mk_7S());
                    oneway_CtoD_downB.setElement(y, x, u_Downc2dB);

                    // left
                    auto u_Leftc2dA = std::make_shared<OnewayUnit6and>("default","end");
                    u_Leftc2dA->setInternalElements(mk_7S());
                    oneway_CtoD_leftA.setElement(y, x, u_Leftc2dA);

                    auto u_Leftc2dB = std::make_shared<OnewayUnit6and>("default","end");
                    u_Leftc2dB->setInternalElements(mk_7S());
                    oneway_CtoD_leftB.setElement(y, x, u_Leftc2dB);

                    // up
                    auto u_Upc2dA = std::make_shared<OnewayUnit6and>("default","end");
                    u_Upc2dA->setInternalElements(mk_7S());
                    oneway_CtoD_upA.setElement(y, x, u_Upc2dA);

                    auto u_Upc2dB = std::make_shared<OnewayUnit6and>("default","end");
                    u_Upc2dB->setInternalElements(mk_7S());
                    oneway_CtoD_upB.setElement(y, x, u_Upc2dB);

                    // right
                    auto u_Rightc2dA = std::make_shared<OnewayUnit6and>("default","end");
                    u_Rightc2dA->setInternalElements(mk_7S());
                    oneway_CtoD_rightA.setElement(y, x, u_Rightc2dA);

                    auto u_Rightc2dB = std::make_shared<OnewayUnit6and>("default","end");
                    u_Rightc2dB->setInternalElements(mk_7S());
                    oneway_CtoD_rightB.setElement(y, x, u_Rightc2dB);

                    // ----------- 衝突判定から命令方向 --------------------------------------------------------------------
                    // down to left
                    auto u_Downd2cA = std::make_shared<OnewayUnit6and>("default","start");
                    u_Downd2cA->setInternalElements(mk_7S());
                    oneway_DtoC_downtoleftA.setElement(y, x, u_Downd2cA);

                    auto u_Downd2cB = std::make_shared<OnewayUnit6and>("default","start");
                    u_Downd2cB->setInternalElements(mk_7S());
                    oneway_DtoC_downtoleftB.setElement(y, x, u_Downd2cB);

                    // left to up
                    auto u_Leftd2cA = std::make_shared<OnewayUnit6and>("default","start");
                    u_Leftd2cA->setInternalElements(mk_7S());
                    oneway_DtoC_lefttoupA.setElement(y, x, u_Leftd2cA);

                    auto u_Leftd2cB = std::make_shared<OnewayUnit6and>("default","start");
                    u_Leftd2cB->setInternalElements(mk_7S());
                    oneway_DtoC_lefttoupB.setElement(y, x, u_Leftd2cB);

                    // up to right
                    auto u_Upd2cA = std::make_shared<OnewayUnit6and>("default","start");
                    u_Upd2cA->setInternalElements(mk_7S());
                    oneway_DtoC_uptorightA.setElement(y, x, u_Upd2cA);

                    auto u_Upd2cB = std::make_shared<OnewayUnit6and>("default","start");
                    u_Upd2cB->setInternalElements(mk_7S());
                    oneway_DtoC_uptorightB.setElement(y, x, u_Upd2cB);

                    // right to down
                    auto u_Rightd2cA = std::make_shared<OnewayUnit6and>("default","start");
                    u_Rightd2cA->setInternalElements(mk_7S());
                    oneway_DtoC_righttodownA.setElement(y, x, u_Rightd2cA);

                    auto u_Rightd2cB = std::make_shared<OnewayUnit6and>("default","start");
                    u_Rightd2cB->setInternalElements(mk_7S());
                    oneway_DtoC_righttodownB.setElement(y, x, u_Rightd2cB);

                    // --- パラメータ設定＆接続 ---
                    // ----------- 命令方向 --------------------------------------------------------------------
                    u_cmdDownA->setOnewaySeoParam(R,Rj,cj_leg2,cj_leg3,C,Vd_oneway);
                    u_cmdDownB->setOnewaySeoParam(R,Rj,cj_leg2,cj_leg3,C,Vd_oneway);
                    u_cmdLeftA->setOnewaySeoParam(R,Rj,cj_leg2,cj_leg3,C,Vd_oneway);
                    u_cmdLeftB->setOnewaySeoParam(R,Rj,cj_leg2,cj_leg3,C,Vd_oneway);
                    u_cmdUpA->setOnewaySeoParam(R,Rj,cj_leg2,cj_leg3,C,Vd_oneway);
                    u_cmdUpB->setOnewaySeoParam(R,Rj,cj_leg2,cj_leg3,C,Vd_oneway);
                    u_cmdRightA->setOnewaySeoParam(R,Rj,cj_leg2,cj_leg3,C,Vd_oneway);
                    u_cmdRightB->setOnewaySeoParam(R,Rj,cj_leg2,cj_leg3,C,Vd_oneway);

                    // ----------- 命令方向から衝突判定 --------------------------------------------------------------------
                    // down
                    u_Downc2dA->setAdditionalElement_seo(R,Rj,cj_leg3,C,Vd_seo,leg3);
                    u_Downc2dA->setOnewaySeoParam(R,Rj,cj_leg2,cj_leg3,C,Vd_oneway);
                    u_Downc2dB->setAdditionalElement_seo(R,Rj,cj_leg3,C,Vd_seo,leg3);
                    u_Downc2dB->setOnewaySeoParam(R,Rj,cj_leg2,cj_leg3,C,Vd_oneway);

                    // left
                    u_Leftc2dA->setAdditionalElement_seo(R,Rj,cj_leg3,C,Vd_seo,leg3);
                    u_Leftc2dA->setOnewaySeoParam(R,Rj,cj_leg2,cj_leg3,C,Vd_oneway);
                    u_Leftc2dB->setAdditionalElement_seo(R,Rj,cj_leg3,C,Vd_seo,leg3);
                    u_Leftc2dB->setOnewaySeoParam(R,Rj,cj_leg2,cj_leg3,C,Vd_oneway);

                    // up
                    u_Upc2dA->setAdditionalElement_seo(R,Rj,cj_leg3,C,Vd_seo,leg3);
                    u_Upc2dA->setOnewaySeoParam(R,Rj,cj_leg2,cj_leg3,C,Vd_oneway);
                    u_Upc2dB->setAdditionalElement_seo(R,Rj,cj_leg3,C,Vd_seo,leg3);
                    u_Upc2dB->setOnewaySeoParam(R,Rj,cj_leg2,cj_leg3,C,Vd_oneway);

                    // right
                    u_Rightc2dA->setAdditionalElement_seo(R,Rj,cj_leg3,C,Vd_seo,leg3);
                    u_Rightc2dA->setOnewaySeoParam(R,Rj,cj_leg2,cj_leg3,C,Vd_oneway);
                    u_Rightc2dB->setAdditionalElement_seo(R,Rj,cj_leg3,C,Vd_seo,leg3);
                    u_Rightc2dB->setOnewaySeoParam(R,Rj,cj_leg2,cj_leg3,C,Vd_oneway);

                    // ----------- 衝突判定から命令方向 --------------------------------------------------------------------
                    // down
                    u_Downd2cA->setAdditionalElement_seo(R,Rj,cj_leg4,C,Vd_adjust,leg4);
                    u_Downd2cA->setOnewaySeoParam(R,Rj,cj_leg2,cj_leg3,C,Vd_oneway);
                    u_Downd2cB->setAdditionalElement_seo(R,Rj,cj_leg4,C,Vd_adjust,leg4);
                    u_Downd2cB->setOnewaySeoParam(R,Rj,cj_leg2,cj_leg3,C,Vd_oneway);

                    // left
                    u_Leftd2cA->setAdditionalElement_seo(R,Rj,cj_leg4,C,Vd_adjust,leg4);
                    u_Leftd2cA->setOnewaySeoParam(R,Rj,cj_leg2,cj_leg3,C,Vd_oneway);
                    u_Leftd2cB->setAdditionalElement_seo(R,Rj,cj_leg4,C,Vd_adjust,leg4);
                    u_Leftd2cB->setOnewaySeoParam(R,Rj,cj_leg2,cj_leg3,C,Vd_oneway);

                    // up
                    u_Upd2cA->setAdditionalElement_seo(R,Rj,cj_leg4,C,Vd_adjust,leg4);
                    u_Upd2cA->setOnewaySeoParam(R,Rj,cj_leg2,cj_leg3,C,Vd_oneway);
                    u_Upd2cB->setAdditionalElement_seo(R,Rj,cj_leg4,C,Vd_adjust,leg4);
                    u_Upd2cB->setOnewaySeoParam(R,Rj,cj_leg2,cj_leg3,C,Vd_oneway);

                    // right
                    u_Rightd2cA->setAdditionalElement_seo(R,Rj,cj_leg4,C,Vd_adjust,leg4);
                    u_Rightd2cA->setOnewaySeoParam(R,Rj,cj_leg2,cj_leg3,C,Vd_oneway);
                    u_Rightd2cB->setAdditionalElement_seo(R,Rj,cj_leg4,C,Vd_adjust,leg4);
                    u_Rightd2cB->setOnewaySeoParam(R,Rj,cj_leg2,cj_leg3,C,Vd_oneway);

                    // 接続
                    // ----------- 命令方向 --------------------------------------------------------------------
                    if (x>0 && x<command_downA.numCols()-1 && y>0 && y<command_downA.numRows()-1) {
                    u_cmdDownA->setOnewayConnections(command_downA.getElement(y,x), command_downA.getElement(y+1,x));
                    u_cmdDownB->setOnewayConnections(command_downB.getElement(y,x), command_downB.getElement(y+1,x));
                    u_cmdLeftA->setOnewayConnections(command_leftA.getElement(y,x), command_leftA.getElement(y,x+1));
                    u_cmdLeftB->setOnewayConnections(command_leftB.getElement(y,x), command_leftB.getElement(y,x+1));
                    u_cmdUpA->setOnewayConnections(command_upA.getElement(y,x), command_upA.getElement(y+1,x));
                    u_cmdUpB->setOnewayConnections(command_upB.getElement(y,x), command_upB.getElement(y+1,x));
                    u_cmdRightA->setOnewayConnections(command_rightA.getElement(y,x), command_rightA.getElement(y,x+1));
                    u_cmdRightB->setOnewayConnections(command_rightB.getElement(y,x), command_rightB.getElement(y,x+1));
                    }

                    // ----------- 命令方向から衝突判定 --------------------------------------------------------------------
                    if (x>0 && x<command_downA.numCols()-1 && y>0 && y<command_downA.numRows()-1) {
                        // down
                        u_Downc2dA->setOnewayConnections(
                            { command_downA.getElement(y,x) },
                            { detection_down.getElement(y,x), u_Downd2cB->getInternalElement(0) }
                        );
                        u_Downc2dB->setOnewayConnections(
                            { command_downB.getElement(y,x) },
                            { detection_down.getElement(y,x), u_Downd2cA->getInternalElement(0) }
                        );
                        // left
                        u_Leftc2dA->setOnewayConnections(
                            { command_leftA.getElement(y,x) },
                            { detection_left.getElement(y,x), u_Leftd2cB->getInternalElement(0) }
                        );
                        u_Leftc2dB->setOnewayConnections(
                            { command_leftB.getElement(y,x) },
                            { detection_left.getElement(y,x), u_Leftd2cA->getInternalElement(0) }
                        );
                        // up
                        u_Upc2dA->setOnewayConnections(
                            { command_upA.getElement(y,x) },
                            { detection_up.getElement(y,x), u_Upd2cB->getInternalElement(0) }
                        );
                        u_Upc2dB->setOnewayConnections(
                            { command_upB.getElement(y,x) },
                            { detection_up.getElement(y,x), u_Upd2cA->getInternalElement(0) }
                        );
                        // right
                        u_Rightc2dA->setOnewayConnections(
                            { command_rightA.getElement(y,x) },
                            { detection_right.getElement(y,x), u_Rightd2cB->getInternalElement(0) }
                        );
                        u_Rightc2dB->setOnewayConnections(
                            { command_rightB.getElement(y,x) },
                            { detection_right.getElement(y,x), u_Rightd2cA->getInternalElement(0) }
                        );
                    }

                    // ----------- 衝突判定から命令方向 --------------------------------------------------------------------
                    if (x>0 && x<command_leftA.numCols()-1 && y>0 && y<command_leftA.numRows()-1) {
                        // down
                        u_Downd2cA->setOnewayConnections(
                            { detection_down.getElement(y,x), u_Downc2dB->getInternalElement(6), u_Downd2cB->getInternalElement(0) },
                            { command_leftA.getElement(y,x) }
                        );
                        u_Downd2cB->setOnewayConnections(
                            { detection_down.getElement(y,x), u_Downc2dA->getInternalElement(6), u_Downd2cA->getInternalElement(0) },
                            { command_leftB.getElement(y,x) }
                        );
                        // left
                        u_Leftd2cA->setOnewayConnections(
                            { detection_left.getElement(y,x), u_Leftc2dB->getInternalElement(6), u_Leftd2cB->getInternalElement(0) },
                            { command_upA.getElement(y,x) }
                        );
                        u_Leftd2cB->setOnewayConnections(
                            { detection_left.getElement(y,x), u_Leftc2dA->getInternalElement(6), u_Leftd2cA->getInternalElement(0) },
                            { command_upB.getElement(y,x) }
                        );
                        // up
                        u_Upd2cA->setOnewayConnections(
                            { detection_up.getElement(y,x), u_Upc2dB->getInternalElement(6), u_Upd2cB->getInternalElement(0) },
                            { command_rightA.getElement(y,x) }
                        );
                        u_Upd2cB->setOnewayConnections(
                            { detection_up.getElement(y,x), u_Upc2dA->getInternalElement(6), u_Upd2cA->getInternalElement(0) },
                            { command_rightB.getElement(y,x) }
                        );
                        // right
                        u_Rightd2cA->setOnewayConnections(
                            { detection_right.getElement(y,x), u_Rightc2dB->getInternalElement(6), u_Rightd2cB->getInternalElement(0) },
                            { command_leftA.getElement(y,x) }
                        );
                        u_Rightd2cB->setOnewayConnections(
                            { detection_right.getElement(y,x), u_Rightc2dA->getInternalElement(6), u_Rightd2cA->getInternalElement(0) },
                            { command_leftB.getElement(y,x) }
                        );
                    }
                }
            }


            // === 接続情報 ===
            // 命令方向回路・衝突判定回路
            for (int y = 1; y < command_downA.numRows() - 1; ++y) {
                for (int x = 1; x < command_downA.numCols() - 1; ++x) {
                    // ----------- 命令方向回路 --------------------------------------------------------------------
                    {
                        // command_downA
                        auto elem = command_downA.getElement(y, x);
                        std::vector<std::shared_ptr<BaseElement>> neighbors;
                        neighbors.push_back(oneway_command_downA.getElement(y - 1,x)->getInternalElement(5)); // 下方向命令の一方通行（前）
                        neighbors.push_back(oneway_command_downA.getElement(y,x)->getInternalElement(0)); // 下方向命令の一方通行（次）
                        neighbors.push_back(oneway_CtoD_downA.getElement(y,x)->getInternalElement(0)); // 命令から衝突まで
                        neighbors.push_back(oneway_DtoC_righttodownA.getElement(y,x)->getInternalElement(6)); // 右衝突判定から
                        elem->setConnections(neighbors);
                    }
                    {
                        // command_downB
                        auto elem = command_downB.getElement(y, x);
                        std::vector<std::shared_ptr<BaseElement>> neighbors;
                        neighbors.push_back(oneway_command_downB.getElement(y - 1,x)->getInternalElement(5)); // 下方向命令の一方通行（前）
                        neighbors.push_back(oneway_command_downB.getElement(y,x)->getInternalElement(0)); // 下方向命令の一方通行（次）
                        neighbors.push_back(oneway_CtoD_downB.getElement(y,x)->getInternalElement(0)); // 命令から衝突まで
                        neighbors.push_back(oneway_DtoC_righttodownB.getElement(y,x)->getInternalElement(6)); // 右衝突判定から
                        elem->setConnections(neighbors);
                    }
                    {
                        // command_upA
                        auto elem = command_upA.getElement(y, x);
                        std::vector<std::shared_ptr<BaseElement>> neighbors;
                        neighbors.push_back(oneway_command_upA.getElement(y - 1,x)->getInternalElement(5)); // 上方向命令の一方通行（前）
                        neighbors.push_back(oneway_command_upA.getElement(y,x)->getInternalElement(0)); // 上方向命令の一方通行（次）
                        neighbors.push_back(oneway_CtoD_upA.getElement(y,x)->getInternalElement(0)); // 命令から衝突まで
                        neighbors.push_back(oneway_DtoC_lefttoupA.getElement(y,x)->getInternalElement(6)); // 左衝突判定から
                        elem->setConnections(neighbors);
                    }
                    {
                        // command_upB
                        auto elem = command_upB.getElement(y, x);
                        std::vector<std::shared_ptr<BaseElement>> neighbors;
                        neighbors.push_back(oneway_command_upB.getElement(y - 1,x)->getInternalElement(5)); // 上方向命令の一方通行（前）
                        neighbors.push_back(oneway_command_upB.getElement(y,x)->getInternalElement(0)); // 上方向命令の一方通行（次）
                        neighbors.push_back(oneway_CtoD_upB.getElement(y,x)->getInternalElement(0)); // 命令から衝突まで
                        neighbors.push_back(oneway_DtoC_lefttoupB.getElement(y,x)->getInternalElement(6)); // 左衝突判定から
                        elem->setConnections(neighbors);
                    }
                    {
                        // command_leftA
                        auto elem = command_leftA.getElement(y, x);
                        std::vector<std::shared_ptr<BaseElement>> neighbors;
                        neighbors.push_back(oneway_DtoC_downtoleftA.getElement(y,x)->getInternalElement(6)); // 下方向衝突判定
                        neighbors.push_back(oneway_command_leftA.getElement(y,x - 1)->getInternalElement(5)); // 左方向命令の一方通行（前）
                        neighbors.push_back(oneway_command_leftA.getElement(y,x)->getInternalElement(0)); // 左方向命令の一方通行（次）
                        neighbors.push_back(oneway_CtoD_leftA.getElement(y,x)->getInternalElement(0)); // 命令から衝突まで
                        elem->setConnections(neighbors);
                    }
                    {
                        // command_leftB
                        auto elem = command_leftB.getElement(y, x);
                        std::vector<std::shared_ptr<BaseElement>> neighbors;
                        neighbors.push_back(oneway_DtoC_downtoleftB.getElement(y,x)->getInternalElement(6)); // 下方向衝突判定
                        neighbors.push_back(oneway_command_leftB.getElement(y,x - 1)->getInternalElement(5)); // 左方向命令の一方通行（前）
                        neighbors.push_back(oneway_command_leftB.getElement(y,x)->getInternalElement(0)); // 左方向命令の一方通行（次）
                        neighbors.push_back(oneway_CtoD_leftB.getElement(y,x)->getInternalElement(0)); // 命令から衝突まで
                        elem->setConnections(neighbors);
                    }
                    {
                        // command_rightA
                        auto elem = command_rightA.getElement(y, x);
                        std::vector<std::shared_ptr<BaseElement>> neighbors;
                        neighbors.push_back(oneway_DtoC_uptorightA.getElement(y,x)->getInternalElement(6)); // 右方向衝突判定
                        neighbors.push_back(oneway_command_rightA.getElement(y,x - 1)->getInternalElement(5)); // 右方向命令の一方通行（前）
                        neighbors.push_back(oneway_command_rightA.getElement(y,x)->getInternalElement(0)); // 右方向命令の一方通行（次）
                        neighbors.push_back(oneway_CtoD_rightA.getElement(y,x)->getInternalElement(0)); // 命令から衝突まで
                        elem->setConnections(neighbors);
                    }
                    {
                        // command_rightB
                        auto elem = command_rightB.getElement(y, x);
                        std::vector<std::shared_ptr<BaseElement>> neighbors;
                        neighbors.push_back(oneway_DtoC_uptorightB.getElement(y,x)->getInternalElement(6)); // 右方向衝突判定
                        neighbors.push_back(oneway_command_rightB.getElement(y,x - 1)->getInternalElement(5)); // 右方向命令の一方通行（前）
                        neighbors.push_back(oneway_command_rightB.getElement(y,x)->getInternalElement(0)); // 右方向命令の一方通行（次）
                        neighbors.push_back(oneway_CtoD_rightB.getElement(y,x)->getInternalElement(0)); // 命令から衝突まで
                        elem->setConnections(neighbors);
                    }
                    // ----------- 衝突判定回路 --------------------------------------------------------------------
                    {
                        // down
                        auto elem = detection_down.getElement(y, x);
                        std::vector<std::shared_ptr<BaseElement>> neighbors;
                        neighbors.push_back(oneway_CtoD_downA.getElement(y, x)->getInternalElement(6)); // 命令から衝突まで
                        neighbors.push_back(oneway_CtoD_downB.getElement(y, x)->getInternalElement(6));
                        neighbors.push_back(oneway_DtoC_downtoleftA.getElement(y, x)->getInternalElement(0));
                        neighbors.push_back(oneway_DtoC_downtoleftB.getElement(y, x)->getInternalElement(0));
                        elem->setConnections(neighbors);
                    }
                    {
                        // left
                        auto elem = detection_left.getElement(y, x);
                        std::vector<std::shared_ptr<BaseElement>> neighbors;
                        neighbors.push_back(oneway_CtoD_leftA.getElement(y, x)->getInternalElement(6)); // 命令から衝突まで
                        neighbors.push_back(oneway_CtoD_leftB.getElement(y, x)->getInternalElement(6));
                        neighbors.push_back(oneway_DtoC_lefttoupA.getElement(y, x)->getInternalElement(0));
                        neighbors.push_back(oneway_DtoC_lefttoupB.getElement(y, x)->getInternalElement(0));
                        elem->setConnections(neighbors);
                    }
                    {
                        // up
                        auto elem = detection_up.getElement(y, x);
                        std::vector<std::shared_ptr<BaseElement>> neighbors;
                        neighbors.push_back(oneway_CtoD_upA.getElement(y, x)->getInternalElement(6)); // 命令から衝突まで
                        neighbors.push_back(oneway_CtoD_upB.getElement(y, x)->getInternalElement(6));
                        neighbors.push_back(oneway_DtoC_uptorightA.getElement(y, x)->getInternalElement(0));
                        neighbors.push_back(oneway_DtoC_uptorightB.getElement(y, x)->getInternalElement(0));
                        elem->setConnections(neighbors);
                    }
                    {
                        // right
                        auto elem = detection_right.getElement(y, x);
                        std::vector<std::shared_ptr<BaseElement>> neighbors;
                        neighbors.push_back(oneway_CtoD_rightA.getElement(y, x)->getInternalElement(6)); // 命令から衝突まで
                        neighbors.push_back(oneway_CtoD_rightB.getElement(y, x)->getInternalElement(6));
                        neighbors.push_back(oneway_DtoC_righttodownA.getElement(y, x)->getInternalElement(0));
                        neighbors.push_back(oneway_DtoC_righttodownB.getElement(y, x)->getInternalElement(0));
                        elem->setConnections(neighbors);
                    }
                }
            }
         
            // バイアス電圧を迷路状に設定
            // cmd
            setMazeBiasForEachParticle(command_downA,maze,Vd_seo);
            setMazeBiasForEachParticle(command_downB,maze,Vd_seo);
            setMazeBiasForEachParticle(command_leftA,maze,Vd_seo);
            setMazeBiasForEachParticle(command_leftB,maze,Vd_seo);
            setMazeBiasForEachParticle(command_upA,maze,Vd_seo);
            setMazeBiasForEachParticle(command_upB,maze,Vd_seo);
            setMazeBiasForEachParticle(command_rightA,maze,Vd_seo);
            setMazeBiasForEachParticle(command_rightB,maze,Vd_seo);

            // detec
            setMazeBiasWithDirection(detection_down,maze,"down",-Vd_detec);
            setMazeBiasWithDirection(detection_left,maze,"left",-0);
            setMazeBiasWithDirection(detection_up,maze,"up",-0);
            setMazeBiasWithDirection(detection_right,maze,"right",-0);

            // === シミュレーション初期化 ===
            Sim sim(dt, endtime);
            sim.addGrid({
                command_downA, command_downB, command_leftA, command_leftB,command_upA, command_upB,command_rightA, command_rightB,
                detection_down, detection_left, detection_up, detection_right,
                oneway_command_downA,oneway_command_downB, oneway_CtoD_downA, oneway_CtoD_downB, oneway_DtoC_downtoleftA,oneway_DtoC_downtoleftB,
                oneway_command_leftA,oneway_command_leftB, oneway_CtoD_leftA, oneway_CtoD_leftB, oneway_DtoC_lefttoupA,oneway_DtoC_lefttoupB,
                oneway_command_upA,oneway_command_upB, oneway_CtoD_upA, oneway_CtoD_upB, oneway_DtoC_uptorightA,oneway_DtoC_uptorightB,
                oneway_command_rightA,oneway_command_rightB, oneway_CtoD_rightA, oneway_CtoD_rightB, oneway_DtoC_righttodownA,oneway_DtoC_righttodownB,
            });

            // === 特定素子の出力設定 ===
            // std::vector<std::shared_ptr<BaseElement>> tracked = {
            //     command_downA.getElement(3,6), // oA1 A
            //     command_downA.getElement(5,8), // oA2 B
            //     command_downB.getElement(5,8), // oB3 C
            //     command_downA.getElement(6,8), // oA4 D
            //     command_downB.getElement(6,8), // oB5 E
            //     command_downA.getElement(8,12), // oA6 F
            //     command_downB.getElement(8,12), // oB7 G
            //     command_downA.getElement(9,12), // oA8 H
            //     command_downB.getElement(9,12), // oB9 I

            //     detection_down.getElement(2,6), // xAB10 J
            //     detection_down.getElement(3,6), // oA11 K
            //     detection_down.getElement(5,8), // oAB12 L
            //     detection_down.getElement(6,8), // oA13 M
            //     detection_down.getElement(8,12), // oAB14 N
            //     detection_down.getElement(9,12), // oB15 O

            //     command_leftA.getElement(3,6), // oA16 P
            //     command_leftB.getElement(3,6), // xB17 Q
            //     command_leftA.getElement(5,8), // xA18 R
            //     command_leftB.getElement(5,8), // oB19 S
            //     command_leftA.getElement(6,8), // oA20 T
            //     command_leftB.getElement(6,8), // xB21 U
            //     command_leftA.getElement(8,12), // oA22 V
            //     command_leftB.getElement(8,12), // xB23 W
            //     command_leftA.getElement(9,12), // xA24 X
            //     command_leftB.getElement(9,12), // oB25 Y
            // };
            // sim.addTrackedElements(tracked);

            // === トリガ設定 ===
            sim.addVoltageTrigger(100, &command_downA, 1, 8, 0.002);//A
            sim.addVoltageTrigger(100, &command_downB, 1, 10, 0.002);//B

            // トリガとしてパルス波を入力
            for (int y = 0; y < detection_down.numRows(); ++y) {
                for (int x = 0; x < detection_down.numCols(); ++x) {
                    for (int interval = 10; interval < endtime; interval += 10){
                        sim.addVoltageTrigger(interval, &detection_down, y, x, -0.0025, 1);
                        sim.addVoltageTrigger(interval, &detection_left, y, x, -0.0025, 1);
                        sim.addVoltageTrigger(interval, &detection_up, y, x, -0.0025, 1);
                        sim.addVoltageTrigger(interval, &detection_right, y, x, -0.0025, 1);
                    }
                }
            }

            // gnuplot追跡用
            std::vector<std::shared_ptr<BaseElement>> trackElements = {
                command_downA.getElement(1,8), // トリガA
                command_downB.getElement(1,10), // トリガB
                command_downA.getElement(4,8), 
                command_downB.getElement(4,10),
                detection_down.getElement(4,8), // xAB10 J
                detection_down.getElement(4,10), // oA11 K
                command_leftA.getElement(4,8),
                command_leftB.getElement(4,8),
                command_leftA.getElement(4,10),
                command_leftB.getElement(4,10),
            };
            // std::vector<std::shared_ptr<BaseElement>> tracked_detec = {
            //     detection_down.getElement(2,6), // xAB10 J
            //     detection_down.getElement(3,6), // oA11 K
            //     detection_down.getElement(5,8), // oAB12 L
            //     detection_down.getElement(6,8), // oA13 M
            //     detection_down.getElement(8,12), // oAB14 N
            //     detection_down.getElement(9,12), // oB15 O
            // };
            // std::vector<std::shared_ptr<BaseElement>> tracked_command_left = {
            //     command_leftA.getElement(3,6), // oA16 P
            //     command_leftB.getElement(3,6), // xB17 Q
            //     command_leftA.getElement(5,8), // xA18 R
            //     command_leftB.getElement(5,8), // oB19 S
            //     command_leftA.getElement(6,8), // oA20 T
            //     command_leftB.getElement(6,8), // xB21 U
            //     command_leftA.getElement(8,12), // oA22 V
            //     command_leftB.getElement(8,12), // xB23 W
            //     command_leftA.getElement(9,12), // xA24 X
            //     command_leftB.getElement(9,12), // oB25 Y
            // };

            // std::vector<std::shared_ptr<BaseElement>> tracked_oneway = {
            //     oneway_CtoD_downA.getElement(6,8)->getInternalElement(0),
            //     oneway_CtoD_downA.getElement(6,8)->getInternalElement(6),
            //     oneway_CtoD_downB.getElement(6,8)->getInternalElement(0),
            //     oneway_CtoD_downB.getElement(6,8)->getInternalElement(6),
            //     detection_down.getElement(6,8),
            //     oneway_DtoC_downtoleftA.getElement(6,8)->getInternalElement(0),
            //     oneway_DtoC_downtoleftA.getElement(6,8)->getInternalElement(1),
            //     oneway_DtoC_downtoleftA.getElement(6,8)->getInternalElement(2),
            //     oneway_DtoC_downtoleftA.getElement(6,8)->getInternalElement(3),
            //     oneway_DtoC_downtoleftA.getElement(6,8)->getInternalElement(4),
            //     oneway_DtoC_downtoleftA.getElement(6,8)->getInternalElement(5),
            //     oneway_DtoC_downtoleftA.getElement(6,8)->getInternalElement(6),
            //     oneway_DtoC_downtoleftB.getElement(6,8)->getInternalElement(0),
            //     oneway_DtoC_downtoleftB.getElement(6,8)->getInternalElement(1),
            //     oneway_DtoC_downtoleftB.getElement(6,8)->getInternalElement(2),
            //     oneway_DtoC_downtoleftB.getElement(6,8)->getInternalElement(3),
            //     oneway_DtoC_downtoleftB.getElement(6,8)->getInternalElement(4),
            //     oneway_DtoC_downtoleftB.getElement(6,8)->getInternalElement(5),
            //     oneway_DtoC_downtoleftB.getElement(6,8)->getInternalElement(6),
            //     command_leftA.getElement(6,8),
            //     command_leftB.getElement(6,8)
            // };

            auto ofs1 = std::make_shared<std::ofstream>("output/commandDowntoLeft.txt");
            sim.addSelectedElements(ofs1, trackElements);
            sim.generateGnuplotScript("output/commandDowntoLeft.txt", {"1-8A","1-10B","4-8A","4-10B","4-8detec","4-10detec","4-8A","4-8B","4-10A","4-10B"});

            // auto ofs2 = std::make_shared<std::ofstream>("output/new_structure_detec"+ std::to_string(trial) +".txt");
            // sim.addSelectedElements(ofs2, tracked_detec);
            // sim.generateGnuplotScript("output/new_structure_detec"+ std::to_string(trial) +".txt", {"2-6","3-6","5-8","6-8","8-12","9-12"});

            // auto ofs3 = std::make_shared<std::ofstream>("output/new_structure_commL.txt");
            // sim.addSelectedElements(ofs3, tracked_command_left);
            // sim.generateGnuplotScript("output/new_structure_commL.txt", {"3-6A","3-6B","5-8A","5-8B","6-8A","6-8B","8-12A","8-12B","9-12A","9-12B"});

            // auto ofs4 = std::make_shared<std::ofstream>("output/new_structure_oneway.txt");
            // sim.addSelectedElements(ofs4, tracked_oneway);
            // sim.generateGnuplotScript("output/new_structure_oneway.txt", {"0A","6A","0B","6B","detec","0A","1A","2A","3A","4A","5A","6A","0B","1B","2B","3B","4B","5B","6B","LA","LB"});

            // dE追跡用
            // auto ofsdE = std::make_shared<std::ofstream>("output/dEcheccker.csv");
            // sim.addSelecteddEElements(ofsdE, {oneway_DtoC_downtoleftA.getElement(6,8)->getInternalElement(0),oneway_DtoC_downtoleftB.getElement(6,8)->getInternalElement(0)});
            // // 時刻記録
            sim.run();
            allResults.push_back(sim.getTunnelTimes());
            // 動画出力
            const auto &outputs = sim.getOutputs();
            for (const auto &[label, data] : outputs)
            {
                auto normalized = oyl::normalizeto255(data);
                oyl::VideoClass video(normalized);
                video.set_filename("output/" + label + std::to_string(trial) +  ".mp4");
                video.set_codec(cv::VideoWriter::fourcc('m', 'p', '4', 'v'));
                video.set_fps(30.0);
                video.makevideo();
            }

        }

        // // === CSV出力 ===
        // std::string filename = "output/single100.csv";
        // std::ofstream ofs(filename);
        // for (const auto& row : allResults) {
        //     for (size_t i = 0; i < row.size(); ++i) {
        //         ofs << row[i];
        //         if (i != row.size() - 1) ofs << ",";
        //     }
        //     ofs << "\n";
        // }

        
    }
    return 0;
}
