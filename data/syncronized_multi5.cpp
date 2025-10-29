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
constexpr int size_y = 12;
constexpr double Vd_seo = 0.0039;
constexpr double Vd_oneway = 0.0039;
constexpr double Vd_adjust = 0.0036;
constexpr double R = 1;
constexpr double R_detec = 0.4;
constexpr double R_small = 0.8;
constexpr double Rj = 0.0001;
constexpr double C = 2.0;
constexpr double C_detec = 1;
constexpr double Cj_detec = 12;
constexpr double dt = 0.1;
constexpr double endtime = 160;
constexpr double setVth = 0.004;

double cj_leg2 = seo_junction_cj_calc(leg2, C, setVth);
double cj_leg3 = seo_junction_cj_calc(leg3, C, setVth);
double cj_leg4 = seo_junction_cj_calc(leg4, C, setVth);
double cj_leg5 = seo_junction_cj_calc(leg5, C, setVth);
double cj_leg6 = seo_junction_cj_calc(leg6, C, setVth);
constexpr int repeat_count = 10;
constexpr int junction_num_max = 5;
constexpr double max_pulse_voltage = 0;

using Grid = Grid2D<BaseElement>;
using Sim = Simulation2D<BaseElement>;

int main()
{
    std::vector<std::vector<int>> maze = {
        {0,0,0,0,0,1,0,1,0,0,0,1,0,0,0},
        {1,1,1,1,1,1,0,1,0,0,0,1,0,0,0},
        {1,1,1,1,1,1,0,1,0,0,0,1,0,0,0},
        {0,0,0,0,0,0,0,1,0,0,0,1,0,0,0},
        {1,1,1,1,1,1,1,1,0,0,0,1,0,0,0},
        {1,1,1,1,1,1,1,1,0,0,0,1,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,1,0,0,0},
        {1,1,1,1,1,1,1,1,1,1,1,1,0,0,0},
        {1,1,1,1,1,1,1,1,1,1,1,1,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    };

    for(double pulse_voltage = 0; pulse_voltage <= max_pulse_voltage; pulse_voltage+=0.001){
        int multi_num = junction_num_max;
        double multi_cj_leg2 = multi_junction_cj_calc(multi_num, leg2, C, setVth); // 引数の条件に合わせたCjを定義
        double multi_cj_leg3 = multi_junction_cj_calc(multi_num, leg3, C, setVth);
        double multi_cj_leg4 = multi_junction_cj_calc(multi_num, leg4, C, setVth);
        double multi_cj_leg5 = multi_junction_cj_calc(multi_num, leg5, C, setVth);
        double multi_cj_leg6 = multi_junction_cj_calc(multi_num, leg6, C, setVth);
        // double Vd_detec_multi = Vd_seo - 2 * multi_tunnelV(C,leg4,multi_cj_leg4,multi_cj_leg3,junction_num_max);
        double Vd_detec = Vd_seo - 0.64 * tunnelV(C,leg5,leg3,cj_leg5,cj_leg3);
        // double Vd_detec = Vd_seo;

        std::vector<std::vector<double>> allResults;
        for(int trial = 0; trial < repeat_count; trial++){
            std::cout << "multi_num = " << pulse_voltage << " trial = " << trial << std::endl;
            // === Gridを生成 ===
            Grid command_downA(size_y, size_x); // 命令方向回路A（下）
            Grid command_downB(size_y, size_x); // 命令方向回路B（下）
            Grid detection_down(size_y, size_x); // 衝突方向回路（下）
            Grid command_leftA(size_y, size_x); // 命令方向回路A（左）
            Grid command_leftB(size_y, size_x); // 命令方向回路A（左）
            Grid oneway_command_downA(size_y, size_x, false);  // 命令方向回路（下）における一方通行回路A
            Grid oneway_command_downB(size_y, size_x, false);  // 命令方向回路（下）における一方通行回路B
            Grid oneway_CtoD_downA(size_y, size_x, false);     // 命令方向回路（下）から衝突判定回路（下）をつなぐ一方通行回路A
            Grid oneway_CtoD_downB(size_y, size_x, false);     // 命令方向回路（下）から衝突判定回路（下）をつなぐ一方通行回路B
            Grid oneway_DtoC_downtoleftA(size_y, size_x, false);   // 衝突判定回路（下）から命令方向回路（左）をつなぐ一方通行回路A
            Grid oneway_DtoC_downtoleftB(size_y, size_x, false);   // 衝突判定回路（下）から命令方向回路（左）をつなぐ一方通行回路B
            Grid oneway_command_leftA(size_y , size_x, false);  // 命令方向回路（左）における一方通行回路A
            Grid oneway_command_leftB(size_y , size_x, false);  // 命令方向回路（左）における一方通行回路B

            // 動画出力するgridに名前をつける
            command_downA.setOutputLabel("command_downA");
            command_downB.setOutputLabel("command_downB");
            detection_down.setOutputLabel("detection_down");
            command_leftA.setOutputLabel("command_leftA");
            command_leftB.setOutputLabel("command_leftB");

            // === 全グリッド初期化 ===
            // 命令方向回路 (command_down, command_up)
            for (int y = 0; y < command_downA.numRows(); ++y) {
                for (int x = 0; x < command_downA.numCols(); ++x) {
                    // MultiSEO
                    // {
                    //     auto seo = std::make_shared<MultiSEO>();
                    //     seo->setUp(R, Rj, multi_cj_leg6, C, Vd_seo, leg6, multi_num);
                    //     command_down.setElement(y, x, seo);
                    // }
                    // SEO
                    {
                        auto seo = std::make_shared<SEO>();
                        seo->setUp(R, Rj, cj_leg3, C, Vd_seo, leg3);
                        command_downA.setElement(y, x, seo);
                    }
                    // SEO
                    {
                        auto seo = std::make_shared<SEO>();
                        seo->setUp(R, Rj, cj_leg3, C, Vd_seo, leg3);
                        command_downB.setElement(y, x, seo);
                    }
                }
            }
            // 命令方向回路(command_left, command_right)
            for (int y = 0; y < command_leftA.numRows(); ++y) {
                for (int x = 0; x < command_leftA.numCols(); ++x) {
                    // MultiSEO
                    // {
                    //     auto seo = std::make_shared<MultiSEO>();
                    //     seo->setUp(R, Rj, multi_cj_leg6, C, Vd_seo, leg6, multi_num);
                    //     command_left.setElement(y, x, seo);
                    // }
                    // SEO
                    {
                        auto seo = std::make_shared<SEO>();
                        seo->setUp(R, Rj, cj_leg3, C, Vd_seo, leg3);
                        command_leftA.setElement(y, x, seo);
                    }
                    // SEO
                    {
                        auto seo = std::make_shared<SEO>();
                        seo->setUp(R, Rj, cj_leg3, C, Vd_seo, leg3);
                        command_leftB.setElement(y, x, seo);
                    }
                }
            }

            // 衝突判定回路 (detection_down, detection_up, detection_left, detection_right)
            for (int y = 0; y < detection_down.numRows(); ++y) {
                for (int x = 0; x < detection_down.numCols(); ++x) {
                    // MultiSEO
                    // {
                    //     auto seo = std::make_shared<MultiSEO>();
                    //     seo->setUp(R, Rj, multi_cj_leg4, C, Vd_detec, leg4, multi_num);
                    //     detection_down.setElement(y, x, seo);
                    // }
                    //SEO
                    {
                        auto seo = std::make_shared<SEO>();
                        seo->setUp(R_detec, Rj, Cj_detec, C_detec, Vd_detec, leg5);
                        detection_down.setElement(y, x, seo);
                    }
                }
            }

            // // oneway回路 (oneway_command_down, oneway_CtoD_down, ...)
            // for (int y = 0; y < oneway_command_downA.numRows(); ++y) {
            //     for (int x = 0; x < oneway_command_downA.numCols(); ++x) {
            //         {
            //             // onway_command_downA(MultiSEO)
            //             auto unit = std::make_shared<OnewayUnit6>();
            //             std::array<std::shared_ptr<BaseElement>, 6> internal_seos;
            //             for (int i = 0; i < 6; ++i) internal_seos[i] = std::make_shared<MultiSEO>();
            //             unit->setInternalElements(internal_seos);
            //             unit->setOnewayMultiSeoParam(R, Rj, multi_cj_leg2, multi_cj_leg3, C, Vd_oneway, multi_num);
            //             oneway_command_downA.setElement(y, x, unit);
            //             if(x > 0 && x < command_downA.numCols() && y > 0 && y < command_downA.numRows() - 1){
            //                 unit->setOnewayConnections(command_downA.getElement(y,x),command_downA.getElement(y+1,x));
            //             }
            //         }
            //         {
            //             // onway_command_downB(MultiSEO)
            //             auto unit = std::make_shared<OnewayUnit6>();
            //             std::array<std::shared_ptr<BaseElement>, 6> internal_seos;
            //             for (int i = 0; i < 6; ++i) internal_seos[i] = std::make_shared<MultiSEO>();
            //             unit->setInternalElements(internal_seos);
            //             unit->setOnewayMultiSeoParam(R, Rj, multi_cj_leg2, multi_cj_leg3, C, Vd_oneway, multi_num);
            //             oneway_command_downB.setElement(y, x, unit);
            //             if(x > 0 && x < command_downB.numCols() && y > 0 && y < command_downB.numRows() - 1){
            //                 unit->setOnewayConnections(command_downB.getElement(y,x),command_downB.getElement(y+1,x));
            //             }
            //         }
                    
            //         {
            //             // oneway_CtoD_downA(SEO)
            //             auto unit = std::make_shared<OnewayUnit6and>("default","end");
            //             std::array<std::shared_ptr<BaseElement>, 7> internal_seos;
            //             for (int i = 0; i < 7; ++i) internal_seos[i] = std::make_shared<SEO>();
            //             unit->setInternalElements(internal_seos);
            //             unit->setAdditionalElement_seo(R, Rj, cj_leg3, C, Vd_seo, leg3);
            //             unit->setOnewaySeoParam(R, Rj, cj_leg2, cj_leg3, C, Vd_oneway);
            //             oneway_CtoD_downA.setElement(y, x, unit);
            //             if(x > 0 && x < command_downA.numCols() && y > 0 && y < command_downA.numRows()){
            //                 unit->setOnewayConnections(
            //                     {command_downA.getElement(y,x)},
            //                     {detection_down.getElement(y,x),oneway_DtoC_downtoleftB.getElement(y,x)->getInternalElement(0)}
            //                 );
            //             }
            //         }
            //         {
            //             // oneway_CtoD_downB(SEO)
            //             auto unit = std::make_shared<OnewayUnit6and>("default","end");
            //             std::array<std::shared_ptr<BaseElement>, 7> internal_seos;
            //             for (int i = 0; i < 7; ++i) internal_seos[i] = std::make_shared<SEO>();
            //             unit->setInternalElements(internal_seos);
            //             unit->setAdditionalElement_seo(R, Rj, cj_leg3, C, Vd_seo, leg3);
            //             unit->setOnewaySeoParam(R, Rj, cj_leg2, cj_leg3, C, Vd_oneway);
            //             oneway_CtoD_downB.setElement(y, x, unit);
            //             if(x > 0 && x < command_downB.numCols() && y > 0 && y < command_downB.numRows()){
            //                 unit->setOnewayConnections(
            //                     {command_downB.getElement(y,x)},
            //                     {detection_down.getElement(y,x),oneway_DtoC_downtoleftA.getElement(y,x)->getInternalElement(0)}
            //                 );
            //             }
            //         }

            //         {
            //             // oneway_DtoC_downtoleftA(SEO)
            //             auto unit = std::make_shared<OnewayUnit6and>("default","start");
            //             std::array<std::shared_ptr<BaseElement>, 7> internal_seos;
            //             for (int i = 0; i < 7; ++i) internal_seos[i] = std::make_shared<SEO>();
            //             unit->setInternalElements(internal_seos);
            //             unit->setAdditionalElement_seo(R, Rj, cj_leg4, C, Vd_seo, leg4);
            //             unit->setOnewaySeoParam(R, Rj, cj_leg2, cj_leg3, C, Vd_oneway);
            //             oneway_DtoC_downtoleftA.setElement(y, x, unit);
            //             if(x > 0 && x < command_leftA.numCols() && y > 0 && y < command_leftA.numRows()){
            //                 unit->setOnewayConnections(
            //                     {detection_down.getElement(y,x),oneway_CtoD_downB.getElement(y,x)->getInternalElement(6),oneway_DtoC_downtoleftB.getElement(y,x)->getInternalElement(0)},
            //                     {command_leftA.getElement(y,x)}
            //                 );
            //             }
            //         }
            //         {
            //             // oneway_DtoC_downtoleftB(SEO)
            //             auto unit = std::make_shared<OnewayUnit6and>("default","start");
            //             std::array<std::shared_ptr<BaseElement>, 7> internal_seos;
            //             for (int i = 0; i < 7; ++i) internal_seos[i] = std::make_shared<SEO>();
            //             unit->setInternalElements(internal_seos);
            //             unit->setAdditionalElement_seo(R, Rj, cj_leg4, C, Vd_seo, leg4);
            //             unit->setOnewaySeoParam(R, Rj, cj_leg2, cj_leg3, C, Vd_oneway);
            //             oneway_DtoC_downtoleftB.setElement(y, x, unit);
            //             if(x > 0 && x < command_leftB.numCols() && y > 0 && y < command_leftB.numRows()){
            //                 unit->setOnewayConnections(
            //                     {detection_down.getElement(y,x),oneway_CtoD_downA.getElement(y,x)->getInternalElement(6),oneway_DtoC_downtoleftA.getElement(y,x)->getInternalElement(0)},
            //                     {command_leftB.getElement(y,x)}
            //                 );
            //             }
            //         }

            //         {
            //             // onway_command_leftA(MultiSEO)
            //             auto unit = std::make_shared<OnewayUnit6>("reverse");
            //             std::array<std::shared_ptr<BaseElement>, 6> internal_seos;
            //             for (int i = 0; i < 6; ++i) internal_seos[i] = std::make_shared<MultiSEO>();
            //             unit->setInternalElements(internal_seos);
            //             unit->setOnewayMultiSeoParam(R, Rj, multi_cj_leg2, multi_cj_leg3, C, Vd_oneway, multi_num);
            //             oneway_command_leftA.setElement(y, x, unit);
            //             if(x > 0 && x < command_leftA.numCols() && y > 0 && y < command_leftA.numRows() - 1){
            //                 unit->setOnewayConnections(command_leftA.getElement(y,x),command_leftA.getElement(y+1,x));
            //             }
            //         }
            //         {
            //             // onway_command_leftB(MultiSEO)
            //             auto unit = std::make_shared<OnewayUnit6>("reverse");
            //             std::array<std::shared_ptr<BaseElement>, 6> internal_seos;
            //             for (int i = 0; i < 6; ++i) internal_seos[i] = std::make_shared<MultiSEO>();
            //             unit->setInternalElements(internal_seos);
            //             unit->setOnewayMultiSeoParam(R, Rj, multi_cj_leg2, multi_cj_leg3, C, Vd_oneway, multi_num);
            //             oneway_command_leftB.setElement(y, x, unit);
            //             if(x > 0 && x < command_leftB.numCols() && y > 0 && y < command_leftB.numRows() - 1){
            //                 unit->setOnewayConnections(command_leftB.getElement(y,x),command_leftB.getElement(y+1,x));
            //             }
            //         }
            //     }
            // }
            for (int y = 0; y < oneway_command_downA.numRows(); ++y) {
                for (int x = 0; x < oneway_command_downA.numCols(); ++x) {

                    // --- (A) この (y,x) に関係する “全 oneway ユニット” を先に setElement だけする ---
                    auto mk_6  = [] { std::array<std::shared_ptr<BaseElement>,6> a{}; for(int i=0;i<6;++i) a[i]=std::make_shared<MultiSEO>(); return a; };
                    auto mk_7S = [] { std::array<std::shared_ptr<BaseElement>,7> a{}; for(int i=0;i<7;++i) a[i]=std::make_shared<SEO>();     return a; };

                    // 1) ↓命令A/B
                    auto u_cmdDownA = std::make_shared<OnewayUnit6>();
                    u_cmdDownA->setInternalElements(mk_6());
                    oneway_command_downA.setElement(y, x, u_cmdDownA);

                    auto u_cmdDownB = std::make_shared<OnewayUnit6>();
                    u_cmdDownB->setInternalElements(mk_6());
                    oneway_command_downB.setElement(y, x, u_cmdDownB);

                    // 2) 命令→検出（CtoD）A/B
                    auto u_c2dA = std::make_shared<OnewayUnit6and>("default","end");
                    u_c2dA->setInternalElements(mk_7S());
                    oneway_CtoD_downA.setElement(y, x, u_c2dA);

                    auto u_c2dB = std::make_shared<OnewayUnit6and>("default","end");
                    u_c2dB->setInternalElements(mk_7S());
                    oneway_CtoD_downB.setElement(y, x, u_c2dB);

                    // 3) 検出→命令左（DtoC）A/B
                    auto u_d2cA = std::make_shared<OnewayUnit6and>("default","start");
                    u_d2cA->setInternalElements(mk_7S());
                    oneway_DtoC_downtoleftA.setElement(y, x, u_d2cA);

                    auto u_d2cB = std::make_shared<OnewayUnit6and>("default","start");
                    u_d2cB->setInternalElements(mk_7S());
                    oneway_DtoC_downtoleftB.setElement(y, x, u_d2cB);

                    // 4) ←命令A/B
                    auto u_cmdLeftA = std::make_shared<OnewayUnit6>("reverse");
                    u_cmdLeftA->setInternalElements(mk_6());
                    oneway_command_leftA.setElement(y, x, u_cmdLeftA);

                    auto u_cmdLeftB = std::make_shared<OnewayUnit6>("reverse");
                    u_cmdLeftB->setInternalElements(mk_6());
                    oneway_command_leftB.setElement(y, x, u_cmdLeftB);

                    // --- (B) ここから “同じ (y,x)” の中でパラメータ設定＆接続 ---
                    // パラメータ
                    u_cmdDownA->setOnewayMultiSeoParam(R,Rj,multi_cj_leg2,multi_cj_leg3,C,Vd_oneway,multi_num);
                    u_cmdDownB->setOnewayMultiSeoParam(R,Rj,multi_cj_leg2,multi_cj_leg3,C,Vd_oneway,multi_num);
                    u_cmdLeftA->setOnewayMultiSeoParam(R,Rj,multi_cj_leg2,multi_cj_leg3,C,Vd_oneway,multi_num);
                    u_cmdLeftB->setOnewayMultiSeoParam(R,Rj,multi_cj_leg2,multi_cj_leg3,C,Vd_oneway,multi_num);

                    u_c2dA->setAdditionalElement_seo(R,Rj,cj_leg3,C,Vd_seo,leg3);
                    u_c2dA->setOnewaySeoParam(R,Rj,cj_leg2,cj_leg3,C,Vd_oneway);
                    u_c2dB->setAdditionalElement_seo(R,Rj,cj_leg3,C,Vd_seo,leg3);
                    u_c2dB->setOnewaySeoParam(R,Rj,cj_leg2,cj_leg3,C,Vd_oneway);

                    u_d2cA->setAdditionalElement_seo(R,Rj,cj_leg4,C,Vd_adjust,leg4);
                    u_d2cA->setOnewaySeoParam(R,Rj,cj_leg2,cj_leg3,C,Vd_oneway);
                    u_d2cB->setAdditionalElement_seo(R,Rj,cj_leg4,C,Vd_adjust,leg4);
                    u_d2cB->setOnewaySeoParam(R,Rj,cj_leg2,cj_leg3,C,Vd_oneway);

                    // 接続（この時点で u_c2dA/u_d2cB 等 “相互参照先” も既に存在）
                    if (x>0 && x<command_downA.numCols()-1 && y>0 && y<command_downA.numRows()-1) {
                    u_cmdDownA->setOnewayConnections(command_downA.getElement(y,x), command_downA.getElement(y+1,x));
                    u_cmdDownB->setOnewayConnections(command_downB.getElement(y,x), command_downB.getElement(y+1,x));
                    }

                    if (x>0 && x<command_downA.numCols()-1 && y>0 && y<command_downA.numRows()-1) {
                    u_c2dA->setOnewayConnections(
                        { command_downA.getElement(y,x) },
                        { detection_down.getElement(y,x), u_d2cB->getInternalElement(0) }
                    );
                    u_c2dB->setOnewayConnections(
                        { command_downB.getElement(y,x) },
                        { detection_down.getElement(y,x), u_d2cA->getInternalElement(0) }
                    );
                    }

                    if (x>0 && x<command_leftA.numCols()-1 && y>0 && y<command_leftA.numRows()-1) {
                    u_d2cA->setOnewayConnections(
                        { detection_down.getElement(y,x), u_c2dB->getInternalElement(6), u_d2cB->getInternalElement(0) },
                        { command_leftA.getElement(y,x) }
                    );
                    u_d2cB->setOnewayConnections(
                        { detection_down.getElement(y,x), u_c2dA->getInternalElement(6), u_d2cA->getInternalElement(0) },
                        { command_leftB.getElement(y,x) }
                    );
                    // ←命令（縦つなぎ）
                    u_cmdLeftA->setOnewayConnections(command_leftA.getElement(y,x+1), command_leftA.getElement(y,x));
                    u_cmdLeftB->setOnewayConnections(command_leftB.getElement(y,x+1), command_leftB.getElement(y,x));
                    }
                }
            }


            // === 接続情報 ===
            // 命令方向回路 (command_down, command_up)
            for (int y = 1; y < command_downA.numRows() - 1; ++y) {
                for (int x = 1; x < command_downA.numCols() - 1; ++x) {
                    {
                        // command_downA
                        auto elem = command_downA.getElement(y, x);
                        std::vector<std::shared_ptr<BaseElement>> neighbors;
                        neighbors.push_back(oneway_command_downA.getElement(y - 1,x)->getInternalElement(5)); // 下方向命令の一方通行（前）
                        neighbors.push_back(oneway_command_downA.getElement(y,x)->getInternalElement(0)); // 下方向命令の一方通行（次）
                        neighbors.push_back(oneway_CtoD_downA.getElement(y,x)->getInternalElement(0)); // 命令から衝突まで
                        elem->setConnections(neighbors);
                    }
                    {
                        // command_downB
                        auto elem = command_downB.getElement(y, x);
                        std::vector<std::shared_ptr<BaseElement>> neighbors;
                        neighbors.push_back(oneway_command_downB.getElement(y - 1,x)->getInternalElement(5)); // 下方向命令の一方通行（前）
                        neighbors.push_back(oneway_command_downB.getElement(y,x)->getInternalElement(0)); // 下方向命令の一方通行（次）
                        neighbors.push_back(oneway_CtoD_downB.getElement(y,x)->getInternalElement(0)); // 命令から衝突まで
                        elem->setConnections(neighbors);
                    }
                }
            }
            // 命令方向回路(command_left, command_right)
            for (int y = 1; y < command_leftA.numRows() - 1; ++y) {
                for (int x = 1; x < command_leftA.numCols() - 1; ++x) {
                    {
                        // command_leftA
                        auto elem = command_leftA.getElement(y, x);
                        std::vector<std::shared_ptr<BaseElement>> neighbors;
                        neighbors.push_back(oneway_DtoC_downtoleftA.getElement(y,x)->getInternalElement(6)); // 下方向衝突判定
                        neighbors.push_back(oneway_command_leftA.getElement(y,x - 1)->getInternalElement(5)); // 左方向命令の一方通行（前）
                        neighbors.push_back(oneway_command_leftA.getElement(y,x)->getInternalElement(0)); // 左方向命令の一方通行（次）
                        // neighbors.push_back(oneway_CtoD_left.getElement(y,x)->getInternalElement(0)); // 命令から衝突まで
                        elem->setConnections(neighbors);
                    }
                    {
                        // command_leftB
                        auto elem = command_leftB.getElement(y, x);
                        std::vector<std::shared_ptr<BaseElement>> neighbors;
                        neighbors.push_back(oneway_DtoC_downtoleftB.getElement(y,x)->getInternalElement(6)); // 下方向衝突判定
                        neighbors.push_back(oneway_command_leftB.getElement(y,x - 1)->getInternalElement(5)); // 左方向命令の一方通行（前）
                        neighbors.push_back(oneway_command_leftB.getElement(y,x)->getInternalElement(0)); // 左方向命令の一方通行（次）
                        // neighbors.push_back(oneway_CtoD_left.getElement(y,x)->getInternalElement(0)); // 命令から衝突まで
                        elem->setConnections(neighbors);
                    }
                }
            }
            // 衝突判定回路（detection_down, detection_left, detection_up, detection_right）
            for (int y = 1; y < detection_down.numRows() - 1; ++y) {
                for (int x = 1; x < detection_down.numCols() - 1; ++x) {
                    {
                        // detection_down
                        auto elem = detection_down.getElement(y, x);
                        std::vector<std::shared_ptr<BaseElement>> neighbors;
                        neighbors.push_back(oneway_CtoD_downA.getElement(y, x)->getInternalElement(6)); // 命令から衝突まで
                        neighbors.push_back(oneway_CtoD_downB.getElement(y, x)->getInternalElement(6));
                        neighbors.push_back(oneway_DtoC_downtoleftA.getElement(y, x)->getInternalElement(0));
                        neighbors.push_back(oneway_DtoC_downtoleftB.getElement(y, x)->getInternalElement(0));
                        elem->setConnections(neighbors);
                    }
                }
            }

            // バイアス電圧を迷路状に設定
            setMazeBiasForEachParticle(command_downA,maze,Vd_seo);
            setMazeBiasForEachParticle(command_downB,maze,Vd_seo);
            setMazeBiasForEachParticle(command_leftA,maze,Vd_seo);
            setMazeBiasForEachParticle(command_leftB,maze,Vd_seo);

            // MultiSEO
            // setMazeBiasWithDirection_multi(detection_down,maze,"down",Vd_seo, multi_num, multi_cj_leg2, multi_cj_leg3);

            // SEO
            setMazeBiasWithDirection(detection_down,maze,"down",-Vd_detec);

            // === シミュレーション初期化 ===
            Sim sim(dt, endtime);
            sim.addGrid({
                command_downA, command_downB, command_leftA, command_leftB,
                detection_down,
                oneway_command_downA,oneway_command_downB, oneway_CtoD_downA, oneway_CtoD_downB, oneway_DtoC_downtoleftA,oneway_DtoC_downtoleftB,
                oneway_command_leftA,oneway_command_leftB, 
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
            sim.addVoltageTrigger(100, &command_downA, 1, 6, 0.002);//A
            sim.addVoltageTrigger(100, &command_downA, 2, 8, 0.002);//A
            sim.addVoltageTrigger(100, &command_downB, 1, 8, 0.002);//B
            sim.addVoltageTrigger(100, &command_downA, 1, 12, 0.002);//A
            sim.addVoltageTrigger(100, &command_downB, 2, 12, 0.002);//B

            // トリガとしてパルス波を入力
            for (int y = 0; y < detection_down.numRows(); ++y) {
                for (int x = 0; x < detection_down.numCols(); ++x) {
                    for (int interval = 10; interval < endtime; interval += 10){
                        sim.addVoltageTrigger(interval, &detection_down, y, x, -0.0025, 1);
                        // sim.addVoltageTrigger(interval, &detection_down, y, x, 0, 1);
                    }
                }
            }

            // // // gnuplot追跡用
            // std::vector<std::shared_ptr<BaseElement>> tracked_command_down = {
            //     command_downA.getElement(3,6), // oA1 A
            //     command_downA.getElement(5,8), // oA2 B
            //     command_downB.getElement(5,8), // oB3 C
            //     command_downA.getElement(6,8), // oA4 D
            //     command_downB.getElement(6,8), // oB5 E
            //     command_downA.getElement(8,12), // oA6 F
            //     command_downB.getElement(8,12), // oB7 G
            //     command_downA.getElement(9,12), // oA8 H
            //     command_downB.getElement(9,12), // oB9 I
            // };
            std::vector<std::shared_ptr<BaseElement>> tracked_detec = {
                detection_down.getElement(2,6), // xAB10 J
                detection_down.getElement(3,6), // oA11 K
                detection_down.getElement(5,8), // oAB12 L
                detection_down.getElement(6,8), // oA13 M
                detection_down.getElement(8,12), // oAB14 N
                detection_down.getElement(9,12), // oB15 O
            };
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

            // auto ofs1 = std::make_shared<std::ofstream>("output/new_structure_commD.txt");
            // sim.addSelectedElements(ofs1, tracked_command_down);
            // sim.generateGnuplotScript("output/new_structure_commD.txt", {"3-6A","5-8A","5-8B","6-8A","6-8B","8-12A","8-12B","9-12A","9-12B"});

            auto ofs2 = std::make_shared<std::ofstream>("output/new_structure_detec"+ std::to_string(trial) +".txt");
            sim.addSelectedElements(ofs2, tracked_detec);
            sim.generateGnuplotScript("output/new_structure_detec"+ std::to_string(trial) +".txt", {"2-6","3-6","5-8","6-8","8-12","9-12"});

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
        // std::string filename = "output/onlyB_new.csv";
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
