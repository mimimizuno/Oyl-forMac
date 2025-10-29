// seo particle computation test
// --------------------------------------------------------------------------------
#include <iostream>
#include <fstream>
#include <memory>
#include <vector>
#include "seo_class.hpp"
#include "oneway_unit.hpp"
#include "oneway_unit_6.hpp"
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
constexpr double R = 2;
constexpr double R_small = 0.8;
constexpr double Rj = 0.001;
constexpr double C = 2.0;
constexpr double dt = 0.1;
constexpr double endtime = 400;
constexpr double setVth = 0.004;

double cj_leg2 = seo_junction_cj_calc(leg2, C, setVth);
double cj_leg3 = seo_junction_cj_calc(leg3, C, setVth);
double cj_leg4 = seo_junction_cj_calc(leg4, C, setVth);
double cj_leg5 = seo_junction_cj_calc(leg5, C, setVth);
double cj_leg6 = seo_junction_cj_calc(leg6, C, setVth);
double cj_leg7 = seo_junction_cj_calc(leg7, C, setVth);
double cj_leg8 = seo_junction_cj_calc(leg8, C, setVth);

constexpr int repeat_count = 100;
constexpr int junction_num_max = 3;
constexpr double max_pulse_voltage = 0.003;
// constexpr double oneway_tunnel_ratio = 1.5;

using Grid = Grid2D<BaseElement>;
using Sim = Simulation2D<BaseElement>;

int main()
{
    std::vector<std::vector<int>> maze = {
        {0,0,0,0,0,1,0,1,0,1,0,1,0,0,0},
        {0,0,0,0,0,1,0,1,0,1,0,1,0,0,0},
        {1,1,1,1,1,1,0,1,0,1,0,1,0,0,0},
        {0,0,0,0,0,0,0,1,0,1,0,1,0,0,0},
        {1,1,1,1,1,1,1,1,0,1,0,1,0,0,0},
        {0,0,0,0,0,0,0,0,0,1,0,1,0,0,0},
        {1,1,1,1,1,1,1,1,1,1,0,1,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,1,0,0,0},
        {1,1,1,1,1,1,1,1,1,1,1,1,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    };

    for(double pulse_voltage = 0.003; pulse_voltage <= max_pulse_voltage; pulse_voltage+=0.001){
        int multi_num = junction_num_max;
        double multi_cj_leg2 = multi_junction_cj_calc(multi_num, leg2, C, setVth); // 引数の条件に合わせたCjを定義
        double multi_cj_leg3 = multi_junction_cj_calc(multi_num, leg3, C, setVth);
        double multi_cj_leg4 = multi_junction_cj_calc(multi_num, leg4, C, setVth);
        double multi_cj_leg5 = multi_junction_cj_calc(multi_num, leg5, C, setVth);
        double multi_cj_leg6 = multi_junction_cj_calc(multi_num, leg6, C, setVth);
        // double Vd_detec_multi = Vd_seo - 2 * multi_tunnelV(C,leg4,multi_cj_leg4,multi_cj_leg3,junction_num_max);
        double Vd_detec = Vd_seo - tunnelV(C,leg5,leg3,cj_leg5,cj_leg3);

        std::vector<std::vector<double>> allResults;
        for(int trial = 0; trial < repeat_count; trial++){
            std::cout << "multi_num = " << pulse_voltage << " trial = " << trial << std::endl;
            // === Gridを生成 ===
            Grid command_down(size_y, size_x * particles - 2);                    // 命令方向回路（下）
            Grid detection_downA(size_y, size_x,false);                  // 衝突方向回路（下）A
            Grid detection_downB(size_y, size_x,false);                  // 衝突方向回路（下）B
            Grid command_left(size_y * particles - 2, size_x);                    // 命令方向回路（左）
            Grid oneway_command_down(size_y * particles, size_x * particles, false);  // 命令方向回路（下）における一方通行回路
            Grid oneway_CtoD_downA(size_y * particles, size_x * particles, false);     // 命令方向回路（下）から衝突判定回路（下）をつなぐ一方通行回路A
            Grid oneway_CtoD_downB(size_y * particles, size_x * particles, false);     // 命令方向回路（下）から衝突判定回路（下）をつなぐ一方通行回路B
            Grid oneway_DtoC_downtoleftA(size_y * particles, size_x * particles, false);   // 衝突判定回路（下）から命令方向回路（左）をつなぐ一方通行回路A
            Grid oneway_DtoC_downtoleftB(size_y * particles, size_x * particles, false);   // 衝突判定回路（下）から命令方向回路（左）をつなぐ一方通行回路B
            Grid oneway_command_left(size_y * particles, size_x * particles, false);  // 命令方向回路（左）における一方通行回路

            // 動画出力するgridに名前をつける
            command_down.setOutputLabel("command_down");
            command_left.setOutputLabel("command_left");

            // === 全グリッド初期化 ===
            // 命令方向回路 (command_down, command_up)
            for (int y = 0; y < command_down.numRows(); ++y) {
                for (int x = 0; x < command_down.numCols(); ++x) {
                    // MultiSEO
                    // {
                    //     auto seo = std::make_shared<MultiSEO>();
                    //     seo->setUp(R, Rj, multi_cj_leg6, C, Vd_seo, leg6, multi_num);
                    //     command_down.setElement(y, x, seo);
                    // }
                    // SEO
                    {
                        auto seo = std::make_shared<SEO>();
                        seo->setUp(R, Rj, cj_leg8, C, Vd_seo, leg8);
                        command_down.setElement(y, x, seo);
                    }
                }
            }
            // 命令方向回路(command_left, command_right)
            for (int y = 0; y < command_left.numRows(); ++y) {
                for (int x = 0; x < command_left.numCols(); ++x) {
                    // MultiSEO
                    // {
                    //     auto seo = std::make_shared<MultiSEO>();
                    //     seo->setUp(R, Rj, multi_cj_leg6, C, Vd_seo, leg6, multi_num);
                    //     command_left.setElement(y, x, seo);
                    // }
                    // SEO
                    {
                        auto seo = std::make_shared<SEO>();
                        seo->setUp(R, Rj, cj_leg8, C, Vd_seo, leg8);
                        command_left.setElement(y, x, seo);
                    }
                }
            }

            // 衝突判定回路 (detection_down, detection_up, detection_left, detection_right)
            for (int y = 0; y < detection_downA.numRows(); ++y) {
                for (int x = 0; x < detection_downA.numCols(); ++x) {
                    // MultiSEO
                    // {
                    //     auto seo = std::make_shared<MultiSEO>();
                    //     seo->setUp(R, Rj, multi_cj_leg4, C, Vd_detec, leg4, multi_num);
                    //     detection_down.setElement(y, x, seo);
                    // }
                    //SEO
                    {
                        // pulse無し
                        auto seo = std::make_shared<SEO>();
                        seo->setUp(R, Rj, cj_leg5, C, Vd_seo, leg5);
                        detection_downA.setElement(y, x, seo);
                    }
                    {
                        // pulse無し
                        auto seo = std::make_shared<SEO>();
                        seo->setUp(R, Rj, cj_leg5, C, Vd_seo, leg5);
                        detection_downB.setElement(y, x, seo);
                    }
                }
            }

            // oneway回路 (oneway_command_down, oneway_CtoD_down, ...)
            for (int y = 0; y < oneway_command_down.numRows(); ++y) {
                for (int x = 0; x < oneway_command_down.numCols(); ++x) {
                    {
                        // onway_command_down(MultiSEO)
                        auto unit = std::make_shared<OnewayUnit6>();
                        std::array<std::shared_ptr<BaseElement>, 6> internal_seos;
                        for (int i = 0; i < 6; ++i) internal_seos[i] = std::make_shared<MultiSEO>();
                        unit->setInternalElements(internal_seos);
                        unit->setOnewayMultiSeoParam(R, Rj, multi_cj_leg2, multi_cj_leg3, C, Vd_oneway, multi_num);
                        oneway_command_down.setElement(y, x, unit);
                        if(x > 0 && x < command_down.numCols() && y > 0 && y < command_down.numRows() - 1){
                            unit->setOnewayConnections(command_down.getElement(y,x),command_down.getElement(y+1,x));
                        }
                    }
                    {
                        // onway_command_left(MultiSEO)
                        auto unit = std::make_shared<OnewayUnit6>("reverse");
                        std::array<std::shared_ptr<BaseElement>, 6> internal_seos;
                        for (int i = 0; i < 6; ++i) internal_seos[i] = std::make_shared<MultiSEO>();
                        unit->setInternalElements(internal_seos);
                        unit->setOnewayMultiSeoParam(R, Rj, multi_cj_leg2, multi_cj_leg3, C, Vd_oneway, multi_num);
                        oneway_command_left.setElement(y, x, unit);
                        if(x > 0 && x < command_left.numCols() - 1 && y > 0 && y < command_left.numRows()){
                            unit->setOnewayConnections(command_left.getElement(y,x),command_left.getElement(y,x+1));
                        }
                    }
                }
            }
            // 衝突判定回路に接続される一方通行回路
            for (int y = 0; y < oneway_CtoD_downA.numRows(); ++y) {
                for (int x = 0; x < oneway_CtoD_downA.numCols(); ++x) {
                    {
                        // oneway_CtoD_downA(SEO) detecAに接続する一方通行
                        auto unit = std::make_shared<OnewayUnit6>();
                        std::array<std::shared_ptr<BaseElement>, 6> internal_seos;
                        for (int i = 0; i < 6; ++i) internal_seos[i] = std::make_shared<SEO>();
                        unit->setInternalElements(internal_seos);
                        unit->setOnewaySeoParam(R, Rj, cj_leg2, cj_leg3, C, Vd_oneway);
                        oneway_CtoD_downA.setElement(y, x, unit);
                        if(x > 0 && x < command_down.numCols() && y > 0 && y < command_down.numRows()){
                            int cordinated_x = x / 2;
                            if(x % particles == 1){
                                unit->setOnewayConnections(command_down.getElement(y,x),detection_downA.getElement(y,cordinated_x + 1));
                            }
                            else {
                                unit->setOnewayConnections(command_down.getElement(y,x),detection_downA.getElement(y,cordinated_x));
                            }
                        }
                    }
                    {
                        // oneway_CtoD_downB(SEO) detecBに接続する一方通行
                        auto unit = std::make_shared<OnewayUnit6>();
                        std::array<std::shared_ptr<BaseElement>, 6> internal_seos;
                        for (int i = 0; i < 6; ++i) internal_seos[i] = std::make_shared<SEO>();
                        unit->setInternalElements(internal_seos);
                        unit->setOnewaySeoParam(R, Rj, cj_leg2, cj_leg3, C, Vd_oneway);
                        oneway_CtoD_downB.setElement(y, x, unit);
                        if(x > 0 && x < command_down.numCols() && y > 0 && y < command_down.numRows()){
                            int cordinated_x = x / 2;
                            if(x % particles == 1){
                                unit->setOnewayConnections(command_down.getElement(y,x),detection_downB.getElement(y,cordinated_x + 1));
                            }
                            else {
                                unit->setOnewayConnections(command_down.getElement(y,x),detection_downB.getElement(y,cordinated_x));
                            }
                        }
                    }
                    {
                        // oneway_DtoC_downtoleftA(SEO) detecAから出る一方通行
                        auto unit = std::make_shared<OnewayUnit6>();
                        std::array<std::shared_ptr<BaseElement>, 6> internal_seos;
                        for (int i = 0; i < 6; ++i) internal_seos[i] = std::make_shared<SEO>();
                        unit->setInternalElements(internal_seos);
                        unit->setOnewaySeoParam(R, Rj, cj_leg2, cj_leg3, C, Vd_oneway);
                        oneway_DtoC_downtoleftA.setElement(y, x, unit);
                        if(x > 0 && x < command_left.numCols() && y > 0 && y < command_left.numRows()){
                            int cordinated_y = y / 2;
                            if(y % particles == 1){
                                unit->setOnewayConnections(detection_downA.getElement(cordinated_y + 1, x),command_left.getElement(y, x));
                            }
                            else {
                                unit->setOnewayConnections(detection_downA.getElement(cordinated_y, x),command_left.getElement(y, x));
                            }
                        }
                    }
                    {
                        // oneway_DtoC_downtoleftB(SEO) detecBから出る一方通行
                        auto unit = std::make_shared<OnewayUnit6>();
                        std::array<std::shared_ptr<BaseElement>, 6> internal_seos;
                        for (int i = 0; i < 6; ++i) internal_seos[i] = std::make_shared<SEO>();
                        unit->setInternalElements(internal_seos);
                        unit->setOnewaySeoParam(R, Rj, cj_leg2, cj_leg3, C, Vd_oneway);
                        oneway_DtoC_downtoleftB.setElement(y, x, unit);
                        if(x > 0 && x < command_left.numCols() && y > 0 && y < command_left.numRows()){
                            int cordinated_y = y / 2;
                            if(y % particles == 1){
                                unit->setOnewayConnections(detection_downB.getElement(cordinated_y + 1, x),command_left.getElement(y, x));
                            }
                            else {
                                unit->setOnewayConnections(detection_downB.getElement(cordinated_y, x),command_left.getElement(y, x));
                            }
                        }
                    }
                }
            }

            // === 接続情報 ===
            // 命令方向回路 (command_down, command_up)
            for (int y = 1; y < command_down.numRows() - 1; ++y) {
                for (int x = 1; x < command_down.numCols() - 1; ++x) { // xが2倍-2
                    int cordinated_x = x / 2;
                    int cordinated_y = y * 2; 
                    {
                        // command_down
                        auto elem = command_down.getElement(y, x);
                        std::vector<std::shared_ptr<BaseElement>> neighbors;
                        if(x % particles == 1){ // 奇数インデックス
                            // neighbors.push_back(command_right.getElement(cordinated_y, cordinated_x + 1));
                            neighbors.push_back(command_left.getElement(cordinated_y, cordinated_x + 1));
                        }
                        else { // 偶数インデックス
                            // neighbors.push_back(command_right.getElement(cordinated_y - 1, cordinated_x));
                            neighbors.push_back(command_left.getElement(cordinated_y - 1, cordinated_x));
                        }
                        // neighbors.push_back(oneway_DtoC_righttodown.getElement(y,x)->getInternalElement(5)); // 右方向衝突判定
                        neighbors.push_back(oneway_command_down.getElement(y - 1,x)->getInternalElement(5)); // 下方向命令の一方通行（前）
                        neighbors.push_back(oneway_command_down.getElement(y,x)->getInternalElement(0)); // 下方向命令の一方通行（次）
                        neighbors.push_back(oneway_CtoD_downA.getElement(y,x)->getInternalElement(0)); // 命令から衝突までA
                        neighbors.push_back(oneway_CtoD_downB.getElement(y,x)->getInternalElement(0)); // 命令から衝突までB
                        elem->setConnections(neighbors);
                    }
                }
            }
            // 命令方向回路(command_left, command_right)
            for (int y = 1; y < command_left.numRows() - 1; ++y) {
                for (int x = 1; x < command_left.numCols() - 1; ++x) {
                    int cordinated_x = x * 2;
                    int cordinated_y = y / 2; 
                    {
                        // command_left
                        auto elem = command_left.getElement(y, x);
                        std::vector<std::shared_ptr<BaseElement>> neighbors;
                        if(y % particles == 1){ // 奇数インデックス
                            // neighbors.push_back(command_up.getElement(cordinated_y + 1, cordinated_x));
                            neighbors.push_back(command_down.getElement(cordinated_y + 1, cordinated_x));
                        }
                        else { // 偶数インデックス
                            // neighbors.push_back(command_up.getElement(cordinated_y, cordinated_x - 1));
                            neighbors.push_back(command_down.getElement(cordinated_y, cordinated_x - 1));
                        }
                        neighbors.push_back(oneway_DtoC_downtoleftA.getElement(y,x)->getInternalElement(5)); // 下方向衝突判定A
                        neighbors.push_back(oneway_DtoC_downtoleftB.getElement(y,x)->getInternalElement(5)); // 下方向衝突判定A
                        neighbors.push_back(oneway_command_left.getElement(y,x - 1)->getInternalElement(5)); // 左方向命令の一方通行（前）
                        neighbors.push_back(oneway_command_left.getElement(y,x)->getInternalElement(0)); // 左方向命令の一方通行（次）
                        // neighbors.push_back(oneway_CtoD_left.getElement(y,x)->getInternalElement(0)); // 命令から衝突まで
                        elem->setConnections(neighbors);
                    }
                }
            }
            // 衝突判定回路（detection_down, detection_left, detection_up, detection_right）
            for (int y = 1; y < detection_downA.numRows() - 1; ++y) {
                for (int x = 1; x < detection_downA.numCols() - 1; ++x) {
                    int cordinated_x = x * 2;
                    int cordinated_y = y * 2;
                    {
                        // detection_downA
                        auto elem = detection_downA.getElement(y, x);
                        std::vector<std::shared_ptr<BaseElement>> neighbors;
                        neighbors.push_back(oneway_CtoD_downA.getElement(y, cordinated_x - 1)->getInternalElement(5)); // 命令から衝突まで
                        neighbors.push_back(oneway_CtoD_downA.getElement(y, cordinated_x)->getInternalElement(5));
                        neighbors.push_back(oneway_DtoC_downtoleftA.getElement(cordinated_y - 1, x)->getInternalElement(0));
                        neighbors.push_back(oneway_DtoC_downtoleftA.getElement(cordinated_y, x)->getInternalElement(0));
                        elem->setConnections(neighbors);
                    }
                    {
                        // detection_downB
                        auto elem = detection_downB.getElement(y, x);
                        std::vector<std::shared_ptr<BaseElement>> neighbors;
                        neighbors.push_back(oneway_CtoD_downB.getElement(y, cordinated_x - 1)->getInternalElement(5)); // 命令から衝突まで
                        neighbors.push_back(oneway_CtoD_downB.getElement(y, cordinated_x)->getInternalElement(5));
                        neighbors.push_back(oneway_DtoC_downtoleftB.getElement(cordinated_y - 1, x)->getInternalElement(0));
                        neighbors.push_back(oneway_DtoC_downtoleftB.getElement(cordinated_y, x)->getInternalElement(0));
                        elem->setConnections(neighbors);
                    }
                }
            }

            // バイアス電圧を迷路状に設定
            setMazeBias(command_down,maze,"down",Vd_seo);
            setMazeBias(command_left,maze,"left",Vd_seo);
            // MultiSEO
            // setMazeBiasWithDirection_multi(detection_down,maze,"down",Vd_seo, multi_num, multi_cj_leg2, multi_cj_leg3);

            // SEO
            // setMazeBiasWithDirection(detection_downA,maze,"down",Vd_detec);
            setMazeBiasDetec(detection_downA,maze,"down", Vd_seo);
            setMazeBiasDetec(detection_downB,maze,"down",Vd_detec);


            // === シミュレーション初期化 ===
            Sim sim(dt, endtime);
            sim.addGrid({
                command_down, command_left,
                detection_downA,detection_downB,
                oneway_command_down, oneway_CtoD_downA, oneway_CtoD_downB, oneway_DtoC_downtoleftB, oneway_DtoC_downtoleftA,
                oneway_command_left, 
            });

            // === 特定素子の出力設定 ===
            std::vector<std::shared_ptr<BaseElement>> tracked = {
                command_down.getElement(3,11),
                command_down.getElement(3,12),
                command_down.getElement(5,15),
                command_down.getElement(5,16),
                command_down.getElement(7,19),
                command_down.getElement(7,20),
                command_down.getElement(9,23),
                command_down.getElement(9,24),

                detection_downA.getElement(3,6),
                detection_downB.getElement(3,6),
                detection_downB.getElement(5,8),
                detection_downA.getElement(5,8),
                detection_downB.getElement(7,10),
                detection_downA.getElement(7,10),
                detection_downB.getElement(9,12),
                detection_downA.getElement(9,12),

                command_left.getElement(5,6),
                command_left.getElement(6,6),
                command_left.getElement(9,8),
                command_left.getElement(10,8),
                command_left.getElement(13,10),
                command_left.getElement(14,10),
                command_left.getElement(17,12),
                command_left.getElement(18,12),

                command_left.getElement(5,1),
                command_left.getElement(6,1),
                command_left.getElement(9,1),
                command_left.getElement(10,1),
                command_left.getElement(13,1),
                command_left.getElement(14,1),
                command_left.getElement(17,1),
                command_left.getElement(18,1),
            };
            sim.addTrackedElements(tracked);

            // === トリガ設定 ===
            sim.addVoltageTrigger(250, &command_down, 1, 11, 0.002);
            sim.addVoltageTrigger(250, &command_down, 1, 15, 0.002);
            sim.addVoltageTrigger(250, &command_down, 1, 16, 0.002);
            sim.addVoltageTrigger(250, &command_down, 1, 20, 0.002);
            sim.addVoltageTrigger(250, &command_down, 1, 23, 0.002);
            sim.addVoltageTrigger(250, &command_down, 1, 24, 0.002);

            // トリガとしてパルス波を入力
            // for (int y = 0; y < detection_down.numRows(); ++y) {
            //     for (int x = 0; x < detection_down.numCols(); ++x) {
            //         for (int interval = 10; interval < endtime; interval += 20){
            //             sim.addVoltageTrigger(interval, &detection_down, y, x, pulse_voltage, 10);
            //         }
            //     }
            // }

            // auto selectedElements = {
            //     command_down.getElement(5,15),
            //     command_down.getElement(5,16),
            //     detection_downA.getElement(5,8),
            //     detection_downB.getElement(5,8),
            //     command_left.getElement(9,8),
            //     command_left.getElement(10,8),
            // };

            // auto selectedElements_solo = {
            //     command_down.getElement(3,11),
            //     command_down.getElement(3,12),
            //     detection_downA.getElement(3,6),
            //     detection_downB.getElement(3,6),
            //     command_left.getElement(5,6),
            //     command_left.getElement(6,6),
            // };

            // // gnuplot追跡用
            // auto ofsB = std::make_shared<std::ofstream>("output/speedcheckB"+std::to_string(pulse_voltage)+".txt");
            // sim.addSelectedElements(ofsB, selectedElements);
            // sim.generateGnuplotScript("output/speedcheckB"+std::to_string(pulse_voltage)+".txt", {"commandD5-15","commandD5-16","detecAD5-8","detecBD5-8","commandL9-8","commandL10-8"});

            // auto ofsA = std::make_shared<std::ofstream>("output/speedcheckA"+std::to_string(pulse_voltage)+".txt");
            // sim.addSelectedElements(ofsA, selectedElements_solo);
            // sim.generateGnuplotScript("output/speedcheckA"+std::to_string(pulse_voltage)+".txt", {"commandD5-15","commandD5-16","detecAD5-8","detecBD4-8","commandL9-8","commandL10-8"});
            // // // dE追跡用
            // auto ofsdE_B = std::make_shared<std::ofstream>("output/dEchecckerB"+std::to_string(pulse_voltage)+".csv");
            // sim.addSelecteddEElements(ofsdE_B, selectedElements);
            // auto ofsdE_A = std::make_shared<std::ofstream>("output/dEchecckerA"+std::to_string(pulse_voltage)+".csv");
            // sim.addSelecteddEElements(ofsdE_A, selectedElements_solo);

            // auto ofsC = std::make_shared<std::ofstream>("output/speedcheckC"+std::to_string(pulse_voltage)+".txt");
            // sim.addSelectedElements(ofsC, {
            //     command_down.getElement(7,20),
            //     detection_down.getElement(7,10),
            //     detection_down.getElement(6,10),
            //     command_left.getElement(13,10),
            //     command_left.getElement(14,10),
            // });
            // sim.generateGnuplotScript("output/speedcheckC"+std::to_string(pulse_voltage)+".txt", {"commandD7-20","detecD7-10","detecD6-10","commandL13-10","commandL14-10"});

            // auto ofs = std::make_shared<std::ofstream>("output/check-detec"+std::to_string(pulse_voltage)+".txt");
            // sim.addSelectedElements(ofs, {
            //     detection_down.getElement(5,8),
            //     oneway_CtoD_down.getElement(5, 15)->getInternalElement(5),
            //     oneway_CtoD_down.getElement(5, 16)->getInternalElement(5),
            //     oneway_DtoC_downtoleft.getElement(9, 8)->getInternalElement(0),
            //     oneway_DtoC_downtoleft.getElement(10, 8)->getInternalElement(0),
            //     detection_down.getElement(4,8),
            //     oneway_CtoD_down.getElement(4, 15)->getInternalElement(5),
            //     oneway_CtoD_down.getElement(4, 16)->getInternalElement(5),
            //     oneway_DtoC_downtoleft.getElement(7, 8)->getInternalElement(0),
            //     oneway_DtoC_downtoleft.getElement(8, 8)->getInternalElement(0),
            // });
            // sim.generateGnuplotScript("output/check-detec"+std::to_string(pulse_voltage)+".txt", 
            // {"detec5-8","onewayCtoD5-15","onewayCtoD5-16","onewayDtoC9-8","onewayDtoC10-8",
            // "detec4-8","onewayCtoD4-15","onewayCtoD4-16","onewayDtoC7-8","onewayDtoC8-8"});

            // === 実行 ===
            // while(sim.getTime() < endtime){
            //     // double rectangularV = getRectangularV(sim.getTime(),tunnelV(C,4,3,cj_leg4,cj_leg3),20,20);
            //     // double rectangularV = getRectangularV(sim.getTime(),multi_tunnelV(C,leg4,multi_cj_leg4,multi_cj_leg3,multi_junction_var), 10, 20);
            //     // double rectangularV = getRectangularV(sim.getTime(),0.00005, 10, 20);


            //     sim.runStep();
            //     sim.printProgressBar();
            // }
            // // 時刻記録
            sim.run();
            allResults.push_back(sim.getTunnelTimes());
        }

        // === CSV出力 ===
        // std::string filename = "output/onlyB_junction_" + std::to_string(pulse_voltage) + ".csv";
        std::string filename = "output/onlyB_junction_detecw.csv";
        std::ofstream ofs(filename);
        for (const auto& row : allResults) {
            for (size_t i = 0; i < row.size(); ++i) {
                ofs << row[i];
                if (i != row.size() - 1) ofs << ",";
            }
            ofs << "\n";
        }
    }
    return 0;
}
