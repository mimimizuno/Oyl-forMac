// seo particle computation test
// --------------------------------------------------------------------------------
#include <iostream>
#include <fstream>
#include <memory>
#include <vector>
#include "seo_class.hpp"
#include "oneway_unit.hpp"
#include "grid_2dim.hpp"
#include "simulation_2d.hpp"
#include "oyl_video.hpp"
#include "constants.hpp"
#include "particle_computation_methods.hpp"

constexpr double Vd_seo = 0.0039;
constexpr double Vd_oneway = 0.0039;
constexpr double R = 1.5;
constexpr double R_small = 0.8;
constexpr double Rj = 0.001;
constexpr double C = 2.0;
constexpr double dt = 0.1;
constexpr double endtime = 300;
constexpr double setVth = 0.004;

constexpr int test_size_y = 10;
constexpr int test_size_x = 2;

double cj_leg2 = seo_junction_cj_calc(leg2, C, setVth);
double cj_leg3 = seo_junction_cj_calc(leg3, C, setVth);
double cj_leg4 = seo_junction_cj_calc(leg4, C, setVth);
double cj_leg5 = seo_junction_cj_calc(leg5, C, setVth);
double cj_leg6 = seo_junction_cj_calc(leg6, C, setVth);
constexpr int repeat_count = 100;
constexpr int junction_num = 3;
constexpr double max_ratio = 1.0;

using Grid = Grid2D<BaseElement>;
using Sim = Simulation2D<BaseElement>;

int main()
{
    for(double  ratio = 1.0; ratio <= max_ratio; ratio+=0.01){
        int multi_num = junction_num;
        double multi_cj_leg2 = multi_junction_cj_calc(multi_num, leg2, C, setVth); // 引数の条件に合わせたCjを定義
        double multi_cj_leg3 = multi_junction_cj_calc(multi_num, leg3, C, setVth);
        double multi_cj_leg4 = multi_junction_cj_calc(multi_num, leg4, C, setVth);
        double multi_cj_leg5 = multi_junction_cj_calc(multi_num, leg5, C, setVth);
        double multi_cj_leg6 = multi_junction_cj_calc(multi_num, leg6, C, setVth);
        
        double ratio_multi = ratio - 0.2;

        std::vector<std::vector<double>> allResults;
        for(int trial = 0; trial < repeat_count; trial++){
            std::cout << "ratio = " << ratio << " ratio_multi = " << ratio_multi << " trial = " << trial << std::endl;
            // === Gridを生成 ===
            Grid sample_seo_single(test_size_y,test_size_x,false);
            Grid oneway_tester_single(test_size_y,test_size_x,false);
            Grid sample_seo_multi(test_size_y,test_size_x,false);
            Grid oneway_tester_multi(test_size_y,test_size_x,false);

            // === 全グリッド初期化 ===
            for (int y = 0; y < test_size_y; ++y) {
                for (int x = 0; x < test_size_x; ++x) {
                    // MultiSEO
                    {
                        auto seo = std::make_shared<MultiSEO>();
                        seo->setUp(R, Rj, multi_cj_leg6, C, Vd_seo, leg6, multi_num);
                        sample_seo_multi.setElement(y, x, seo);
                    }
                    // SEO
                    {
                        auto seo = std::make_shared<SEO>();
                        seo->setUp(R, Rj, cj_leg6, C, Vd_seo, leg6);
                        sample_seo_single.setElement(y, x, seo);
                    }
                }
            }

            // oneway回路
            for (int y = 0; y < test_size_y; ++y) {
                for (int x = 0; x < test_size_x; ++x) {
                    // MultiSEO
                    if(y < 5){
                        auto unit = std::make_shared<OnewayUnit>();
                        std::array<std::shared_ptr<BaseElement>, 4> internal_seos;
                        for (int i = 0; i < 4; ++i) internal_seos[i] = std::make_shared<MultiSEO>();
                        unit->setInternalElements(internal_seos);
                        unit->setOnewayMultiSeoParam(R, Rj, multi_cj_leg2, multi_cj_leg3, C, Vd_oneway, multi_num, ratio_multi);
                        oneway_tester_multi.setElement(y, x, unit);
                        if(x > 0 && y > 0 && y < test_size_y - 1){
                            unit->setOnewayConnections(sample_seo_multi.getElement(y,x),sample_seo_multi.getElement(y+1,x));
                        }
                    }
                    else{
                        auto unit = std::make_shared<OnewayUnit>("reverse");
                        std::array<std::shared_ptr<BaseElement>, 4> internal_seos;
                        for (int i = 0; i < 4; ++i) internal_seos[i] = std::make_shared<MultiSEO>();
                        unit->setInternalElements(internal_seos);
                        unit->setOnewayMultiSeoParam(R, Rj, multi_cj_leg2, multi_cj_leg3, C, Vd_oneway, multi_num, ratio_multi);
                        oneway_tester_multi.setElement(y, x, unit);
                        if(x > 0 && y > 0 && y < test_size_y - 1){
                            unit->setOnewayConnections(sample_seo_multi.getElement(y,x),sample_seo_multi.getElement(y+1,x));
                        }
                    }
                    // SEO
                    if(y < 5){
                        auto unit = std::make_shared<OnewayUnit>();
                        std::array<std::shared_ptr<BaseElement>, 4> internal_seos;
                        for (int i = 0; i < 4; ++i) internal_seos[i] = std::make_shared<SEO>();
                        unit->setInternalElements(internal_seos);
                        unit->setOnewaySeoParam(R, Rj, cj_leg2, cj_leg3, C, Vd_oneway, ratio);
                        oneway_tester_single.setElement(y, x, unit);
                        if(x > 0 && y > 0 && y < test_size_y - 1){
                            unit->setOnewayConnections(sample_seo_single.getElement(y,x),sample_seo_single.getElement(y+1,x));
                        }
                    }
                    else{
                        auto unit = std::make_shared<OnewayUnit>("reverse");
                        std::array<std::shared_ptr<BaseElement>, 4> internal_seos;
                        for (int i = 0; i < 4; ++i) internal_seos[i] = std::make_shared<SEO>();
                        unit->setInternalElements(internal_seos);
                        unit->setOnewaySeoParam(R, Rj, cj_leg2, cj_leg3, C, Vd_oneway, ratio);
                        oneway_tester_single.setElement(y, x, unit);
                        if(x > 0 && y > 0 && y < test_size_y - 1){
                            unit->setOnewayConnections(sample_seo_single.getElement(y,x),sample_seo_single.getElement(y+1,x));
                        }
                    }
                }
            }

            // === 接続情報 ===
            for (int y = 1; y < test_size_y; ++y) {
                for (int x = 0; x < test_size_x; ++x) {
                    {
                        auto elem = sample_seo_multi.getElement(y, x);
                        std::vector<std::shared_ptr<BaseElement>> neighbors;
                        neighbors.push_back(oneway_tester_multi.getElement(y - 1,x)->getInternalElement(3)); // 前
                        neighbors.push_back(oneway_tester_multi.getElement(y,x)->getInternalElement(0)); // 次
                        elem->setConnections(neighbors);
                    }
                    {
                        auto elem = sample_seo_single.getElement(y, x);
                        std::vector<std::shared_ptr<BaseElement>> neighbors;
                        neighbors.push_back(oneway_tester_single.getElement(y - 1,x)->getInternalElement(3)); // 前
                        neighbors.push_back(oneway_tester_single.getElement(y,x)->getInternalElement(0)); // 次
                        elem->setConnections(neighbors);
                    }
                }
            }
            
            // === シミュレーション初期化 ===
            Sim sim(dt, endtime);
            sim.addGrid({
                sample_seo_multi,
                sample_seo_single,
                oneway_tester_multi,
                oneway_tester_single,
            });

            // === 特定素子の出力設定 ===
            std::vector<std::shared_ptr<BaseElement>> tracked = {
                sample_seo_multi.getElement(5,1),
                sample_seo_multi.getElement(6,1),
                sample_seo_single.getElement(5,1),
                sample_seo_single.getElement(6,1),
            };
            sim.addTrackedElements(tracked);

            // === トリガ設定 ===
            sim.addVoltageTrigger(200, &sample_seo_multi, 1, 1, 0.004);
            sim.addVoltageTrigger(200, &sample_seo_single, 1, 1, 0.004);

            // auto selectedElements_multi = {
            //     sample_seo_multi.getElement(1,1),
            //     oneway_tester_multi.getElement(1, 1)->getInternalElement(0),
            //     oneway_tester_multi.getElement(1, 1)->getInternalElement(1),
            //     oneway_tester_multi.getElement(1, 1)->getInternalElement(2),
            //     oneway_tester_multi.getElement(1, 1)->getInternalElement(3),

            //     oneway_tester_multi.getElement(4, 1)->getInternalElement(0),
            //     oneway_tester_multi.getElement(4, 1)->getInternalElement(1),
            //     oneway_tester_multi.getElement(4, 1)->getInternalElement(2),
            //     oneway_tester_multi.getElement(4, 1)->getInternalElement(3),
            //     sample_seo_multi.getElement(5,1),
            //     oneway_tester_multi.getElement(5, 1)->getInternalElement(0),
            //     oneway_tester_multi.getElement(5, 1)->getInternalElement(1),
            //     oneway_tester_multi.getElement(5, 1)->getInternalElement(2),
            //     oneway_tester_multi.getElement(5, 1)->getInternalElement(3),
            // };

            // auto selectedElements_single = {
            //     oneway_tester_single.getElement(4, 1)->getInternalElement(0),
            //     oneway_tester_single.getElement(4, 1)->getInternalElement(1),
            //     oneway_tester_single.getElement(4, 1)->getInternalElement(2),
            //     oneway_tester_single.getElement(4, 1)->getInternalElement(3),
            //     sample_seo_single.getElement(5,1),
            //     oneway_tester_single.getElement(5, 1)->getInternalElement(0),
            //     oneway_tester_single.getElement(5, 1)->getInternalElement(1),
            //     oneway_tester_single.getElement(5, 1)->getInternalElement(2),
            //     oneway_tester_single.getElement(5, 1)->getInternalElement(3),
            // };

            // // gnuplot追跡用
            // auto ofs_multi = std::make_shared<std::ofstream>("output/multi_"+std::to_string(ratio_multi)+".txt");
            // sim.addSelectedElements(ofs_multi, selectedElements_multi);
            // sim.generateGnuplotScript("output/multi_"+std::to_string(ratio_multi)+".txt", {"1","1-0","1-1","1-2","1-3","4-0","4-1","4-2","4-3","seo","5-0","5-1","5-2","5-3"});

            // auto ofs_single = std::make_shared<std::ofstream>("output/single_"+std::to_string(ratio)+".txt");
            // sim.addSelectedElements(ofs_single, selectedElements_single);
            // sim.generateGnuplotScript("output/single_"+std::to_string(ratio)+".txt", {"4-0","4-1","4-2","4-3","seo","5-0","5-1","5-2","5-3"});
            
            // // 時刻記録
            sim.run();
            allResults.push_back(sim.getTunnelTimes());
        }

        // === CSV出力 ===
        std::string filename = "output/oneway_single" + std::to_string(ratio) + "_multi" + std::to_string(ratio_multi) + ".csv";
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
