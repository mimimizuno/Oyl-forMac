//------------------------------------------------------------------------------------------
// パルスのテスト
#include <iostream>
#include <fstream>
#include <memory>
#include <vector>
#include "seo_class.hpp"
#include "oneway_unit.hpp"
#include "grid_2dim.hpp"
#include "simulation_2d.hpp"
#include "constants.hpp"

constexpr int particles = 2;
constexpr double Vd_seo = 0.0039;
constexpr double Vd_oneway = 0.0039;
constexpr double R = 1.5;
constexpr double R_large = 2.5;
constexpr double R_small = 0.8;
constexpr double Rj = 0.001;
constexpr double C = 2.0;
constexpr double dt = 0.1;
constexpr double endtime = 600;
constexpr double Vth = 0.004;
double cj_leg6 = seo_junction_cj_calc(leg6,C,Vth);
double cj_leg4 = seo_junction_cj_calc(leg4,C,Vth);
double cj_leg3 = seo_junction_cj_calc(leg3,C,Vth);
double cj_leg2 = seo_junction_cj_calc(leg2,C,Vth);

using Grid = Grid2D<BaseElement>;
using Sim = Simulation2D<BaseElement>;

int main()
{
    Grid command(2, 4, false);
    Grid detec(2, 1, false);
    Grid oneway(2, 4, false);

    // 命令方向
    for (int y=0; y<2; y++){
        for(int x=0;x<4;x++){
            auto seo = std::make_shared<SEO>();
            seo->setUp(R, Rj, cj_leg6, C, Vd_seo, leg6);
            command.setElement(y, x, seo);
        }
    }
    
    // 衝突判定
    for(int y = 0;y < 2;y++){
        double vias = Vd_seo - 2 * tunnelV(C,4,3,cj_leg4,cj_leg3);
        auto seo = std::make_shared<SEO>();
        seo->setUp(R_large,Rj,cj_leg4,C,vias,leg4);
        detec.setElement(y,0,seo);
    }

    // 一方通行
    for(int y = 0;y < 2;y++){
        for(int x=0;x<4;x++){
            auto unit = std::make_shared<OnewayUnit>();
            std::array<std::shared_ptr<BaseElement>, 4> internal_seos;
            for (int i = 0; i < 4; ++i) internal_seos[i] = std::make_shared<SEO>();
            unit->setInternalElements(internal_seos);
            unit->setOnewaySeoParam(R, Rj, cj_leg2, cj_leg3, C, Vd_oneway);
            if(x == 0 || x == 1){
                unit->setOnewayConnections(command.getElement(y,x),detec.getElement(y,0));
            }
            else{
                unit->setOnewayConnections(detec.getElement(y,0),command.getElement(y,x));
            }
            oneway.setElement(y, x, unit);
        }
    }

    // 接続
    for(int y = 0; y < 2;y++){
        command.getElement(y,0)->setConnections({
            oneway.getElement(y,0)->getInternalElement(0),
            command.getElement(y,3),
        });
        command.getElement(y,1)->setConnections({
            oneway.getElement(y,1)->getInternalElement(0),
            command.getElement(y,2),
        });
        command.getElement(y,2)->setConnections({
            command.getElement(y,1),
            oneway.getElement(y,2)->getInternalElement(3),
        });
        command.getElement(y,3)->setConnections({
            command.getElement(y,0),
            oneway.getElement(y,3)->getInternalElement(3),
        });

        detec.getElement(y,0)->setConnections({
            oneway.getElement(y,0)->getInternalElement(3),
            oneway.getElement(y,1)->getInternalElement(3),
            oneway.getElement(y,2)->getInternalElement(0),
            oneway.getElement(y,3)->getInternalElement(0),
        });
    }

    // シミュレーション準備
    Sim sim(dt, endtime);
    sim.addGrid({command,detec,oneway});

    // トリガ
    sim.addVoltageTrigger(400, &command, 0, 0, 0.004);
    sim.addVoltageTrigger(410, &command, 1, 0, 0.004);

    // 出力ファイル設定
    auto ofs = std::make_shared<std::ofstream>("output/synchronized-test.txt");
    sim.addSelectedElements(ofs, {
        command.getElement(0,0),
        detec.getElement(0,0),
        command.getElement(0,2),
        command.getElement(1,0),
        detec.getElement(1,0),
        command.getElement(1,2),
    });
    sim.generateGnuplotScript("output/synchronized-test.txt", {"seo00","detec0","seo02","seo10","detec1","seo12"});

    auto ofs2 = std::make_shared<std::ofstream>("output/oneway-test2.txt");
    sim.addSelectedElements(ofs2, {
        detec.getElement(1,0),
        oneway.getElement(1,2)->getInternalElement(0),
        oneway.getElement(1,2)->getInternalElement(1),
        oneway.getElement(1,2)->getInternalElement(2),
        oneway.getElement(1,2)->getInternalElement(3),
        command.getElement(1,2),
    });
    sim.generateGnuplotScript("output/oneway-test2.txt", {"detec","oneway0","oneway1","oneway2","oneway3","seo2"});

    auto ofs3 = std::make_shared<std::ofstream>("output/oneway-test3.txt");
    sim.addSelectedElements(ofs3, {
        detec.getElement(1,0),
        oneway.getElement(1,0)->getInternalElement(3),
        oneway.getElement(1,1)->getInternalElement(3),
        oneway.getElement(1,2)->getInternalElement(0),
        oneway.getElement(1,3)->getInternalElement(0),
    });
    sim.generateGnuplotScript("output/oneway-test3.txt", {"detec","oneway0","oneway1","oneway2","oneway3"});

    // auto ofs1 = std::make_shared<std::ofstream>("../output/oneway-test1.txt");
    // sim.addSelectedElements(ofs1, {
    //     command.getElement(0,0),
    //     oneway.getElement(0,0)->getInternalElement(0),
    //     oneway.getElement(0,0)->getInternalElement(1),
    //     oneway.getElement(0,0)->getInternalElement(2),
    //     oneway.getElement(0,0)->getInternalElement(3),
    //     detec.getElement(0,0),
    // });
    // sim.generateGnuplotScript("../output/oneway-test1.txt", {"seo0","oneway0","oneway1","oneway2","oneway3","detec"});

    // 実行
    while(sim.getTime() < endtime){
        // double rectangularV = getRectangularV(sim.getTime(),tunnelV(C,4,3,cj_leg4,cj_leg3),20,20);
        double rectangularV = getRectangularV(sim.getTime(),tunnelV(C,4,3,cj_leg4,cj_leg3),20,50);
        sim.addVoltageTrigger(sim.getTime(), &detec, 0, 0, rectangularV);
        sim.addVoltageTrigger(sim.getTime(),&detec, 1, 0, rectangularV);
        // detec.getElement(0,0)->setVias(vias + rectangularV);
        sim.runStep();
    }

    std::cout << "Simulation finished. Output saved to shynchronized_test.txt" << std::endl;
    return 0;
}
