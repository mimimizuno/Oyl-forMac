#ifndef SIMULATION_2D_HPP
#define SIMULATION_2D_HPP

#include <fstream>
#include <vector>
#include <memory>
#include <utility>
#include <map>
#include <cmath>
#include "seo_class.hpp"
#include "grid_2dim.hpp"

template <typename Element>
class Simulation2D
{
private:
    double t;                           // 現在の時間（不定期に増える）
    double dt;                          // 基本刻み（参考値）
    double endtime;                     // 終了時刻
    double outputInterval;              // 出力間隔（例: 0.1）
    double nextOutputTime;              // 次に出力すべき時刻（0.1, 0.2, ...）
    std::vector<Grid2D<Element>> grids; // Grid2Dのインスタンス配列
    // oyl-video形式のデータ
    std::map<
        std::string,                                  // ラベル名
        std::vector<std::vector<std::vector<double>>> // [timeframe][y][x]
        >
        outputs;
    // トリガを表すベクトル（どのgridか、時刻、位置、値)
    std::vector<std::tuple<Grid2D<Element>*,double, int, int, double, double>> voltageTriggers; // (grid, time, x, y, V, applyTime(初期値1ns))
    // ファイル出力する素子をファイル名とともに格納するベクトル
    // std::vector<std::pair<std::ofstream*, std::shared_ptr<Element>>> selectedElements;
    // 複数素子を1つのファイルに出力するための構造
    std::vector<std::pair<std::shared_ptr<std::ofstream>, std::vector<std::shared_ptr<Element>>>> selectedElements;
    // dE出力対象
    std::vector<std::pair<std::shared_ptr<std::ofstream>, std::vector<std::shared_ptr<Element>>>> selecteddEElements;
    // 全データ出力対象
    std::vector<std::pair<std::shared_ptr<std::ofstream>, std::vector<std::shared_ptr<Element>>>> selectedAllDataElements;
    // トンネル情報を追跡する素子のベクトル
    std::vector<std::shared_ptr<Element>> trackedElements;
    // トンネルした素子を記録するための変数
    std::vector<double> tunnelTimes; 

public:
    // コンストラクタ(刻み時間,シミュレーションの終了タイミング)
    Simulation2D(double dT, double EndTime);

    // wtの比較（gridごと）
    std::pair<bool, std::shared_ptr<Grid2D<Element>>> comparewt();

    // トンネルの処理
    void handleTunnels(Grid2D<Element> &tunnelgrid);

    // ファイル作成
    void openFiles() const;

    // ファイル閉じる
    void closeFiles() const;

    // ファイル出力
    void outputToFile();

    // oyl-video形式に合わせた出力を生成
    void outputTooyl();

    // シミュレーションの1ステップ
    void runStep();

    // シミュレーションにgridを追加
    void addGrid(const std::vector<Grid2D<Element>> &Gridinstance);

    // シミュレーションの実行
    void run();

    // グリッド取得
    std::vector<Grid2D<Element>> &getGrids();

    // outputsを取得
    const std::map<std::string, std::vector<std::vector<std::vector<double>>>> &getOutputs() const;

    // トリガーを追加する
    void addVoltageTrigger(double triggerTime, Grid2D<Element>* grid, int y, int x, double voltage, double applyTime = 1.0);

    // トリガを適用させる
    void applyVoltageTriggers();

    // ファイル出力する素子とファイルを格納するベクトルに追加する
    void addSelectedElements(std::shared_ptr<std::ofstream> ofs, const std::vector<std::shared_ptr<Element>>& elems);

    // ファイル出力する素子とファイルを格納するベクトルに追加する（dEの出力用）
    void addSelecteddEElements(std::shared_ptr<std::ofstream> ofs, const std::vector<std::shared_ptr<Element>>& elems);

    // ファイル出力する素子とファイルを格納するベクトルに追加する（素子全データ出力用）
    void addSelectedAllDataElements(std::shared_ptr<std::ofstream> ofs, const std::vector<std::shared_ptr<Element>>& elems);

    // selectedElementsから該当する素子のVnを記録するファイル出力
    void outputSelectedElements();

    // selectedElementsから該当する素子のdEを記録するファイル出力
    void outputSelecteddE();

    // selectedElementsから該当する素子の全データを記録するファイル出力
    void outputSelectedAllData();

    // gnuplot用データ出力
    void generateGnuplotScript(const std::string& dataFilename, const std::vector<std::string>& labels);

    // 実行中の進捗状況を表示する
    void printProgressBar();

    // tを取得する
    double getTime() const;

    // トンネルしたかどうかを追跡する素子を追加する
    void addTrackedElements(const std::vector<std::shared_ptr<Element>>& elems);

    // トンネル状況を記録する変数を返す
    const std::vector<double>& getTunnelTimes() const;

};

// コンストラクタ
template <typename Element>
Simulation2D<Element>::Simulation2D(double dT, double EndTime)
    : t(0.0), dt(dT), endtime(EndTime), outputInterval(dT), nextOutputTime(0.0) {}

// 最小wtをもつgridを探索する（最小wtがdtより小さいかどうかのbool, 最小のwtを持つgrid）
template <typename Element>
std::pair<bool, std::shared_ptr<Grid2D<Element>>> Simulation2D<Element>::comparewt()
{
    double minwt = dt;
    std::shared_ptr<Grid2D<Element>> tunnelgrid = nullptr;
    for (auto &grid : grids)
    {
        if (grid.gridminwt(dt))
        {
            double candidate = grid.getMinWT();
            if (candidate < minwt)
            {
                minwt = candidate;
                tunnelgrid = std::make_shared<Grid2D<Element>>(grid);
            }
        }
    }
    if (minwt < dt) return {true, tunnelgrid};
    return {false, nullptr};
}


// トンネル処理を実行
template <typename Element>
void Simulation2D<Element>::handleTunnels(Grid2D<Element> &tunnelgrid)
{
    auto ptr = tunnelgrid.getTunnelPlace();
    if (!trackedElements.empty()) {
        // インデックス順に記録
        auto it = std::find(trackedElements.begin(), trackedElements.end(), ptr);
        if (it != trackedElements.end()) {
            size_t index = std::distance(trackedElements.begin(), it);
            if (tunnelTimes[index] == 0) {
                tunnelTimes[index] = t;
            }
        }
    }
    // 実際のトンネル処理
    ptr->setTunnel(tunnelgrid.getTunnelDirection());
    
}

template <typename Element>
void Simulation2D<Element>::addSelectedElements(std::shared_ptr<std::ofstream> ofs, const std::vector<std::shared_ptr<Element>>& elems)
{
    selectedElements.emplace_back(ofs, elems);
}

template <typename Element>
void Simulation2D<Element>::addSelecteddEElements(std::shared_ptr<std::ofstream> ofs, const std::vector<std::shared_ptr<Element>>& elems)
{
    selecteddEElements.emplace_back(ofs, elems);
}

template <typename Element>
void Simulation2D<Element>::addSelectedAllDataElements(std::shared_ptr<std::ofstream> ofs, const std::vector<std::shared_ptr<Element>>& elems)
{
    selectedAllDataElements.emplace_back(ofs, elems);
}

template <typename Element>
void Simulation2D<Element>::outputSelectedElements()
{
    for (auto& [ofsPtr, elemPtrs] : selectedElements)
    {
        if (!ofsPtr || !(*ofsPtr)) continue;  // ファイルが開けていない場合はスキップ

        (*ofsPtr) << t;
        for (const auto& elemPtr : elemPtrs)
        {
            if (elemPtr)
                (*ofsPtr) << " " << elemPtr->getVn();
            else
                (*ofsPtr) << " nan";  // nullポインタの場合の保険
        }
        (*ofsPtr) << std::endl;
    }
}

template <typename Element>
void Simulation2D<Element>::outputSelecteddE()
{
    for (auto& [ofsPtr, elemPtrs] : selecteddEElements)
    {
        if (!ofsPtr || !(*ofsPtr)) continue;  // ファイルが開けていない場合はスキップ

        (*ofsPtr) << t;
        for (const auto& elemPtr : elemPtrs)
        {
            if (elemPtr){
                (*ofsPtr) << "," << elemPtr->getdE().at("up");
                (*ofsPtr) << "," << elemPtr->getdE().at("down");
            }
            else (*ofsPtr) << ",nan,nan";  // nullポインタの場合の保険
        }
        (*ofsPtr) << std::endl;
    }
}

template <typename Element>
void Simulation2D<Element>::outputSelectedAllData()
{
    for (auto& [ofsPtr, elemPtrs] : selecteddEElements)
    {
        if (!ofsPtr || !(*ofsPtr)) continue;

        // --- 最初の1回だけヘッダーを書く ---
        if (ofsPtr->tellp() == 0)
        {
            (*ofsPtr)
                << "t"
                << ",Q"
                << ",Vn"
                << ",Vd"
                << ",R"
                << ",Rj"
                << ",Cj"
                << ",C"
                << ",legs"
                << ",V_sum"
                << ",dE_up"
                << ",dE_down"
                << ",wt_up"
                << ",wt_down"
                << "\n";
        }

        (*ofsPtr) << t;

        const auto& elemPtr = elemPtrs[0];

        if (!elemPtr)
        {
            // null の場合は全項目 nan
            (*ofsPtr) << ",nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan\n";
            continue;
        }

        // --- 基本パラメータ ---
        (*ofsPtr)
            << "," << elemPtr->getQ()
            << "," << elemPtr->getVn()
            << "," << elemPtr->getVd()
            << "," << elemPtr->getR()
            << "," << elemPtr->getRj()
            << "," << elemPtr->getCj()
            << "," << elemPtr->getC()
            << "," << elemPtr->getlegs()
            << "," << elemPtr->getSurroundingVsum();

        // --- dE(up/down) ---
        double dE_up   = elemPtr->getdE().count("up")   ? elemPtr->getdE().at("up")   : NAN;
        double dE_down = elemPtr->getdE().count("down") ? elemPtr->getdE().at("down") : NAN;

        (*ofsPtr)
            << "," << dE_up
            << "," << dE_down;

        // --- wt(up/down) ---
        double wt_up   = elemPtr->getwt().count("up")   ? elemPtr->getwt().at("up")   : NAN;
        double wt_down = elemPtr->getwt().count("down") ? elemPtr->getwt().at("down") : NAN;

        (*ofsPtr)
            << "," << wt_up
            << "," << wt_down
            << "\n";
    }
}

// oyl-video形式に合わせた出力を生成
// 1 <= x <= max-1, 1 <= y <= max-1の範囲で出力される(sizex=32,sizey=32の場合は1から31までの範囲で30×30になる)
template <typename Element>
void Simulation2D<Element>::outputTooyl()
{
    if (t >= nextOutputTime)
    {
        // 出力形式に合わせて整数値にならす
        int timeframe = static_cast<int>(std::round(nextOutputTime / outputInterval));
        int outputIndex = 0; // 出力順にindex付けするカウンタ

        for (const auto &grid : grids)
        {
            if (!grid.isOutputEnabled())
                continue;

            std::string label;
            if (grid.hasOutputLabel())
            {
                label = grid.getOutputLabel();
            }
            else
            {
                label = "output" + std::to_string(outputIndex);
                ++outputIndex;
            }

            int rows = grid.numRows();
            int cols = grid.numCols();

            std::vector<std::vector<double>> vnGrid(rows - 2, std::vector<double>(cols - 2));
            for (int i = 1; i < rows - 1; ++i)
            {
                for (int j = 1; j < cols - 1; ++j)
                {
                    auto elem = grid.getElement(i, j);
                    double vn = elem->getVn();
                    double vd = elem->getVd();
            
                    // Vdが負のとき、Vnを反転して記録
                    if (vd < 0) {
                        vn *= -1.0;
                    }
            
                    vnGrid[i-1][j-1] = vn;
                }
            }            

            outputs[label].resize(timeframe + 1);
            outputs[label][timeframe] = vnGrid;
        }
        nextOutputTime += outputInterval;
    }
}

// シミュレーションの1ステップを実行
template <typename Element>
void Simulation2D<Element>::runStep()
{
    double steptime = dt;

    // oyl-video形式に出力
    outputTooyl();
    // ファイル出力
    outputSelectedElements();
    outputSelecteddE();
    outputSelectedAllData();

    // grid全体のVn計算(5回計算してならす)
    for (int i = 0; i < 5; i++)
    {
        for (auto &grid : grids)
        {
            // 接続されている電圧を更新
            grid.updateGridSurVn();
            // トリガの適用
            applyVoltageTriggers();
            // 電圧を更新
            grid.updateGridVn();
        }
    }

    // grid全体のdE計算
    for (auto &grid : grids)
    {    
        grid.updateGriddE();
    }



    // wtの計算と比較
    auto compared = this->comparewt();
    if (compared.first)
    {
        handleTunnels(*compared.second);
        steptime = compared.second->getMinWT();
    }

    // チャージの計算
    for (auto &grid : grids)
    {
        grid.updateGridQn(steptime);
    }

    // tの増加
    t += steptime;
}

// Gridインスタンスの配列を登録
template <typename Element>
void Simulation2D<Element>::addGrid(const std::vector<Grid2D<Element>> &Gridinstance)
{
    grids = Gridinstance;
}

// 全体シミュレーションの実行
template <typename Element>
void Simulation2D<Element>::run()
{
    while (t < endtime)
    {
        runStep();
        printProgressBar();
    }
}

// グリッド取得
template <typename Element>
std::vector<Grid2D<Element>> &Simulation2D<Element>::getGrids()
{
    return grids;
}

template <typename Element>
const std::map<std::string, std::vector<std::vector<std::vector<double>>>> &Simulation2D<Element>::getOutputs() const
{
    return outputs;
}

template <typename Element>
void Simulation2D<Element>::addVoltageTrigger(double triggerTime, Grid2D<Element>* grid, int y, int x, double voltage, double applyTime) {
    voltageTriggers.emplace_back(grid, triggerTime, y, x, voltage, applyTime);
}

template <typename Element>
void Simulation2D<Element>::applyVoltageTriggers()
{
    for (const auto& [gridPtr, triggerTime, y, x, voltage, applyTime] : voltageTriggers) {
        if (t >= triggerTime && t < triggerTime + applyTime) {
            if (!gridPtr) {
                throw std::invalid_argument("Trigger references a null grid pointer.");
            }

            if (x < 0 || x >= gridPtr->numCols() || y < 0 || y >= gridPtr->numRows()) {
                throw std::out_of_range(
                    "Trigger coordinates (x=" + std::to_string(x) +
                    ", y=" + std::to_string(y) + ") are out of grid bounds (" +
                    std::to_string(gridPtr->numCols()) + "x" + std::to_string(gridPtr->numRows()) + ")."
                );
            }

            auto elem = gridPtr->getElement(y, x);
            elem->setVsum(elem->getSurroundingVsum() + voltage);
        }
    }
}

template <typename Element>
void Simulation2D<Element>::generateGnuplotScript(const std::string& dataFilename, const std::vector<std::string>& labels)
{
    std::string scriptFilename = dataFilename.substr(0, dataFilename.find_last_of('.')) + "_gnu.txt";
    std::ofstream gnuFile(scriptFilename);
    if (!gnuFile.is_open()) {
        std::cerr << "[ERROR] Failed to create gnuplot script: " << scriptFilename << std::endl;
        return;
    }
    // 空のラベルがあるかチェック
    for (size_t i = 0; i < labels.size(); ++i) {
        if (labels[i].empty()) {
            std::cerr << "[ERROR] Label for index " << i << " is empty. Cannot generate gnuplot script.\n";
            return;
        }
    }

    std::string plotFilename = dataFilename;
    const std::string prefix = "output/";
    if (plotFilename.rfind(prefix, 0) == 0) {
        plotFilename = plotFilename.substr(prefix.size());
    }

    gnuFile << "#unset key\n";
    gnuFile << "#set title 'no'\n";
    gnuFile << "set terminal qt font \"Arial,10\"\n";
    gnuFile << "set xl 't[ns]'\n";
    gnuFile << "set yl 'V[V]'\n";
    gnuFile << "\n";

    gnuFile << "p ";
    for (size_t i = 0; i < labels.size(); ++i) {
        gnuFile << "'" << plotFilename << "' u 1:" << (i + 2) << " title '" << labels[i] << "' w steps lw 3";
        if (i != labels.size() - 1)
            gnuFile << ",\\\n  ";
    }
    gnuFile << "\n";

    std::cout << "[INFO] Gnuplot script generated: " << scriptFilename << std::endl;
}

// 実行中の進捗状況を表示する
template <typename Element>
void Simulation2D<Element>::printProgressBar(){
    static int last_stage = -1;
    int stage = static_cast<int>((t / endtime) * 10);  // 0〜10（10%刻み）

    if (stage != last_stage) {
        last_stage = stage;
        int percent = stage * 10;

        std::string bar = "[";
        for (int i = 0; i < 10; ++i)
            bar += (i < stage) ? "=" : " ";
        bar += "]";

        std::cout << "[INFO] Simulation progress: " << bar
                  << " " << percent << "%" << std::endl;
    }
}

// tを取得する
template <typename Element>
double Simulation2D<Element>::getTime() const {
    return t;
}

// トンネル情報を追跡する素子を追加する
template <typename Element>
void Simulation2D<Element>::addTrackedElements(const std::vector<std::shared_ptr<Element>>& elems) {
    trackedElements = elems;
    tunnelTimes.assign(elems.size(), 0); // 全て t=0 で初期化
}

// トンネル状況を記録する変数を返す
template <typename Element>
const std::vector<double> &Simulation2D<Element>::getTunnelTimes() const{
    return tunnelTimes;
}
#endif // SIMULATION_2D_HPP