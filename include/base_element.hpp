#ifndef BASE_ELEMENT_HPP
#define BASE_ELEMENT_HPP

#include <string>
#include <map>
#include <memory>
#include <vector>

//---------------------------------------------------------------------------------------------------
// BaseElementではメソッドのインターフェースのみを実装しており、実際の処理内容は持たない
// 派生クラス(SEO, multiSEOなど)が必ずオーバーライドして具体的な処理を書く必要がある
// =0で定義しているのは、関数の中身を持っていないことを示し子クラスで実装する設計指示をC++に与えるため
// 本クラスで定義していないメソッドはsimulationクラスやgridクラスで使用することはできない
//---------------------------------------------------------------------------------------------------

class BaseElement {
public:
    virtual ~BaseElement() = default;

    // --- 状態取得 ---
    virtual double getVn() const = 0;
    virtual double getVd() const = 0;
    virtual double getSurroundingVsum() const = 0;
    virtual std::map<std::string, double> getWT() const = 0;
    // oneway用の中要素を取り出すメソッド（seoやmultiseoで呼び出すとエラーが発生する）
    virtual std::shared_ptr<BaseElement> getInternalElement(int index) const = 0;
    virtual std::map<std::string, double> getdE() const = 0;
    // Qの取得
    virtual double getQ() const = 0;
    // Rゲッター
    virtual double getR() const = 0;
    // Rjゲッター
    virtual double getRj() const = 0;
    // Cjゲッター
    virtual double getCj() const = 0;
    // Cゲッター
    virtual double getC() const = 0;
    // legsゲッター
    virtual int getlegs() const = 0;

    // --- 状態設定 ---
    virtual void setVias(const double Vd) = 0;
    virtual void setVsum(double V) = 0;
    virtual void setConnections(const std::vector<std::shared_ptr<BaseElement>>& conns) = 0;

    // --- 計算処理 ---
    virtual void setSurroundingVoltages() = 0;
    virtual void setPcalc() = 0;
    virtual void setdEcalc() = 0;
    virtual bool calculateTunnelWt() = 0;
    virtual void setTunnel(const std::string& direction) = 0;
    virtual void setNodeCharge(double dt) = 0;
};

#endif // BASE_ELEMENT_HPP
