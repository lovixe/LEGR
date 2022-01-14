#ifndef TIMEVARING_H
#define TIMEVARING_H

#include <map>
#include "ns3/nstime.h"
#include <unordered_map>
#include "ns3/random-variable-stream.h"
#include "ns3/simulator.h"
#include "ns3/mobility-model.h"

namespace ns3
{
    
class LinkItem
{
private:
    /* data */
    int startID;
    int endID;
    int startTime;      //使用毫秒作为基本的单位
    int endTime;        //时间段内有效，超过就需要重新计算
    double curLoss;     //当前的扰动值

    void resetLinkNoise();

public:

    int timeRange;

    double rangeL;
    double rangeH;

    Ptr<UniformRandomVariable> random;

    
    LinkItem(int start ,int end, Ptr<UniformRandomVariable> random);
    ~LinkItem(){};

    void init(int timeRange, double rangeL, double rangeH);

    double getCurLoss(bool update);       //获取链路衰减，update为是否更新     
};



//不过考虑线程安全，因为NS3是单线程的
class Timevaring
{
private:
    /* data */
    Timevaring(){};
    static Timevaring* m_ins;
    std::unordered_map<int, LinkItem *> map;
    
public:
    int col;
    int step;
    static Timevaring* getIns(){
        if(m_ins == nullptr){
            m_ins = new Timevaring();
        }
        return m_ins;
    }

    //获取对应的链路
    LinkItem *getLink(int nodeID0, int nodeID1);

    LinkItem *getLink(Ptr<MobilityModel> a, Ptr<MobilityModel> b);

    //新建一个连接并返回这个链接
    LinkItem *addLink(int nodeID0, int nodeID1, Ptr<ns3::UniformRandomVariable> random);
};

class LinkPotentRecorder
{
private:
    LinkPotentRecorder(){}
    static LinkPotentRecorder* m_ins;
    std::unordered_map<uint32_t, double> lps;
    std::unordered_map<uint32_t, double> lpvs;
    std::unordered_map<uint32_t, uint32_t> jumps;
    std::unordered_map<uint32_t, double> atts;      //记录链路衰减量的
public:
    static LinkPotentRecorder* getIns(){
        if(m_ins == nullptr){
            m_ins = new LinkPotentRecorder();
        }
        return m_ins;
    }

    void updateLp(uint32_t id, double lp, double lpv);
    void updateJump(uint32_t id, uint32_t jump);
    void updateatts(uint32_t id, double att);
    double getLp(uint32_t id);
    double getLpv(uint32_t id);
    uint32_t getJump(uint32_t id);
    double getAtt(uint32_t id);
};


}

#endif