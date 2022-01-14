#include "timevaring.h"

namespace ns3{

Timevaring* Timevaring::m_ins = nullptr;
LinkPotentRecorder* LinkPotentRecorder::m_ins = nullptr;


LinkItem::LinkItem(int start ,int end, Ptr<UniformRandomVariable> random)
{
    this->startID = start;
    this->endID = end;
    this->random = random;
}

void LinkItem::resetLinkNoise()
{
    //第一步，计算时间范围
    startTime = ns3::Simulator::Now().GetMilliSeconds();
    endTime = random->GetInteger(1,timeRange) + startTime;   //随机的确定一个范围，在0-range中进行随机变化

    //第二步，计算当前扰动值
    curLoss = random->GetValue(rangeL, rangeH);

    if(startID == 2 && endID == 3){
        std::cout << startTime <<" endtime : "<< endTime << std::endl;
    }
}

double LinkItem::getCurLoss(bool update)
{
    if(update == false){
        return curLoss;
    }
    else{
        if(startTime == endTime && endTime == 0){
            resetLinkNoise();
            return curLoss;
        }
        
        //检查时间是否在区间里
        int now = ns3::Simulator::Now().GetMilliSeconds();
        if(now > endTime){
            resetLinkNoise();
            return curLoss;
        }
        else{
            return curLoss;
        }
    }
}

void LinkItem::init(int timeRange,  double rangeL, double rangeH)
{
    this->timeRange = timeRange;
    this->rangeL = rangeL;
    this->rangeH = rangeH;

    curLoss = 0;
    startTime = endTime = 0;
}


LinkItem *Timevaring::getLink(int node0, int node1)
{
    std::unordered_map<int, LinkItem *>::iterator i = map.find(node0 * 10000 + node1);
    if(i == map.end()){
        return nullptr;
    }
    else{
        return i->second;
    }
}

LinkItem *Timevaring::getLink(Ptr<MobilityModel> a, Ptr<MobilityModel> b)
{
    Vector positiona = a->GetPosition();
    int node0 = (positiona.y * col +  positiona.x) / step;
    Vector positionb = b->GetPosition();
    int node1 = (positionb.y * col + positionb.x) / step;
    return this->getLink(node0, node1);
}

LinkItem *Timevaring::addLink(int node0, int node1, Ptr<ns3::UniformRandomVariable> random)
{
    std::unordered_map<int, LinkItem *>::iterator i = map.find(node0 * 10000 + node1);
    if(i == map.end()){
        //没有，那么需要新建一个
        LinkItem *tmp = new LinkItem(node0, node1, random);
        map.insert(std::pair<int, LinkItem *>(node0 * 10000 + node1, tmp));
        return tmp;
    }
    else{
        return i->second;
    }
}

void 
LinkPotentRecorder::updateLp(uint32_t id, double lp, double lpv)
{
    std::unordered_map<uint32_t, double>::iterator i = lps.find(id);
    if(i == lps.end()){
        lps.insert(std::pair<uint32_t, double>(id, lp));
    }
    else{
        i->second = lp;
    }
    std::unordered_map<uint32_t, double>::iterator iv = lpvs.find(id);
    if(iv == lpvs.end()){
        lpvs.insert(std::pair<uint32_t, double>(id, lpv));
    }
    else{
        iv->second = lp;
    }
}

void LinkPotentRecorder::updateJump(uint32_t id, uint32_t jump)
{
    std::unordered_map<uint32_t, uint32_t>::iterator i = jumps.find(id);
    if(i == jumps.end()){
        jumps.insert(std::pair<uint32_t, uint32_t>(id, jump));
    }
    else{
        i->second = jump;
    }
}

double LinkPotentRecorder::getLp(uint32_t id)
{
    std::unordered_map<uint32_t, double>::iterator i = lps.find(id);
    if(i == lps.end()){
        return 0;
    }
    else{
        return i->second;
    }
}

double LinkPotentRecorder::getLpv(uint32_t id)
{
    std::unordered_map<uint32_t, double>::iterator i = lpvs.find(id);
    if(i == lpvs.end()){
        return 0;
    }
    else{
        return i->second;
    }
}

uint32_t LinkPotentRecorder::getJump(uint32_t id)
{
    std::unordered_map<uint32_t, uint32_t>::iterator i = jumps.find(id);
    if(i == jumps.end()){
        return 0xFFFF;
    }
    else{
        return i->second;
    }
}

void LinkPotentRecorder::updateatts(uint32_t id, double att)
{
    std::unordered_map<uint32_t, double>::iterator i = atts.find(id);
    if(i == atts.end()){
        atts.insert(std::pair<uint32_t, double>(id, att));
    }
    else{
        i->second = att;
    }
}

double LinkPotentRecorder::getAtt(uint32_t id)
{
    std::unordered_map<uint32_t, double>::iterator i = atts.find(id);
    if(i == atts.end()){
        return 0;
    }
    else{
        return i->second;
    }
}

}