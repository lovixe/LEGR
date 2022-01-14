#include "gpsr-ptable.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include <algorithm>
#include "ns3/energy-source.h"
#include "src/propagation/model/timevaring.h"
#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <iomanip>
#include <cmath>

NS_LOG_COMPONENT_DEFINE ("GpsrTable");


namespace ns3 {
namespace gpsr {

/*
  GPSR position table
*/

PositionTable::PositionTable ()
{
  m_txErrorCallback = MakeCallback (&PositionTable::ProcessTxError, this);
  m_entryLifeTime = Seconds (2); //FIXME fazer isto parametrizavel de acordo com tempo de hello
}

Time 
PositionTable::GetEntryUpdateTime (Ipv4Address id)
{
  if (id == Ipv4Address::GetZero ())
    {
      return Time (Seconds (0));
    }
  std::map<Ipv4Address, std::pair<Vector, Time> >::iterator i = m_table.find (id);
  return i->second.second;
}

/**
 * \brief Adds entry in position table
 */
void 
PositionTable::AddEntry (Ipv4Address id, Vector position)
{
  std::map<Ipv4Address, std::pair<Vector, Time> >::iterator i = m_table.find (id);
  if (i != m_table.end () || id.IsEqual (i->first))
    {
      m_table.erase (id);
      m_table.insert (std::make_pair (id, std::make_pair (position, Simulator::Now ())));
      return;
    }
  

  m_table.insert (std::make_pair (id, std::make_pair (position, Simulator::Now ())));
}

void PositionTable::updateOneCycle()
{
    //周期更新事件
    //第一步，遍历更新所有的链路质量，并更新它们的压力和势能（对于没接收到消息的更新，接受到的不更新）
    for(uint64_t i = 0; i < cs.size(); i++){
      //检查是否双向通信了
      if(!cs[i]->hasMe){
        //未完成通信, 更新一下内容
        cs[i]->lpv = cs[i]->lpv * beta;
        cs[i]->lp = cs[i]->lp * beta;  //未完成通信的,这里直接衰减.
        //更新链路质量
        cs[i]->p = alpha * cs[i]->p;
      }
      else{
        //更新链路质量,链路使能不更新,使用自己提供的,更准确
        cs[i]->p = alpha * cs[i]->p + 1 - alpha;
      }
    }

    //第二步，提取所有的链路压力，更新自己的链路压力
    //这里所有的节点链路都已经更新完毕,对于未完成通信的也直接进行了修正.下面是计算哪个链路压力更大,就按照哪个来.
    double max = 0;
    fromID = NO_CONNECT;

    for(uint64_t i = 0; i < cs.size(); i++){
      double lost = 1/cs[i]->p;
      Ptr<EnergySourceContainer> container = node->GetObject<EnergySourceContainer> ();
      double remainEnergy = 1;
      for (EnergySourceContainer::Iterator i = container->Begin (); i != container->End (); ++i)
      {
        remainEnergy = (*i)->GetRemainingEnergy()/10;  //The 10 is the total energy.
      }

      lost = lost * pow(250, (1/remainEnergy));

      double tmpLp = cs[i]->lp - lost;  //减去损耗
      if(tmpLp > max){
        max = tmpLp;
        if(max > 0){
          fromID = cs[i]->connectID;
        }
      }
      cs[i]->hasMe = false;  //顺便清除所有的接受标识.
      //记录剩余电量，需要乘以10恢复成真实数据
      LinkPotentRecorder::getIns()->updateatts(selfID, remainEnergy * 10);

    }
    //更新自己的链路势能
    lp =  max;
    lpv = (1-beta) * max + beta * lpv;

    LinkPotentRecorder::getIns()->updateLp(selfID, lp, lp);

    if(selfID == 50){
      lp = 65535;
      lpv = 65535;
      fromID = NO_CONNECT;
    }

    //删除本周期接受到的节点信息
    lastRecv.clear();
}

/**
 * \brief recv hello, update cs
 */
void PositionTable::AddRecv(uint32_t id, double lp, double lpv, int jump, std::vector<uint32_t> recv)
{
    //接受到新来的广播消息了

    //第一步,搜索cs,有则更新,没有新建
    uint64_t i = 0;
    for(; i < cs.size(); i++){
        if(cs[i]->connectID == id){
          break;
        }
    }

    if(i == cs.size()){
      //没有,那么新建一个.
      T_Connect *con = new T_Connect;
      con->connectID = id;
      con->p = 0.25;  //初始化都修改为0.25
      con->hasMe = false;    //默认是没有自己的
      con->jump = 0xFFFF;
      cs.push_back(con);
    }
    
    cs[i]->lp = lp;
    cs[i]->lpv = lpv;
    cs[i]->jump = jump; //跳数也更新

    uint16_t j = 0;
    for(; j < recv.size(); j++){
      if(recv[j] == selfID){
        break;
      }
    }
    if(j == recv.size()){
      //没有
      cs[i]->hasMe = false;
    }
    else{
      cs[i]->hasMe = true;
    }

    lastRecv.push_back(id); //记录这个周期都接受到谁了
}

/**
 * \brief Deletes entry in position table
 */
void PositionTable::DeleteEntry (Ipv4Address id)
{
  m_table.erase (id);
}

/**
 * \brief Gets position from position table
 * \param id Ipv4Address to get position from
 * \return Position of that id or NULL if not known
 */
Vector 
PositionTable::GetPosition (Ipv4Address id)
{

  NodeList::Iterator listEnd = NodeList::End ();
  for (NodeList::Iterator i = NodeList::Begin (); i != listEnd; i++)
    {
      Ptr<Node> node = *i;
      if (node->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal () == id)
        {
          return node->GetObject<MobilityModel> ()->GetPosition ();
        }
    }
  return PositionTable::GetInvalidPosition ();
}

/**
 * \brief Checks if a node is a neighbour
 * \param id Ipv4Address of the node to check
 * \return True if the node is neighbour, false otherwise
 */
bool
PositionTable::isNeighbour (Ipv4Address id)
{

 std::map<Ipv4Address, std::pair<Vector, Time> >::iterator i = m_table.find (id);
  if (i != m_table.end () || id.IsEqual (i->first))
    {
      return true;
    }

  return false;
}


/**
 * \brief remove entries with expired lifetime
 */
void 
PositionTable::Purge ()
{

  if(m_table.empty ())
    {
      return;
    }

  std::list<Ipv4Address> toErase;

  std::map<Ipv4Address, std::pair<Vector, Time> >::iterator i = m_table.begin ();
  std::map<Ipv4Address, std::pair<Vector, Time> >::iterator listEnd = m_table.end ();
  
  for (; !(i == listEnd); i++)
    {

      if (m_entryLifeTime + GetEntryUpdateTime (i->first) <= Simulator::Now ())
        {
          toErase.insert (toErase.begin (), i->first);

        }
    }
  toErase.unique ();

  std::list<Ipv4Address>::iterator end = toErase.end ();

  for (std::list<Ipv4Address>::iterator it = toErase.begin (); it != end; ++it)
    {

      m_table.erase (*it);

    }
}

/**
 * \brief clears all entries
 */
void 
PositionTable::Clear ()
{
  m_table.clear ();
}

void PositionTable::AddIpv4IDTable(Ipv4Address ip, uint32_t id)
{
  ids.insert(std::make_pair(ip, id));
}

uint32_t PositionTable::GetIpv4ID(Ipv4Address ip)
{
  std::map<Ipv4Address, uint32_t>::iterator i = ids.find(ip);
  return i->second;
}

Ipv4Address PositionTable::GetNextNode()  //这个可以直接获得，无需使用其他的信息
{
  //看看有没有fromID，有用就整一下，没有就找最高的
  if(fromID != NO_CONNECT){
    std::map<Ipv4Address, uint32_t>::iterator i;
      for(i = ids.begin(); i != ids.end(); i++){
        if(i->second == fromID){
          return i->first;
        }
      }
  }

  //遍历,查找所有比自己高的,取最高的发送过去

    double max = lpv;
    uint32_t targetID = NO_CONNECT;
    for(uint64_t i = 0; i < cs.size(); i++){
      if(cs[i]->lpv > max){
        max = cs[i]->lpv;
        targetID = cs[i]->connectID;
      }
    }
    if(max == lpv){
      //std::cout<<selfID<<" To None"<<std::endl;
      return Ipv4Address::GetZero ();
    }
    else{
      //std::cout<<selfID << " To "<<targetID<<std::endl;
      std::map<Ipv4Address, uint32_t>::iterator i;
      for(i = ids.begin(); i != ids.end(); i++){
        if(i->second == targetID){
          return i->first;
        }
      }
      return Ipv4Address::GetZero ();
    }
}


/**
 * \brief Gets next hop according to GPSR protocol
 * \param position the position of the destination node
 * \param nodePos the position of the node that  the packet
 * \return Ipv4Address of the next hop, Ipv4Address::GetZero () if no nighbour was found in greedy mode
 */
Ipv4Address 
PositionTable::BestNeighbor (Vector position, Vector nodePos)
{
  /*Purge ();

  double initialDistance = CalculateDistance (nodePos, position);

  if (m_table.empty ())
    {
      NS_LOG_DEBUG ("BestNeighbor table is empty; Position: " << position);
      return Ipv4Address::GetZero ();
    }     //if table is empty (no neighbours)

  Ipv4Address bestFoundID = m_table.begin ()->first;
  double bestFoundDistance = CalculateDistance (m_table.begin ()->second.first, position);
  std::map<Ipv4Address, std::pair<Vector, Time> >::iterator i;
  for (i = m_table.begin (); !(i == m_table.end ()); i++)
    {
      if (bestFoundDistance > CalculateDistance (i->second.first, position))
        {
          bestFoundID = i->first;
          bestFoundDistance = CalculateDistance (i->second.first, position);
        }
    }

  if(initialDistance > bestFoundDistance)
    return bestFoundID;
  else
    return Ipv4Address::GetZero (); //so it enters Recovery-mode
    */

//看看有没有fromID，有用就整一下，没有就找最高的
  if(fromID != NO_CONNECT){
    std::map<Ipv4Address, uint32_t>::iterator i;
      for(i = ids.begin(); i != ids.end(); i++){
        if(i->second == fromID){
          return i->first;
        }
      }
  }


//遍历,查找所有比自己高的,取最高的发送过去
    double max = lpv;
    uint32_t targetID = NO_CONNECT;
    for(uint64_t i = 0; i < cs.size(); i++){
      if(cs[i]->lpv > max){
        max = cs[i]->lpv;
        targetID = cs[i]->connectID;
      }
    }
    if(max == lpv){
      //std::cout<<selfID<<" To None"<<std::endl;
      return Ipv4Address::GetZero ();
    }
    else{
      std::map<Ipv4Address, uint32_t>::iterator i;
      for(i = ids.begin(); i != ids.end(); i++){
        if(i->second == targetID){
          return i->first;
        }
      }
      return Ipv4Address::GetZero ();
    }
  return Ipv4Address::GetZero ();
}


/**
 * \brief Gets next hop according to GPSR recovery-mode protocol (right hand rule)
 * \param previousHop the position of the node that sent the packet to this node
 * \param nodePos the position of the destination node
 * \return Ipv4Address of the next hop, Ipv4Address::GetZero () if no nighbour was found in greedy mode
 */
Ipv4Address
PositionTable::BestAngle (Vector previousHop, Vector nodePos)
{
  Purge ();

  if (m_table.empty ())
    {
      NS_LOG_DEBUG ("BestNeighbor table is empty; Position: " << nodePos);
      return Ipv4Address::GetZero ();
    }     //if table is empty (no neighbours)

  double tmpAngle;
  Ipv4Address bestFoundID = Ipv4Address::GetZero ();
  double bestFoundAngle = 360;
  std::map<Ipv4Address, std::pair<Vector, Time> >::iterator i;

  for (i = m_table.begin (); !(i == m_table.end ()); i++)
    {
      tmpAngle = GetAngle(nodePos, previousHop, i->second.first);
      if (bestFoundAngle > tmpAngle && tmpAngle != 0)
	{
	  bestFoundID = i->first;
	  bestFoundAngle = tmpAngle;
	}
    }
  if(bestFoundID == Ipv4Address::GetZero ()) //only if the only neighbour is who sent the packet
    {
      bestFoundID = m_table.begin ()->first;
    }
  return bestFoundID;
}


//Gives angle between the vector CentrePos-Refpos to the vector CentrePos-node counterclockwise
double 
PositionTable::GetAngle (Vector centrePos, Vector refPos, Vector node){
  double const PI = 4*atan(1);

  std::complex<double> A = std::complex<double>(centrePos.x,centrePos.y);
  std::complex<double> B = std::complex<double>(node.x,node.y);
  std::complex<double> C = std::complex<double>(refPos.x,refPos.y);   //Change B with C if you want angles clockwise

  std::complex<double> AB; //reference edge
  std::complex<double> AC;
  std::complex<double> tmp;
  std::complex<double> tmpCplx;

  std::complex<double> Angle;

  AB = B - A;
  AB = (real(AB)/norm(AB)) + (std::complex<double>(0.0,1.0)*(imag(AB)/norm(AB)));

  AC = C - A;
  AC = (real(AC)/norm(AC)) + (std::complex<double>(0.0,1.0)*(imag(AC)/norm(AC)));

  tmp = log(AC/AB);
  tmpCplx = std::complex<double>(0.0,-1.0);
  Angle = tmp*tmpCplx;
  Angle *= (180/PI);
  if (real(Angle)<0)
    Angle = 360+real(Angle);

  return real(Angle);

}





/**
 * \ProcessTxError
 */
void PositionTable::ProcessTxError (WifiMacHeader const & hdr)
{
}



//FIXME ainda preciso disto agr que o LS ja n está aqui???????

/**
 * \brief Returns true if is in search for destionation
 */
bool PositionTable::IsInSearch (Ipv4Address id)
{
  return false;
}

bool PositionTable::HasPosition (Ipv4Address id)
{
  return true;
}


}   // gpsr
} // ns3
