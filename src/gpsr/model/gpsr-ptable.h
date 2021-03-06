#ifndef GPSR_PTABLE_H
#define GPSR_PTABLE_H

#include <map>
#include <cassert>
#include <stdint.h>
#include "ns3/ipv4.h"
#include "ns3/timer.h"
#include <sys/types.h>
#include "ns3/node.h"
#include "ns3/node-list.h"
#include "ns3/mobility-model.h"
#include "ns3/vector.h"
#include "ns3/wifi-mac-header.h"
#include "ns3/random-variable-stream.h"
#include <complex>
#include "gpsr-packet.h"

namespace ns3 {
namespace gpsr {

//记录自己能连接上那个节点，以及这个节点的联通集
#define MAX_CS_COUNT 50
#define NO_CONNECT 10000

typedef struct{
  uint32_t connectID;
  double p;         //成功概率
  double lp;        //链路压力值
  double lpv;       //链路势能值
  uint32_t jump;    //跳数
  bool hasMe;       //这个周期里是否收到了我的广播消息
}T_Connect;


/*
 * \ingroup gpsr
 * \brief Position table used by GPSR
 */
class PositionTable
{
public:
  /// c-tor
  PositionTable ();
  std::vector<T_Connect *> cs;
  std::vector<uint32_t> lastRecv;

  Ptr<Node> node; //记录自己,用于后面获取电量用.

  uint32_t selfID;
  uint32_t fromID = NO_CONNECT;

  double beta = 0.90;
  double alpha = 0.9;

  double lp = 0;
  double lpv = 0;

  uint32_t jump = 0xFFFF;    //跳数路由所用的,默认最大值

  void updateOneCycle();

  /**
   * \brief Gets the last time the entry was updated
   * \param id Ipv4Address to get time of update from
   * \return Time of last update to the position
   */
  Time GetEntryUpdateTime (Ipv4Address id);

  /**
   * \brief Adds entry in position table
   */
  void AddEntry (Ipv4Address id, Vector position);

  void AddRecv(uint32_t id, double lp, double lpv, int jump, std::vector<uint32_t> recv);

  void AddIpv4IDTable(Ipv4Address ip, uint32_t id);
  uint32_t GetIpv4ID(Ipv4Address ip);

  /**
   * \brief Deletes entry in position table
   */
  void DeleteEntry (Ipv4Address id);

  /**
   * \brief Gets position from position table
   * \param id Ipv4Address to get position from
   * \return Position of that id or NULL if not known
   */
  Vector GetPosition (Ipv4Address id);

  /**
   * \brief Checks if a node is a neighbour
   * \param id Ipv4Address of the node to check
   * \return True if the node is neighbour, false otherwise
   */
  bool isNeighbour (Ipv4Address id);

  /**
   * \brief remove entries with expired lifetime
   */
  void Purge ();

  /**
   * \brief clears all entries
   */
  void Clear ();

  /**
   * \Get Callback to ProcessTxError
   */
  Callback<void, WifiMacHeader const &> GetTxErrorCallback () const
  {
    return m_txErrorCallback;
  }

  /**
   * \brief Gets next hop according to GPSR protocol
   * \param position the position of the destination node
   * \param nodePos the position of the node that has the packet
   * \return Ipv4Address of the next hop, Ipv4Address::GetZero () if no nighbour was found in greedy mode
   */
  Ipv4Address BestNeighbor (Vector position, Vector nodePos);

  Ipv4Address GetNextNode();

  bool IsInSearch (Ipv4Address id);

  bool HasPosition (Ipv4Address id);

  static Vector GetInvalidPosition ()
  {
    return Vector (-1, -1, 0);
  }


  /**
   * \brief Gets next hop according to GPSR recovery-mode protocol (right hand rule)
   * \param previousHop the position of the node that sent the packet to this node
   * \param nodePos the position of the destination node
   * \return Ipv4Address of the next hop, Ipv4Address::GetZero () if no nighbour was found in greedy mode
   */
  Ipv4Address BestAngle (Vector previousHop, Vector nodePos);

  //Gives angle between the vector CentrePos-Refpos to the vector CentrePos-node counterclockwise
  double GetAngle (Vector centrePos, Vector refPos, Vector node);

private:
  Time m_entryLifeTime;
  std::map<Ipv4Address, std::pair<Vector, Time> > m_table;
  // TX error callback
  Callback<void, WifiMacHeader const &> m_txErrorCallback;
  // Process layer 2 TX error notification
  void ProcessTxError (WifiMacHeader const&);

  std::map<Ipv4Address, uint32_t> ids;
};

}   // gpsr
} // ns3
#endif /* GPSR_PTABLE_H */
