/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/gpsr-module.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/wifi-module.h"
#include "ns3/v4ping-helper.h"
#include "ns3/udp-echo-server.h"
#include "ns3/udp-echo-client.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/udp-echo-helper.h"
#include <sys/stat.h>
#include <iostream>
#include <cmath>
#include "ns3/flow-monitor-module.h"
#include <iomanip>

#include "ns3/wave-net-device.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/wave-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/netanim-module.h"

#include "ns3/basic-energy-source.h"
#include "ns3/wifi-radio-energy-model.h"
#include "ns3/basic-energy-source-helper.h"
#include "ns3/wifi-radio-energy-model-helper.h"
#include "ns3/energy-source-container.h"
#include "ns3/device-energy-model-container.h"

#include "ns3/timevaring.h"
#include "ns3/aodv-module.h"

#include <fstream>

using namespace ns3;

uint32_t totalHello = 0;
uint32_t rx_drop = 0;
uint32_t tx_drop = 0;

static void PRxDrop (Ptr<const Packet> m_phyRxDropTrace){
  
  //std::cout<<"PhyRXDrop at == " << Simulator::Now ().GetSeconds () << "\n";
  rx_drop += 1;
}

static void PTxDrop (Ptr<const Packet> m_phyTxDropTrace){
  
  //std::cout<<"PhyTXDrop at == " << Simulator::Now ().GetSeconds () << "\n";
  tx_drop +=1;
}


void handler (int arg0, DeviceEnergyModelContainer deviceModels)
{
  std::cout << "The simulation is now at: "<<arg0 <<" seconds" << std::endl;

  DeviceEnergyModelContainer::Iterator i;
  double sumption = 0;
  for(i = deviceModels.Begin (); i != deviceModels.End (); ++i)
  {
    sumption = sumption + (*i)->GetTotalEnergyConsumption();  // some DeviceEnergyModel method
  }
  std::cout << "The sumption is : "<< sumption << std::endl;

  double limit = 0.1;
  double min = 50;
  bool hasLow = false;
  LinkPotentRecorder* recoder = LinkPotentRecorder::getIns();
  for(uint32_t i = 0; i < 100; i++){
      if(recoder->getAtt(i) < limit){ 
        hasLow = true;
         std::cout << "Too low???" << std::endl;
        break;
      }
      if(recoder->getAtt(i) < min){
        min = recoder->getAtt(i);
      }
  }

  std::cout<< "The min is " << min << std::endl;

  if(hasLow || arg0 == 50){
    std::ofstream write;
   
    write.open("energy.txt", std::ios::out);
    for(uint32_t i = 0; i < 10; i++){
      for(uint32_t j = 0; j < 10 - 1; j++){
        write << recoder->getAtt(i*10 + j)<<",";
      }
      write << recoder->getAtt(i*10+10-1)<<std::endl;
    }
    write.flush();
    write.close();
    write.open("second.txt", std::ios::app);
    write << arg0 << "\n";
    write.flush();
    write.close();

    Simulator::Stop();
  }

  //???????????????????????????
  /*if(arg0 == 20){
    std::ofstream write;  
    LinkPotentRecorder* recoder = LinkPotentRecorder::getIns();
    write.open("formID1" + std::to_string(arg0) + ".txt", std::ios::out);
    for(uint32_t i = 0; i < 20; i++){
      for(uint32_t j = 0; j < 20 - 1; j++){
        write << recoder->getAtt(i*20 + j)<<",";
      }
      write << recoder->getAtt(i*20+20-1)<<std::endl;
    }
    write.flush();
    write.close();

    write.open("formIDLP1" + std::to_string(arg0) + ".txt", std::ios::out);
    for(uint32_t i = 0; i < 20; i++){
      for(uint32_t j = 0; j < 20 - 1; j++){
        write << recoder->getLp(i*20 + j)<<",";
      }
      write << recoder->getLp(i*20+20-1)<<std::endl;
    }
    write.flush();
    write.close();
  }*/

  //???????????????????????????
  /*if(arg0 > 20){
    std::ofstream write;  
    LinkPotentRecorder* recoder = LinkPotentRecorder::getIns();
    write.open("LPEVFluction0.txt", std::ios::app);
    for(uint32_t i = 0; i < 20; i++){
      for(uint32_t j = 0; j < 20 - 1; j++){
        write << recoder->getLp(i*20 + j)<<",";
      }
      write << recoder->getLp(i*20+20-1)<<std::endl;
    }
    write.flush();
    write.close();
  }*/

  /*std::ofstream write;
  //uint32_t row = 20;
  //uint32_t col = 20;
   LinkPotentRecorder* recoder = LinkPotentRecorder::getIns();
    write.open("lpev" + std::to_string(arg0) + ".txt", std::ios::out);
    for(uint32_t i = 0; i < 20; i++){
      for(uint32_t j = 0; j < 20 - 1; j++){
        write << recoder->getLp(i*20 + j)<<",";
      }
      write << recoder->getLp(i*20+20-1)<<std::endl;
    }
    write.flush();
    write.close();
    
    //Timevaring* linkMng = Timevaring::getIns();
    if(arg0 == 20){
        LinkItem *item;
        for(uint32_t i = 0; i < row *col; i++){
          for(uint32_t j = 0; j < row *col; j++){
            if(i != j){
              item = linkMng->getLink(i,j);
              item->initt(3000, -1,1);
            }
          }
        }
    }
    if(arg0 == 70){
        LinkItem *item;
        for(uint32_t i = 0; i < row *col; i++){
          for(uint32_t j = 0; j < row *col; j++){
            if(i != j){
              item = linkMng->getLink(i,j);
              item->initt(3000, 2, 2.001);
            }
          }
        }
    }
    */
}


static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval )
{
  if (pktCount > 0)
    {
      socket->Send (Create<Packet> (pktSize));
      Simulator::Schedule (pktInterval, &GenerateTraffic,
                                      socket, pktSize,pktCount-1, pktInterval);
    }
  else
    {
      socket->Close ();
    }
}



class LegrExample
{
public:
  LegrExample ();
  /// Configure script parameters, \return true on successful configuration
  bool Configure (int argc, char **argv);
  /// Run simulation
  void Run ();
  /// Report results
  void Report (std::ostream & os);

  void writeToFile(uint32_t lostPackets, uint32_t totalTx, uint32_t totalRx, double hopCount,double count, double delay);

private:
  ///\name parameters
  //\{
  /// Number of nodes
  uint32_t size;
  /// Width of the Node Grid
  uint32_t gridWidth;
  /// Distance between nodes, meters
  double step;
  uint32_t seed;
  std::string path;
  /// Simulation time, seconds
  double totalTime;
  /// Write per-device PCAP traces if true
  bool pcap;
  bool newfile;
  uint32_t nPairs;

  ///clientPosition
  uint16_t clientID;
  //\}

  ///\name network
  //\{
  NodeContainer nodes;
  NetDeviceContainer devices;
  Ipv4InterfaceContainer interfaces;
  DeviceEnergyModelContainer deviceModels;
  //\}

  int m_callbackCount = 0;

private:
  void CreateNodes ();
  void CreateDevices ();
  void InstallInternetStack ();
  void InstallApplications ();
  void DepletionHandler();
};

int main (int argc, char **argv)
{
  LegrExample test;
  if (! test.Configure(argc, argv))
    NS_FATAL_ERROR ("Configuration failed. Aborted.");

  test.Run ();
  test.Report (std::cout);
  return 0;
}

//-----------------------------------------------------------------------------
LegrExample::LegrExample () :
  // Number of Nodes
  size (400),
  // Grid Width
  gridWidth (20),
  // Distance between nodes
  step (50), //TODO Distance changed to the limit between nodes: test to see if there are transmitions
  seed (5450),
  path("outputs/"),
  // Simulation time
  totalTime (51),
  // Generate capture files for each node
  pcap (false),
  newfile(true),
  nPairs(15),
  clientID(219)
{
}

bool
LegrExample::Configure (int argc, char **argv)
{

  SeedManager::SetSeed(seed);
  CommandLine cmd;

  //cmd.AddValue ("pcap", "Write PCAP traces.", pcap);
  cmd.AddValue ("size", "Number of nodes.", size);
  cmd.AddValue ("time", "Simulation time, s.", totalTime);
  cmd.AddValue ("step", "Grid step, m", step);
  cmd.AddValue ("path", "path of results file", path);

  cmd.Parse (argc, argv);
  return true;
}

void LegrExample::writeToFile(uint32_t lostPackets, uint32_t totalTx, uint32_t totalRx, double hopCount,double count, double delay){

  struct stat buf;
  std::string outputfile = "runResult.txt";
  //std::string outputfile = "results/pairs"+std::to_string(nPairs)+"/"+algorithm+std::to_string(size)+"_results.txt";
  //std::string outputfile = "results/teste.txt";
  int exist = stat(outputfile.c_str(), &buf);

  std::ofstream outfile;
  outfile.open(outputfile.c_str(), std::ios::app);

  if (outfile.is_open())
  {
    std::cout << "Output operation successfully performed1\n";
  }
  else
  {
    std::cout << "Error opening file";
  }

  if (newfile == true){

   std::ofstream outfile;
   outfile.open(outputfile.c_str(), std::ios::trunc);

   if (outfile.is_open())
   {
     std::cout << "Output operation successfully performed2\n";
   }
   else
   {
     std::cout << "Error opening file";
   }

    //outfile<< "Seed\t"<<"LostPackets\t"<<"totalTx\t"<<"totalRx\t"<<"PDR (%)\t"<<"HopCount\t"<<"Delay (ms)\t"<<"PhyRxDrop\t"<<"PhyTxDrop\n";
    //outfile.flush();
    exist = 1;
   }

  if (exist == -1){
        //outfile<< "Seed\t"<<"LostPackets\t"<<"totalTx\t"<<"totalRx\t"<<"PDR (%)\t"<<"HopCount\t"<<"Delay (ms)\t"<<"PhyRxDrop\t"<<"PhyTxDrop\n";
        //outfile.flush();
  }

  // write to outfile
  outfile <<seed<<"\t"; //Lost packets
  outfile.flush();
  outfile <<lostPackets<<"\t"; //Lost packets
  outfile.flush();
  outfile <<(double)totalTx<<"\t"; //Total transmited packets
  outfile.flush();
  outfile <<(double)totalRx<<"\t"; //Total received packets
  outfile.flush();
 
  if (count == 0){

    outfile <<0<<"\t"; //PDR
    outfile.flush();
    outfile <<0<<"\t"; //Mean Hop Count
    outfile.flush();
    outfile <<0<<"\t"; //Mean Delay (ms)
    outfile.flush();

  }else{

    outfile <<std::fixed<<std::setprecision(2)<< ((double)totalRx/(double)totalTx)*100.0<<"\t"; //PDR
    outfile.flush();
    outfile <<std::fixed<<std::setprecision(2)<< hopCount/count<<"\t"; //Mean Hop Count
    outfile.flush();
    outfile <<std::fixed<<std::setprecision(2)<< delay/count * 1000<<"\t"; //Mean Delay (ms)
    outfile.flush();

  }

  outfile <<rx_drop<<"\t";
  outfile.flush();
  outfile <<tx_drop<<"\n";
  outfile.flush();

  outfile.close();
}

void
LegrExample::Run ()
{

  Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable> ();

  Timevaring* linkMng = Timevaring::getIns();
  linkMng->col = gridWidth;
  linkMng->step = step;  //??????

  LinkItem *item;
  for(uint32_t i = 0; i < size; i++){
    LinkPotentRecorder::getIns()->updateatts(i, 0);
    for(uint32_t j = 0; j < size; j++){
      if(i != j){
        item = linkMng->addLink(i,j, x);
        item->init(3000, -0.001,0.001);
      }
    }
  }


  std::cout<<"?????????????????????"<<std::endl;

  CreateNodes ();
  CreateDevices ();
  InstallInternetStack ();
  InstallApplications ();

  LegrHelper legr;
  legr.Install ();

  std::cout << "Starting simulation for " << totalTime << " s ...\n";

  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();

  for (int i=1; i<=totalTime; i++){
        Simulator::Schedule(Seconds(i), &handler, i, deviceModels);
    }

  Simulator::Stop (Seconds (totalTime));
  AnimationInterface anim ("animFile.xml");
  //anim.EnablePacketMetadata(true);
  anim.SetMaxPktsPerTraceFile(500000);
  Simulator::Run ();

  //Print per flow statstics
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    std::map<FlowId,FlowMonitor::FlowStats> stats = monitor->GetFlowStats();
    uint32_t lostPackets = 0;
    uint32_t totalRx=0;
    uint32_t totalTx=0;
    double delay=0;
    double count=0;
    double hopCount=0;


    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i= stats.begin();i!=stats.end();++i)
    {
        Ipv4FlowClassifier::FiveTuple t=classifier->FindFlow(i->first);
        Ipv4Address client = Ipv4Address(0x0A000000 + clientID + 1);

        if(t.sourceAddress.IsEqual(client) && t.destinationAddress == "10.0.0.201" && i->second.rxPackets!=0){
          //???????????????????????????
          totalRx+=i->second.rxPackets;
            totalTx+=i->second.txPackets;
            hopCount+=(i->second.timesForwarded/i->second.rxPackets+1); 
            delay+=(i->second.delaySum.GetSeconds()/i->second.rxPackets);
            count++;
            lostPackets += i->second.lostPackets;
        }
    }

    writeToFile(lostPackets, totalTx, totalRx, hopCount, count, delay);


    monitor->SerializeToXmlFile("legr.flowmon",true,true);

  Simulator::Destroy ();
}

void
LegrExample::DepletionHandler()
{
  m_callbackCount++;
  std::cout << "node dead," << m_callbackCount << "," << Simulator::Now().GetMilliSeconds() << std::endl;;
}

void
LegrExample::Report (std::ostream &)
{
}

void
LegrExample::CreateNodes ()
{
  std::cout << "Creating " << (unsigned)size << " nodes " << step << " m apart.\n";
  nodes.Create (size);
  // Name nodes
  for (uint32_t i = 0; i < size; ++i)
     {
       std::ostringstream os;
       // Set the Node name to the corresponding IP host address
       os << "node-" << i+1;
       Names::Add (os.str (), nodes.Get (i));
     }


  // Create static grid
  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                "MinX", DoubleValue (0.0),
                                "MinY", DoubleValue (0.0),
                                "DeltaX", DoubleValue (step),
                                "DeltaY", DoubleValue (step),
                                "GridWidth", UintegerValue (gridWidth),
                                "LayoutType", StringValue ("RowFirst"));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodes);
}

void
LegrExample::CreateDevices ()
{
  WifiMacHelper wifiMac;
  wifiMac.SetType ("ns3::AdhocWifiMac");
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();

  /*    wifiPhy.Set ("TxPowerStart", DoubleValue(20));
      wifiPhy.Set ("TxPowerEnd", DoubleValue(20));
      wifiPhy.Set ("TxPowerLevels", UintegerValue(1));
      wifiPhy.Set ("TxGain", DoubleValue(20));
      wifiPhy.Set ("RxGain", DoubleValue(22));
      

   wifiChannel.AddPropagationLoss ("ns3::ThreeLogDistancePropagationLossModel");*/


  wifiPhy.SetChannel (wifiChannel.Create ());
  WifiHelper wifi;
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue ("OfdmRate6Mbps"), "RtsCtsThreshold", UintegerValue (1560));
  devices = wifi.Install (wifiPhy, wifiMac, nodes);


  /*
   * Create and install energy source and a single basic radio energy model on
   * the node using helpers.
   */
  // source helper
  BasicEnergySourceHelper basicSourceHelper;
  // set energy to 0 so that we deplete energy at the beginning of simulation
  basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (10));
  // set update interval
 // basicSourceHelper.Set ("PeriodicEnergyUpdateInterval",
   //                      TimeValue (Seconds (0.25)));


  // install source
  EnergySourceContainer sources = basicSourceHelper.Install (nodes);

  // device energy model helper
  WifiRadioEnergyModelHelper radioEnergyHelper;
  //set energy depletion callback
  //WifiRadioEnergyModel::WifiRadioEnergyDepletionCallback callback =
  //   MakeCallback (&LegrExample::DepletionHandler, this);
  //radioEnergyHelper.SetDepletionCallback (callback);
  
  radioEnergyHelper.Set("TxCurrentA", DoubleValue (0.229));
  radioEnergyHelper.Set("RxCurrentA", DoubleValue (0.059));
  radioEnergyHelper.Set("IdleCurrentA", DoubleValue (0.000659));
  radioEnergyHelper.Set("CcaBusyCurrentA", DoubleValue (0.059));
  radioEnergyHelper.Set("SleepCurrentA", DoubleValue (0.00012));
  // install on node
  deviceModels = radioEnergyHelper.Install (devices, sources);



  // Enable Captures, if necessary
  if (pcap)
    {
      wifiPhy.EnablePcapAll (std::string ("legr"));
    }
}

void
LegrExample::InstallInternetStack ()
{
  LegrHelper legr;

  InternetStackHelper stack;
  stack.SetRoutingHelper (legr);
  stack.Install (nodes);
  Ipv4AddressHelper address;
  address.SetBase ("10.0.0.0", "255.255.0.0");
  interfaces = address.Assign (devices);
}

void
LegrExample::InstallApplications ()
{

  uint16_t port = 9;  // well-known echo port number
  uint32_t packetSize = 80; // size of the exchanged packets
  uint32_t maxPacketCount = 5000; // number of packets to transmit
  Time interPacketInterval = Seconds (0.05); // interval between packet transmitions

  // Set-up a server Application, to be run on the bottom-right node of the grid
  UdpEchoServerHelper server (port);
  uint16_t serverPosition = 200;
  ApplicationContainer apps = server.Install (nodes.Get(serverPosition));
  apps.Start (Seconds (10)); // Server Start Time
  apps.Stop (Seconds (totalTime-0.1)); // Server Stop Time

  // Set-up a client Application, connected to 'server', to be run on the top-left node of the grid
  UdpEchoClientHelper client (interfaces.GetAddress (serverPosition), port);
  client.SetAttribute ("MaxPackets", UintegerValue (maxPacketCount));
  client.SetAttribute ("Interval", TimeValue (interPacketInterval));
  client.SetAttribute ("PacketSize", UintegerValue (packetSize));
  apps = client.Install (nodes.Get (clientID));
  apps.Start (Seconds (15)); // Client Start Time
  apps.Stop (Seconds (totalTime-0.1)); // Client Stop Time

  //

  Config::ConnectWithoutContext("/NodeList/0/DeviceList/0/$ns3::WifiNetDevice/Phy/PhyRxDrop", MakeCallback(&PRxDrop));
  Config::ConnectWithoutContext("/NodeList/0/DeviceList/0/$ns3::WifiNetDevice/Phy/PhyTxDrop", MakeCallback(&PTxDrop));

}

//exi

