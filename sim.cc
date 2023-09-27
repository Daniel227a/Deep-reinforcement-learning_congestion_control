/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2018 Piotr Gawlowicz
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Piotr Gawlowicz <gawlowicz.p@gmail.com>
 * Based on script: ./examples/tcp/tcp-variants-comparison.cc
 *
 * Topology:
 *
 *   Right Leafs (Clients)                      Left Leafs (Sinks)
 *           |            \                    /        |
 *           |             \    bottleneck    /         |
 *           |              R0--------------R1          |
 *           |             /                  \         |
 *           |   access   /                    \ access |
 *           N -----------                      --------N
 */

#include <iostream>
#include <fstream>
#include <string>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/point-to-point-layout-module.h"
#include "ns3/applications-module.h"
#include "ns3/error-model.h"
#include "ns3/tcp-header.h"
#include "ns3/enum.h"
#include "ns3/event-id.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/traffic-control-module.h"

#include "ns3/opengym-module.h"
#include "tcp-rl.h"
#include <cmath>
using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("TcpVariantsComparison");

static std::vector<uint32_t> rxPkts;//vetor de inteiros armazena contagem de pacotes recebidos

static void
CountRxPkts(uint32_t sinkId, Ptr<const Packet> packet, const Address & srcAddr)//contagem de pacotes recebidos
{
  rxPkts[sinkId]++;
}


// Função que modela um sinal senoide para determinar o número de pacotes a serem enviados em cada período.
double SinusoidalTraffic(double time, double period, double peakRate, double minRate) {
    double amplitude = 0.5 * (peakRate - minRate);
    double offset = minRate + amplitude;
    return amplitude * sin(2 * M_PI * time / period) + offset;
}


static void
PrintRxCount()//mostra pacotes recebidos
{
  uint32_t size = rxPkts.size();
  NS_LOG_UNCOND("RxPkts:");
  for (uint32_t i=0; i<size; i++){
    NS_LOG_UNCOND("---SinkId: "<< i << " RxPkts: " << rxPkts.at(i));
    std::cout << i << std::endl;
  }
}


int main (int argc, char *argv[])
{
  uint32_t openGymPort = 5555;//porta
  double tcpEnvTimeStep = 0.1;//delta t 0

  uint32_t nLeaf = 1;//topologia contem uma folha de cada lado ==ponto a ponto 
  std::string transport_prot = "TcpRl";//nome do protocolo 
  double error_p =0 ;//taxa de erro de pacotes 
  std::string bottleneck_bandwidth = "2Mbps";//largura de banda entre roteadores
  std::string bottleneck_delay = "0.01ms";//atraso  
  std::string access_bandwidth = "10Mbps";//largura de banda host
  std::string access_delay = "20ms";//atrasso de acesso
  std::string prefix_file_name = "TcpVariantsComparison";
  uint64_t data_mbytes = 1000;//Configura o volume de dados em megabytes que serão transmitidos na simulação. Atualmente, está definido como zero, o que significa que não há transmissão de dados específicos planejada na simulação.
  uint32_t mtu_bytes = 1500;//define o MTU o tamanho máximo dos pacotes que podem ser transmitidos na rede
  double duration = 100.0;//duração da simulação em segundos
  uint32_t run = 0;//duração da simulação em segundos
  bool flow_monitor = false;//monitoramento de fluxo
  bool sack = true;//O SACK ajuda o remetente a saber antecipadamente sobre essas lacunas no buffer do receptor
  std::string queue_disc_type = "ns3::PfifoFastQueueDisc";//algoritmo de disciplina de fila que será usado nos dispositivos de rede
  std::string recovery = "ns3::TcpClassicRecovery";//Define o tipo de algoritmo de recuperação TCP que será usado

  CommandLine cmd;//processar argumentos da linha de comando 
  // required parameters for OpenGym interface
  cmd.AddValue ("openGymPort", "Port number for OpenGym env. Default: 5555", openGymPort);//numero da porta openGym
  cmd.AddValue ("simSeed", "Seed for random generator. Default: 1", run);//semente para o gerador de números aleatórios usado na simulação. A descrição informa que o valor padrão é 1.
  cmd.AddValue ("envTimeStep", "Time step interval for time-based TCP env [s]. Default: 0.1s", tcpEnvTimeStep);//intervalo de tempo para o ambiente TCP 
  // other parameters
  cmd.AddValue ("nLeaf",     "Number of left and right side leaf nodes", nLeaf);// número de nós folha 
  cmd.AddValue ("transport_prot", "Transport protocol to use: TcpNewReno, "
                "TcpHybla, TcpHighSpeed, TcpHtcp, TcpVegas, TcpScalable, TcpVeno, "
                "TcpBic, TcpYeah, TcpIllinois, TcpWestwoodPlus, TcpLedbat, "
		            "TcpLp, TcpRl, TcpRlTimeBased", transport_prot);//protocolo de transporte a ser usado na simulação
  cmd.AddValue ("error_p", "Packet error rate", error_p);//representa a taxa de erro de pacote na simulação
  cmd.AddValue ("bottleneck_bandwidth", "Bottleneck bandwidth", bottleneck_bandwidth);//argura de banda do gargalo na simulação
  cmd.AddValue ("bottleneck_delay", "Bottleneck delay", bottleneck_delay);//representa o atraso do gargalo na simulação
  cmd.AddValue ("access_bandwidth", "Access link bandwidth", access_bandwidth);//largura de banda do link de acesso na simulação
  cmd.AddValue ("access_delay", "Access link delay", access_delay);//o atraso do link de acesso na simulação
  cmd.AddValue ("prefix_name", "Prefix of output trace file", prefix_file_name);//prefixo para o nome do arquivo de saída que será gerado pela simulação
  cmd.AddValue ("data", "Number of Megabytes of data to transmit", data_mbytes);//quantidade de dados, em megabytes, que serão transmitidos na simulação
  cmd.AddValue ("mtu", "Size of IP packets to send in bytes", mtu_bytes);// O MTU representa o tamanho máximo dos pacotes IP 
  cmd.AddValue ("duration", "Time to allow flows to run in seconds", duration);// tempo que as simulações de fluxos devem ser executadas em segundos.
  cmd.AddValue ("run", "Run index (for setting repeatable seeds)", run);//eração de números aleatórios na simulação
  cmd.AddValue ("flow_monitor", "Enable flow monitor", flow_monitor);// monitor de fluxo está ativado ou desativado
  cmd.AddValue ("queue_disc_type", "Queue disc type for gateway (e.g. ns3::CoDelQueueDisc)", queue_disc_type);// disciplina de fila a ser usada em um gateway.
  cmd.AddValue ("sack", "Enable or disable SACK option", sack); 
  cmd.AddValue ("recovery", "Recovery algorithm type to use (e.g., ns3::TcpPrrRecovery", recovery);//tipo de algoritmo de recuperação a ser usado em conexões TCP 
  cmd.Parse (argc, argv);//Esta linha analisa os argumentos de linha de comando passados para o programa.

  transport_prot = std::string ("ns3::") + transport_prot;

  SeedManager::SetSeed (1);
  SeedManager::SetRun (run);

  NS_LOG_UNCOND("Ns3Env parameters:");
  if (transport_prot.compare ("ns3::TcpRl") == 0 or transport_prot.compare ("ns3::TcpRlTimeBased") == 0)
  {
    NS_LOG_UNCOND("--openGymPort: " << openGymPort);
  } else {
    NS_LOG_UNCOND("--openGymPort: No OpenGym");
  }

  NS_LOG_UNCOND("--seed: " << run);
  NS_LOG_UNCOND("--Tcp version: " << transport_prot);


  // OpenGym Env --- has to be created before any other thing
  Ptr<OpenGymInterface> openGymInterface;
  if (transport_prot.compare ("ns3::TcpRl") == 0)
  {
    openGymInterface = OpenGymInterface::Get(openGymPort);//ponteiro para OpenGymInterface
    Config::SetDefault ("ns3::TcpRl::Reward", DoubleValue (2.0)); // Reward when increasing congestion window
    Config::SetDefault ("ns3::TcpRl::Penalty", DoubleValue (-30.0)); // Penalty when decreasing congestion window
  }

  if (transport_prot.compare ("ns3::TcpRlTimeBased") == 0)
  {
    openGymInterface = OpenGymInterface::Get(openGymPort);
    Config::SetDefault ("ns3::TcpRlTimeBased::StepTime", TimeValue (Seconds(tcpEnvTimeStep)));
    //TcpEventGymEnv::SetReward(1.0);
    //Config::SetDefault ("ns3::TcpRlTimeBased::Reward", DoubleValue (2.0)); // Reward when increasing congestion window
    //Config::SetDefault ("ns3::TcpRlTimeBased::Penalty", DoubleValue (-30.0));
  // Calculate the ADU size // Time step of TCP env //define o intervalo de tempo entre etapas (steps) do ambiente.
  }

  Header* temp_header = new Ipv4Header ();// Ipv4Header informações sobre o tamanho do cabeçalho IPv4.
  uint32_t ip_header = temp_header->GetSerializedSize ();
  NS_LOG_LOGIC ("IP Header size is: " << ip_header);
  delete temp_header;//é excluído da memória, liberando os recursos alocados dinamicamente.
  temp_header = new TcpHeader ();
  uint32_t tcp_header = temp_header->GetSerializedSize ();
  NS_LOG_LOGIC ("TCP Header size is: " << tcp_header);
  delete temp_header;
  uint32_t tcp_adu_size = mtu_bytes - 20 - (ip_header + tcp_header);
  NS_LOG_LOGIC ("TCP ADU size is: " << tcp_adu_size);

  // Set the simulation start and stop time
  double start_time = 0.1;
  double stop_time = start_time + duration;

  // 4 MB of TCP buffer
  Config::SetDefault ("ns3::TcpSocket::RcvBufSize", UintegerValue (1 << 21));//tamanho do buffer de recepção (RcvBufSize) para os sockets TCP.
  Config::SetDefault ("ns3::TcpSocket::SndBufSize", UintegerValue (1 << 21));
  Config::SetDefault ("ns3::TcpSocketBase::Sack", BooleanValue (sack));
  Config::SetDefault ("ns3::TcpSocket::DelAckCount", UintegerValue (2));


  Config::SetDefault ("ns3::TcpL4Protocol::RecoveryType",
                      TypeIdValue (TypeId::LookupByName (recovery)));//algoritmo de recuperação
  // Select TCP variant
  TypeId tcpTid;
  NS_ABORT_MSG_UNLESS (TypeId::LookupByNameFailSafe (transport_prot, &tcpTid), "TypeId " << transport_prot << " not found");
  Config::SetDefault ("ns3::TcpL4Protocol::SocketType", TypeIdValue (TypeId::LookupByName (transport_prot)));

  // Configure the error model
  // Here we use RateErrorModel with packet error rate
  Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable> ();//gerar números aleatórios uniformemente distribuídos
  uv->SetStream (50);
  RateErrorModel error_model;
  error_model.SetRandomVariable (uv);
  error_model.SetUnit (RateErrorModel::ERROR_UNIT_PACKET);//simular erros de pacote com base em uma taxa de erro especificada.
  error_model.SetRate (error_p);//isso determina a probabilidade de cada pacote ser corrompido durante a simulação

  // Create the point-to-point link helpers
  PointToPointHelper bottleNeckLink;
  bottleNeckLink.SetDeviceAttribute  ("DataRate", StringValue (bottleneck_bandwidth));
  bottleNeckLink.SetChannelAttribute ("Delay", StringValue (bottleneck_delay));
  //bottleNeckLink.SetDeviceAttribute  ("ReceiveErrorModel", PointerValue (&error_model));

  PointToPointHelper pointToPointLeaf;
  pointToPointLeaf.SetDeviceAttribute  ("DataRate", StringValue (access_bandwidth));
  pointToPointLeaf.SetChannelAttribute ("Delay", StringValue (access_delay));

  PointToPointDumbbellHelper d (nLeaf, pointToPointLeaf,
                                nLeaf, pointToPointLeaf,
                                bottleNeckLink);

  // Install IP stack
  InternetStackHelper stack;
  stack.InstallAll ();

  // Traffic Control
  TrafficControlHelper tchPfifo;
  tchPfifo.SetRootQueueDisc ("ns3::PfifoFastQueueDisc");

  TrafficControlHelper tchCoDel;
  tchCoDel.SetRootQueueDisc ("ns3::CoDelQueueDisc");

  DataRate access_b (access_bandwidth);
  DataRate bottle_b (bottleneck_bandwidth);
  Time access_d (access_delay);
  Time bottle_d (bottleneck_delay);

  uint32_t size = static_cast<uint32_t>((std::min (access_b, bottle_b).GetBitRate () / 8) *
    ((access_d + bottle_d + access_d) * 2).GetSeconds ());// o tamanho máximo da fila em bytes.

  Config::SetDefault ("ns3::PfifoFastQueueDisc::MaxSize",
                      QueueSizeValue (QueueSize (QueueSizeUnit::PACKETS, size / mtu_bytes)));//representa o tamanho máximo da unidade de transmissão (MTU) em bytes.
  Config::SetDefault ("ns3::CoDelQueueDisc::MaxSize",
                      QueueSizeValue (QueueSize (QueueSizeUnit::BYTES, size)));

  if (queue_disc_type.compare ("ns3::PfifoFastQueueDisc") == 0)//. A disciplina de fila 
  {
    tchPfifo.Install (d.GetLeft()->GetDevice(1));
    tchPfifo.Install (d.GetRight()->GetDevice(1));
  }
  else if (queue_disc_type.compare ("ns3::CoDelQueueDisc") == 0)// A disciplina de fila 
  {
    tchCoDel.Install (d.GetLeft()->GetDevice(1));
    tchCoDel.Install (d.GetRight()->GetDevice(1));
  }
  else
  {
    NS_FATAL_ERROR ("Queue not recognized. Allowed values are ns3::CoDelQueueDisc or ns3::PfifoFastQueueDisc");
  }

  // Assign IP Addresses
  d.AssignIpv4Addresses (Ipv4AddressHelper ("10.1.1.0", "255.255.255.0"),
                         Ipv4AddressHelper ("10.2.1.0", "255.255.255.0"),
                         Ipv4AddressHelper ("10.3.1.0", "255.255.255.0"));


  NS_LOG_INFO ("Initialize Global Routing.");
  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  // Install apps in left and right nodes
  uint16_t port = 50000;
  Address sinkLocalAddress (InetSocketAddress (Ipv4Address::GetAny (), port));
  PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory", sinkLocalAddress);
  ApplicationContainer sinkApps;
  for (uint32_t i = 0; i < d.RightCount (); ++i)
  {
    sinkHelper.SetAttribute ("Protocol", TypeIdValue (TcpSocketFactory::GetTypeId ()));
    sinkApps.Add (sinkHelper.Install (d.GetRight (i)));
  }
  sinkApps.Start (Seconds (0.0));
  sinkApps.Stop  (Seconds (stop_time));

  for (uint32_t i = 0; i < d.LeftCount (); ++i)
  {
      // Configure os parâmetros do sinal senoide
    double peakRate = 1000.0; // Taxa máxima de envio (pacotes por segundo)
    double minRate = 100.0;   // Taxa mínima de envio (pacotes por segundo)
    double period = 10.0;     // Período do sinal senoide (segundos)

    // Calcule o número de pacotes a serem enviados neste período de tempo
    double currentTime = start_time;

    AddressValue remoteAddress (InetSocketAddress (d.GetRightIpv4Address (i), port));
    Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (tcp_adu_size));
    BulkSendHelper ftp ("ns3::TcpSocketFactory", Address ());
    ftp.SetAttribute ("Remote", remoteAddress);
    ftp.SetAttribute ("SendSize", UintegerValue (tcp_adu_size));
    ftp.SetAttribute ("MaxBytes", UintegerValue (data_mbytes* 1000000));
    while (currentTime < stop_time)
    {
        double packetsToSend = SinusoidalTraffic(currentTime - start_time, period, peakRate, minRate);
        uint32_t packetsToSendInt = static_cast<uint32_t>(packetsToSend);
        
        NS_LOG_UNCOND("--packetsToSend: " << packetsToSend);
        // Instale o aplicativo de envio para este período de tempo
        ApplicationContainer clientApp = ftp.Install (d.GetLeft (i));
        clientApp.Start (Seconds (currentTime));
        clientApp.Stop (Seconds (currentTime + packetsToSendInt * (1.0 / peakRate)));

        // Avance para o próximo período
        currentTime += period;
    }


    // Create an on/off app sending packets to the left side

    
 /*   
    //std::cout << data_mbytes << std::endl;
    //std::cout << data_mbytes << std::endl;
    ApplicationContainer clientApp = ftp.Install (d.GetLeft (i));
    clientApp.Start (Seconds (start_time )); // Start after sink
    clientApp.Stop (Seconds (stop_time )); // Stop before the sink
  
  */
  }

  // Flow monitor
  FlowMonitorHelper flowHelper;
  if (flow_monitor)
  {
    flowHelper.InstallAll ();
  }

  // Count RX packets  
  for (uint32_t i = 0; i < d.RightCount (); ++i)
  {
    rxPkts.push_back(0);
    Ptr<PacketSink> pktSink = DynamicCast<PacketSink>(sinkApps.Get(i));// PacketSink para rastrear o recebimento
    pktSink->TraceConnectWithoutContext ("Rx", MakeBoundCallback (&CountRxPkts, i));// contar o número de pacotes recebidos p
  }
  PrintRxCount();

  Simulator::Stop (Seconds (stop_time));
  Simulator::Run ();

  if (flow_monitor)
    {
      flowHelper.SerializeToXmlFile (prefix_file_name + ".flowmonitor", true, true);
    }

  if (transport_prot.compare ("ns3::TcpRl") == 0 or transport_prot.compare ("ns3::TcpRlTimeBased") == 0)
  {
    openGymInterface->NotifySimulationEnd();
  }

  PrintRxCount();
  Simulator::Destroy ();
  return 0;
}
