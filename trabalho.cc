/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 SEBASTIEN DERONNE
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
 * Author: Sebastien Deronne <sebastien.deronne@gmail.com>
 */

/*
## BIBLIOTECAS ##
#1  - command-line: parse de argumentos de simulação via CLI
#2  - config: permite declarar as funções e classes específicas do NS3
#3  - uinteger: possibilita declarar uinteger
#4  - double: possibilita declarar double
#5  - string: possibilita declarar string   
#6  - log: depurar as mensagens de log
#7  - yans-wifi-helper: facilita o trabalho de objetos da camada PHY utilizados pelo modelo YANS
#8  - ssid: informações para trabalhar com SSID (Service Set Identifier)
#9  - mobility-helper: lida com a mobilidade inerente ao wifi
#10 - internet-stack-helper: agrega as funcionalidades da pilha de protocolos IP/TCP/UDP
#11 - ipv4-address-helper: simplifica a atribuição de endereços IPv4
#12 - udp-client-server-helper: possibilita criar aplicações que enviam pacotes UDP 
#13 - packet-sink-helper: permite criar instâncias de PacketSinkApplication nos nós
#14 - on-off-helper: permite criar instâncias OnOffApplication nos nós
#15 - ipv4-global-routing-helper: classe auxiliar que adiciona roteamento IPv4 aos objetos
#16 - packet-sink: classe que permite receber tráfego gerado para determinado IP e porta
#17 - yans-wifi-channel: utilizada para trabalhar o canal que conecta os objetos da Yans-Wifi
#18 - flow-monitor: classe para monitorar e reportar fluxo de pacotes durante uma simulação
#19 - flow-monitor-helper: habilita o monitoramento de flow-monitor
*/

#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/uinteger.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/log.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/mobility-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/flow-monitor.h"
#include "ns3/flow-monitor-helper.h"

// This is a simple example in order to show how to configure an IEEE 802.11ax Wi-Fi network.
//
// It outputs the UDP or TCP goodput for every HE MCS value, which depends on the MCS value (0 to 11),
// the channel width (20, 40, 80 or 160 MHz) and the guard interval (800ns, 1600ns or 3200ns).
// The PHY bitrate is constant over all the simulation run. The user can also specify the distance between
// the access point and the station: the larger the distance the smaller the goodput.
//
// The simulation assumes a single station in an infrastructure network:
//
//  STA     AP
//    *     *
//    |     |
//   n1     n2
//
//Packets in this simulation aren't marked with a QosTag so they are considered
//belonging to BestEffort Access Class (AC_BE).

using namespace ns3;

// Definição do componente de log: he-wifi-network
NS_LOG_COMPONENT_DEFINE ("he-wifi-network");

// Função principal
int main (int argc, char *argv[])
{
  bool udp = true; // escolha do protocolo da camada de transporte: UDP/true e TCP/false
  bool useRts = false; // habilita o mecanismo de controle de colisões em "true"
  double simulationTime = 10; // tempo de simulação [segundos]
  double distance = 50.0; // distância entre os nós [metros]
  double frequency = 5.0; // frequência utilizada [GHz]
  int mcs = -1; // definição do MCS: -1 para percorrer de 0 a 11 ou 'x', onde a simulação roda para apenas o MCS 'x'
  double minExpectedThroughput = 0; // valores máximo e mínimo para vazão esperada
  double maxExpectedThroughput = 0;
  
  // Definição dos parâmetros de simulação via linha de comando
  CommandLine cmd;
  cmd.AddValue ("frequency", "Whether working in the 2.4 or 5.0 GHz band (other values gets rejected)", frequency);
  cmd.AddValue ("distance", "Distance in meters between the station and the access point", distance);
  cmd.AddValue ("simulationTime", "Simulation time in seconds", simulationTime);
  cmd.AddValue ("udp", "UDP if set to 1, TCP otherwise", udp);
  cmd.AddValue ("useRts", "Enable/disable RTS/CTS", useRts);
  cmd.AddValue ("mcs", "if set, limit testing to a specific MCS (0-7)", mcs);
  cmd.AddValue ("minExpectedThroughput", "if set, simulation fails if the lowest throughput is below this value", minExpectedThroughput);
  cmd.AddValue ("maxExpectedThroughput", "if set, simulation fails if the highest throughput is above this value", maxExpectedThroughput);
  cmd.Parse (argc,argv);

  // Configuração do mecanismo de redução de colisão: RTS
  if (useRts)
    {
      Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("0"));
    }

  // Configuração para percorrer os valores de MCS com base nos valores já calculados
  double prevThroughput [12];
  for (uint32_t l = 0; l < 12; l++)
    {
      prevThroughput[l] = 0;
    }
  std::cout << "MCS value" << "\t\t" << "Channel width" << "\t\t" << "GI" << "\t\t\t" << "Throughput" << '\n';
  int minMcs = 0;
  int maxMcs = 11;
  if (mcs >= 0 && mcs <= 11)
    {
      minMcs = mcs;
      maxMcs = mcs;
    }
  for (int mcs = minMcs; mcs <= maxMcs; mcs++) // Seleção do MCS
    {
      uint8_t index = 0;
      double previous = 0;
      uint8_t maxChannelWidth = frequency == 2.4 ? 40 : 160;
      for (int channelWidth = 20; channelWidth <= maxChannelWidth; ) // Seleção da largura de banda [MHz]
        {
          for (int gi = 3200; gi >= 800; ) // Seleção do Intervalo de Guarda [ns]
            {
              uint32_t payloadSize; // tamanho do pacote: 1500 bytes
              if (udp)
                {
                  payloadSize = 1472; // bytes
                }
              else
                {
                  payloadSize = 1448; // bytes
                  Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (payloadSize));
                }
              
              // Define os nós STA e AP
              NodeContainer wifiStaNode;
              wifiStaNode.Create (1);
              NodeContainer wifiApNode;
              wifiApNode.Create (1);

              // Criação do canal
              YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
              YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
              phy.SetChannel (channel.Create ());

              // Define o intervalo de guarda
              phy.Set ("GuardInterval", TimeValue (NanoSeconds (gi)));

              // Configuração da MAC layer
              WifiMacHelper mac;
              WifiHelper wifi;
              if (frequency == 5.0)
                {
                  wifi.SetStandard (WIFI_PHY_STANDARD_80211ax_5GHZ);
                }
              else if (frequency == 2.4)
                {
                  wifi.SetStandard (WIFI_PHY_STANDARD_80211ax_2_4GHZ);
                  Config::SetDefault ("ns3::LogDistancePropagationLossModel::ReferenceLoss", DoubleValue (40.046));
                }
              else
                {
                  std::cout << "Wrong frequency value!" << std::endl;
                  return 0;
                }

              // Configuração dos dispositivos
              std::ostringstream oss;
              oss << "HeMcs" << mcs;
              wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode", StringValue (oss.str ()),
                                            "ControlMode", StringValue (oss.str ()));

              Ssid ssid = Ssid ("ns3-80211ax");

              mac.SetType ("ns3::StaWifiMac",
                           "Ssid", SsidValue (ssid));

              NetDeviceContainer staDevice;
              staDevice = wifi.Install (phy, mac, wifiStaNode);

              mac.SetType ("ns3::ApWifiMac",
                           "EnableBeaconJitter", BooleanValue (false),
                           "Ssid", SsidValue (ssid));

              NetDeviceContainer apDevice;
              apDevice = wifi.Install (phy, mac, wifiApNode);

              // Define a largura do canal
              Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/ChannelWidth", UintegerValue (channelWidth));

              // Configuração de mobilidade dos objetos que caracterizam os dispositivos
              MobilityHelper mobility;
              Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();

              positionAlloc->Add (Vector (0.0, 0.0, 0.0));
              positionAlloc->Add (Vector (distance, 0.0, 0.0));
              mobility.SetPositionAllocator (positionAlloc);

              // Modelo em que a posição atual não é alterada quando já foi configurada a não ser que seja reconfigurada
              mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
              mobility.Install (wifiApNode);
              mobility.Install (wifiStaNode);


              // Configura pilha de protocolos IP
              // A classe InternetStackHelper agrega funcionalidades IP/TCP/UDP aos nós
              InternetStackHelper stack;
              stack.Install (wifiApNode);
              stack.Install (wifiStaNode);
              Ipv4AddressHelper address;
              address.SetBase ("192.168.1.0", "255.255.255.0");
              Ipv4InterfaceContainer staNodeInterface;
              Ipv4InterfaceContainer apNodeInterface;
              staNodeInterface = address.Assign (staDevice);
              apNodeInterface = address.Assign (apDevice);

              // Configuração da aplicação
              ApplicationContainer serverApp;
              if (udp)
                {
                  // UDP flow
                  uint16_t port = 9;
                  UdpServerHelper server (port);
                  serverApp = server.Install (wifiStaNode.Get (0));
                  serverApp.Start (Seconds (0.0));
                  serverApp.Stop (Seconds (simulationTime + 1));

                  UdpClientHelper client (staNodeInterface.GetAddress (0), port);
                  client.SetAttribute ("MaxPackets", UintegerValue (4294967295u));
                  client.SetAttribute ("Interval", TimeValue (Time ("0.00001"))); //packets/s
                  client.SetAttribute ("PacketSize", UintegerValue (payloadSize));
                  ApplicationContainer clientApp = client.Install (wifiApNode.Get (0));
                  clientApp.Start (Seconds (1.0));
                  clientApp.Stop (Seconds (simulationTime + 1));
                }
              else
                {
                  // TCP flow
                  uint16_t port = 50000;
                  Address localAddress (InetSocketAddress (Ipv4Address::GetAny (), port));
                  PacketSinkHelper packetSinkHelper ("ns3::TcpSocketFactory", localAddress);
                  serverApp = packetSinkHelper.Install (wifiStaNode.Get (0));
                  serverApp.Start (Seconds (0.0));
                  serverApp.Stop (Seconds (simulationTime + 1));

                  OnOffHelper onoff ("ns3::TcpSocketFactory", Ipv4Address::GetAny ());
                  onoff.SetAttribute ("OnTime",  StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
                  onoff.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
                  onoff.SetAttribute ("PacketSize", UintegerValue (payloadSize));
                  onoff.SetAttribute ("DataRate", DataRateValue (1000000000)); //bit/s
                  AddressValue remoteAddress (InetSocketAddress (staNodeInterface.GetAddress (0), port));
                  onoff.SetAttribute ("Remote", remoteAddress);
                  ApplicationContainer clientApp = onoff.Install (wifiApNode.Get (0));
                  clientApp.Start (Seconds (1.0));
                  clientApp.Stop (Seconds (simulationTime + 1));
                }

              Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

              // Configura a utilização do monitoramento com Flow Monitor
              Ptr<FlowMonitor> flowMonitor;
              FlowMonitorHelper flowHelper;
              flowMonitor = flowHelper.InstallAll();
              Simulator::Stop (Seconds (simulationTime + 1));
              flowMonitor->SerializeToXmlFile("he-wifi-network.xml", true, true);
              Simulator::Run ();

              uint64_t rxBytes = 0;
              if (udp)
                {
                  rxBytes = payloadSize * DynamicCast<UdpServer> (serverApp.Get (0))->GetReceived ();
                }
              else
                {
                  rxBytes = DynamicCast<PacketSink> (serverApp.Get (0))->GetTotalRx ();
                }
              double throughput = (rxBytes * 8) / (simulationTime * 1000000.0); //Mbit/s

              Simulator::Destroy ();

              std::cout << mcs << "\t\t\t" << channelWidth << " MHz\t\t\t" << gi << " ns\t\t\t" << throughput << " Mbit/s" << std::endl;

              // Confere o primeiro elemento p/ possível erro
              if (mcs == 0 && channelWidth == 20 && gi == 3200)
                {
                  if (throughput < minExpectedThroughput)
                    {
                      NS_LOG_ERROR ("Obtained throughput " << throughput << " is not expected!");
                      exit (1);
                    }
                }
              // Confere o último elemento p/ possível erro
              if (mcs == 11 && channelWidth == 160 && gi == 800)
                {
                  if (maxExpectedThroughput > 0 && throughput > maxExpectedThroughput)
                    {
                      NS_LOG_ERROR ("Obtained throughput " << throughput << " is not expected!");
                      exit (1);
                    }
                }
              // Confere se o valor anterior de vazão era menor para o mesmo MCS
              if (throughput > previous)
                {
                  previous = throughput;
                }
              else
                {
                  NS_LOG_ERROR ("Obtained throughput " << throughput << " is not expected!");
                  exit (1);
                }
              // Confere se o valor anterior de vazão era menor para mesma BW e GI
              if (throughput > prevThroughput [index])
                {
                  prevThroughput [index] = throughput;
                }
              else
                {
                  NS_LOG_ERROR ("Obtained throughput " << throughput << " is not expected!");
                  exit (1);
                }
              index++;
              gi /= 2;
            }
          channelWidth *= 2;
        }
    }
  return 0;
}
