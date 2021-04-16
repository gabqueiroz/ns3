//
// The orientation is:
//     n2  ---------> n0 <---------- n1
//  interferer      receiver       transmitter
//
// The configurable parameters are:
//   - Prss (primary rss) (-80 dBm default)
//   - Irss (interfering rss) (-95 dBm default)
//   - delta (microseconds, (t1-t0), may be negative, default 0)
//   - PpacketSize (primary packet size) (bytes, default 1000)
//   - IpacketSize (interferer packet size) (bytes, default 1000)
//
// For instance, for this configuration, the interfering frame arrives
// at -90 dBm with a time offset of 3.2 microseconds:
//
// ./waf --run "wifi-simple-interference --Irss=-90 --delta=3.2"
//

/*
## CLASSES ##
#1  - command-line: parse de argumentos de simulação via CLI
#2  - config: permite declarar as funções e classes específicas do NS3
#3  - double: possibilita declarar double
#4  - string: possibilita declarar string
#5  - log: depurar as mensagens de log
#6  - yans-wifi-helper: facilita o trabalho de objetos da camada PHY utilizados pelo modelo YANS
#7  - ssid: informações para trabalhar com SSID (Service Set Identifier)
#8  - mobility-helper: lida com a mobilidade inerente ao wifi
#9  - yans-wifi-channel: utilizada para trabalhar o canal que conecta os objetos da Yans-Wifi
#10 - mobility-model: trabalha informações de posição e velocidade de um objeto
#11 - internet-stack-helper: agrega as funcionalidades da pilha de protocolos IP/TCP/UDP

*/

#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/log.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/mobility-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/mobility-model.h"
#include "ns3/internet-stack-helper.h"

using namespace ns3;

// Definição do componente de log: WifiSimpleInterference
NS_LOG_COMPONENT_DEFINE ("WifiSimpleInterference");

// Função utilizada para retornar as informações do(s) pacote(s) recebido(s): socket e porta
static inline std::string PrintReceivedPacket (Ptr<Socket> socket)
{
  Address addr;

  std::ostringstream oss;

  while (socket->Recv ())
    {
      socket->GetSockName (addr);
      InetSocketAddress iaddr = InetSocketAddress::ConvertFrom (addr);

      oss << "Received one packet!  Socket: " << iaddr.GetIpv4 () << " port: " << iaddr.GetPort ();
    }
// Retorna as informações pertinentes 
  return oss.str ();
}

// Função para printar a mensagem de saída incondicionalmente com NS_LOG_UNCOND
static void ReceivePacket (Ptr<Socket> socket)
{
  NS_LOG_UNCOND (PrintReceivedPacket (socket));
}

/* Função para gerar o tráfego com base nas informações ligadas ao(s) pacote(s):
   # socket utilizado
   # tamanho do pacote
   # flag de ordem dos pacotes
   # instante de envio
   Caso haja pacotes para transmissão, são enviados ao socket designado, considerando
   as informações de instante de envio e tamanho.
*/
static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval )
{
  if (pktCount > 0)
    {
      socket->Send (Create<Packet> (pktSize));
      Simulator::Schedule (pktInterval, &GenerateTraffic,
                           socket, pktSize,pktCount - 1, pktInterval);
    }
  else
    {
      socket->Close ();
    }
}

int main (int argc, char *argv[])
{
  std::string phyMode ("DsssRate1Mbps");
  double Prss = -50;  // Primary RSS - referente ao transmissor [-dBm]
  double Irss = -65;  // Interfering RSS - referente à fonte de interferência [-dBm]
  double delta = 0;  // Diferença de tempo entre o envio do transmissor e fonte de interferência [ms]
  uint32_t PpacketSize = 1000; // Primary Packet Size - tamanho do pacote do tranmissor [bytes]
  uint32_t IpacketSize = 1000; // Interfering Packet Size - tamanho do pacote interferente [bytes]
  bool verbose = false; // Configurar informações detalhadas

  // these are not command line arguments for this version
  uint32_t numPackets = 1; // Número de pacotes enviados
  double interval = 1.0; // Intervalo de envio [s]
  double startTime = 10.0; // Início do tráfego/envio de pacote(s) [s]
  double distanceToRx = 300.0; // Distância para o receptor [m]

  // This is a magic number used to set the transmit power, based on other configuration
  double offset = 91;

  // Informações para parse via CLI
  CommandLine cmd;
  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode); // protocolo 802.11: a, b, g, n, ac
  cmd.AddValue ("Prss", "Intended primary received signal strength (dBm)", Prss); // Primary RSS
  cmd.AddValue ("Irss", "Intended interfering received signal strength (dBm)", Irss); // Interfering RSS
  cmd.AddValue ("delta", "time offset (microseconds) for interfering signal", delta); // t1-t0
  cmd.AddValue ("PpacketSize", "size of application packet sent", PpacketSize); // Primary Packet Size
  cmd.AddValue ("IpacketSize", "size of interfering packet sent", IpacketSize); // Interfering Packet Size
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose); // Detalhamento
  cmd.Parse (argc, argv);
  // Converte o intervalo de envio do pacote para segundos
  Time interPacketInterval = Seconds (interval);

  // WifiRemoteStationManager: configuração de estado do dispositivo
  // Fix non-unicast data rate to be the same as that of unicast
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",
                      StringValue (phyMode));

  NodeContainer c;
  c.Create (3);

  // The below set of helpers will help us to put together the wifi NICs we want
  // Configurações para controlar a interface de rede wireless
  WifiHelper wifi;
  if (verbose)
    {
      wifi.EnableLogComponents ();  // Configura os logs detalhados do Wifi
    }
  wifi.SetStandard (WIFI_PHY_STANDARD_80211b); // Padrão utilizado: 802.11 n

  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  /* Definir valores de ganho no receptor e
     CCA (Clear Channel Assessment) para determinar quando um frame da transmissão
     deve ser recebido por um dispositivo. Quando o meio estiver ocupado, haverá
     tentativa de ressincronizar AP e STA. Além disso, serve para determinar antes
     da transmissão se o meio está ocupado ou não.
     Os valores são definidos em zero para não utilizar nenhuma das funcionalidades.
  */
  wifiPhy.Set ("RxGain", DoubleValue (0) );
  wifiPhy.Set ("CcaMode1Threshold", DoubleValue (0.0) );

  // ns-3 supports RadioTap and Prism tracing extensions for 802.11b
  // DLT_IEEE802_11_RADIO inclui informações dos frames capturados: Radiotap link layer
  wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);

  // Definição da Velocidade de Propagação e Perda na Propagação
  // ConstantSpeedPropagationDelayModel: a velocidade é constante
  // LogDistancePropagationLossModel: modelo de perda de propagação baseado na distância
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::LogDistancePropagationLossModel");
  wifiPhy.SetChannel (wifiChannel.Create ());

  // Add a mac and disable rate control
  // ConstantRateWifiManager: utiliza taxa constante para transmissão de dados
  WifiMacHelper wifiMac;
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (phyMode),
                                "ControlMode",StringValue (phyMode));
  // Set it to adhoc mode
  wifiMac.SetType ("ns3::AdhocWifiMac");
  NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, c.Get (0));
  // This will disable these sending devices from detecting a signal so that they do not backoff
  wifiPhy.Set ("EnergyDetectionThreshold", DoubleValue (0.0) );
  wifiPhy.Set ("TxGain", DoubleValue (offset + Prss) );
  devices.Add (wifi.Install (wifiPhy, wifiMac, c.Get (1)));
  wifiPhy.Set ("TxGain", DoubleValue (offset + Irss) );
  devices.Add (wifi.Install (wifiPhy, wifiMac, c.Get (2)));

  // Note that with FixedRssLossModel, the positions below are not
  // used for received signal strength.
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  positionAlloc->Add (Vector (distanceToRx, 0.0, 0.0));
  positionAlloc->Add (Vector (-1 * distanceToRx, 0.0, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (c);

  InternetStackHelper internet;
  internet.Install (c);

  // Configuração de socket e porta
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> recvSink = Socket::CreateSocket (c.Get (0), tid);
  InetSocketAddress local = InetSocketAddress (Ipv4Address ("10.1.1.1"), 80);
  recvSink->Bind (local);
  recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));

  Ptr<Socket> source = Socket::CreateSocket (c.Get (1), tid);
  InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
  source->SetAllowBroadcast (true);
  source->Connect (remote);

  // Interferer will send to a different port; we will not see a "Received packet" message
  Ptr<Socket> interferer = Socket::CreateSocket (c.Get (2), tid);
  InetSocketAddress interferingAddr = InetSocketAddress (Ipv4Address ("255.255.255.255"), 49000);
  interferer->SetAllowBroadcast (true);
  interferer->Connect (interferingAddr);

  // Tracing
  wifiPhy.EnablePcap ("wifi-simple-interference", devices.Get (0));

  // Output what we are doing
  // Retorna as informações com NS_LOG_UNCOND: PRSS, IRSS e DELTA
  NS_LOG_UNCOND ("Primary packet RSS=" << Prss << " dBm and interferer RSS=" << Irss << " dBm at time offset=" << delta << " ms");

  // Schedula a simulação com os parâmetros pertinentes ao transmissor
  Simulator::ScheduleWithContext (source->GetNode ()->GetId (),
                                  Seconds (startTime), &GenerateTraffic,
                                  source, PpacketSize, numPackets, interPacketInterval);

  // Schedula a simulação com os parâmetros pertinentes à fonte de interferência
  Simulator::ScheduleWithContext (interferer->GetNode ()->GetId (),
                                  Seconds (startTime + delta / 1000000.0), &GenerateTraffic,
                                  interferer, IpacketSize, numPackets, interPacketInterval);

  Simulator::Run (); // Roda a simulação até que um comando de STOP seja invocado
  Simulator::Destroy ();

  return 0;
}
