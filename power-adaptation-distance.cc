/*

## RESUMO ##

Código adaptado de: rate-adaptation-distance.cc
		    power-adaptation-distance.cc

A Potência utilizada para o cálculo do RSSI (Received Signal Strength Indication)
neste código é a Potência Média Transmitida definida como uma média da potência
consumida por intervalo de medição, sendo dada em Watts. 

A mobilidade da STA (station) em relação ao AP (access point) é configurada da
seguinte forma:
> Define-se a posição do AP como referência fixa:
ap_x = 0.0 (m)
ap_y = 0.0 (m)
ap_z = 0.0 (m)
> Varia-se a posição de STA conforme os pontos de medição, por exemplo:
sta_x = -2.0 (m)
sta_y = +1.5 (m)
sta_z =  0.0 (m)
* Adotou-se sta_z = 0.0 em razão de todas as medições terem sido realizadas
na mesma altura do AP.
> Há duas formas de operação:
(1) Cálculo de múltiplos pontos conforme "steps"
- A cada "stepTime", a energia é dividida pelo intervalo de tempo definido
por "stepTime" e otem-se a potência em [W]. Desta forma, a quantidade de pontos
de medição será equivalente a "steps", variando em "stepSize" a cada "stepTime".
Desta forma, pode-se fazer uma varredura geral de um ambiente. Por exemplo:
sta_x = 1.0; sta_y = 2.0
steps = 4; stepSize = 0.5 (m); stepTime = 1 (s)
P1: 1.5;2.5
P2: 2.0;3.0
P3: 2.5;3.5
P4: 3.0;4.0 
(2) Cálculo ponto a ponto minimizando "steps"
- Este foi o método utilizado para calcular os valores via simulação, pois há
maior controle sobre os pontos desejados, mesmo que seja ponto a ponto. O valor
mínimo de "steps" é 1, de forma que pode-se configurar "stepsTime" para o tempo
desejado e "stepsSize" para valores menores, por exemplo, 0.1, visando calcular
o mais próximo de sta_x,sta_y. Por exemplo:
sta_x = 1.0; sta_y = 2.0
steps = 1; stepSize = 0.1 (m); stepTime = 1 (s)
P: 1.1;2.1

/*
## BIBLIOTECAS ##
#1  - gnuplot: permite utilizar os comandos de gnuplot para plotar conjunto de dados
#2  - command-line: parse de argumentos de simulação via CLI
#3  - config: permite declarar as funções e classes específicas do NS3
#4  - uinteger: possibilita declarar uinteger
#5  - double: possibilita declarar double
#6  - log: depurar as mensagens de log
#7  - yans-wifi-helper: facilita o trabalho de objetos da camada PHY utilizados pelo modelo YANS
#8  - ssid: informações para trabalhar com SSID (Service Set Identifier)
#9  - mobility-helper: lida com a mobilidade inerente ao wifi
#10 - internet-stack-helper: agrega as funcionalidades da pilha de protocolos IP/TCP/UDP
#11 - ipv4-address-helper: simplifica a atribuição de endereços IPv4
#12 - packet-sink-helper: permite criar instâncias de PacketSinkApplication nos nós
#13 - on-off-helper: permite criar instâncias OnOffApplication nos nós
#14 - yans-wifi-channel: utilizada para trabalhar o canal que conecta os objetos da Yans-Wifi
#15 - wifi-net-device: permite trabalhar com todos objetos ligados ao Wifi: ns3::Channel, ns3::WifiPhy e ns3:WifiMac
#16 - wifi-mac: trabalha os objetos relacionadas ao MAC address
#17 - wifi-mac-header: implementa o cabeçalho do MAC address
#18 - mobility-model: trabalha informações de posição e velocidade de um objeto
*/

#include "ns3/gnuplot.h"
#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/mobility-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-mac.h"
#include "ns3/wifi-mac-header.h"
#include "ns3/mobility-model.h"

using namespace ns3;
using namespace std;

// Definição do componente de log: PowerAdaptationDistance
NS_LOG_COMPONENT_DEFINE ("PowerAdaptationDistance");

// Tamanho do pacote gerado no AP (bytes)
static const uint32_t packetSize = 1420;

// Classe para definir os parâmetros referentes aos nós da rede 
class NodeStatistics
{
// Método público
// Informações referentes a vazão, potência e posição para
// registrar os dados de duas maneiras:
// 1) Vazão vs tempo conforme a mobilidade (não será utilizado)
// 2) Potência vs tempo conforme a mobilidade (será utilizado para o cálculo do RSSI)
public:
  NodeStatistics (NetDeviceContainer aps, NetDeviceContainer stas);

  void PhyCallback (std::string path, Ptr<const Packet> packet);
  void RxCallback (std::string path, Ptr<const Packet> packet, const Address &from);
  void PowerCallback (std::string path, double oldPower, double newPower, Mac48Address dest);
  void RateCallback (std::string path, DataRate oldRate, DataRate newRate, Mac48Address dest);
  void SetPosition (Ptr<Node> node, Vector position);
  void AdvancePosition (Ptr<Node> node, int stepsSize, int stepsTime);
  Vector GetPosition (Ptr<Node> node);

  Gnuplot2dDataset GetDatafile ();
  Gnuplot2dDataset GetPowerDatafile ();

// Método privado
/*
   1) Define-se um par ordenado ou conjunto de palavra-chave 
   para associar DataRate (Vazão) e TxTime (Tempo de transmissão).
   2) Mapeia os valores atuais de Potência e Vazão.
   3) Definição de: vazão total, energia e tempo totais.
*/
private:
  typedef std::vector<std::pair<Time, DataRate> > TxTime;
  void SetupPhy (Ptr<WifiPhy> phy);
  Time GetCalcTxTime (DataRate rate);

  std::map<Mac48Address, double> currentPower;
  std::map<Mac48Address, DataRate> currentRate;
  uint32_t m_bytesTotal;
  double totalEnergy;
  double totalTime;
  Ptr<WifiPhy> myPhy;
  TxTime timeTable;
  Gnuplot2dDataset m_output;
  Gnuplot2dDataset m_output_power;
};

/*
  1) Utilizando a classe "NodeStatistics" os parâmetros são os NetDeviceContainers 
     tanto para o AP quanto para a STA (dispositivo utilizando o WiFi).
  2) Os NetDeviceContainers inicializam os dispositivos, adicionando o endereço MAC
     e realizando a instalação do nó da rede. 
*/
NodeStatistics::NodeStatistics (NetDeviceContainer aps, NetDeviceContainer stas)
{
// NetDevice e WifiNetDevice resguardam todos os objetos relacionados ao WiFi,
// ou seja, atributos como: canal, configuração das camadas PHY e MAC atribuídos
// ao NetDevice, além de funções de controle remoto (RemoteStationManager). 
  Ptr<NetDevice> device = aps.Get (0); // Configurando AP
  Ptr<WifiNetDevice> wifiDevice = DynamicCast<WifiNetDevice> (device);
  Ptr<WifiPhy> phy = wifiDevice->GetPhy ();
  myPhy = phy;
  SetupPhy (phy);
// Por exemplo, com base na configuração do NetDevice, a Vazão tem por base os 
// parâmetros da camada PHY e largura do canal. 
  DataRate dataRate = DataRate (phy->GetMode (0).GetDataRate (phy->GetChannelWidth ()));
  double power = phy->GetTxPowerEnd ();
  for (uint32_t j = 0; j < stas.GetN (); j++) // Configurando STA
    {
      Ptr<NetDevice> staDevice = stas.Get (j);
      Ptr<WifiNetDevice> wifiStaDevice = DynamicCast<WifiNetDevice> (staDevice);
      Mac48Address addr = wifiStaDevice->GetMac ()->GetAddress ();
// Dados atuais de potência e vazão para STA
      currentPower[addr] = power;
      currentRate[addr] = dataRate;
    }
  currentRate[Mac48Address ("ff:ff:ff:ff:ff:ff")] = dataRate;
  totalEnergy = 0;
  totalTime = 0;
  m_bytesTotal = 0;
// Define a saída no arquivo de dados para o gnuplot: 
// Vazão (Mbps) e Potência Média (W)
  m_output.SetTitle ("Throughput [Mbits/s]");
  m_output_power.SetTitle ("Potência Transmitida [W]");
}

// Função para configuração da camada PHY
void
NodeStatistics::SetupPhy (Ptr<WifiPhy> phy)
{
  uint32_t nModes = phy->GetNModes ();
  for (uint32_t i = 0; i < nModes; i++)
    {
      WifiMode mode = phy->GetMode (i);
      WifiTxVector txVector;
      txVector.SetMode (mode);
/* O PREAMBLE_TYPE é o tempo de espera e sincronização
   antes da transmissão de um quadro. Sendo assim, é
   utilizado como sincronismo que configura confiabilidade
   na transmissão, ou seja, não há transmissão de dados.
*/
      txVector.SetPreambleType (WIFI_PREAMBLE_LONG);
// Configuração da largura do canal
      txVector.SetChannelWidth (phy->GetChannelWidth ());
// Vazão
      DataRate dataRate = DataRate (mode.GetDataRate (phy->GetChannelWidth ()));
// Duração da transmissão
      Time time = phy->CalculateTxDuration (packetSize, txVector, phy->GetFrequency ());
      NS_LOG_DEBUG (i << " " << time.GetSeconds () << " " << dataRate);
      timeTable.push_back (std::make_pair (time, dataRate));
    }
}

// Verifica o intervalo de transmissão
Time
NodeStatistics::GetCalcTxTime (DataRate rate)
{
  for (TxTime::const_iterator i = timeTable.begin (); i != timeTable.end (); i++)
    {
      if (rate == i->second)
        {
          return i->first;
        }
    }
  NS_ASSERT (false);
  return Seconds (0);
}

// Cálculo da energia e do tempo total de transmissão para cálculo da potência média.

void
NodeStatistics::PhyCallback (std::string path, Ptr<const Packet> packet)
{
  WifiMacHeader head;
  packet->PeekHeader (head);
  Mac48Address dest = head.GetAddr1 ();

  if (head.GetType () == WIFI_MAC_DATA)
    {
      totalEnergy += pow (10.0, currentPower[dest] / 10.0) * GetCalcTxTime (currentRate[dest]).GetSeconds ();
      totalTime += GetCalcTxTime (currentRate[dest]).GetSeconds ();
    }
}

// Atribuição de valores para Potência
void
NodeStatistics::PowerCallback (std::string path, double oldPower, double newPower, Mac48Address dest)
{
  currentPower[dest] = newPower;
}

// Atribuição de valores para Vazão
void
NodeStatistics::RateCallback (std::string path, DataRate oldRate, DataRate newRate, Mac48Address dest)
{
  currentRate[dest] = newRate;
}

// Atribuição de valor a M_bytes com base no tamanho de pacotes transmitidos
void
NodeStatistics::RxCallback (std::string path, Ptr<const Packet> packet, const Address &from)
{
  m_bytesTotal += packet->GetSize ();
}

// Configuração da mobilidade do nó STA
void
NodeStatistics::SetPosition (Ptr<Node> node, Vector position)
{
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
  mobility->SetPosition (position);
}
Vector
NodeStatistics::GetPosition (Ptr<Node> node)
{
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
  return mobility->GetPosition ();
}

void
NodeStatistics::AdvancePosition (Ptr<Node> node, int stepsSize, int stepsTime)
{
  Vector pos = GetPosition (node);
  double mbs = ((m_bytesTotal * 8.0) / (1000000 * stepsTime)); // cálculo de mbs
  m_bytesTotal = 0; // inicialização de m_bytesTotal
  double atp = totalEnergy / stepsTime; // average transmission power (atp)
  totalEnergy = 0; // inicialização de totalEnergy
  totalTime = 0; // inicialização de totalTime
  m_output_power.Add (pos.x, atp);
  m_output.Add (pos.x, mbs);
// A posição do nó é incrementada com base no tamanho do passo (stepsSize)
// Para realizar medições ponto a ponto, um de cada vez, será utilizado 1 passo apenas.
  pos.x += stepsSize;
  SetPosition (node, pos);
  NS_LOG_INFO ("No intervalo de " << Simulator::Now ().GetSeconds () << " segundos; configurando nova posição para " << pos);
  Simulator::Schedule (Seconds (stepsTime), &NodeStatistics::AdvancePosition, this, node, stepsSize, stepsTime);
}

// Chamada de Gnuplot para o conjunto de dados quando utilizado.
Gnuplot2dDataset
NodeStatistics::GetDatafile ()
{
  return m_output;
}

Gnuplot2dDataset
NodeStatistics::GetPowerDatafile ()
{
  return m_output_power;
}

// Informações de Log para Potência e Vazão durante a simulação.
void PowerCallback (std::string path, double oldPower, double newPower, Mac48Address dest)
{
  NS_LOG_INFO ((Simulator::Now ()).GetSeconds () << " " << dest << " Potência anterior=" << oldPower << " Nova potência=" << newPower);
}

void RateCallback (std::string path, DataRate oldRate, DataRate newRate, Mac48Address dest)
{
  NS_LOG_INFO ((Simulator::Now ()).GetSeconds () << " " << dest << " Throughput anterior=" << oldRate << " Nova throughput=" <<  newRate);
}

// Função principal
int main (int argc, char *argv[])
{
  double maxPower = -40; // valor máximo de potência
  double minPower = -70; // valor mínimo de potência
  uint32_t powerLevels = 30; // níveis de potência
  uint32_t rtsThreshold = 2346;
  std::string manager = "ns3::ParfWifiManager"; // PARF Rate control algorithm
  std::string outputFileName = "COMODO01_POSICAO01"; // nome do arquivo salvo
  int ap1_x = 0; // posição 'x' do AP
  int ap1_y = 0; // posição 'y' do AP
  int sta1_x = -1.4; // posição 'x' para STA
  int sta1_y = 3.0; // posição 'y' para STA
  uint32_t steps = 1; // quantidade de passos
  uint32_t stepsSize = 0.1; // tamanho do passo (mínimo para não interferir na posição atual)
  uint32_t stepsTime = 1; // tempo para cada passo

// Caso não haja uma quantidade de passos definidas, a simulação é interrompida.
  if (steps == 0)
    {
      std::cout << "Finalizando sem executar a simulação; steps = 0" << std::endl;
    }

// Definição do tempo de simulação a partir da quantidade de passos e sua duração.
  uint32_t simuTime = (steps + 1) * stepsTime;

  // Define o AP utilizando a classe NodeContainer, que contém todas as propriedades pertinentes
  NodeContainer wifiApNodes;
  wifiApNodes.Create (1);

  // Define o STA da mesma forma que o AP
  NodeContainer wifiStaNodes;
  wifiStaNodes.Create (1);

  // Configuração do WiFi
  WifiHelper wifi; // classe para criar e configurar objetivos WiFi necessários aos WifiNetDevices
  wifi.SetStandard (WIFI_PHY_STANDARD_80211a); // configura o padrão utilizado: 802.11n
  WifiMacHelper wifiMac; // classe para configurar e instalar os objetos de WifiMac nos nós da rede
// Configurações da camada PHY e do canal utilizado: classes WifiPhy e WifiChannel
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
// Criação do canal
  wifiPhy.SetChannel (wifiChannel.Create ());

// Instancia os dispositivos com suas propriedades: AP e STA
  NetDeviceContainer wifiApDevices;
  NetDeviceContainer wifiStaDevices;
  NetDeviceContainer wifiDevices;

  // Configura o nó STA
/* Configuração do RemoteStationManager para o RTS/CTS Threshold.
   Os roteadores e APs apresentam o gerenciamento ou manipulação dos pacotes na rede.
   Desta forma, as funções RTS (Request To Send) e CTS (Clear To Send) controlam
   o acesso das estações ao meio de transmissão. De maneira simplificada, o mecanismo
   atua como um anúncio ao AP que há desejo de transmitir os dados.
   Utilizar o modo Threshold permite administrar quais pacotes acima do tamanho limite (threshold)
   são anunciados.
*/
  wifi.SetRemoteStationManager ("ns3::MinstrelWifiManager", "RtsCtsThreshold", UintegerValue (rtsThreshold));
  wifiPhy.Set ("TxPowerStart", DoubleValue (maxPower)); // potência de transmissão
  wifiPhy.Set ("TxPowerEnd", DoubleValue (maxPower));

  Ssid ssid = Ssid ("AP"); // SSID para o AP
  wifiMac.SetType ("ns3::StaWifiMac",
                   "Ssid", SsidValue (ssid));
  wifiStaDevices.Add (wifi.Install (wifiPhy, wifiMac, wifiStaNodes.Get (0)));

  // Configura o nó AP
  // Configura o Threshold e os níveis de potência de maneira semelhante ao STA
  wifi.SetRemoteStationManager (manager, "DefaultTxPowerLevel", UintegerValue (powerLevels - 1), "RtsCtsThreshold", UintegerValue (rtsThreshold));
  wifiPhy.Set ("TxPowerStart", DoubleValue (minPower));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (maxPower));
  wifiPhy.Set ("TxPowerLevels", UintegerValue (powerLevels));

  ssid = Ssid ("AP"); // SSID para o AP
  wifiMac.SetType ("ns3::ApWifiMac",
                   "Ssid", SsidValue (ssid));
  wifiApDevices.Add (wifi.Install (wifiPhy, wifiMac, wifiApNodes.Get (0)));

  wifiDevices.Add (wifiStaDevices); // adiciona o nó STA
  wifiDevices.Add (wifiApDevices); // adiciona o nó AP

  // Configuração do esquema de mobilidade
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> (); // alocação de posição
  positionAlloc->Add (Vector (ap1_x, ap1_y, 0.0)); // Posições iniciais AP
  NS_LOG_INFO ("Setting initial AP position to " << Vector (ap1_x, ap1_y, 0.0));
  positionAlloc->Add (Vector (sta1_x, sta1_y, 0.0)); // Posições iniciais STA
  NS_LOG_INFO ("Setting initial STA position to " << Vector (sta1_x, sta1_y, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  // Modelo em que a posição atual não é alterada quando já foi configurada a não ser que seja reconfigurada
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (wifiApNodes.Get (0));
  mobility.Install (wifiStaNodes.Get (0));

  // Statistics counter
  NodeStatistics statistics = NodeStatistics (wifiApDevices, wifiStaDevices);

  // Configura a posição de STA de acordo com 'stepSize' (metros) a cada 'stepsTime' (segundos)
  Simulator::Schedule (Seconds (0.5 + stepsTime), &NodeStatistics::AdvancePosition, &statistics, wifiStaNodes.Get (0), stepsSize, stepsTime);

  // Configura pilha de protocolos IP
  // A classe InternetStackHelper agrega funcionalidades IP/TCP/UDP aos nós 
  InternetStackHelper stack;
  stack.Install (wifiApNodes);
  stack.Install (wifiStaNodes);
  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = address.Assign (wifiDevices);
  Ipv4Address sinkAddress = i.GetAddress (0);
  uint16_t port = 9;

  // Configure the CBR generator
  PacketSinkHelper sink ("ns3::UdpSocketFactory", InetSocketAddress (sinkAddress, port));
  ApplicationContainer apps_sink = sink.Install (wifiStaNodes.Get (0));

  OnOffHelper onoff ("ns3::UdpSocketFactory", InetSocketAddress (sinkAddress, port));
  onoff.SetConstantRate (DataRate ("54Mb/s"), packetSize);
  onoff.SetAttribute ("StartTime", TimeValue (Seconds (0.5)));
  onoff.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  ApplicationContainer apps_source = onoff.Install (wifiApNodes.Get (0));

  apps_sink.Start (Seconds (0.5));
  apps_sink.Stop (Seconds (simuTime));

  // Registros de dados

  // Registro de pacotes recebidos para calcular a Vazão/Throughput
  Config::Connect ("/NodeList/1/ApplicationList/*/$ns3::PacketSink/Rx",
                   MakeCallback (&NodeStatistics::RxCallback, &statistics));

  // Registro de Potência e Intervalo de tempo para calcular a Potência Média Transmitida
  Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + manager + "/PowerChange",
                   MakeCallback (&NodeStatistics::PowerCallback, &statistics));
  Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + manager + "/RateChange",
                   MakeCallback (&NodeStatistics::RateCallback, &statistics));

  Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxBegin",
                   MakeCallback (&NodeStatistics::PhyCallback, &statistics));

  // Chamado para registrar cada mudança de Potência e Tempo
  Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + manager + "/PowerChange",
                   MakeCallback (PowerCallback));
  Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + manager + "/RateChange",
                   MakeCallback (RateCallback));

  Simulator::Stop (Seconds (simuTime));
  Simulator::Run ();

  // Gera os arquivos com os dados para utilizar o gnuplot se desejado
  std::ofstream outfile (("throughput-" + outputFileName + ".plt").c_str ());
  Gnuplot gnuplot = Gnuplot (("throughput-" + outputFileName + ".eps").c_str (), "Throughput");
  gnuplot.SetTerminal ("post eps color enhanced");
  gnuplot.SetLegend ("Tempo (segundos)", "Throughput (Mb/s)");
  gnuplot.SetTitle ("Throughput (AP -> STA) em função do tempo");
  gnuplot.AddDataset (statistics.GetDatafile ());
  gnuplot.GenerateOutput (outfile);

  if (manager.compare ("ns3::ParfWifiManager") == 0
      || manager.compare ("ns3::AparfWifiManager") == 0
      || manager.compare ("ns3::RrpaaWifiManager") == 0)
    {
      std::ofstream outfile2 (("power-" + outputFileName + ".plt").c_str ());
      gnuplot = Gnuplot (("power-" + outputFileName + ".eps").c_str (), "Potência transmitida");
      gnuplot.SetTerminal ("post eps color enhanced");
      gnuplot.SetLegend ("Tempo (segundos)", "Potência (W)");
      gnuplot.SetTitle ("Potência Média de Transmissão (AP -> STA) em função do tempo");
      gnuplot.AddDataset (statistics.GetPowerDatafile ());
      gnuplot.GenerateOutput (outfile2);
    }

  Simulator::Destroy ();

  return 0;
}
