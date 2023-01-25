/* -- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -- */
/*
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
 */

// Author: Nuno F. Moreira @INESCTEC, based on udmsim.cc submited to WiMob 2020.
// 20/06/2020
// Version 2 -> works on multiple data mules (more than 2, 3 at least) 
// ChangeLog: BugFix. In new_state_machine() function the time is correctly adjusted to deal with receiving data earlier! 
// 	          Prints state machine status.
//            Returns after app timeout.
// Network topology
// Network topology
//	data:
//       Central-----------------mule 1-----------------AUV
//		 10.0.0.1				 mule 2                 10.0.0.(nmules+1)
//							 	 mule n
//  control:
//       Central-----------------mule 1-----------------AUV
//		 192.168.1.1		     mule 2                 192.168.1.(nmules+1)
//							 	 mule n
#include <iostream>
#include <fstream>
#include <vector>
#include <iterator>
#include <iomanip>
#include <ctime>
#include <cmath>
#include <sstream>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include "boost/lexical_cast.hpp"
#include "ns3/core-module.h"
#include "ns3/wifi-module.h"
#include "ns3/applications-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store.h"
#include <iomanip>

NS_LOG_COMPONENT_DEFINE("UDMSim");

#define AUV "01"
#define CENTRAL "00"
using namespace ns3;
uint32_t totalTxBytes = 500000000; // failsafe for no argument launch
std::vector<std::vector<std::string> > posicoes_ida, posicoes_volta, tempos_mov;
class CSVReader {
	std::string fileName;
	std::string delimeter;
public:
	CSVReader(std::string filename, std::string delm = ",") :
			fileName(filename), delimeter(delm) {
	}
// Function to fetch data from a CSV File
	std::vector<std::vector<std::string> > getData();
};
/*
 * Parses through csv file line by line and returns the data
 * in vector of vector of strings.
 */
std::vector<std::vector<std::string> > CSVReader::getData() {
	std::ifstream file(fileName);
	std::vector<std::vector<std::string> > dataList;
	std::string line = "";
// Iterate through each line and split the content using delimeter
	while (getline(file, line)) {
		std::vector<std::string> vec;
		boost::algorithm::split(vec, line, boost::is_any_of(delimeter));
		dataList.push_back(vec);
	}
// Close the File
	file.close();
	return dataList;
}

Ptr<Socket> sources[500];
uint32_t nmules = 1;
uint32_t nact = 0;
uint32_t h = 0;
double DockSpeed = 0, UndockSpeed = 0, TravelSpeed = 0; // speeds used for D=V*t aproximation
// used to switch states on the state machine
bool init = true, back = false, finished = false, last = false;
bool tracing = false, failed = false;
Ipv4InterfaceContainer Interfaces, Interfaces_control;
NodeContainer mules;
double docked_distance = 0.15;
double end_time;
double distance, seed;
uint16_t channel_width = 5;
uint16_t phy_type = 0;
uint32_t data_size, total_data=0, state_k = 1; // used to select the correct data mule on the state machine
ApplicationContainer APPS_auv, Sinks, Srcs;

static void updateRssOfNodeXFromNodeY(double rss, uint32_t receiverNodeX,
		uint32_t senderNodeY); // From Helder Fontes TraceBasedPropagationLossModel
void arranca(uint32_t n);
void new_state_machine(uint32_t maxBytes);
static void GenerateTraffic(Ptr<Socket> socket, std::string datas);
int dispatched = 0, id = 0; // used to correctly identify the control socket to use
void ReceivePacket(Ptr<Socket> socket);
double dock_time; // UDMP message time
double data_time;
std::string csv_cras_ida, csv_cras_volta, tempos, tipo;

int main(int argc, char *argv[]) {
	seed = 1;
	distance = 100.0; // failsafe
	double txpower = 0; // in order to use Helder Fontes TraceBasedPropagationLossModel
	double antenna_gain = 0; // in order to use Helder Fontes TraceBasedPropagationLossModel

	data_size = 100;
	CommandLine cmd;
	cmd.AddValue("seed", "Set seed", seed);
	cmd.AddValue("undock", "Set undock speed", UndockSpeed);
	cmd.AddValue("dock", "Set dock speed", DockSpeed);
	cmd.AddValue("travel", "Set travel speed", TravelSpeed);
	cmd.AddValue("txpower", "Tx power in dB", txpower);
	cmd.AddValue("antenna_gain", "RF gain of the antenna in dBi", antenna_gain);
	cmd.AddValue("data", "Set amount of Mbytes to send", data_size);
	cmd.AddValue("mules", "Set number of Data Mules", nmules);
	cmd.AddValue("distance",
			"Set Distance between Central Station and Survey Unit", distance);
	cmd.AddValue("Dock distance",
			"Set Distance between Central Station and Survey Unit",
			docked_distance);
	cmd.AddValue("tracing", "Enables PCAP tracing", tracing);
	cmd.AddValue("go_to_auv", "The csv file from cras", csv_cras_ida);
	cmd.AddValue("return_from_auv", "The csv file from cras", csv_cras_volta);
	cmd.AddValue("travel_times", "The csv file with times from cras", tempos);
	cmd.AddValue("type", "Perfect Nav (PFT) or Imperfect Nav (IPFT)", tipo);
	cmd.AddValue("freq", "Channel BandWidth", channel_width);
	cmd.AddValue("phy",
			"Wifi Phy type -WIFI_PHY_STANDARD_80211n_2_4GHZ -> 0 or -WIFI_PHY_STANDARD_80211_5MHZ -> 1",
			phy_type);
	cmd.Parse(argc, argv);
	RngSeedManager::SetSeed(seed);
	RngSeedManager::SetRun(seed);
	NodeContainer auv;
	NodeContainer central;
	NodeContainer controler;
	if (data_size != 0)
		totalTxBytes = data_size * 1e6;
	central.Create(1);
	mules.Create(nmules);
	auv.Create(1);
	controler.Create(1); // used to receive the control UDMP messages

	// data
	WifiMacHelper wifiMac;
	WifiHelper wifiHelper;
	if (phy_type == 0) {
		wifiHelper.SetStandard(ns3::WIFI_PHY_STANDARD_80211n_2_4GHZ); // n_2_4GHZ
	} else if (phy_type == 1) {
		wifiHelper.SetStandard(WIFI_PHY_STANDARD_80211_5MHZ); // 5MHZ
	}

	wifiHelper.SetRemoteStationManager("ns3::MinstrelHtWifiManager"); //auto-rate ns3::MinstrelHtWifiManager AarfWifiManager
	/* Set up Legacy Channel */
	YansWifiChannelHelper wifiChannel;
	wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
	wifiChannel.AddPropagationLoss("ns3::TraceBasedPropagationLossModel");
	/* Setup Physical Layer */
	uint16_t freq_MHz = 2462;

	YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
	wifiPhy.SetPcapDataLinkType(YansWifiPhyHelper::DLT_IEEE802_11_RADIO);
	wifiPhy.SetChannel(wifiChannel.Create());
	wifiPhy.SetErrorRateModel("ns3::YansErrorRateModel");
	wifiPhy.Set("ChannelWidth", UintegerValue(channel_width));
	wifiPhy.Set("RxGain", DoubleValue(antenna_gain));
	wifiPhy.Set("TxGain", DoubleValue(antenna_gain));
	wifiPhy.Set("TxPowerStart", DoubleValue(txpower));
	wifiPhy.Set("TxPowerEnd", DoubleValue(txpower));
	wifiPhy.Set("Frequency", UintegerValue(freq_MHz));
	wifiPhy.Set("Antennas", UintegerValue(1));
	wifiPhy.Set("RxSensitivity", DoubleValue(-200.0));
	wifiPhy.Set("MaxSupportedTxSpatialStreams", UintegerValue(1));
	wifiPhy.Set("MaxSupportedRxSpatialStreams", UintegerValue(1));
	wifiMac.SetType("ns3::AdhocWifiMac");



	// control UDMP
	std::string phyMode("DsssRate1Mbps");
	WifiHelper wifi_control;
	wifi_control.SetStandard(WIFI_PHY_STANDARD_80211b);
	YansWifiPhyHelper wifiPhy_control = YansWifiPhyHelper::Default();
	// This is one parameter that matters when using FixedRssLossModel
	// set it to zero; otherwise, gain will be added
	wifiPhy_control.Set("RxGain", DoubleValue(2000));
	// ns-3 supports RadioTap and Prism tracing extensions for 802.11b
	wifiPhy_control.SetPcapDataLinkType(
			YansWifiPhyHelper::DLT_IEEE802_11_RADIO);

	YansWifiChannelHelper wifiChannel_control;
	wifiChannel_control.SetPropagationDelay(
			"ns3::ConstantSpeedPropagationDelayModel");
	// The below FixedRssLossModel will cause the rss to be fixed regardless
	// of the distance between the two stations, and the transmit power
	wifiChannel_control.AddPropagationLoss("ns3::FixedRssLossModel", "Rss",
			DoubleValue(-80));
	wifiPhy_control.SetChannel(wifiChannel_control.Create());

	// Add a mac and disable rate control
	WifiMacHelper wifiMac_control;
	wifi_control.SetRemoteStationManager("ns3::ConstantRateWifiManager",
			"DataMode", StringValue(phyMode), "ControlMode",
			StringValue(phyMode));


	NetDeviceContainer devices;
	devices = wifiHelper.Install(wifiPhy, wifiMac, central);
	devices.Add(wifiHelper.Install(wifiPhy, wifiMac, mules));
	devices.Add(wifiHelper.Install(wifiPhy, wifiMac, auv));


	// Set it to adhoc mode
	wifiMac_control.SetType("ns3::AdhocWifiMac");
	NetDeviceContainer devices_control = wifi_control.Install(wifiPhy_control,
			wifiMac_control, central);
	devices_control.Add(
			wifi_control.Install(wifiPhy_control, wifiMac_control, mules));
	devices_control.Add(
			wifi_control.Install(wifiPhy_control, wifiMac_control, auv));
	devices_control.Add(
			wifi_control.Install(wifiPhy_control, wifiMac_control, controler));


	/*
	 * devices: [0] central, [1] mula 0 [2] mula 1 [nmules] auv
	 */

	InternetStackHelper internet;
	internet.InstallAll();

	// Later, we add IP addresses.
	Ipv4AddressHelper ipv4, ipv4_control;
	ipv4.SetBase("10.0.0.0", "255.255.255.0");
	Interfaces = ipv4.Assign(devices);

	// control
	ipv4_control.SetBase("192.168.1.0", "255.255.255.0");
	Interfaces_control = ipv4_control.Assign(devices_control);
	/*
	 * Interfaces: [0] central, [1] mula 0 [2] mula 1 [nmules] auv
	 */

	Ipv4GlobalRoutingHelper::PopulateRoutingTables();

	/* Mobility model */
	CSVReader reader_ida(csv_cras_ida);
	// Get the data from CSV File
	posicoes_ida = reader_ida.getData();
	CSVReader reader_volta(csv_cras_volta);
	// Get the data from CSV File
	posicoes_volta = reader_volta.getData();
	CSVReader reader_tempos(tempos);
	// Get the data from CSV File
	tempos_mov = reader_tempos.getData();
	dock_time = boost::lexical_cast<double>(tempos_mov[0][0]);
	data_time = boost::lexical_cast<double>(tempos_mov[0][1]);
	MobilityHelper mobility;
	Ptr<ListPositionAllocator> positionAlloc = CreateObject<
			ListPositionAllocator>();
	positionAlloc->Add(
			Vector(boost::lexical_cast<double>(posicoes_ida[0][0]),
					boost::lexical_cast<double>(posicoes_ida[0][1]),
					boost::lexical_cast<double>(posicoes_ida[0][2]))); //Central node
	positionAlloc->Add(
			Vector(boost::lexical_cast<double>(posicoes_ida[0][0]),
					boost::lexical_cast<double>(posicoes_ida[0][1]),
					boost::lexical_cast<double>(posicoes_ida[0][2]))); //Mule node

	//positionAlloc->Add(Vector(0.0, 0.0, 0.0)); //Central node
	//positionAlloc->Add(Vector(0.0, 0.0, 0.0)); //Mule 0 node
	for (uint32_t i = 0; i < nmules - 1; i++) { //- 1
		positionAlloc->Add(
				Vector(boost::lexical_cast<double>(posicoes_ida.back()[0]),
						boost::lexical_cast<double>(posicoes_ida.back()[1]),
						boost::lexical_cast<double>(posicoes_ida.back()[2]))); //Other Mules nodes
	}

	positionAlloc->Add(
			Vector(boost::lexical_cast<double>(posicoes_ida.back()[0]),
					boost::lexical_cast<double>(posicoes_ida.back()[1]),
					boost::lexical_cast<double>(posicoes_ida.back()[2]))); //AUV node
	//positionAlloc->Add(Vector(0.0, distance - 1, 0.0)); // Mule 1 node
	//positionAlloc->Add(Vector(0.0, distance, 0.0)); // AUV node

	mobility.SetPositionAllocator(positionAlloc);
	mobility.SetMobilityModel("ns3::WaypointMobilityModel"); //ns3::ConstantVelocityMobilityModel
	mobility.InstallAll();

	// add the waypoints based on the provided csv
	Ptr<WaypointMobilityModel> mob_mule0;
	mob_mule0 = mules.Get(0)->GetObject<WaypointMobilityModel>();
	for (uint32_t i = 0; i < posicoes_ida.size(); i++) { // não deve colocar a última posição
        //std::cout << " i: " << i << std::endl;		
        mob_mule0->AddWaypoint(
				Waypoint(Seconds(double(::atof(posicoes_ida[i][3].c_str()))),
						Vector(double(::atof(posicoes_ida[i][0].c_str())),
								double(::atof(posicoes_ida[i][1].c_str())),
								double(::atof(posicoes_ida[i][2].c_str())))));
	}

//	Config::SetDefault("ns3::TcpSocket::ConnCount", UintegerValue(6));
//	Config::SetDefault("ns3::TcpSocket::DataRetries", UintegerValue(6));
//	Config::SetDefault("ns3::TcpSocket::TcpNoDelay", BooleanValue(true));

	/* Install TCP Receiver on central and auv */

	Ipv4Address IPs[2 * nmules + 1];
	/* NOT USED IN THIS VERSION!
	 * IP [ M0, M1, M2, M3, M0, M1, M2, M3, A, C ]
	 * -> pode não ser necessário extender o vetor
	 * -> 18/02/2020
	 */
	for (uint32_t i = 0; i < nmules; i++) {
		/* IPs das mulas até IPs[nmules]
		 * IPs[nmules+1] auv
		 * IPs[nmules+2] central
		 */
		IPs[i] = mules.Get(i)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
		IPs[i + nmules] =
				mules.Get(i)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
	}
	IPs[2 * nmules] =
			auv.Get(0)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
	IPs[2 * nmules + 1] =
			central.Get(0)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();

	uint32_t k = 1, j = 1;
	for (uint32_t i = 0; i <= 2 * nmules - 1; i++) {
		if (i == 0) {
			PacketSinkHelper sink("ns3::TcpSocketFactory",
					InetSocketAddress(Ipv4Address::GetAny(), i + 9)); // IPs[2 * nmules]
			Sinks.Add(sink.Install(auv.Get(0))); // A0 Sink A9

			BulkSendHelper source("ns3::TcpSocketFactory",
					InetSocketAddress(
							auv.Get(0)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal(),
							9 + i));
			source.SetAttribute("MaxBytes",
					UintegerValue(totalTxBytes / nmules));
			Srcs.Add(source.Install(mules.Get(0))); // A0 Src M0 9
			Srcs.Get(i)->TraceConnectWithoutContext("MaxBytesSent",
					MakeCallback(new_state_machine));

		} else if (i % 2 == 0 && i != 0) { // par
			PacketSinkHelper sink("ns3::TcpSocketFactory",
					InetSocketAddress(Ipv4Address::GetAny(), i + 9)); // IPs[2 * nmules] mules.Get(i - j)
			Sinks.Add(sink.Install(auv.Get(0)));
			BulkSendHelper source1("ns3::TcpSocketFactory",
					InetSocketAddress(
							auv.Get(0)->GetObject<Ipv4>()->GetAddress(1,
									0).GetLocal(), 9 + i));
			source1.SetAttribute("MaxBytes",
					UintegerValue(totalTxBytes / nmules));
			Srcs.Add(source1.Install(mules.Get(i - j)));
			j++;
			Srcs.Get(i)->TraceConnectWithoutContext("MaxBytesSent",
					MakeCallback(new_state_machine));

		} else if (i % 2 != 0 && i != 0) { // impar
			PacketSinkHelper sink1("ns3::TcpSocketFactory",
					InetSocketAddress(Ipv4Address::GetAny(), i + 9));
			Sinks.Add(sink1.Install(mules.Get(i - k)));
			BulkSendHelper source2("ns3::TcpSocketFactory",
					InetSocketAddress(
							mules.Get(i - k)->GetObject<Ipv4>()->GetAddress(1,
									0).GetLocal(), 9 + i));
			source2.SetAttribute("MaxBytes",
					UintegerValue(totalTxBytes / nmules));
			Srcs.Add(source2.Install(central.Get(0))); //
			k++;
			Srcs.Get(i)->TraceConnectWithoutContext("MaxBytesSent",
					MakeCallback(new_state_machine));
		}

	}
	// control

	TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
	Ptr<Socket> recvSink = Socket::CreateSocket(controler.Get(0), tid);
	InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), 80);
    recvSink->SetAllowBroadcast(true);
	recvSink->Bind(local);
	recvSink->SetRecvCallback(MakeCallback(&ReceivePacket));

	InetSocketAddress remote = InetSocketAddress(Ipv4Address("255.255.255.255"),
			80);
	for (uint32_t i = 0; i < nmules; i++) {
		sources[i] = Socket::CreateSocket(mules.Get(i), tid);
		sources[i]->SetAllowBroadcast(true);
		sources[i]->Connect(remote);
	}
	sources[nmules] = Socket::CreateSocket(central.Get(0), tid);
	sources[nmules]->SetAllowBroadcast(true);
	sources[nmules]->Connect(remote);
	sources[nmules + 1] = Socket::CreateSocket(auv.Get(0), tid);
	sources[nmules + 1]->SetAllowBroadcast(true);
	sources[nmules + 1]->Connect(remote);

	if (tracing) {
		wifiPhy_control.EnablePcap("output/control/Controler",
				controler.Get(0)->GetDevice(1), false);
		wifiPhy_control.EnablePcap("output/control/Central",
				central.Get(0)->GetDevice(1), false);
		wifiPhy_control.EnablePcap("output/control/Auv",
				auv.Get(0)->GetDevice(1), false);
		for (int i = 0; i < int(nmules); i++) {
			wifiPhy_control.EnablePcap(
					"output/control/Mule_" + std::to_string(i),
					mules.Get(i)->GetDevice(1), false);
		}
		wifiPhy.EnablePcap("output/data/Central", central.Get(0)->GetDevice(0),
				false);
		wifiPhy.EnablePcap("output/data/Auv", auv.Get(0)->GetDevice(0), false);
		for (int i = 0; i < int(nmules); i++) {
			wifiPhy.EnablePcap("output/data/Mule_" + std::to_string(i),
					mules.Get(i)->GetDevice(0), false);
		}
	}
	/**
	 * Prepare mobility for RSS -> From Helder Fontes TraceBasedPropagationLossModel
	 */
	Config::MatchContainer matchCont =
			Config::LookupMatches(
					"/NodeList/0/DeviceList/0/$ns3::WifiNetDevice/Channel/$ns3::YansWifiChannel/PropagationLossModel/$ns3::TraceBasedPropagationLossModel");
	//NS_LOG_UNCOND("Number TraceBaded Models running:");
	//NS_LOG_UNCOND(matchCont.GetN());
	Ptr<TraceBasedPropagationLossModel> modelTraceBased = DynamicCast<
			TraceBasedPropagationLossModel>(matchCont.Get(0));
	modelTraceBased->InitMobilityModelForEachNode();
	Simulator::Schedule(Seconds(0.0), &new_state_machine, 0);
	//Simulator::Stop(Seconds(2500));
    Simulator::Run();
    if (failed) {
        double Now = Simulator::Now().GetSeconds();
	    std::ostringstream pcap;
	    pcap << tipo;
	    if (distance < 1000) {
		    pcap << ":Seed:0" << seed << "_Mules:0" << nmules << "_Data:0"
				    << data_size << "_Distance:0" << distance << "_Phy_Type:"
				    << phy_type << "_BW:" << channel_width;
	    } else if (distance >= 1000) {
		    pcap << ":Seed:0" << seed << "_Mules:0" << nmules << "_Data:0"
				    << data_size << "_Distance:" << distance << "_Phy_Type:"
				    << phy_type << "_BW:" << channel_width;
	    }
	    double Rb;
	    std::cout << "FAIL!\nEnd time: " << Now << std::endl;
	    Rb = (8 * totalTxBytes) / (Now);
	    std::cout << "Rb: " << Rb << " bit/s" << std::scientific << std::endl;
	    std::ofstream myfile;
	    pcap << ".txt";
	    std::cout << pcap.str() << std::endl;
	    myfile.open("output/" + pcap.str(), std::ios::out | std::ios::app); // make sure the file exists
	    myfile.close();
	    myfile.open("output/" + pcap.str(),
			    std::ios::in | std::ios::out | std::ios::app);
	    myfile.precision(4);
	    myfile << "FAIL_"<<Now << "_"<< tipo << ":" << seed << ":" << nmules << ":" << distance << ":" 
			    << total_data <<":" << phy_type << ":"
			    << channel_width << "\n";
	    myfile.close();
    }
	Simulator::Destroy();
}
void arranca(uint32_t n) {
	Sinks.Get(n)->SetState(Seconds(0.0));
	Srcs.Get(n)->SetState(Seconds(0.0));
}
// h counts the number of times the new_state_machine was called
//	-> the 1st time is the scheduler
//	the next times are the callbacks of the bulk send app
// mob_mul0 -> waypoint mobility of the first data mule docking at the AUV, and the data mule coming from the AUV
// mob_mule1 -> waypoint mobility of the the data mule docking at the AUV
// state_k -> used to iterate over the mule nodes, to get the mobb_mule0
// id -> used to get the data mule UDMP id
// nact -> is the counter used to select the applications that are transmitting data -> data applications (bulk send, sink) in the nodes
void new_state_machine(uint32_t maxBytes) {
	double Now = Simulator::Now().GetSeconds();
    if (maxBytes == 0 && !init){
        failed == true;
    }
    total_data = total_data + maxBytes;
	Ptr<WaypointMobilityModel> mob_mule0, mob_mule1;

	double index_posicao_fix_volta_mule1_ida = 0, posicao_match_volta = 0,
			dock_time_index_vector_ida, dock_time_index_vector_volta,
			data_time_index_vector_volta;
	bool earlier = false;
	ns3::Time time_next;
	h++;
	std::cout << "State Machine\nStatus\nNow: " << Now << "\nINIT: " << init
			<< "\nBACK: " << back << "\nLAST: " << last << "\nFINISHED: "
			<< finished << "\nH: " << h << "\nNACT: " << nact << std::endl;
	if (nact == 0 && (nmules > 1)) {
		mob_mule0 = mules.Get(nact)->GetObject<WaypointMobilityModel>();
	} else if (nact == 1 && (nmules > 1)) {
		mob_mule0 = mules.Get(nact - 1)->GetObject<WaypointMobilityModel>();
		mob_mule1 = mules.Get(nact)->GetObject<WaypointMobilityModel>();
	} else if (nact > 1 && !last && !finished && (nmules > 1)) {
		mob_mule0 = mules.Get(nact - state_k - 1)->GetObject<
				WaypointMobilityModel>();
		mob_mule1 =
				mules.Get(nact - state_k)->GetObject<WaypointMobilityModel>();
		state_k += 1;
	} else if (nact > 1 && last && (nmules > 1) && !finished) {
		mob_mule0 = mules.Get(nmules - 1)->GetObject<WaypointMobilityModel>();
	} else if ((nmules == 1)) {
		mob_mule0 = mules.Get(0)->GetObject<WaypointMobilityModel>();
	}

	if (init) {
		init = false;
		back = true;

		// first data mule waypoints already set in the configuration
		// udmp
		std::ostringstream message;
		message << CENTRAL << "_" << AUV << "_" << "0";
		Simulator::Schedule(Seconds(0.0), &GenerateTraffic, sources[nmules],
				message.str()); // data size

		std::ostringstream message0;
		message0 << "02" << "_" << AUV << "_06_0"
				<< boost::lexical_cast<std::string>(data_size / nmules).length()
				<< "_" << boost::lexical_cast<std::string>(data_size / nmules);
		// rss
		uint32_t senderNodeY = Srcs.Get(nact)->GetNode()->GetId();
		uint32_t receiverNodeX = Sinks.Get(nact)->GetNode()->GetId();
		for (uint32_t i = 0; i < posicoes_ida.size(); i++) {
			Simulator::Schedule(
					Seconds(double(::atof(posicoes_ida[i][3].c_str()))),
					&updateRssOfNodeXFromNodeY,
					double(::atof(posicoes_volta[i][4].c_str())), receiverNodeX,
					senderNodeY);
			Simulator::Schedule(
					Seconds(double(::atof(posicoes_ida[i][3].c_str()))),
					&updateRssOfNodeXFromNodeY,
					double(::atof(posicoes_volta[i][4].c_str())), senderNodeY,
					receiverNodeX);
		}
		// data
		Simulator::Schedule(Seconds(dock_time), &GenerateTraffic, sources[0],
				message0.str()); // dock request
		Simulator::Schedule(Seconds(data_time), &arranca, nact); // A0

		nact += 1;
		return;
	}
	for (uint32_t i = 0; i < posicoes_ida.size(); i++) {
		if (!init && !finished && !earlier) {
			if (mob_mule0->WaypointsLeft() > 2) { // received the data before te csv ended it is needed to fix the vector positions
				time_next = mob_mule0->GetNextWaypoint().time;
				earlier = true;
				mob_mule0->ClearMobility();
			}
		}
		if (double(::atof(posicoes_ida[i][3].c_str())) == dock_time) {
			dock_time_index_vector_ida = i;
			dock_time_index_vector_volta = posicoes_volta.size() - i;
		} else if (double(::atof(posicoes_volta[i][3].c_str())) == data_time) {
			data_time_index_vector_volta = i;
		} else if (earlier
				&& ((mob_mule0->GetNextWaypoint().position.x
						== double(::atof(posicoes_volta[i][0].c_str())))
						&& (mob_mule0->GetNextWaypoint().position.y
								== double(::atof(posicoes_volta[i][1].c_str())))
						&& (mob_mule0->GetNextWaypoint().position.z
								== double(::atof(posicoes_volta[i][2].c_str()))))) {
			posicao_match_volta = i; // use in the data mule that is leaving the auv
			index_posicao_fix_volta_mule1_ida = posicoes_volta.size() - i; //use in the data mule that is docking at the auv
		}
	}
	std::cout << "EARLIER: " << earlier << std::endl;
	if (!init && back) {
		if (nmules == 1) {

			//UDMP
			std::ostringstream message0;
			message0 << AUV << "_" << CENTRAL << "_" << "08";
			// Data Chunk Sent
			Simulator::Schedule(Seconds(1), &GenerateTraffic, sources[nmules + 1],
					message0.str()); // Data Chunk Sent
			std::ostringstream dock_mula_0;
			dock_mula_0 << "0" << boost::lexical_cast<std::string>(id + 2)
					<< "_" << CENTRAL << "_" << "06_0"
					<< boost::lexical_cast<std::string>(data_size / nmules).length()
					<< "_"
					<< boost::lexical_cast<std::string>(data_size / nmules); // Dock Req Mula 0
			uint32_t senderNodeY = Srcs.Get(1)->GetNode()->GetId();
			uint32_t receiverNodeX = Sinks.Get(1)->GetNode()->GetId();

			if (earlier) {
				//Waypoints
				for (uint32_t i = posicao_match_volta + 1;
						i < posicoes_volta.size(); i++) {
					mob_mule0->AddWaypoint(
							Waypoint(
									Seconds(
											Now
													+ double(
															abs(
																	::atof(
																			posicoes_volta[posicao_match_volta][3].c_str())
																			- ::atof(
																					posicoes_volta[i][3].c_str())))),
									Vector(
											double(
													::atof(
															posicoes_volta[i][0].c_str())),
											double(
													::atof(
															posicoes_volta[i][1].c_str())),
											double(
													::atof(
															posicoes_volta[i][2].c_str())))));

					//RSS
					Simulator::Schedule(
							Seconds(
									double(
											abs(
													::atof(
															posicoes_volta[posicao_match_volta][3].c_str())
															- ::atof(
																	posicoes_volta[i][3].c_str())))),
							&updateRssOfNodeXFromNodeY,
							double(::atof(posicoes_volta[i][4].c_str())),
							receiverNodeX, senderNodeY);

					Simulator::Schedule(
							Seconds(

									double(
											abs(
													::atof(
															posicoes_volta[posicao_match_volta][3].c_str())
															- ::atof(
																	posicoes_volta[i][3].c_str())))),
							&updateRssOfNodeXFromNodeY,
							double(::atof(posicoes_volta[i][4].c_str())),
							senderNodeY, receiverNodeX);
				}
				//UDMP
				Simulator::Schedule(
						Seconds(

								abs(
										::atof(
												posicoes_volta[posicao_match_volta][3].c_str())
												- dock_time)), &GenerateTraffic,
						sources[id], dock_mula_0.str()); // dock req mula 0
				// Data
				Simulator::Schedule(
						Seconds(

								abs(
										::atof(
												posicoes_volta[posicao_match_volta][3].c_str())
												- data_time)), &arranca, 1);
			} else if (!earlier) {
				for (uint32_t i = 0; i < posicoes_volta.size(); i++) {
					// Waypoints
					mob_mule0->AddWaypoint(
							Waypoint(
									Seconds(
											Now
													+ double(
															::atof(
																	posicoes_volta[i][3].c_str()))),
									Vector(
											double(
													::atof(
															posicoes_volta[i][0].c_str())),
											double(
													::atof(
															posicoes_volta[i][1].c_str())),
											double(
													::atof(
															posicoes_volta[i][2].c_str())))));

					//RSS
					Simulator::Schedule(
							Seconds(
									double(
											::atof(
													posicoes_volta[i][3].c_str()))),
							&updateRssOfNodeXFromNodeY,
							double(::atof(posicoes_volta[i][4].c_str())),
							receiverNodeX, senderNodeY);

					Simulator::Schedule(Seconds(

					double(::atof(posicoes_volta[i][3].c_str()))),
							&updateRssOfNodeXFromNodeY,
							double(::atof(posicoes_volta[i][4].c_str())),
							senderNodeY, receiverNodeX);
				}
				// UDMP
				Simulator::Schedule(Seconds(dock_time), &GenerateTraffic,
						sources[id], dock_mula_0.str());	// dock req mula 0
				// DATA
				Simulator::Schedule(Seconds(abs(data_time)), &arranca, 1);
			}
			nact += 1;
			earlier = false;
			back = false;
			finished = true;
			return;
		}

		else if (nmules > 1) {

			//UDMP
			std::ostringstream message0;
			message0 << AUV << "_" << CENTRAL << "_" << "08";
			// Data Chunk Sent
			Simulator::ScheduleNow(&GenerateTraffic, sources[nmules + 1],
					message0.str());						// Data Chunk Sent
			std::ostringstream dock_mula_0;
			dock_mula_0 << "0" << boost::lexical_cast<std::string>(id + 2)
					<< "_" << CENTRAL << "_" << "06_0"
					<< boost::lexical_cast<std::string>(data_size / nmules).length()
					<< "_"
					<< boost::lexical_cast<std::string>(data_size / nmules);// Dock Req Mula 0

			std::ostringstream dock_mula_1;
			dock_mula_1 << "0" << boost::lexical_cast<std::string>(id + 3)
					<< "_" << AUV << "_" << "06_0"
					<< boost::lexical_cast<std::string>(data_size / nmules).length()
					<< "_"
					<< boost::lexical_cast<std::string>(data_size / nmules);// Dock Req Mula 1
			uint32_t senderNodeY = Srcs.Get(nact)->GetNode()->GetId();// apps mula 0
			uint32_t receiverNodeX = Sinks.Get(nact)->GetNode()->GetId();

            Simulator::ScheduleNow(&GenerateTraffic, sources[id + 1],
					dock_mula_1.str());		
			if (earlier) {
				//mula 0
				//Waypoints
				for (uint32_t i = posicao_match_volta + 1;
						i < posicoes_volta.size(); i++) {
					mob_mule0->AddWaypoint(
							Waypoint(
									Seconds(
											Now
													+ double(
															abs(
																	::atof(
																			posicoes_volta[posicao_match_volta][3].c_str())
																			- ::atof(
																					posicoes_volta[i][3].c_str())))),
									Vector(
											double(
													::atof(
															posicoes_volta[i][0].c_str())),
											double(
													::atof(
															posicoes_volta[i][1].c_str())),
											double(
													::atof(
															posicoes_volta[i][2].c_str())))));

					//RSS
					Simulator::Schedule(
							Seconds(
									double(
													abs(
															::atof(
																	posicoes_volta[posicao_match_volta][3].c_str())
																	- ::atof(
																			posicoes_volta[i][3].c_str())))),
							&updateRssOfNodeXFromNodeY,
							double(::atof(posicoes_volta[i][4].c_str())),
							receiverNodeX, senderNodeY);

					Simulator::Schedule(
							Seconds(
									double(
													abs(
															::atof(
																	posicoes_volta[posicao_match_volta][3].c_str())
																	- ::atof(
																			posicoes_volta[i][3].c_str())))),
							&updateRssOfNodeXFromNodeY,
							double(::atof(posicoes_volta[i][4].c_str())),
							senderNodeY, receiverNodeX);
				}
				//UDMP
				Simulator::Schedule(
						Seconds(
								abs(
												::atof(
														posicoes_volta[posicao_match_volta][3].c_str())
														- dock_time)),
						&GenerateTraffic, sources[id], dock_mula_0.str());// dock req mula 0
				// Data
				Simulator::Schedule(
						Seconds(
								abs(
												::atof(
														posicoes_volta[posicao_match_volta][3].c_str())
														- data_time)), &arranca,
						nact);
				// mula 1
				senderNodeY = Srcs.Get(nact + 1)->GetNode()->GetId();// apps mula 0
				receiverNodeX = Sinks.Get(nact + 1)->GetNode()->GetId();

				for (uint32_t i = dock_time_index_vector_ida + 1;
						i < posicoes_ida.size(); i++) {
					mob_mule1->AddWaypoint(
							Waypoint(
									Seconds(
											Now
													+ double(
															abs(
																	::atof(
																			posicoes_ida[dock_time_index_vector_ida][3].c_str())
																			- ::atof(
																					posicoes_ida[i][3].c_str())))),
									Vector(
											double(
													::atof(
															posicoes_ida[i][0].c_str())),
											double(
													::atof(
															posicoes_ida[i][1].c_str())),
											double(
													::atof(
															posicoes_ida[i][2].c_str())))));

					//RSS
					Simulator::Schedule(
							Seconds(
									double(
													abs(
															::atof(
																	posicoes_ida[dock_time_index_vector_ida][3].c_str())
																	- ::atof(
																			posicoes_ida[i][3].c_str())))),
							&updateRssOfNodeXFromNodeY,
							double(::atof(posicoes_ida[i][4].c_str())),
							receiverNodeX, senderNodeY);

					Simulator::Schedule(
							Seconds(
									double(
													abs(
															::atof(
																	posicoes_ida[dock_time_index_vector_ida][3].c_str())
																	- ::atof(
																			posicoes_ida[i][3].c_str())))),
							&updateRssOfNodeXFromNodeY,
							double(::atof(posicoes_ida[i][4].c_str())),
							senderNodeY, receiverNodeX);
				}
				//UDMP
							// dock req mula 0
				id++;
				// Data
				Simulator::Schedule(
						Seconds(
								abs(
												::atof(
														posicoes_ida[dock_time_index_vector_ida][3].c_str())
														- data_time)), &arranca,
						nact + 1);
			} else if (!earlier) {
				//mula 0
				//Waypoints
				for (uint32_t i = 0; i < posicoes_volta.size(); i++) {
					mob_mule0->AddWaypoint(
							Waypoint(
									Seconds(
											Now
													+ double(
															::atof(
																	posicoes_volta[i][3].c_str()))),
									Vector(
											double(
													::atof(
															posicoes_volta[i][0].c_str())),
											double(
													::atof(
															posicoes_volta[i][1].c_str())),
											double(
													::atof(
															posicoes_volta[i][2].c_str())))));

					//RSS
					Simulator::Schedule(
							Seconds(
									double(
											::atof(
													posicoes_volta[i][3].c_str()))),
							&updateRssOfNodeXFromNodeY,
							double(::atof(posicoes_volta[i][4].c_str())),
							receiverNodeX, senderNodeY);

					Simulator::Schedule(Seconds(

					double(::atof(posicoes_volta[i][3].c_str()))),
							&updateRssOfNodeXFromNodeY,
							double(::atof(posicoes_volta[i][4].c_str())),
							senderNodeY, receiverNodeX);
				}
				//UDMP
				Simulator::Schedule(Seconds(dock_time), &GenerateTraffic,
						sources[id], dock_mula_0.str());	// dock req mula 0
				// Data
				Simulator::Schedule(Seconds(data_time), &arranca, nact);
				// mula 1
				senderNodeY = Srcs.Get(nact + 1)->GetNode()->GetId();// apps mula 0
				receiverNodeX = Sinks.Get(nact + 1)->GetNode()->GetId();

				for (uint32_t i = dock_time_index_vector_ida + 1;
						i < posicoes_ida.size(); i++) {
					mob_mule1->AddWaypoint(
							Waypoint(
									Seconds(
											Now
													+ double(
															abs(
																	dock_time
																			- ::atof(
																					posicoes_ida[i][3].c_str())))),
									Vector(
											double(
													::atof(
															posicoes_ida[i][0].c_str())),
											double(
													::atof(
															posicoes_ida[i][1].c_str())),
											double(
													::atof(
															posicoes_ida[i][2].c_str())))));

					//RSS
					Simulator::Schedule(
							Seconds(

									double(
											abs(
													dock_time
															- ::atof(
																	posicoes_ida[i][3].c_str())))),
							&updateRssOfNodeXFromNodeY,
							double(::atof(posicoes_ida[i][4].c_str())),
							receiverNodeX, senderNodeY);

					Simulator::Schedule(
							Seconds(

									double(
											abs(
													dock_time
															- ::atof(
																	posicoes_ida[i][3].c_str())))),
							&updateRssOfNodeXFromNodeY,
							double(::atof(posicoes_ida[i][4].c_str())),
							senderNodeY, receiverNodeX);
				}
				//UDMP
				Simulator::Schedule(Seconds(1), &GenerateTraffic, sources[id + 1],
						dock_mula_1.str());					// dock req mula 0
				id++;
				// Data
				Simulator::Schedule(Seconds(abs(dock_time - data_time)),
						&arranca, nact + 1);

			}
			if ((nact + 2) == (2 * nmules - 1)) {
				// last iteration
				back = false;
				last = true;
			}
			earlier = false;
			nact += 2;
			return;
		}
	}
	if (!init && !back && last) {
		std::ostringstream message0;
		message0 << AUV << "_" << CENTRAL << "_" << "08";
		Simulator::Schedule(Seconds(1), &GenerateTraffic, sources[nmules + 1],
				message0.str()); // Data Chunk Sent
		std::ostringstream dock_mula_0;
		dock_mula_0 << "0" << boost::lexical_cast<std::string>(id + 2) << "_"
				<< CENTRAL << "_" << "06_0"
				<< boost::lexical_cast<std::string>(data_size / nmules).length()
				<< "_" << boost::lexical_cast<std::string>(data_size / nmules); // Dock Req Mula 0
		if (earlier) {
			Simulator::Schedule(Seconds(dock_time), &GenerateTraffic,
					sources[id], dock_mula_0.str()); // dock req mula 0
			for (uint32_t i = posicao_match_volta + 1;
					i < posicoes_volta.size(); i++) {
				mob_mule0->AddWaypoint(
						Waypoint(
								Seconds(Now + 
										double(
														abs(
																::atof(
																		posicoes_volta[posicao_match_volta][3].c_str())
																		- ::atof(
																				posicoes_volta[i][3].c_str())))),
								Vector(
										double(
												::atof(
														posicoes_volta[i][0].c_str())),
										double(
												::atof(
														posicoes_volta[i][1].c_str())),
										double(
												::atof(
														posicoes_volta[i][2].c_str())))));
				// rss
				Simulator::Schedule(
						Seconds(
								 double(
												abs(
														::atof(
																posicoes_volta[posicao_match_volta][3].c_str())
																- ::atof(
																		posicoes_volta[i][3].c_str())))),
						&updateRssOfNodeXFromNodeY,
						double(::atof(posicoes_volta[i][4].c_str())),
						Sinks.Get(nact)->GetNode()->GetId(),
						Srcs.Get(nact)->GetNode()->GetId());
				Simulator::Schedule(
						Seconds(
								 double(
												abs(
														::atof(
																posicoes_volta[posicao_match_volta][3].c_str())
																- ::atof(
																		posicoes_volta[i][3].c_str())))),
						&updateRssOfNodeXFromNodeY,
						double(::atof(posicoes_volta[i][4].c_str())),
						Srcs.Get(nact)->GetNode()->GetId(),
						Sinks.Get(nact)->GetNode()->GetId());
			}
			// Data
			Simulator::Schedule(
					Seconds(
							 abs(
											::atof(
													posicoes_volta[posicao_match_volta][3].c_str())
													- data_time)), &arranca,
					nact);
		} else {
			Simulator::Schedule(Seconds(dock_time), &GenerateTraffic,
					sources[id], dock_mula_0.str()); // dock req mula 0
            
			Simulator::Schedule(Seconds(data_time), &arranca, nact);

			
			for (uint32_t i = 0; i < posicoes_volta.size(); i++) {
				mob_mule0->AddWaypoint(
						Waypoint(
								Seconds(
										Now
												+ double(
														::atof(
																posicoes_volta[i][3].c_str()))),
								Vector(
										double(
												::atof(
														posicoes_volta[i][0].c_str())),
										double(
												::atof(
														posicoes_volta[i][1].c_str())),
										double(
												::atof(
														posicoes_volta[i][2].c_str())))));

				// rss
				Simulator::Schedule(Seconds(

				double(::atof(posicoes_volta[i][3].c_str()))),
						&updateRssOfNodeXFromNodeY,
						double(::atof(posicoes_volta[i][4].c_str())),
						Sinks.Get(nact)->GetNode()->GetId(),
						Srcs.Get(nact)->GetNode()->GetId());
				Simulator::Schedule(Seconds(

				double(::atof(posicoes_volta[i][3].c_str()))),
						&updateRssOfNodeXFromNodeY,
						double(::atof(posicoes_volta[i][4].c_str())),
						Srcs.Get(nact)->GetNode()->GetId(),
						Sinks.Get(nact)->GetNode()->GetId());
			}

		}

		Simulator::Schedule(Seconds(1), &GenerateTraffic, sources[id + 1],
				dock_mula_0.str()); // dock req mula 1

		id++;
		last = false;
		finished = true;
       

		return;
	}
	if (finished && !last && (h == 2 * nmules + 1)) {
		std::ostringstream message;
		message << CENTRAL << "_" << AUV << "_" << "09";
		Simulator::Schedule(Seconds(1), &GenerateTraffic, sources[nmules],
				message.str()); // Data Chunk Sent
		return;
	}

}
void ReceivePacket(Ptr<Socket> socket) {
	Ptr<Packet> p = socket->Recv();
	uint8_t *buffer = new uint8_t[p->GetSize()];
	uint32_t size = p->CopyData(buffer, p->GetSize());
	std::string s = std::string(buffer, buffer + p->GetSize());
	std::vector<std::string> results;
	boost::split(results, s, [](char c) {
		return c == '_';
	});
	std::cout << "Received: " << s << "\nAt time: "
			<< Simulator::Now().GetSeconds() << std::endl;
	int type, src, dst;
	std::istringstream(results[2]) >> type;
	std::istringstream(results[0]) >> src;
	std::istringstream(results[1]) >> dst;
	std::cout << "Type: " << type << std::endl;

	if (type == 0) {
		std::cout << "Filesize Request" << std::endl;
		std::ostringstream message;
		message << AUV << "_" << CENTRAL << "_" << "01" << "_0"
				<< boost::lexical_cast<std::string>(data_size).length() << "_"
				<< boost::lexical_cast<std::string>(data_size);

		Simulator::ScheduleNow(&GenerateTraffic, sources[nmules + 1],
				message.str()); // Filesize Response auv responde
	} else if (type == 1) {
		std::cout << "Filesize Response" << std::endl;
		std::ostringstream message;
		message << CENTRAL << "_" << AUV << "_" << "02" << "_0"
				<< boost::lexical_cast<std::string>(nmules).length() << "_"
				<< boost::lexical_cast<std::string>(nmules);

		Simulator::ScheduleNow(&GenerateTraffic, sources[nmules],
				message.str()); // Data Mules being dispatched central responde
	} else if (type == 2) {
		std::cout << "Number of Mules Being Dispatch" << std::endl;
		std::ostringstream message;
		message << AUV << "_" << CENTRAL << "_" << "03" << "_0"
				<< boost::lexical_cast<std::string>("OK").length() << "_"
				<< boost::lexical_cast<std::string>("OK");

		Simulator::ScheduleNow(&GenerateTraffic, sources[nmules + 1],
				message.str()); // Data Mules being dispatched ack auv responde
	} else if (type == 3) {
		std::cout << "Number of Mules Being Dispatch Ack" << std::endl;
		std::ostringstream message;
		message << CENTRAL << "_0"
				<< boost::lexical_cast<std::string>(2 + dispatched) << "_"
				<< "04" << "_0"
				<< boost::lexical_cast<std::string>(data_size / nmules).length()
				<< "_" << boost::lexical_cast<std::string>(data_size / nmules);
		Simulator::ScheduleNow(&GenerateTraffic, sources[nmules],
				message.str()); // Send Mules Request
	} else if (type == 4) {
		std::cout << "Send Mule Request" << std::endl;
		if (dispatched < nmules) {
			std::ostringstream message;
			message << "0" << boost::lexical_cast<std::string>(2 + dispatched)
					<< "_" << CENTRAL << "_" << "05" << "_0"
					<< boost::lexical_cast<std::string>("OK").length() << "_"
					<< boost::lexical_cast<std::string>("OK");
			dispatched++;

			Simulator::ScheduleNow(&GenerateTraffic, sources[dispatched - 1],
					message.str()); // Send Mules Response
		}

	} else if (type == 5) {
		std::cout << "Send Mule Response" << std::endl;
		if (dispatched < nmules) {
			std::ostringstream message;
			message << CENTRAL << "_0"
					<< boost::lexical_cast<std::string>(2 + dispatched) << "_"
					<< "04" << "_0"
					<< boost::lexical_cast<std::string>(data_size / nmules).length()
					<< "_"
					<< boost::lexical_cast<std::string>(data_size / nmules);

			Simulator::ScheduleNow(&GenerateTraffic, sources[nmules],
					message.str()); // Send Mules Response
		}
	} else if (type == 6) {
		std::cout << "Dock Request ID: " << src << std::endl;
		std::ostringstream message;
		message << results[1] << "_" << results[0] << "_" << "07" << "_0"
				<< boost::lexical_cast<std::string>("OK").length() << "_"
				<< boost::lexical_cast<std::string>("OK");

		if (dst == 1) { //  dock auv -> auv "says" ok
			Simulator::ScheduleNow(&GenerateTraffic, sources[nmules + 1],
					message.str()); // Dock Response
		} else if (dst == 0) { // dock a central -> central "says" ok
			Simulator::ScheduleNow(&GenerateTraffic, sources[nmules],
					message.str()); // Dock Response
		}
	} else if (type == 7) {
		std::cout << "Dock Response" << "\n---------" << std::endl;
	} else if (type == 8) {
		std::cout << "Data Chunk Sent" << "\n---------" << std::endl;
	} else if (type == 9) {
// simulation output log
		std::cout << "Data Received" << "\n---------" << std::endl;
        if(!failed){		
            double Now = Simulator::Now().GetSeconds();
		    std::ostringstream pcap;
		    pcap << tipo;
		    if (distance < 1000) {
			    pcap << ":Seed:0" << seed << "_Mules:0" << nmules << "_Data:0"
					    << data_size << "_Distance:0" << distance << "_Phy_Type:"
					    << phy_type << "_BW:" << channel_width;
		    } else if (distance >= 1000) {
			    pcap << ":Seed:0" << seed << "_Mules:0" << nmules << "_Data:0"
					    << data_size << "_Distance:" << distance << "_Phy_Type:"
					    << phy_type << "_BW:" << channel_width;
		    }
		    double Rb;

		    std::cout << "End time: " << Now << std::endl;
		    Rb = (8 * totalTxBytes) / (Now);
		    std::cout << "Rb: " << Rb << " bit/s" << std::scientific << std::endl;
		    std::ofstream myfile;
		    pcap << ".txt";
		    std::cout << pcap.str() << std::endl;
		    myfile.open("output/" + pcap.str(), std::ios::out | std::ios::app); // make sure the file exists
		    myfile.close();
		    myfile.open("output/" + pcap.str(),
				    std::ios::in | std::ios::out | std::ios::app);
		    myfile.precision(4);
		    myfile << tipo << ":" << seed << ":" << nmules << ":" << distance << ":"
				    << Rb << ":" << data_size << ":" << phy_type << ":"
				    << channel_width << "\n";
		    myfile.close();
        }
		Simulator::Stop(Seconds(0.0));
	}
}

static void GenerateTraffic(Ptr<Socket> socket, std::string datas) {
	std::stringstream msgx;
	msgx << datas;
	uint16_t packetSize = msgx.str().length() + 1;
	Ptr<Packet> packet = Create<Packet>((uint8_t*) msgx.str().c_str(),
			packetSize);
	socket->Send(packet);
}

static void updateRssOfNodeXFromNodeY(double rss, uint32_t receiverNodeX,
		uint32_t senderNodeY) {
	Config::MatchContainer matchCont =
			Config::LookupMatches(
					"/NodeList/0/DeviceList/0/$ns3::WifiNetDevice/Channel/$ns3::YansWifiChannel/PropagationLossModel/$ns3::TraceBasedPropagationLossModel");
	Ptr<TraceBasedPropagationLossModel> modelTraceBased = DynamicCast<
			TraceBasedPropagationLossModel>(matchCont.Get(0));
	modelTraceBased->SetRssOfNodeXFromNodeY(rss, receiverNodeX, senderNodeY);

}

