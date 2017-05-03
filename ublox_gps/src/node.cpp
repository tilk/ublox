//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <ublox_gps/gps.h>
#include <ublox_gps/navsat_conversions.h>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/serial_port.hpp>

#include <boost/regex.hpp>

#include <ros/ros.h>
#include <ros/serialization.h>
#include <ublox_msgs/CfgGNSS.h>
#include <ublox_msgs/CfgODO.h>
#include <ublox_msgs/CfgINF.h>
#include <ublox_msgs/NavPOSLLH.h>
#include <ublox_msgs/NavSOL.h>
#include <ublox_msgs/NavDOP.h>
#include <ublox_msgs/NavSAT.h>
#include <ublox_msgs/NavPVT.h>
#include <ublox_msgs/NavSTATUS.h>
#include <ublox_msgs/NavVELNED.h>
#include <ublox_msgs/NavORB.h>
#include <ublox_msgs/NavCLOCK.h>
#include <ublox_msgs/NavHPPOSLLH.h>
#include <ublox_msgs/NavRELPOSNED.h>
#include <ublox_msgs/NavSVIN.h>
#include <ublox_msgs/RxmRAWX.h>
#include <ublox_msgs/RxmRAWX_Meas.h>
#include <ublox_msgs/ublox_msgs.h>
#include <rtcm_msgs/Message.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>

const static uint32_t kROSQueueSize = 1;

using namespace ublox_gps;

boost::shared_ptr<ros::NodeHandle> nh;
boost::shared_ptr<diagnostic_updater::Updater> updater;
boost::shared_ptr<diagnostic_updater::TopicDiagnostic> freq_diag;
Gps gps;
std::map<std::string, bool> enabled;
std::string frame_id, odom_frame_id;

sensor_msgs::NavSatFix fix;
nav_msgs::Odometry odometry;

double uere;

template<typename MessageT>
class UbloxPublisher {
public:
  UbloxPublisher(std::string topicName): topicName(topicName) {
  }
  void subscribe(int rate = 1) {
    if (publisher) throw std::runtime_error("double subscription for " + topicName);
    publisher = nh->advertise<MessageT>(topicName, kROSQueueSize);
    gps.subscribe<MessageT>(boost::bind(&UbloxPublisher<MessageT>::publish, this, _1), rate);
  }
  const MessageT &lastValue() {
    return last;
  }
  bool isCurrent(unsigned int iTOW) {
    return publisher && last.iTOW == iTOW;
  }
private:
  void publish(const MessageT &m) {
    last = m;
    publisher.publish(m);
  }
  std::string topicName;
  ros::Publisher publisher;
  MessageT last;
};

UbloxPublisher<ublox_msgs::NavSTATUS> pubNavStatus("navstatus");
UbloxPublisher<ublox_msgs::NavSOL> pubNavSol("navsol");
UbloxPublisher<ublox_msgs::NavDOP> pubNavDOP("navdop");
UbloxPublisher<ublox_msgs::NavPVT> pubNavPVT("navpvt");
UbloxPublisher<ublox_msgs::NavSBAS> pubNavSBAS("navsbas");
UbloxPublisher<ublox_msgs::NavDGPS> pubNavDGPS("navdgps");
UbloxPublisher<ublox_msgs::NavVELNED> pubNavVelNED("navvelned");
UbloxPublisher<ublox_msgs::NavPOSLLH> pubNavPosLLH("navposllh");
UbloxPublisher<ublox_msgs::NavHPPOSLLH> pubNavHPPosLLH("navhpposllh");
UbloxPublisher<ublox_msgs::NavSAT> pubNavSat("navsat");
UbloxPublisher<ublox_msgs::NavSVINFO> pubNavSVINFO("navsvinfo");
UbloxPublisher<ublox_msgs::NavORB> pubNavORB("navorb");
UbloxPublisher<ublox_msgs::NavTIMEGPS> pubNavTimeGPS("navtimegps");
UbloxPublisher<ublox_msgs::NavTIMEUTC> pubNavTimeUTC("navtimeutc");
UbloxPublisher<ublox_msgs::NavCLOCK> pubNavClock("navclock");
UbloxPublisher<ublox_msgs::NavRELPOSNED> pubNavRelPosNED("navrelposned");
UbloxPublisher<ublox_msgs::NavSVIN> pubNavSVIN("navsvin");
UbloxPublisher<ublox_msgs::RxmRAW> pubRxmRAW("rxmraw");
UbloxPublisher<ublox_msgs::RxmRAWX> pubRxmRAWX("rxmrawx");
UbloxPublisher<ublox_msgs::RxmSFRB> pubRxmSFRB("rxmsfrb");
UbloxPublisher<ublox_msgs::RxmSFRBX> pubRxmSFRBX("rxmsfrbx");
UbloxPublisher<ublox_msgs::RxmSVSI> pubRxmSVSI("rxmsvsi");
UbloxPublisher<ublox_msgs::RxmALM> pubRxmALM("rxmalm");
UbloxPublisher<ublox_msgs::RxmEPH> pubRxmEPH("rxmeph");
UbloxPublisher<ublox_msgs::AidALM> pubAidALM("aidalm");
UbloxPublisher<ublox_msgs::AidEPH> pubAidEPH("aideph");
UbloxPublisher<ublox_msgs::AidHUI> pubAidHUI("aidhui");

ros::Subscriber subRTCM;

struct NavData {
    unsigned int iTOW;
    double lon, lat, height, hMSL, hAcc, vAcc;
    double nDOP, eDOP, vDOP;
    double gSpeed, velN, velE, velD, sAcc, heading, headAcc;
    int numSV;
} navData;

void endOfEpoch(const ublox_msgs::NavEOE &m) {
  // update nav data
  navData.iTOW = m.iTOW;

  if (pubNavHPPosLLH.isCurrent(m.iTOW)) {
    const auto &nm = pubNavHPPosLLH.lastValue();
    navData.lon = nm.lon * 1e-7 + nm.lonHp * 1e-9;
    navData.lat = nm.lat * 1e-7 + nm.latHp * 1e-9;
    navData.height = nm.height * 1e-3 + nm.heightHp * 1e-4;
    navData.hMSL = nm.hMSL * 1e-3 + nm.hMSLHp * 1e-4;
    navData.hAcc = nm.hAcc * 1e-4;
    navData.vAcc = nm.vAcc * 1e-4;
  } else if (pubNavPVT.isCurrent(m.iTOW)) {
    const auto &nm = pubNavPVT.lastValue();
    navData.lon = nm.lon * 1e-7;
    navData.lat = nm.lat * 1e-7;
    navData.height = nm.height * 1e-3;
    navData.hMSL = nm.hMSL * 1e-3;
    navData.hAcc = nm.hAcc * 1e-3;
    navData.vAcc = nm.vAcc * 1e-3;
  } else if (pubNavPosLLH.isCurrent(m.iTOW)) {
    const auto &nm = pubNavPosLLH.lastValue();
    navData.lon = nm.lon * 1e-7;
    navData.lat = nm.lat * 1e-7;
    navData.height = nm.height * 1e-3;
    navData.hMSL = nm.hMSL * 1e-3;
    navData.hAcc = nm.hAcc * 1e-3;
    navData.vAcc = nm.vAcc * 1e-3;
  } else ROS_WARN_THROTTLE(5, "No current position data!");

  if (pubNavDOP.isCurrent(m.iTOW)) {
    const auto &nm = pubNavDOP.lastValue();
    navData.nDOP = nm.nDOP * 0.01;
    navData.eDOP = nm.eDOP * 0.01;
    navData.vDOP * nm.vDOP * 0.01;
  } else if (pubNavPVT.isCurrent(m.iTOW)) {
    const auto &nm = pubNavPVT.lastValue();
    navData.nDOP = navData.eDOP = navData.vDOP = nm.pDOP * 0.01;
  } else if (pubNavSol.isCurrent(m.iTOW)) {
    const auto &nm = pubNavPVT.lastValue();
    navData.nDOP = navData.eDOP = navData.vDOP = nm.pDOP * 0.01;
  } else ROS_WARN_THROTTLE(5, "No current DOP data!");

  if (pubNavPVT.isCurrent(m.iTOW)) {
    const auto &nm = pubNavPVT.lastValue();
    navData.velN = nm.velN * 1e-3;
    navData.velE = nm.velE * 1e-3;
    navData.velD = nm.velD * 1e-3;
    navData.sAcc = nm.sAcc * 1e-3;
    navData.heading = nm.headMot * 1e-5;
    navData.headAcc = nm.headAcc * 1e-5;
    navData.gSpeed = nm.gSpeed * 1e-3;
  } else if (pubNavVelNED.isCurrent(m.iTOW)) {
    const auto &nm = pubNavVelNED.lastValue();
    navData.velN = nm.velN * 1e-2;
    navData.velE = nm.velE * 1e-2;
    navData.velD = nm.velD * 1e-2;
    navData.sAcc = nm.sAcc * 1e-2;
    navData.heading = nm.heading * 1e-5;
    navData.headAcc = nm.cAcc * 1e-5;
    navData.gSpeed = nm.gSpeed * 1e-2;
  } else ROS_WARN_THROTTLE(5, "No current velocity data!");
  
  if (pubNavPVT.isCurrent(m.iTOW)) 
    navData.numSV = pubNavPVT.lastValue().numSV;
  else if (pubNavSol.isCurrent(m.iTOW))
    navData.numSV = pubNavSol.lastValue().numSV;

  const auto &status = pubNavStatus.lastValue();

  // Position message
  static ros::Publisher fixPublisher =
      nh->advertise<sensor_msgs::NavSatFix>("fix", kROSQueueSize);
  fix.header.stamp = ros::Time::now();
  fix.header.frame_id = frame_id;
  fix.header.seq++;
  fix.latitude = navData.lat;
  fix.longitude = navData.lon;
  fix.altitude = navData.height;
  if (status.gpsFix < status.GPS_2D_FIX)
    fix.status.status = fix.status.STATUS_NO_FIX;
  else if (status.flags & status.FLAGS_DIFFSOLN)
    fix.status.status = fix.status.STATUS_SBAS_FIX; // TODO SBAS/GBAS
  else
    fix.status.status = fix.status.STATUS_FIX;
  fix.status.service = fix.status.SERVICE_GPS; // TODO

  // calculate covariance (convert from mm to m too)
  const double stdHa = (navData.hAcc / 1000.0) * 3.0;
  const double stdVa = (navData.vAcc / 1000.0) * 3.0;

  const double stdNp = navData.nDOP * uere;
  const double stdEp = navData.eDOP * uere;
  const double stdVp = navData.vDOP * uere;

  fix.position_covariance[0] = stdHa * stdHa + stdNp * stdNp;
  fix.position_covariance[4] = stdHa * stdHa + stdEp * stdEp;
  fix.position_covariance[8] = stdVa * stdVa + stdVp * stdVp;
  fix.position_covariance_type =
      sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

  fix.status.service = fix.status.SERVICE_GPS;
  fixPublisher.publish(fix);

  static ros::Publisher odometryPublisher =
      nh->advertise<nav_msgs::Odometry>("fix_odom", kROSQueueSize);
  odometry.header.stamp = ros::Time::now();
  odometry.header.frame_id = odom_frame_id;
  odometry.header.seq++;
  odometry.child_frame_id = frame_id;

  std::string utmzone;
  double northing, easting;
  NavsatConversions::LLtoUTM(navData.lat, navData.lon, northing, easting, utmzone);
  odometry.pose.pose.position.x = easting;
  odometry.pose.pose.position.y = northing;
  odometry.pose.pose.position.z = navData.height;

  tf2::Quaternion q;
  q.setRPY(0, 0, (90 - navData.heading) / 180 * M_PI);

  odometry.pose.pose.orientation.x = q.x();
  odometry.pose.pose.orientation.y = q.y();
  odometry.pose.pose.orientation.z = q.z();
  odometry.pose.pose.orientation.w = q.w();
  
  tf2::Transform rot(q);
  rot = rot.inverse();
  tf2::Vector3 vel(navData.velE, navData.velN, -navData.velD);
  vel = rot(vel);

  odometry.twist.twist.linear.x = vel.x();
  odometry.twist.twist.linear.y = vel.y();
  odometry.twist.twist.linear.z = vel.z();

  const double stdHead = navData.headAcc / 180 * M_PI 
    + M_PI/20/(navData.gSpeed+0.0001); // add uncertainty for low speed
  const double stdSpeed = navData.sAcc * 3;

  const int cols = 6;
  odometry.pose.covariance[cols * 0 + 0] = stdHa * stdHa + stdNp * stdNp;
  odometry.pose.covariance[cols * 1 + 1] = stdHa * stdHa + stdEp * stdEp;
  odometry.pose.covariance[cols * 2 + 2] = stdVa * stdVa + stdVp * stdVp;
  odometry.pose.covariance[cols * 3 + 3] = -1; // roll unsupported
  odometry.pose.covariance[cols * 4 + 4] = -1; // pitch unsupported
  odometry.pose.covariance[cols * 5 + 5] = stdHead * stdHead;
  odometry.twist.covariance[cols * 0 + 0] = stdSpeed * stdSpeed;
  odometry.twist.covariance[cols * 1 + 1] = stdSpeed * stdSpeed;
  odometry.twist.covariance[cols * 2 + 2] = stdSpeed * stdSpeed;
  odometry.twist.covariance[cols * 3 + 3] = -1;  //  angular rate unsupported
  odometry.twist.covariance[cols * 4 + 4] = -1;  //  angular rate unsupported
  odometry.twist.covariance[cols * 5 + 5] = -1;  //  angular rate unsupported

  odometryPublisher.publish(odometry);

  //  update diagnostics
  freq_diag->tick(fix.header.stamp);
  updater->update();
}

void pollMessages(const ros::TimerEvent& event) {
  static std::vector<uint8_t> payload(1, 1);
  if (enabled["aid_alm"]) {
    gps.poll(ublox_msgs::Class::AID, ublox_msgs::Message::AID::ALM, payload);
  }
  if (enabled["aid_eph"]) {
    gps.poll(ublox_msgs::Class::AID, ublox_msgs::Message::AID::EPH, payload);
  }
  if (enabled["aid_hui"]) {
    gps.poll(ublox_msgs::Class::AID, ublox_msgs::Message::AID::HUI);
  }
  payload[0]++;
  if (payload[0] > 32) {
    payload[0] = 1;
  }
}

void fix_diagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat) {
  //  check the last message, convert to diagnostic
  const ublox_msgs::NavSTATUS &status = pubNavStatus.lastValue();
  if (status.gpsFix == ublox_msgs::NavSTATUS::GPS_NO_FIX) {
    stat.level = diagnostic_msgs::DiagnosticStatus::ERROR;
    stat.message = "No fix";
  } else if (status.gpsFix == ublox_msgs::NavSTATUS::GPS_DEAD_RECKONING_ONLY) {
    stat.level = diagnostic_msgs::DiagnosticStatus::WARN;
    stat.message = "Dead reckoning only";
  } else if (status.gpsFix == ublox_msgs::NavSTATUS::GPS_2D_FIX) {
    stat.level = diagnostic_msgs::DiagnosticStatus::OK;
    stat.message = "2D fix";
  } else if (status.gpsFix == ublox_msgs::NavSTATUS::GPS_3D_FIX) {
    stat.level = diagnostic_msgs::DiagnosticStatus::OK;
    stat.message = "3D fix";
  } else if (status.gpsFix ==
             ublox_msgs::NavSTATUS::GPS_GPS_DEAD_RECKONING_COMBINED) {
    stat.level = diagnostic_msgs::DiagnosticStatus::OK;
    stat.message = "GPS and dead reckoning combined";
  } else if (status.gpsFix == ublox_msgs::NavSTATUS::GPS_TIME_ONLY_FIX) {
    stat.level = diagnostic_msgs::DiagnosticStatus::WARN;
    stat.message = "Time fix only";
  }

  //  append last fix position
  const ublox_msgs::NavPVT &pvt = pubNavPVT.lastValue();
  stat.add("iTOW", navData.iTOW);
  stat.add("lon", navData.lon);
  stat.add("lat", navData.lat);
  stat.add("height", navData.height);
  stat.add("hMSL", navData.hMSL);
  stat.add("hAcc", navData.hAcc);
  stat.add("vAcc", navData.vAcc);
  stat.add("numSV", navData.numSV);
}

void rtcmCallback(const rtcm_msgs::Message::ConstPtr &msg)
{
  gps.sendRtcm(msg->message);
}

void handleLog(uint8_t level, const ublox_msgs::Inf &msg)
{
  ros::console::Level severity;
  switch(level) {
    default:
    case ublox_msgs::Message::INF::DEBUG: severity = ros::console::levels::Debug; break;
    case ublox_msgs::Message::INF::ERROR: severity = ros::console::levels::Error; break;
    case ublox_msgs::Message::INF::NOTICE: severity = ros::console::levels::Info; break;
    case ublox_msgs::Message::INF::TEST: severity = ros::console::levels::Debug; break;
    case ublox_msgs::Message::INF::WARNING: severity = ros::console::levels::Warn; break;
  }
  ROS_LOG(severity, std::string(ROSCONSOLE_NAME_PREFIX) + ".ublox", msg.str.c_str());
}

int main(int argc, char** argv) {
  boost::asio::io_service io_service;
  ros::Timer poller;
  boost::shared_ptr<boost::asio::ip::tcp::socket> tcp_handle;
  boost::shared_ptr<boost::asio::serial_port> serial_handle;
  bool setup_ok = true;

  ros::init(argc, argv, "ublox_gps");
  nh.reset(new ros::NodeHandle("~"));
  if (!nh->hasParam("diagnostic_period")) {
    nh->setParam("diagnostic_period", 0.2);  //  5Hz diagnostic period
  }
  updater.reset(new diagnostic_updater::Updater());
  updater->setHardwareID("ublox");

  std::string port;
  int baudrate;
  int rate, meas_rate;
  bool enable_ppp, enable_sbas;
  bool init_reset, autobaud;
  std::map<ublox_msgs::CfgGNSS_Block::_gnssId_type, std::map<std::string, int> > gnss_enabled;
  XmlRpc::XmlRpcValue gnss_enabled_xmlrpc;
  std::string dynamic_model, fix_mode;
  int dr_limit;
  int ublox_version;
  ros::NodeHandle param_nh("~");
  param_nh.param("gnss_port", port, std::string("/dev/ttyACM0"));
  param_nh.param("gnss_frame_id", frame_id, std::string("gps"));
  param_nh.param("odom_frame_id", odom_frame_id, std::string("map"));
  param_nh.param("gnss_baudrate", baudrate, 9600);
  param_nh.param("gnss_aquire_rate", rate, 4);  //  in Hz
  param_nh.getParam("gnss_enabled", gnss_enabled_xmlrpc);
  param_nh.param("gnss_enable_sbas", enable_sbas, false);
  param_nh.param("gnss_enable_ppp", enable_ppp, false);
  param_nh.param("gnss_dynamic_model", dynamic_model, std::string("portable"));
  param_nh.param("gnss_fix_mode", fix_mode, std::string("both"));
  param_nh.param("gnss_dr_limit", dr_limit, 0);
  param_nh.param("gnss_ublox_version", ublox_version, 6);
  param_nh.param("gnss_uere", uere, 6.0);
  param_nh.param("init_reset", init_reset, false);
  param_nh.param("autobaud", autobaud, true);

  bool odo_enable, odo_lp_vel, odo_lp_cog, odo_lowspeed_cog;
  double odo_lp_vel_gain, odo_lp_cog_gain, odo_lowspeed_max_speed;
  int odo_lowspeed_max_pos_acc;
  param_nh.param("odo_enable", odo_enable, false);
  param_nh.param("odo_lp_vel", odo_lp_vel, false);
  param_nh.param("odo_lp_cog", odo_lp_cog, false);
  param_nh.param("odo_lowspeed_cog", odo_lowspeed_cog, false);
  param_nh.param("odo_lp_vel_gain", odo_lp_vel_gain, 0.0);
  param_nh.param("odo_lp_cog_gain", odo_lp_cog_gain, 0.0);
  param_nh.param("odo_lowspeed_max_speed", odo_lowspeed_max_speed, 0.0);
  param_nh.param("odo_lowspeed_max_pos_acc", odo_lowspeed_max_pos_acc, 0);

  std::string rtcm_topic;
  param_nh.param("rtcm_topic", rtcm_topic, std::string("rtcm"));
  subRTCM = nh->subscribe(rtcm_topic, 10, rtcmCallback);

  for (auto it : gnss_enabled_xmlrpc) {
    auto gnssId = ublox_msgs::gnssIdFromString(it.first);
    gnss_enabled[gnssId] = std::map<std::string, int>();
    for (auto kv : it.second) {
      gnss_enabled[gnssId][kv.first] = kv.second;
    }
  }

  if (enable_ppp) {
    ROS_WARN("Warning: PPP is enabled - this is an expert setting.");
  }

  if (rate <= 0) {
    ROS_ERROR("Invalid settings: rate must be > 0");
    return 1;
  }
  //  measurement rate param for ublox, units of ms
  meas_rate = 1000 / rate;

  if (dr_limit < 0 || dr_limit > 255) {
    ROS_ERROR("Invalid settings: dr_limit must be between 0 and 255");
    return 1;
  }

  DynamicModel dmodel;
  FixMode fmode;
  try {
    dmodel = ublox_gps::modelFromString(dynamic_model);
    fmode = ublox_gps::fixModeFromString(fix_mode);
  } catch (std::exception& e) {
    ROS_ERROR("Invalid settings: %s", e.what());
    return 1;
  }

  //  configure diagnostic updater for frequency
  updater->add("fix", &fix_diagnostic);
  updater->force_update();

  const double target_freq = 1000.0 / meas_rate;  //  actual update frequency
  double min_freq = target_freq;
  double max_freq = target_freq;
  diagnostic_updater::FrequencyStatusParam freq_param(&min_freq, &max_freq,
                                                      0.05, 10);
  diagnostic_updater::TimeStampStatusParam time_param(0,
                                                      meas_rate * 1e-3 * 0.05);
  freq_diag.reset(new diagnostic_updater::TopicDiagnostic(
      std::string("fix"), *updater, freq_param, time_param));

  boost::smatch match;
  if (boost::regex_match(port, match,
                         boost::regex("(tcp|udp)://(.+):(\\d+)"))) {
    std::string proto(match[1]);
    std::string host(match[2]);
    std::string port(match[3]);
    ROS_INFO("Connecting to %s://%s:%s ...", proto.c_str(), host.c_str(),
             port.c_str());

    if (proto == "tcp") {
      boost::asio::ip::tcp::resolver::iterator endpoint;

      try {
        boost::asio::ip::tcp::resolver resolver(io_service);
        endpoint =
            resolver.resolve(boost::asio::ip::tcp::resolver::query(host, port));
      } catch (std::runtime_error& e) {
        ROS_ERROR("Could not resolve %s:%s: %s", host.c_str(), port.c_str(),
                  e.what());
        return 1;  //  exit
      }

      boost::asio::ip::tcp::socket* socket =
          new boost::asio::ip::tcp::socket(io_service);
      tcp_handle.reset(socket);

      try {
        socket->connect(*endpoint);
      } catch (std::runtime_error& e) {
        ROS_ERROR("Could not connect to %s:%s: %s",
                  endpoint->host_name().c_str(),
                  endpoint->service_name().c_str(), e.what());
        return 1;  //  exit
      }

      ROS_INFO("Connected to %s:%s.", endpoint->host_name().c_str(),
               endpoint->service_name().c_str());
      gps.initialize(*socket, io_service);
    } else {
      ROS_ERROR("Protocol '%s' is unsupported", proto.c_str());
      return 1;  //  exit
    }
  } else {
    boost::asio::serial_port* serial = new boost::asio::serial_port(io_service);
    serial_handle.reset(serial);

    // open serial port
    try {
      serial->open(port);
    } catch (std::runtime_error& e) {
      ROS_ERROR("Could not open serial port %s: %s", port.c_str(), e.what());
      return 1;  //  exit
    }

    ROS_INFO("Opened serial port %s", port.c_str());
    gps.setBaudrate(baudrate);
    gps.initialize(*serial, io_service);
    if (autobaud && !gps.autobaud(*serial))
      throw std::runtime_error("Failed autobauding");
  }

  //  apply all requested settings
  try {
    if (init_reset) {
        gps.reset();
        ROS_INFO("Receiver reset issued");
    }
    if (!gps.isInitialized()) {
      throw std::runtime_error("Failed to initialize.");
    }
    if (!gps.setMeasRate(meas_rate)) {
      std::stringstream ss;
      ss << "Failed to set measurement rate to " << meas_rate << "ms.";
      throw std::runtime_error(ss.str());
    }
    if (!gps.enableSBAS(enable_sbas)) {
      throw std::runtime_error(std::string("Failed to ") +
                               ((enable_sbas) ? "enable" : "disable") +
                               " SBAS.");
    }
    if (!gps.setPPPEnabled(enable_ppp)) {
      throw std::runtime_error(std::string("Failed to ") +
                               ((enable_ppp) ? "enable" : "disable") + " PPP.");
    }
    if (!gps.setDynamicModel(dmodel)) {
      throw std::runtime_error("Failed to set model: " + dynamic_model + ".");
    }
    if (!gps.setFixMode(fmode)) {
      throw std::runtime_error("Failed to set fix mode: " + fix_mode + ".");
    }
    if (!gps.setDeadReckonLimit(dr_limit)) {
      std::stringstream ss;
      ss << "Failed to set dead reckoning limit: " << dr_limit << ".";
      throw std::runtime_error(ss.str());
    }

    ublox_msgs::CfgODO cfgODO;
    cfgODO.version = 0;
    cfgODO.cogMaxSpeed = odo_lowspeed_max_speed * 10;
    cfgODO.cogMaxPosAcc = odo_lowspeed_max_pos_acc;
    cfgODO.velLpGain = odo_lp_vel_gain * 255;
    cfgODO.cogLpGain = odo_lp_cog_gain * 255;
    cfgODO.flags = (odo_enable ? ublox_msgs::CfgODO::FLAGS_USE_ODO : 0)
                 | (odo_lowspeed_cog ? ublox_msgs::CfgODO::FLAGS_USE_COG : 0)
                 | (odo_lp_vel ? ublox_msgs::CfgODO::FLAGS_OUT_LP_VEL : 0)
                 | (odo_lp_cog ? ublox_msgs::CfgODO::FLAGS_OUT_LP_COG : 0);
    cfgODO.odoCfg = 0;

    if (!gps.configure(cfgODO)) {
      throw std::runtime_error("Failed to setup odometer");
    }

    for (int t = 0; t <= 4; t++) {
      gps.subscribeInf(t, boost::bind(handleLog, t, _1));
    }

    ublox_msgs::CfgINF cfgINF;
    cfgINF.block.resize(1);
    cfgINF.block[0].protocolID = 0; // TODO
    for (int x = 0; x < 6; x++) cfgINF.block[0].infMsgMask[x] = 0xff; // TODO

    if (!gps.configure(cfgINF)) {
      throw std::runtime_error("Failed to setup information messages");
    }

    if (ublox_version >= 7) {
      ublox_msgs::CfgGNSS cfgGNSS;
      if (gps.poll(cfgGNSS)) {
        ROS_INFO("Read GNSS config.");
        ROS_INFO("Num. tracking channels in hardware: %i", cfgGNSS.numTrkChHw);
        ROS_INFO("Num. tracking channels to use: %i", cfgGNSS.numTrkChUse);
        ROS_INFO("GNSS systems: %i", cfgGNSS.numConfigBlocks);
        for (auto block : cfgGNSS.block) {
            ROS_INFO("GNSS %i: %i reserved, %i maximum, %x", block.gnssId, block.resTrkCh, block.maxTrkCh, block.flags);
        }
      } else {
        throw std::runtime_error("Failed to read the GNSS config.");
      }

      for (auto &block : cfgGNSS.block) {
        auto m = gnss_enabled[block.gnssId];
        if (!m["sig"]) {
            block.flags = 0;
        } else {
            block.flags = 1 | (m["sig"] << 16);
            block.resTrkCh = m["min"];
            block.maxTrkCh = m["max"];
        }
      }

      if (!gps.configure(cfgGNSS)) {
        throw std::runtime_error("Failed to setup GNSS");
      }
    } else {
      ROS_WARN("ublox_version < 7, ignoring GNSS settings");
    }
  } catch (std::exception& e) {
    setup_ok = false;
    ROS_ERROR("Error configuring port: %s", e.what());
  }

  if (setup_ok) {
    ROS_INFO("U-Blox configured successfully.");

    // subscribe messages
    param_nh.param("all", enabled["all"], false);
    param_nh.param("rxm", enabled["rxm"], false);
    param_nh.param("nav", enabled["nav"], false);
    param_nh.param("nav_rtk", enabled["nav_rtk"], false);
    param_nh.param("aid", enabled["aid"], false);

    param_nh.param("nav_sol", enabled["nav_sol"], enabled["all"]);
    if (enabled["nav_sol"]) pubNavSol.subscribe();
    param_nh.param("nav_dop", enabled["nav_dop"], enabled["all"]);
    if (enabled["nav_dop"]) pubNavDOP.subscribe();
    param_nh.param("nav_pvt", enabled["nav_pvt"], enabled["all"]);
    if (enabled["nav_pvt"]) pubNavPVT.subscribe();
    param_nh.param("nav_sbas", enabled["nav_sbas"], enabled["all"]);
    if (enabled["nav_sbas"]) pubNavSBAS.subscribe(rate);
    param_nh.param("nav_dgps", enabled["nav_dgps"], enabled["all"]);
    if (enabled["nav_dgps"]) pubNavDGPS.subscribe(rate);
    param_nh.param("nav_status", enabled["nav_status"], enabled["all"]);
    if (enabled["nav_status"]) pubNavStatus.subscribe();
    param_nh.param("nav_sat", enabled["nav_sat"], enabled["all"]);
    if (enabled["nav_sat"]) pubNavSat.subscribe(rate);
    param_nh.param("nav_svinfo", enabled["nav_svinfo"], enabled["all"]);
    if (enabled["nav_svinfo"]) pubNavSVINFO.subscribe(rate);
    param_nh.param("nav_clk", enabled["nav_clk"], enabled["all"]);
    if (enabled["nav_clk"]) pubNavClock.subscribe();

    param_nh.param("nav_timegps", enabled["nav_timegps"], enabled["all"] || enabled["nav"]);
    if (enabled["nav_timegps"]) pubNavTimeGPS.subscribe();
    param_nh.param("nav_timeutc", enabled["nav_timeutc"], enabled["all"] || enabled["nav"]);
    if (enabled["nav_timeutc"]) pubNavTimeUTC.subscribe();

    param_nh.param("nav_posllh", enabled["nav_posllh"], enabled["all"]);
    if (enabled["nav_posllh"]) pubNavPosLLH.subscribe();
    param_nh.param("nav_velned", enabled["nav_velned"], enabled["all"]);
    if (enabled["nav_velned"]) pubNavVelNED.subscribe();
    param_nh.param("nav_orb", enabled["nav_orb"], enabled["all"]);
    if (enabled["nav_orb"]) pubNavORB.subscribe(rate);

    param_nh.param("nav_hpposllh", enabled["nav_hpposllh"], enabled["nav_rtk"]);
    if (enabled["nav_hpposllh"]) pubNavHPPosLLH.subscribe();
    param_nh.param("nav_svin", enabled["nav_svin"], enabled["nav_rtk"]);
    if (enabled["nav_svin"]) pubNavSVIN.subscribe();
    param_nh.param("nav_relposned", enabled["nav_relposned"], enabled["nav_rtk"]);
    if (enabled["nav_relposned"]) pubNavRelPosNED.subscribe();

    param_nh.param("rxm_raw", enabled["rxm_raw"], enabled["all"] || enabled["rxm"]);
    if (enabled["rxm_raw"]) pubRxmRAW.subscribe();
    param_nh.param("rxm_rawx", enabled["rxm_rawx"], enabled["all"] || enabled["rxm"]);
    if (enabled["rxm_rawx"]) pubRxmRAWX.subscribe();
    param_nh.param("rxm_sfrb", enabled["rxm_sfrb"], enabled["all"] || enabled["rxm"]);
    if (enabled["rxm_sfrb"]) pubRxmSFRB.subscribe();
    param_nh.param("rxm_sfrbx", enabled["rxm_sfrbx"], enabled["all"] || enabled["rxm"]);
    if (enabled["rxm_sfrbx"]) pubRxmSFRBX.subscribe();
    param_nh.param("rxm_svsi", enabled["rxm_svsi"], enabled["all"] || enabled["rxm"]);
    if (enabled["rxm_svsi"]) pubRxmSVSI.subscribe(rate);

    param_nh.param("aid_alm", enabled["aid_alm"], enabled["all"] || enabled["aid"]);
    if (enabled["aid_alm"]) pubAidALM.subscribe();
    param_nh.param("aid_eph", enabled["aid_eph"], enabled["all"] || enabled["aid"]);
    if (enabled["aid_eph"]) pubAidEPH.subscribe();
    param_nh.param("aid_hui", enabled["aid_hui"], enabled["all"] || enabled["aid"]);
    if (enabled["aid_hui"]) pubAidHUI.subscribe();

    gps.subscribe<ublox_msgs::NavEOE>(&endOfEpoch, 1);

    poller = nh->createTimer(ros::Duration(1.0), &pollMessages);
    poller.start();
    ros::spin();
  }

  if (gps.isInitialized()) {
    gps.close();
    ROS_INFO("Closed connection to %s.", port.c_str());
  }
  return 0;
}
