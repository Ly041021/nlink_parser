#ifndef TOFSENSEMINIT_H
#define TOFSENSEMINIT_H

#include "protocol_extracter/nprotocol_extracter.h"
#include <nlink_parser/TofsenseMFrame0.h>
#include <nlink_parser/TofsenseMCascade.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <map>
#include <unordered_map>

namespace tofsensem {
class Init {
public:
  explicit Init(NProtocolExtracter *protocol_extraction,
                serial::Serial *serial = nullptr);

private:
  void InitFrame0(NProtocolExtracter *protocol_extraction);

  std::unordered_map<NProtocolBase *, ros::Publisher> publishers_;
  std::map<int, nlink_parser::TofsenseMFrame0> frame0_map_;

  serial::Serial *serial_;

  const int frequency_ = 15;
  bool is_inquire_mode_ = false;

  ros::NodeHandle nh_;
  ros::Timer timer_scan_;
  ros::Timer timer_read_;
  uint8_t node_index_ = 0;
};

} // namespace tofsensem
#endif // TOFSENSEMINIT_H
