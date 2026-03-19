#ifndef TOFSENSEMINIT_H
#define TOFSENSEMINIT_H

#include "protocol_extracter/nprotocol_extracter.h"
#include <nlink_parser/TofsenseMFrame0.h>
#include <nlink_parser/TofsenseMCascade.h>
#include <ros/ros.h>
#include <serial/serial.h>

#include <array>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

namespace tofsensem {
class Init {
public:
  explicit Init(NProtocolExtracter *protocol_extraction,
                serial::Serial *serial = nullptr);

private:
  void InitFrame0(NProtocolExtracter *protocol_extraction);
  void PublishCascadeIfReady(NProtocolBase *protocol);
  bool ParseInquireIds(const std::string &raw, std::vector<uint8_t> *out) const;
  bool IsInquireTargetId(uint8_t id) const;

  std::unordered_map<NProtocolBase *, ros::Publisher> publishers_;
  std::map<int, nlink_parser::TofsenseMFrame0> frame0_map_;

  serial::Serial *serial_;
  bool round_published_ = false;

  const int frequency_ = 15;
  bool is_inquire_mode_ = false;
  std::vector<uint8_t> inquire_ids_;
  std::array<bool, 256> inquire_id_mask_{};
  double inquire_query_interval_sec_ = 0.010;

  ros::NodeHandle nh_;
  ros::Timer timer_scan_;
  ros::Timer timer_read_;
  size_t node_index_ = 0;
};

} // namespace tofsensem
#endif // TOFSENSEMINIT_H
