#include "init.h"

#include "nlink_protocol.h"
#include "nlink_unpack/nlink_tofsensem_frame0.h"
#include "nlink_unpack/nlink_utils.h"
#include "nutils.h"

#include <algorithm>
#include <cctype>
#include <exception>
#include <sstream>

namespace {
class ProtocolFrame0 : public NLinkProtocolVLength {
public:
  ProtocolFrame0()
      : NLinkProtocolVLength(
            true, g_ntsm_frame0.fixed_part_size,
            {g_ntsm_frame0.frame_header, g_ntsm_frame0.function_mark}) {}

protected:
  bool UpdateLength(const uint8_t *data, size_t available_bytes) override {
    if (available_bytes < g_ntsm_frame0.fixed_part_size)
      return false;
    return set_length(tofm_frame0_size(data));
  }

  void UnpackFrameData(const uint8_t *data) override {
    g_ntsm_frame0.UnpackData(data, length());
  }
};

#pragma pack(push, 1)
struct {
  char header[2]{0x57, 0x10};
  uint8_t reserved0[2]{0xff, 0xff};
  uint8_t id{};
  uint8_t reserved1[2]{0xff, 0xff};
  uint8_t checkSum{};
} g_command_read;
#pragma pack(pop)
} // namespace

namespace tofsensem {
nlink_parser::TofsenseMFrame0 g_msg_tofmframe0;

Init::Init(NProtocolExtracter *protocol_extraction, serial::Serial *serial)
    : serial_(serial) {
  is_inquire_mode_ =
      serial_ ? ros::param::param<bool>("~inquire_mode", false) : false;
  inquire_query_interval_sec_ =
      ros::param::param<double>("~inquire_query_interval_sec", 0.010);

  if (is_inquire_mode_) {
    std::string raw_ids =
        ros::param::param<std::string>("~inquire_ids", "0,1,2,3,4,5");
    if (!ParseInquireIds(raw_ids, &inquire_ids_) || inquire_ids_.empty()) {
      ROS_WARN("tofsensem inquire_ids invalid: '%s', disable inquire_mode",
               raw_ids.c_str());
      is_inquire_mode_ = false;
    } else {
      for (const auto id : inquire_ids_) {
        inquire_id_mask_[id] = true;
      }
      ROS_INFO("tofsensem inquire_mode: ids='%s', query_interval=%.3fs, freq=%dHz",
               raw_ids.c_str(), std::max(0.001, inquire_query_interval_sec_),
               frequency_);
    }
  }

  InitFrame0(protocol_extraction);
}

bool Init::ParseInquireIds(const std::string &raw, std::vector<uint8_t> *out) const {
  if (!out) {
    return false;
  }
  out->clear();

  std::array<bool, 256> seen{};
  std::stringstream ss(raw);
  std::string token;
  while (std::getline(ss, token, ',')) {
    token.erase(std::remove_if(token.begin(), token.end(),
                               [](unsigned char c) { return std::isspace(c); }),
                token.end());
    if (token.empty()) {
      continue;
    }

    int value = -1;
    try {
      value = std::stoi(token);
    } catch (const std::exception &) {
      return false;
    }

    if (value < 0 || value > 255) {
      return false;
    }

    const auto index = static_cast<size_t>(value);
    if (seen[index]) {
      continue;
    }
    seen[index] = true;
    out->push_back(static_cast<uint8_t>(value));
  }

  return !out->empty();
}

bool Init::IsInquireTargetId(uint8_t id) const {
  return inquire_id_mask_[id];
}

void Init::PublishCascadeIfReady(NProtocolBase *protocol) {
  if (round_published_) {
    return;
  }

  for (const auto id : inquire_ids_) {
    if (frame0_map_.find(id) == frame0_map_.end()) {
      return;
    }
  }

  nlink_parser::TofsenseMCascade msg_cascade;
  for (const auto id : inquire_ids_) {
    msg_cascade.nodes.push_back(frame0_map_.at(id));
  }
  publishers_.at(protocol).publish(msg_cascade);
  round_published_ = true;
}

void Init::InitFrame0(NProtocolExtracter *protocol_extraction) {
  static auto protocol = new ProtocolFrame0;
  protocol_extraction->AddProtocol(protocol);
  protocol->SetHandleDataCallback([=] {
    if (!publishers_[protocol]) {
      ros::NodeHandle nh_;
      if (is_inquire_mode_) {
        auto topic = "nlink_tofsensem_cascade";
        publishers_[protocol] =
            nh_.advertise<nlink_parser::TofsenseMCascade>(topic, 50);
        TopicAdvertisedTip(topic);
      } else {
        auto topic = "nlink_tofsensem_frame0";
        publishers_[protocol] =
            nh_.advertise<nlink_parser::TofsenseMFrame0>(topic, 50);
        TopicAdvertisedTip(topic);
      }
    }

    const auto &data = g_ntsm_frame0;
    g_msg_tofmframe0.id = data.id;
    g_msg_tofmframe0.system_time = data.system_time;
    g_msg_tofmframe0.pixel_count = data.pixel_count;
    g_msg_tofmframe0.pixels.resize(data.pixel_count);
    for (int i = 0; i < data.pixel_count; ++i) {
      const auto &src_pixel = data.pixels[i];
      auto &pixel = g_msg_tofmframe0.pixels[i];
      pixel.dis = src_pixel.dis;
      pixel.dis_status = src_pixel.dis_status;
      pixel.signal_strength = src_pixel.signal_strength;
    }

    if (is_inquire_mode_) {
      const bool is_target = IsInquireTargetId(data.id);
      if (is_target) {
        frame0_map_[data.id] = g_msg_tofmframe0;
      }
      ROS_INFO_THROTTLE(1.0,
                        "rx id=%u, target=%d, got=%zu/%zu",
                        data.id,
                        is_target ? 1 : 0,
                        frame0_map_.size(),
                        inquire_ids_.size());
      if (is_target) {
        PublishCascadeIfReady(protocol);
      }
    } else {
      publishers_.at(protocol).publish(g_msg_tofmframe0);
    }
  });

  if (is_inquire_mode_) {
    timer_scan_ = nh_.createTimer(
        ros::Duration(1.0 / frequency_),
        [=](const ros::TimerEvent &) {
          timer_read_.stop();
          frame0_map_.clear();
          node_index_ = 0;
          round_published_ = false;
          timer_read_.start();
        },
        false, true);

    const auto query_interval =
        ros::Duration(std::max(0.001, inquire_query_interval_sec_));
    timer_read_ = nh_.createTimer(
        query_interval,
        [=](const ros::TimerEvent &) {
          if (!serial_) {
            timer_read_.stop();
            return;
          }

          if (node_index_ >= inquire_ids_.size()) {
            timer_read_.stop();
            return;
          }

          g_command_read.id = inquire_ids_[node_index_];
          auto raw = reinterpret_cast<uint8_t *>(&g_command_read);
          NLink_UpdateCheckSum(raw, sizeof(g_command_read));
          serial_->write(raw, sizeof(g_command_read));
          ++node_index_;
        },
        false, false);
  }
}

} // namespace tofsensem
