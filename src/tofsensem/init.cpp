#include "init.h"

#include "nlink_protocol.h"
#include "nlink_unpack/nlink_tofsensem_frame0.h"
#include "nlink_unpack/nlink_utils.h"
#include "nutils.h"

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

  InitFrame0(protocol_extraction);
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
      frame0_map_[data.id] = g_msg_tofmframe0;
    } else {
      publishers_.at(protocol).publish(g_msg_tofmframe0);
    }
  });

  if (is_inquire_mode_) {
    timer_scan_ = nh_.createTimer(
        ros::Duration(1.0 / frequency_),
        [=](const ros::TimerEvent &) {
          frame0_map_.clear();
          node_index_ = 0;
          timer_read_.start();
        },
        false, true);
    timer_read_ = nh_.createTimer(
        ros::Duration(0.006),
        [=](const ros::TimerEvent &) {
          if (node_index_ >= 6) {
            if (!frame0_map_.empty()) {
              nlink_parser::TofsenseMCascade msg_cascade;
              for (const auto &msg : frame0_map_) {
                msg_cascade.nodes.push_back(msg.second);
              }
              publishers_.at(protocol).publish(msg_cascade);
            }
            timer_read_.stop();
          } else {
            g_command_read.id = node_index_;
            auto data = reinterpret_cast<uint8_t *>(&g_command_read);
            NLink_UpdateCheckSum(data, sizeof(g_command_read));
            serial_->write(data, sizeof(g_command_read));
            ++node_index_;
          }
        },
        false, false);
  }
}

} // namespace tofsensem
