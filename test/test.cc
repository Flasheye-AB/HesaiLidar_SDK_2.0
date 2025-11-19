// OT128 RemakeConfig Test - Working Version
// Tests ordered point cloud with RemakeConfig ENABLED

#include "hesai_lidar_sdk.hpp"
#include <fstream>
#include <iomanip>
#include <map>
#include <cfloat>
#include <cmath>

#define PCAP_PARSER_TEST

uint32_t last_frame_time = 0;
uint32_t cur_frame_time = 0;
bool running = true;
int frame_counter = 0;
const int MAX_FRAMES_TO_EXPORT = 10;

// CSV export for organized point cloud (includes row/col)
void exportFrameToCSV(const LidarDecodedFrame<LidarPointXYZICRT>& frame,
                      const std::string& filename) {
  std::ofstream file(filename);
  if (!file.is_open()) {
    printf("ERROR: Could not open %s\n", filename.c_str());
    return;
  }

  // Header includes row,col for grid position
  file << "point_id,row,col,x,y,z,intensity,timestamp,ring,distance\n";

  for (uint32_t i = 0; i < frame.points_num; i++) {
    const auto& pt = frame.points[i];
    float dist = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);

    // Calculate row/col if we had grid info (we don't in SDK output)
    // For now, just use sequential indexing
    int row = i / 3600;  // Approximate grid position
    int col = i % 3600;

    file << i << "," << row << "," << col << ","
         << std::fixed << std::setprecision(3) << pt.x << ","
         << pt.y << "," << pt.z << ","
         << (int)pt.intensity << ","
         << std::setprecision(9) << pt.timestamp << ","
         << pt.ring << ","
         << std::setprecision(3) << dist << "\n";
  }

  file.close();
  printf("✓ Exported %u points to %s\n", frame.points_num, filename.c_str());
}

// Analyze frame with RemakeConfig
void analyzeFrame(const LidarDecodedFrame<LidarPointXYZICRT>& frame, int frame_num) {
  printf("\n=== OT128 RemakeConfig Frame %d ===\n", frame_num);
  printf("Points: %u\n", frame.points_num);
  printf("Packets: %u\n", frame.packet_num);

  std::map<uint16_t, int> ring_counts;
  float min_dist = FLT_MAX, max_dist = 0;
  int zero_dist = 0, valid = 0;

  for (uint32_t i = 0; i < frame.points_num; i++) {
    const auto& pt = frame.points[i];
    ring_counts[pt.ring]++;

    float dist = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
    if (dist < 0.001) {
      zero_dist++;
    } else {
      valid++;
      if (dist < min_dist) min_dist = dist;
      if (dist > max_dist) max_dist = dist;
    }
  }

  printf("Rings: %zu (ID %u to %u)\n", ring_counts.size(),
         ring_counts.begin()->first, ring_counts.rbegin()->first);
  printf("Range: %.2f - %.2f m\n", min_dist, max_dist);
  printf("Valid: %d (%.1f%%), Zero-dist: %d\n",
         valid, 100.0f*valid/frame.points_num, zero_dist);

  // Expected grid for OT128: 3600 x 320 = 1,152,000
  int expected_grid_size = 3600 * 320;
  if (frame.points_num != expected_grid_size) {
    printf("⚠ WARNING: Expected %d points (3600x320 grid), got %u\n",
           expected_grid_size, frame.points_num);
  }

  float fill_rate = 100.0f * valid / frame.points_num;
  if (fill_rate < 60.0f) {
    printf("⚠ WARNING: Low fill rate (%.1f%%) - sparse grid!\n", fill_rate);
  }

  printf("=================================\n\n");
}

void lidarCallback(const LidarDecodedFrame<LidarPointXYZICRT>& frame) {
  cur_frame_time = GetMicroTickCount();
  if (last_frame_time == 0) last_frame_time = GetMicroTickCount();

  printf("%ld -> frame:%d points:%u packets:%u\n",
         GetMicroTimeU64(), frame.frame_index, frame.points_num, frame.packet_num);

  if (frame_counter < MAX_FRAMES_TO_EXPORT) {
    std::string csv = "ot128_remakeconfig_frame_" + std::to_string(frame_counter) + ".csv";
    exportFrameToCSV(frame, csv);
    analyzeFrame(frame, frame_counter);
  }

  frame_counter++;
  last_frame_time = cur_frame_time;
}

void faultMessageCallback(const FaultMessageInfo& fault_message_info) {
  return;
}

bool IsPlayEnded(HesaiLidarSdk<LidarPointXYZICRT>& sdk) {
  return sdk.lidar_ptr_->IsPlayEnded();
}

int main(int argc, char *argv[]) {
#ifndef _MSC_VER
  // Not needed for PCAP playback - only for live sensor UDP reception
  // system("sudo sh -c \"echo 562144000 > /proc/sys/net/core/rmem_max\"");
#endif

  printf("\n=== OT128 RemakeConfig Test ===\n");
  printf("Expected: 3600 x 320 grid (1,152,000 points)\n\n");

  HesaiLidarSdk<LidarPointXYZICRT> sample;
  DriverParam param;

  param.use_gpu = (argc > 1);

#ifdef PCAP_PARSER_TEST
  param.input_param.source_type = DATA_FROM_PCAP;
  param.input_param.pcap_path = "/home/jf/Work/Flasheye/Hesai/repo/OT128/Large Crossroad Turning - 360° View.pcap";
  param.input_param.correction_file_path = "/home/jf/Work/Flasheye/Hesai/repo/OT128/OT128_Angle-Correction-File.csv";
  param.input_param.firetimes_path = "/home/jf/Work/Flasheye/Hesai/repo/OT128/OT128_Firetime-Correction-File.csv";

  param.decoder_param.pcap_play_synchronization = true;
  param.decoder_param.play_rate_ = 1.0;
  param.decoder_param.pcap_play_in_loop = false;

  // ENABLE RemakeConfig for ordered point cloud
  param.decoder_param.remake_config.flag = true;

  // OT128 default configuration (from udp1_4_parser.cc)
  param.decoder_param.remake_config.min_azi = 0.0f;
  param.decoder_param.remake_config.max_azi = 360.0f;
  param.decoder_param.remake_config.ring_azi_resolution = 0.1f;
  param.decoder_param.remake_config.max_azi_scan = 3600;      // Width

  param.decoder_param.remake_config.min_elev = -25.0f;
  param.decoder_param.remake_config.max_elev = 15.0f;
  param.decoder_param.remake_config.ring_elev_resolution = 0.125f;
  param.decoder_param.remake_config.max_elev_scan = 320;      // Height
#endif

  param.decoder_param.enable_packet_loss_tool = false;
  param.decoder_param.socket_buffer_size = 262144000;

  sample.Init(param);
  sample.RegRecvCallback(lidarCallback);
  sample.RegRecvCallback(faultMessageCallback);
  sample.Start();

  if (sample.lidar_ptr_->GetInitFinish(FailInit)) {
    sample.Stop();
    return -1;
  }

  while (!IsPlayEnded(sample) || GetMicroTickCount() - last_frame_time < 1000000) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  printf("\n=== Test Complete: %d frames ===\n\n", frame_counter);
  return 0;
}
