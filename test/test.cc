// OT128 RemakeConfig Test - Ring-Based with Sparse Ring Filling
// Tests ordered point cloud with RemakeConfig ENABLED using:
// - Ring-based vertical binning (128 bins, one per physical ring)
// - Sparse ring duplication (fills gaps in outer rings by duplicating to adjacent azimuth bins)

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

    // Calculate row/col from SDK grid layout (azimuth × ring storage)
    // SDK stores as: point_idx = azimuth * 128 + ring
    // Row = ring number (vertical: 0-127), Col = azimuth bin (horizontal: 0-3599)
    int col = i / 128;   // Azimuth bin (horizontal position)
    int row = i % 128;   // Ring number (vertical position, should equal pt.ring)

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
  printf("Return mode: 0x%02X ", frame.return_mode);
  if (frame.return_mode >= 0x39) {
    printf("(Dual/Multi return)\n");
  } else {
    printf("(Single return)\n");
  }

  // Display RemakeConfig mode
  if (frame.fParam.remake_config.use_ring_for_vertical) {
    printf("RemakeConfig: Ring-based vertical binning\n");
    printf("Vertical bins: %d (rings %d to %d)\n",
           frame.fParam.remake_config.vertical_bins,
           frame.fParam.remake_config.min_ring,
           frame.fParam.remake_config.max_ring);
  } else {
    printf("RemakeConfig: Angle-based vertical binning\n");
    printf("Vertical bins: %d (%.1f° to %.1f°)\n",
           frame.fParam.remake_config.max_elev_scan,
           frame.fParam.remake_config.min_elev,
           frame.fParam.remake_config.max_elev);
  }

  // Display sparse ring handling
  if (frame.fParam.remake_config.duplicate_sparse_rings) {
    printf("Sparse rings: Duplicating to adjacent azimuth bins\n");
    printf("Dense rings: %d to %d\n",
           frame.fParam.remake_config.dense_ring_start,
           frame.fParam.remake_config.dense_ring_end);
    printf("Sparse rings: 0-%d and %d-127\n",
           frame.fParam.remake_config.dense_ring_start - 1,
           frame.fParam.remake_config.dense_ring_end + 1);
  } else {
    printf("Sparse rings: No duplication\n");
  }

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

  // Expected grid for OT128 with ring-based mode: 128 x 3600 = 460,800
  int expected_grid_size = 128 * 3600;
  if (frame.points_num != expected_grid_size) {
    printf("⚠ WARNING: Expected %d points (128x3600 grid), got %u\n",
           expected_grid_size, frame.points_num);
  }

  float fill_rate = 100.0f * valid / frame.points_num;
  printf("Fill rate: %.1f%%\n", fill_rate);
  if (fill_rate < 50.0f) {
    printf("⚠ WARNING: Low fill rate - sparse grid!\n");
  }

  printf("=================================\n\n");
}

void lidarCallback(const LidarDecodedFrame<LidarPointXYZICRT>& frame) {
  cur_frame_time = GetMicroTickCount();
  if (last_frame_time == 0) last_frame_time = GetMicroTickCount();

  printf("%ld -> frame:%d points:%u packets:%u\n",
         GetMicroTimeU64(), frame.frame_index, frame.points_num, frame.packet_num);

  if (frame_counter < MAX_FRAMES_TO_EXPORT) {
    std::string csv = "ot128_remakeconfig_filled_frame_" + std::to_string(frame_counter) + ".csv";
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

  printf("\n=== OT128 RemakeConfig Test (Ring-Based + Sparse Fill) ===\n");
  printf("Expected: 128 x 3600 grid (460,800 points)\n");
  printf("Grid layout: 128 rows (rings) x 3600 cols (azimuth bins)\n");
  printf("Using ring-based vertical binning with sparse ring duplication\n");
  printf("Sparse rings (0-23, 88-127) duplicated to adjacent azimuth bins\n\n");

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

  // Use OT128 defaults from udp1_4_parser.cc (ring-based vertical binning)
  // The SDK will automatically apply:
  //   - Horizontal: 3600 bins (0-360°, 0.1° resolution)
  //   - Vertical: 128 bins (ring-based, rings 0-127)
  //   - use_ring_for_vertical = true
  // No explicit config needed - defaults are correct!
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
