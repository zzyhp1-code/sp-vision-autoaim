// Linux PTY only. No hardware access and no calls to XucSender::send().
#include "io/xuc.hpp"
#include "tools/crc.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>
#include <pty.h>
#include <unistd.h>

int main()
{
  using namespace std::chrono;
  int master = -1, slave = -1;
  char port[128]{};
  assert(openpty(&master, &slave, port, nullptr, nullptr) == 0);
  assert(std::string(port).rfind("/dev/pts/", 0) == 0);
  char config_name[] = "/tmp/xuc-packet-test-XXXXXX";
  const int config_fd = mkstemp(config_name);
  assert(config_fd >= 0);
  close(config_fd);
  {
    std::ofstream config(config_name);
    config << "xuc_enable: true\nxuc_serial_port: " << port
           << "\nxuc_serial_baud: 460800\nxuc_allow_control: false\n"
              "xuc_allow_shoot: false\nxuc_imu_angle_unit: rad\n";
  }
  {
    io::XucSender xuc(config_name);
    assert(xuc.enabled());
    const auto packet = [](int count) {
      io::XucTxPacket p{};
      p.head[0] = 'S'; p.head[1] = 'P'; p.mode = 1; p.robot_id = 3;
      p.bullet_speed = 19.8f; p.bullet_count = count;
      p.imu_pitch = .1f; p.imu_yaw = .2f;
      p.crc16 = tools::get_crc16(reinterpret_cast<const uint8_t *>(&p), offsetof(io::XucTxPacket, crc16));
      return p;
    };
    const auto wait_for = [&](uint64_t count, int expected_count = -1) {
      const auto end = steady_clock::now() + milliseconds(500);
      while ((xuc.valid_packets() < count || !xuc.rx_valid() ||
              (expected_count >= 0 && xuc.bullet_count() != expected_count)) && steady_clock::now() < end)
        std::this_thread::sleep_for(microseconds(100));
      assert(xuc.valid_packets() == count);
      assert(xuc.rx_valid() && (expected_count < 0 || xuc.bullet_count() == expected_count));
    };
    // Feed the next packet shortly after each completed read. The old 64-byte
    // request waits for the rest of its 10ms timeout on these whole packets.
    std::vector<double> latencies;
    for (int i = 1; i <= 25; ++i) {
      std::this_thread::sleep_for(milliseconds(2));
      const auto p = packet(i);
      const auto start = steady_clock::now();
      assert(write(master, &p, sizeof(p)) == sizeof(p));
      wait_for(i);
      latencies.push_back(duration<double, std::milli>(steady_clock::now() - start).count());
    }
    std::sort(latencies.begin(), latencies.end());
    const double median = latencies[latencies.size() / 2];
    assert(median < 5.0);
    // Noise + bad CRC + fragmented good frame exercise actual parser recovery.
    auto bad = packet(26); bad.crc16 ^= 1;
    const uint8_t noise[] = {0xff, 'S', 0, 0x33, 0x7f};
    assert(write(master, noise, sizeof(noise)) == sizeof(noise));
    assert(write(master, &bad, sizeof(bad)) == sizeof(bad));
    const auto p = packet(27);
    const auto * bytes = reinterpret_cast<const uint8_t *>(&p);
    assert(write(master, bytes, 7) == 7);
    std::this_thread::sleep_for(milliseconds(15));
    assert(xuc.valid_packets() == 25);
    assert(write(master, bytes + 7, sizeof(p) - 7) == sizeof(p) - 7);
    wait_for(26, 27);
    assert(xuc.crc_errors() >= 1 && xuc.bullet_count() == 27 && xuc.rx_valid());
    // A burst is still parsed as individual 20-byte frames.
    const auto a = packet(28), b = packet(29);
    uint8_t burst[2 * sizeof(a)];
    std::memcpy(burst, &a, sizeof(a)); std::memcpy(burst + sizeof(a), &b, sizeof(b));
    assert(write(master, burst, sizeof(burst)) == sizeof(burst));
    wait_for(28, 29);
    assert(xuc.bullet_count() == 29);
    std::cout << "xuc_packet_timing_test: PASS median_ms=" << median
              << " packets=" << xuc.valid_packets() << " crc_errors=" << xuc.crc_errors() << '\n';
  }
  close(master); close(slave);
  std::filesystem::remove(config_name);
}
