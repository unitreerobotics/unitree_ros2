/**
 * This example demonstrates how to use ROS2 to call audio client api of unitree
 *g1 robot
 **/
#include <fstream>
#include <rclcpp/executors.hpp>
#include <thread>
#include <vector>

#include "common/time_tools.hpp"
#include "g1/g1_audio_client.hpp"

// wave reader start
struct WaveHeader {
  void SeekToDataChunk(std::istream &is) {
    while (is && subchunk2_id != 0x61746164) {
      is.seekg(subchunk2_size, std::istream::cur);
      is.read(reinterpret_cast<char *>(&subchunk2_id), sizeof(int32_t));
      is.read(reinterpret_cast<char *>(&subchunk2_size), sizeof(int32_t));
    }
  }

  int32_t chunk_id;
  int32_t chunk_size;
  int32_t format;
  int32_t subchunk1_id;
  int32_t subchunk1_size;
  int16_t audio_format;
  int16_t num_channels;
  int32_t sample_rate;
  int32_t byte_rate;
  int16_t block_align;
  int16_t bits_per_sample;
  int32_t subchunk2_id;    // a tag of this chunk
  int32_t subchunk2_size;  // size of subchunk2
};

static_assert(sizeof(WaveHeader) == 44, "WaveHeader Should be 44 bytes");

std::vector<uint8_t> ReadWaveImpl(std::istream &is, int32_t *sampling_rate,
                                  int8_t *channel_count, bool *is_ok) {
  WaveHeader header{};
  is.read(reinterpret_cast<char *>(&header.chunk_id), sizeof(header.chunk_id));

  //                        F F I R
  if (header.chunk_id != 0x46464952) {
    printf("Expected chunk_id RIFF. Given: 0x%08x\n", header.chunk_id);
    *is_ok = false;
    return {};
  }

  is.read(reinterpret_cast<char *>(&header.chunk_size),
          sizeof(header.chunk_size));

  is.read(reinterpret_cast<char *>(&header.format), sizeof(header.format));

  //                      E V A W
  if (header.format != 0x45564157) {
    printf("Expected format WAVE. Given: 0x%08x\n", header.format);
    *is_ok = false;
    return {};
  }

  is.read(reinterpret_cast<char *>(&header.subchunk1_id),
          sizeof(header.subchunk1_id));

  is.read(reinterpret_cast<char *>(&header.subchunk1_size),
          sizeof(header.subchunk1_size));

  if (header.subchunk1_id == 0x4b4e554a) {
    // skip junk padding
    is.seekg(header.subchunk1_size, std::istream::cur);

    is.read(reinterpret_cast<char *>(&header.subchunk1_id),
            sizeof(header.subchunk1_id));

    is.read(reinterpret_cast<char *>(&header.subchunk1_size),
            sizeof(header.subchunk1_size));
  }

  if (header.subchunk1_id != 0x20746d66) {
    printf("Expected subchunk1_id 0x20746d66. Given: 0x%08x\n",
           header.subchunk1_id);
    *is_ok = false;
    return {};
  }

  if (header.subchunk1_size != 16 &&
      header.subchunk1_size != 18) {  // 16 for PCM
    printf("Expected subchunk1_size 16. Given: %d\n", header.subchunk1_size);
    *is_ok = false;
    return {};
  }

  is.read(reinterpret_cast<char *>(&header.audio_format),
          sizeof(header.audio_format));

  if (header.audio_format != 1) {  // 1 for PCM
    printf("Expected audio_format 1. Given: %d\n", header.audio_format);
    *is_ok = false;
    return {};
  }

  is.read(reinterpret_cast<char *>(&header.num_channels),
          sizeof(header.num_channels));

  *channel_count = static_cast<int8_t>(header.num_channels);

  is.read(reinterpret_cast<char *>(&header.sample_rate),
          sizeof(header.sample_rate));

  is.read(reinterpret_cast<char *>(&header.byte_rate),
          sizeof(header.byte_rate));

  is.read(reinterpret_cast<char *>(&header.block_align),
          sizeof(header.block_align));

  is.read(reinterpret_cast<char *>(&header.bits_per_sample),
          sizeof(header.bits_per_sample));

  if (header.byte_rate !=
      (header.sample_rate * header.num_channels * header.bits_per_sample / 8)) {
    printf("Incorrect byte rate: %d. Expected: %d", header.byte_rate,
           (header.sample_rate * header.num_channels * header.bits_per_sample /
            8));
    *is_ok = false;
    return {};
  }

  if (header.block_align !=
      (header.num_channels * header.bits_per_sample / 8)) {
    printf("Incorrect block align: %d. Expected: %d\n", header.block_align,
           (header.num_channels * header.bits_per_sample / 8));
    *is_ok = false;
    return {};
  }

  if (header.bits_per_sample != 16) {  // we support only 16 bits per sample
    printf("Expected bits_per_sample 16. Given: %d\n", header.bits_per_sample);
    *is_ok = false;
    return {};
  }

  if (header.subchunk1_size == 18) {
    int16_t extra_size = -1;
    is.read(reinterpret_cast<char *>(&extra_size), sizeof(int16_t));
    if (extra_size != 0) {
      printf(
          "Extra size should be 0 for wave from NAudio. Current extra size "
          "%d\n",
          extra_size);
      *is_ok = false;
      return {};
    }
  }

  is.read(reinterpret_cast<char *>(&header.subchunk2_id),
          sizeof(header.subchunk2_id));

  is.read(reinterpret_cast<char *>(&header.subchunk2_size),
          sizeof(header.subchunk2_size));

  header.SeekToDataChunk(is);
  if (!is) {
    *is_ok = false;
    return {};
  }

  *sampling_rate = header.sample_rate;

  // header.subchunk2_size contains the number of bytes in the data.
  // As we assume each sample contains two bytes, so it is divided by 2 here
  std::vector<int16_t> samples(header.subchunk2_size / 2);

  is.read(reinterpret_cast<char *>(samples.data()), header.subchunk2_size);
  if (!is) {
    *is_ok = false;
    return {};
  }

  std::vector<uint8_t> ans(samples.size() * 2);
  for (int32_t i = 0; i != static_cast<int32_t>(samples.size()); ++i) {
    ans[i * 2] = samples[i] & 0xFF;
    ans[i * 2 + 1] = (samples[i] >> 8) & 0xFF;
  }

  *is_ok = true;
  return ans;
}

std::vector<uint8_t> ReadWave(const std::string &filename,
                              int32_t *sampling_rate, int8_t *channel_count,
                              bool *is_ok) {
  std::ifstream is(filename, std::ifstream::binary);
  auto samples = ReadWaveImpl(is, sampling_rate, channel_count, is_ok);
  return samples;
}
// wave reader end

int64_t GetCurrentTimeMillisecond() {
  auto now = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
      now.time_since_epoch());
  return duration.count();
}

auto Control(std::shared_ptr<unitree::ros2::g1::AudioClient> client,
             const std::string &audio_file_path) {
  /*TTS Example*/
  int32_t ret =
      client->TtsMaker("你好。我是宇树科技的机器人G1。例程启动成功", 0);
  std::cout << "TtsMaker ret:" << ret << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(10));

  /*Volume Example*/
  uint8_t volume = 0;
  ret = client->GetVolume(volume);
  std::cout << "GetVolume ret:" << ret
            << "  volume = " << std::to_string(volume) << std::endl;
  ret = client->SetVolume(100);
  std::cout << "SetVolume to 100% , ret:" << ret << std::endl;

  /*Audio Play Example*/
  int32_t sample_rate = -1;
  int8_t num_channels = 0;
  bool filestate = false;
  std::vector<uint8_t> pcm =
      ReadWave(audio_file_path, &sample_rate, &num_channels, &filestate);
  std::cout << "pcm data size : " << pcm.size() << std::endl;

  std::cout << "sample_rate = " << sample_rate
            << " num_channels =  " << std::to_string(num_channels)
            << " filestate =" << filestate << std::endl;

  if (filestate && sample_rate == 16000 && num_channels == 1) {
    ret = client->PlayStream(
        "example",
        std::to_string(unitree::common::GetCurrentTimeMilliseconds()), pcm);
    std::cout << "start play ,ret : " << ret << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(3));
    std::cout << "stop play" << std::endl;
    ret = client->PlayStop("example");
  } else {
    std::cout << "audio file format error, please check!" << std::endl;
  }

  /*LED Control Example*/
  std::cout << "led control start" << std::endl;
  client->LedControl(0, 255, 0);
  std::this_thread::sleep_for(std::chrono::seconds(2));
  client->LedControl(0, 0, 0);
  std::this_thread::sleep_for(std::chrono::seconds(2));
  client->LedControl(0, 0, 255);
  std::cout << "led control end" << std::endl;
  std::cout << "finish!" << std::endl;
}

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "Error: Audio file path not provided!\n"
              << "Usage: g1_audio_client_example audio_file_path" << std::endl;
    return 1;
  }
  std::string audio_file_path = argv[1];
  std::cout << "audio file path : " << audio_file_path << std::endl;

  try {
    rclcpp::init(argc, argv);
    auto client = std::make_shared<unitree::ros2::g1::AudioClient>();
    auto thread_ = std::thread([client, audio_file_path]() {
      using namespace std::chrono_literals;
      std::this_thread::sleep_for(1s);
      Control(client, audio_file_path);
    });
    rclcpp::spin(client);
    rclcpp::shutdown();
  } catch (const rclcpp::exceptions::RCLError &e) {
    std::cerr << "RCLError caught: " << e.what() << std::endl;
    return 1;
  }
  return 0;
}