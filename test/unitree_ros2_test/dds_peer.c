// Copyright 2026 Yusuf Guenena
// SPDX-License-Identifier: BSD-3-Clause
/* Plain Cyclone DDS peer. Test topics never address a robot endpoint. */
#include <dds/dds.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "unitree_api/msg/dds_connext/Request_.h"
#include "unitree_api/msg/dds_connext/Response_.h"
#include "unitree_go/msg/dds_connext/WirelessController_.h"
#include "unitree_go/msg/dds_connext/LowState_.h"
#include "unitree_go/msg/dds_connext/SportModeState_.h"
#include "unitree_hg/msg/dds_connext/LowState_.h"

static dds_entity_t checked(dds_entity_t entity)
{
  if (entity < 0) {
    fprintf(stderr, "DDS: %s\n", dds_strretcode(-entity));
    exit(2);
  }
  return entity;
}

static dds_entity_t test_topic(
  dds_entity_t participant,
  const dds_topic_descriptor_t * descriptor,
  const char * suffix)
{
  const char * prefix = getenv("UNITREE_DDS_TEST_NAMESPACE");
  char name[256];
  int length = snprintf(
    name, sizeof(name), "rt/%s/%s",
    prefix ? prefix : "unitree_test", suffix);
  if (length < 0 || (size_t)length >= sizeof(name)) {exit(8);}
  return checked(dds_create_topic(participant, descriptor, name, NULL, NULL));
}

int main(void)
{
  const char * domain = getenv("ROS_DOMAIN_ID");
  dds_entity_t participant = checked(
    dds_create_participant(
      domain ? (dds_domainid_t)strtoul(domain, NULL, 10) : 173, NULL, NULL));
  dds_qos_t * qos = dds_create_qos();
  dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(2));
  dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 10);
  dds_data_representation_id_t representation = DDS_DATA_REPRESENTATION_XCDR1;
  dds_qset_data_representation(qos, 1, &representation);
  dds_entity_t request_topic = test_topic(
    participant,
    &unitree_api_msg_dds__Request__desc, "request");
  dds_entity_t response_topic = test_topic(
    participant,
    &unitree_api_msg_dds__Response__desc, "response");
  dds_entity_t wireless_topic = test_topic(
    participant,
    &unitree_go_msg_dds__WirelessController__desc, "wireless");
  dds_entity_t reader = checked(dds_create_reader(participant, request_topic, qos, NULL));
  dds_entity_t response_writer = checked(dds_create_writer(participant, response_topic, qos, NULL));
  dds_entity_t wireless_writer = checked(dds_create_writer(participant, wireless_topic, qos, NULL));
  dds_entity_t go_topic = test_topic(
    participant,
    &unitree_go_msg_dds__LowState__desc, "go_lowstate");
  dds_entity_t hg_topic = test_topic(
    participant,
    &unitree_hg_msg_dds__LowState__desc, "hg_lowstate");
  dds_entity_t sport_topic = test_topic(
    participant,
    &unitree_go_msg_dds__SportModeState__desc, "sport");
  dds_entity_t go_writer = checked(dds_create_writer(participant, go_topic, qos, NULL));
  dds_entity_t hg_writer = checked(dds_create_writer(participant, hg_topic, qos, NULL));
  dds_entity_t sport_writer = checked(dds_create_writer(participant, sport_topic, qos, NULL));
  dds_delete_qos(qos);
  unitree_go_msg_dds__LowState_ go_state = {0};
  go_state.head_[0] = 0xfe; go_state.head_[1] = 0xef;
  go_state.tick_ = UINT32_C(0xfedcba98);
  go_state.motor_state_[19].q_ = -1.25f;
  go_state.foot_force_[3] = -1234;
  go_state.wireless_remote_[39] = 0xa5;
  go_state.crc_ = UINT32_C(0x89abcdef);
  unitree_hg_msg_dds__LowState_ hg_state = {0};
  hg_state.tick_ = UINT32_C(0x12345678);
  hg_state.motor_state_[34].q_ = 2.5f;
  hg_state.motor_state_[34].temperature_[1] = -2345;
  hg_state.reserve_[3] = UINT32_C(0xdeadbeef);
  hg_state.crc_ = UINT32_C(0x76543210);
  unitree_go_msg_dds__SportModeState_ sport = {0};
  sport.stamp_.sec_ = 123456;
  sport.stamp_.nanosec_ = 789012345;
  sport.position_[2] = -3.5f;
  sport.foot_speed_body_[11] = 6.25f;
  unitree_go_msg_dds__WirelessController_ wireless = {0};
  wireless.lx_ = 0.25f; wireless.ly_ = -0.5f;
  wireless.rx_ = 0.75f; wireless.ry_ = 1.0f; wireless.keys_ = 0xabcd;
  int received = 0;
  for (int attempt = 0; attempt < 300; ++attempt) {
    if (dds_write(wireless_writer, &wireless) < 0 ||
      dds_write(go_writer, &go_state) < 0 ||
      dds_write(hg_writer, &hg_state) < 0 ||
      dds_write(sport_writer, &sport) < 0) {return 3;}
    void * samples[1] = {NULL};
    dds_sample_info_t info[1];
    int count = dds_take(reader, samples, info, 1, 1);
    if (count < 0) {return 4;}
    if (count && info[0].valid_data) {
      unitree_api_msg_dds__Request_ * request = samples[0];
      if (request->header_.identity_.id_ != INT64_C(0x1234567887654321) ||
        request->header_.identity_.api_id_ != 1001 ||
        request->header_.lease_.id_ != 42 ||
        request->header_.policy_.priority_ != -7 ||
        request->header_.policy_.noreply_ ||
        strcmp(request->parameter_, "{\"probe\":true}") ||
        request->binary_._length != 4 ||
        memcmp(request->binary_._buffer, "\x00\x7f\x80\xff", 4))
      {
        fprintf(stderr, "Request field mismatch\n");
        return 5;
      }
      uint8_t binary[] = {128, 255, 0, 127};
      unitree_api_msg_dds__Response_ response = {0};
      response.header_.identity_ = request->header_.identity_;
      response.header_.status_.code_ = -23;
      response.data_ = "{\"native_dds\":true}";
      response.binary_._buffer = binary;
      response.binary_._length = response.binary_._maximum = 4;
      if (dds_write(response_writer, &response) < 0) {return 6;}
      received = 1;
    }
    if (count) {dds_return_loan(reader, samples, count);}
    if (received) {break;}
    dds_sleepfor(DDS_MSECS(50));
  }
  if (received) {dds_sleepfor(DDS_MSECS(500));}
  dds_delete(participant);
  return received ? 0 : 7;
}
