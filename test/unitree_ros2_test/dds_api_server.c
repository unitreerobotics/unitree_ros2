// Copyright 2026 Yusuf Guenena
// SPDX-License-Identifier: BSD-3-Clause
/* Plain Cyclone DDS Unitree API server with adversarial response timing.
 * Test topics never address a robot endpoint. Behaviour by request api_id:
 *   2001  reply at once, preceded by wrong-id and wrong-api decoys, then a duplicate
 *   2002  held until 2003 arrives; 2003 is answered first (out of order)
 *   2004  reply after 800 ms (the client times out at 300 ms)
 *   2005  decoy with 2005's id and api 2004 at once, real reply after 700 ms
 * UNITREE_DDS_API_SERVER_EXPECT (hex bit mask of api - 2001, default 1f) selects
 * which of these must be answered before a successful exit.
 * UNITREE_DDS_API_SERVER_REPLIES=N switches to echo mode: answer any request at
 * once, exit cleanly after N replies (used to restart the server under a client).
 */
#include <dds/dds.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "unitree_api/msg/dds_connext/Request_.h"
#include "unitree_api/msg/dds_connext/Response_.h"

typedef unitree_api_msg_dds__RequestIdentity_ identity_t;

static dds_entity_t checked(dds_entity_t entity)
{
  if (entity < 0) {
    fprintf(stderr, "DDS: %s\n", dds_strretcode(-entity));
    exit(2);
  }
  return entity;
}

static dds_entity_t test_topic(
  dds_entity_t participant, const dds_topic_descriptor_t * descriptor, const char * suffix)
{
  const char * prefix = getenv("UNITREE_DDS_TEST_NAMESPACE");
  char name[256];
  int length = snprintf(
    name, sizeof(name), "rt/%s/%s", prefix ? prefix : "unitree_test", suffix);
  if (length < 0 || (size_t)length >= sizeof(name)) {exit(8);}
  return checked(dds_create_topic(participant, descriptor, name, NULL, NULL));
}

static void respond(dds_entity_t writer, identity_t identity, int64_t echoed_api, int is_decoy)
{
  char data[96];
  snprintf(
    data, sizeof(data), "{\"api\":%" PRId64 ",\"decoy\":%s}",
    echoed_api, is_decoy ? "true" : "false");
  unitree_api_msg_dds__Response_ response;
  memset(&response, 0, sizeof(response));
  response.header_.identity_ = identity;
  response.data_ = data;
  if (dds_write(writer, &response) < 0) {exit(6);}
}

typedef struct {dds_time_t due; identity_t identity; int used;} delayed_t;

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
  dds_entity_t reader = checked(dds_create_reader(
      participant, test_topic(participant, &unitree_api_msg_dds__Request__desc, "request"),
      qos, NULL));
  dds_entity_t writer = checked(dds_create_writer(
      participant, test_topic(participant, &unitree_api_msg_dds__Response__desc, "response"),
      qos, NULL));
  dds_delete_qos(qos);

  delayed_t delayed[8];
  memset(delayed, 0, sizeof(delayed));
  identity_t held;
  int have_held = 0;
  unsigned done = 0;  /* bit (api - 2001) set once that api was answered */
  const char * expect = getenv("UNITREE_DDS_API_SERVER_EXPECT");
  const unsigned all = expect ? (unsigned)strtoul(expect, NULL, 16) : 0x1f;
  const char * replies_env = getenv("UNITREE_DDS_API_SERVER_REPLIES");
  const long echo_replies = replies_env ? strtol(replies_env, NULL, 10) : 0;
  long echoed = 0;
  const dds_time_t deadline = dds_time() + DDS_SECS(25);
  while (dds_time() < deadline &&
    (echo_replies ? echoed < echo_replies : (done != all || delayed[0].used || delayed[1].used)))
  {
    void * samples[1] = {NULL};
    dds_sample_info_t info[1];
    int count = dds_take(reader, samples, info, 1, 1);
    if (count < 0) {return 4;}
    if (count && info[0].valid_data) {
      const unitree_api_msg_dds__Request_ * request = samples[0];
      identity_t identity = request->header_.identity_;
      fprintf(stderr, "request api=%" PRId64 " id=%" PRId64 "\n", identity.api_id_, identity.id_);
      identity_t decoy = identity;
      if (echo_replies) {
        respond(writer, identity, identity.api_id_, 0);
        ++echoed;
        dds_return_loan(reader, samples, count);
        continue;
      }
      switch (identity.api_id_) {
        case 2001:
          decoy.id_ = identity.id_ + 1;
          respond(writer, decoy, identity.api_id_, 1);
          decoy = identity;
          decoy.api_id_ = identity.api_id_ + 1;
          respond(writer, decoy, decoy.api_id_, 1);
          respond(writer, identity, identity.api_id_, 0);
          respond(writer, identity, identity.api_id_, 0);
          done |= 1u << 0;
          break;
        case 2002:
          held = identity;
          have_held = 1;
          break;
        case 2003:
          respond(writer, identity, identity.api_id_, 0);
          if (have_held) {respond(writer, held, held.api_id_, 0); done |= 1u << 1;}
          done |= 1u << 2;
          break;
        case 2004:
          delayed[0] = (delayed_t){dds_time() + DDS_MSECS(800), identity, 1};
          break;
        case 2005:
          decoy.api_id_ = 2004;
          respond(writer, decoy, 2004, 1);
          delayed[1] = (delayed_t){dds_time() + DDS_MSECS(700), identity, 1};
          break;
        default:
          fprintf(stderr, "unexpected api_id\n");
          return 5;
      }
    }
    if (count) {dds_return_loan(reader, samples, count);}
    for (int i = 0; i < 2; ++i) {
      if (delayed[i].used && dds_time() >= delayed[i].due) {
        respond(writer, delayed[i].identity, delayed[i].identity.api_id_, 0);
        delayed[i].used = 0;
        done |= 1u << (unsigned)(delayed[i].identity.api_id_ - 2001);
      }
    }
    dds_sleepfor(DDS_MSECS(5));
  }
  dds_sleepfor(DDS_MSECS(500));
  dds_delete(participant);
  if (echo_replies) {return echoed == echo_replies ? 0 : 7;}
  return done == all ? 0 : 7;
}
