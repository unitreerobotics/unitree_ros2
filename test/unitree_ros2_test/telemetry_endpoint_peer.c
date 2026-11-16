// Copyright 2026 Yusuf Guenena
// SPDX-License-Identifier: BSD-3-Clause
/* Plain DDS endpoints with firmware-like topic names, for the telemetry
 * preflight tests. It creates writers and never writes a sample. It refuses to
 * start outside a private domain on the loopback interface. */
#include <dds/dds.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "unitree_go/msg/dds_connext/LowCmd_.h"
#include "unitree_go/msg/dds_connext/LowState_.h"
#include "unitree_go/msg/dds_connext/SportModeState_.h"
#include "unitree_go/msg/dds_connext/WirelessController_.h"

static volatile sig_atomic_t running = 1;

static void stop(int signal_number)
{
  (void)signal_number;
  running = 0;
}

static dds_entity_t checked(dds_entity_t entity)
{
  if (entity < 0) {
    fprintf(stderr, "DDS: %s\n", dds_strretcode(-entity));
    exit(2);
  }
  return entity;
}

static void writer(
  dds_entity_t participant, const dds_topic_descriptor_t * descriptor, const char * name)
{
  checked(dds_create_writer(
      participant, checked(dds_create_topic(participant, descriptor, name, NULL, NULL)),
      NULL, NULL));
}

int main(int argc, char ** argv)
{
  const char * domain = getenv("ROS_DOMAIN_ID");
  const char * uri = getenv("CYCLONEDDS_URI");
  if (!domain || atoi(domain) < 100 || !uri || !strstr(uri, "<NetworkInterface name=\"lo\"")) {
    fprintf(stderr, "Refusing to start outside a private loopback domain\n");
    return 3;
  }
  signal(SIGTERM, stop);
  signal(SIGINT, stop);
  dds_entity_t participant = checked(
    dds_create_participant((dds_domainid_t)strtoul(domain, NULL, 10), NULL, NULL));
  writer(participant, &unitree_go_msg_dds__LowState__desc, "rt/lowstate");
  writer(participant, &unitree_go_msg_dds__SportModeState__desc, "rt/lf/sportmodestate");
  writer(participant, &unitree_go_msg_dds__WirelessController__desc, "rt/wirelesscontroller");
  if (argc > 1 && strcmp(argv[1], "with-lowcmd") == 0) {
    writer(participant, &unitree_go_msg_dds__LowCmd__desc, "rt/lowcmd");
  }
  printf("ready\n");
  fflush(stdout);
  while (running) {
    dds_sleepfor(DDS_MSECS(20));
  }
  dds_delete(participant);
  return 0;
}
