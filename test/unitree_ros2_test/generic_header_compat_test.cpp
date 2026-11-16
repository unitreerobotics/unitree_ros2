// Copyright 2026 Yusuf Guenena
// SPDX-License-Identifier: BSD-3-Clause
#include <type_traits>
class BaseClient;
#include "common/base_client.hpp"
#include "g1/g1_loco_client.hpp"

template<typename T>
constexpr bool value_semantics = std::is_copy_constructible<T>::value &&
  std::is_move_constructible<T>::value && std::is_copy_assignable<T>::value &&
  std::is_move_assignable<T>::value;

static_assert(value_semantics<BaseClient>, "Preserve generic client value semantics");
static_assert(value_semantics<unitree::robot::g1::LocoClient>,
              "Preserve G1 client value semantics");
static_assert(UT_ROBOT_TASK_TIMEOUT == -1, "Preserve the published timeout value");
static_assert(UT_ROBOT_TASK_NOT_PUBLISHED == 3102, "Match the SDK2 send-request error");
int main() {return 0;}
