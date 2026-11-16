#pragma once

#include "request_client.hpp"

// Preserve the public class, including downstream forward declarations.
class BaseClient : public unitree::ros2::RequestClient {
 public:
  using unitree::ros2::RequestClient::RequestClient;
};
