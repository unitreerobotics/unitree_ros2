# Native ROS 2 Jazzy

Ubuntu 24.04 with ROS 2 Jazzy builds the official `unitree_go`, `unitree_hg`
and `unitree_api` interfaces, examples and regression tests directly. A Jazzy
application uses `rmw_cyclonedds_cpp` and native DDS topics to communicate with
Unitree firmware. No Humble installation, bridge or SDK wrapper is required.
The message definitions and Unitree request/response protocol are unchanged.
This repository defines messages; it currently defines no ROS services or actions.

## Install and build

Install ROS 2 Jazzy using the [official Ubuntu installation instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
Start a new terminal with no other ROS distribution or custom DDS overlay sourced.
Then install the development tools and native middleware:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep \
  ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-rosidl-generator-dds-idl
source /opt/ros/jazzy/setup.bash

git clone https://github.com/unitreerobotics/unitree_ros2.git
cd unitree_ros2
```

Initialize rosdep once per machine with `sudo rosdep init` if it has not already
been initialized, then resolve all declared dependencies and build:

```bash
rosdep update --rosdistro jazzy
rosdep install --from-paths cyclonedds_ws/src example/src test \
  --ignore-src --rosdistro jazzy -y
colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/local_setup.bash
```

The commands build one workspace at the repository root, including every example
and the test package. A separate interface workspace under `cyclonedds_ws` remains
usable; select its install when sourcing the helper scripts.

Jazzy supplies Cyclone DDS 0.10.5 with its matching `rmw_cyclonedds_cpp` binary
package. Use that matched pair. Firmware using Cyclone DDS 0.10.2 does not require
replacing Jazzy's system library with 0.10.2. DDS interoperability and local
library ABI compatibility are different requirements. Do not overlay a different
Cyclone DDS library beneath the packaged Jazzy RMW implementation, and do not
source Humble into a Jazzy shell. If apt reports the ROSIDL package unavailable, check the configured ROS 2 Noble
repository and run `sudo apt update`; install the package through apt in the
selected environment. Lack of permission to install a package is a separate
issue from package availability. Use a disposable container when host package
installation is restricted; do not extract ROS packages into a custom overlay.

The `rosidl_generator_dds_idl` dependency emits
DDS IDL used by the plain DDS regression peer; it does not select the Connext RMW.

## Offline validation

For the repository-root workspace, select its install explicitly:

```bash
export UNITREE_ROS_WS="$PWD"
source setup_local.sh jazzy
colcon test --packages-select unitree_ros2_test --executor sequential \
  --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --test-result-base build/unitree_ros2_test --all --verbose
```

`setup_local.sh` selects Cyclone DDS, loopback networking, unicast discovery and
domain 173 unless `ROS_DOMAIN_ID` is already set. It explicitly selects automatic
DDS participant indexes so unicast discovery also works with Humble middleware, and allows
indexes up to 32 (33 ROS processes on the host, as Jazzy does for localhost discovery);
Cyclone DDS otherwise stops at 10. It does not discover a robot
interface. The automatic tests use telemetry or private test topics. They do not
publish motion commands or change a robot control mode.

The tests cover generated interface serialization, ROS publishers/subscribers,
a plain Cyclone DDS peer without ROS client libraries, API request/response
correlation, setup scripts and subscriber example startup. Inspect the generated
JUnit results and colcon logs for the checks actually executed. A passing offline
suite establishes software behavior under the tested conditions, not firmware or
hardware qualification.

External recordings may be validated without adding datasets to Git. After
sourcing `setup_local.sh` as above, pass copied or read-only recordings explicitly:

```bash
python3 test/unitree_ros2_test/check_bags.py /path/to/recording \
  --ros --report /path/to/results/bag-regression.json
```

Omit `--ros` for deserialization and field checks without creating ROS nodes.
With `--ros`, only representative telemetry samples are published, on remapped
`/unitree_regression/telemetry/` topics using the loopback configuration. This
checks subscriber processing; it is not a timed replay of the entire recording.
The utility inventories all topics and reports missing message definitions.
Missing required Unitree telemetry definitions fail validation. It compares all
known fields through independent CDR decoding and a ROS serialization roundtrip,
records timestamp/payload digests and checks that the original file is unchanged.
Report output must be outside source recording directories and must not alias an
input recording, including through a symlink or hardlink.
Observed firmware serialization trailers are reported explicitly; unexplained
trailing payloads fail validation. Recordings may also contain robot commands,
which this utility does not publish. Recorded-data tests do not establish native
firmware communication.

For timed playback through the actual `ros2 bag play` executable, use a SQLite3
recording directory with `metadata.yaml`:

```bash
python3 test/unitree_ros2_test/replay_bags.py /path/to/recording \
  --rate 0.25 --report /path/to/results/timed-replay.json
```

Keep the report outside the source recording. This verifier requires loopback
Cyclone DDS, unicast discovery and domain 173. It explicitly whitelists supported
telemetry types and remaps every selected topic under a unique
`/unitree_regression/replay/` namespace. It compares every received message with
the source in order, including embedded stamps and sequence fields, requires
exact counts and matching semantic digests, and verifies unchanged source file
hashes. Player errors fail the check even when the player exits successfully.
No command topic or `/clock` is replayed. MCAP timed playback is not implemented;
the offline field checker can read MCAP when its storage plugin is installed.

To compare decoding between distributions, run the same read-only recording in
each environment with a different report path, then compare each topic's
`source_records_sha256`, `fields_sha256`, counts and errors:

```bash
python3 test/unitree_ros2_test/compare_bags.py /path/to/recording.db3 \
  --standard-type sensor_msgs/msg/Imu \
  --standard-type nav_msgs/msg/Odometry --report /path/to/results/jazzy.json
```

## Cyclone DDS version interoperability

The normal tests use the distribution's Cyclone DDS. To test a separate plain
DDS 0.10.2 process against the installed Jazzy RMW, use the optional pinned-source
test after selecting the loopback setup above:

```bash
scripts/test_cyclone_0102.sh /tmp/unitree-cyclone-0102
```

The scratch directory contains the source, build, private install and JUnit
result. Only the peer process loads DDS 0.10.2. The ROS process continues to load
Jazzy's DDS library. This exercises XCDR1 telemetry, nested GO and HG states,
sport timestamps and API request/response envelopes without robot hardware.
It also runs the generic and B2 API clients against a plain DDS API server
(`dds_api_server.c`) that sends decoys, duplicates, out-of-order and late
responses and is restarted under a running client.
The Jazzy x86_64 CI job also runs this test.

API discovery on a slow link can outlast a client's startup. To model that,
`scripts/test_api_latency.sh 250ms` reruns the API server tests with added
loopback latency. It changes the `lo` qdisc for the whole host, so it refuses to
run unless `UNITREE_ALLOW_LO_NETEM=1` is set: use a container or disposable VM.

## Robot-facing configuration

Choose the Ethernet interface connected to the robot explicitly. Configure its
address according to the robot model's official network documentation. Start a
clean shell, source Jazzy and select the workspace:

```bash
source /opt/ros/jazzy/setup.bash
cd unitree_ros2
export UNITREE_ROS_WS="$PWD"
source setup.sh YOUR_ETHERNET_INTERFACE jazzy
```

Replace `YOUR_ETHERNET_INTERFACE` with the actual interface name. The helper
rejects an absent or invalid interface and does not guess one. Set
`ROS_DOMAIN_ID` to the domain required by the firmware before starting nodes.
Local regression domain 173 is separate from the firmware domain. The script
selects `rmw_cyclonedds_cpp` and binds Cyclone DDS to the chosen interface.
`setup_default.sh jazzy` instead allows Cyclone DDS to choose its default
interface, which is useful only when that selection is appropriate.

The telemetry examples subscribe with sensor-data (best-effort) QoS, which
matches both reliable and best-effort publishers; the tests cover both. A recorded
GO2 session offers `/lowstate` and `/sportmodestate` as reliable. Evidence for
`/wirelesscontroller` conflicts: one recording shows a reliable publisher, while a
live GO2 observation found it best-effort, which a reliable subscription would not
match. Confirm publisher QoS on the robot with `ros2 topic info -v`. API request/response topics retain their existing Unitree message
schemas and topic conventions. Verify QoS and the firmware's topic names for the
specific robot. Restart a previously running ROS CLI daemon after changing RMW,
domain or DDS configuration so discovery reflects the current environment.

Start with read-only telemetry under the robot's documented safety procedure.
Control examples can actuate hardware. Native firmware interoperability and every
control function require separate hardware qualification before production use.

## Telemetry-only GO2 session

Only `read_low_state`, `read_motion_state` and `read_wireless_controller` may run
in this session. Each subscribes to one telemetry topic and logs it. Like every
`rclcpp` node, each also has `/rosout` and `/parameter_events` publishers and the
standard parameter services; none has a Unitree request or command endpoint. Do
not run `go2_robot_state_client` here: it calls `SetReportFreq` and toggles
`sport_mode` through `ServiceSwitch`.

Follow the robot's documented safety procedure, with the robot at rest and an
operator present. The commands below connect to the robot network as a DDS
participant, so run them only in a supervised session. From the repository root
of a built workspace, in a clean shell:

```bash
source /opt/ros/jazzy/setup.bash
export UNITREE_ROS_WS="$PWD"
source setup.sh ROBOT_INTERFACE jazzy
export ROS_DOMAIN_ID=0   # The domain your firmware uses.
mkdir -p go2-telemetry
python3 scripts/go2_telemetry_preflight.py --domain 0 --interface ROBOT_INTERFACE \
  --report go2-telemetry/preflight-before.json
```

Continue only after `PREFLIGHT PASS` (exit status 0). The check starts DDS only
after its local checks pass, then observes the graph for a bounded time. It fails if an
example is not approved, the environment is not the expected one, a binary refers
to another message type or a message publisher, a telemetry topic has no plain-DDS
publisher of the expected type, or a command topic has a writer. It uses a unique
private node with no writer and does not use the ROS daemon. The report records
binary digests and the discovered publishers with their QoS, including
`/wirelesscontroller`.

Firmware endpoints have no ROS node name, so a plain-DDS writer on a command topic
cannot be attributed from the graph. It may be a robot process. By default it fails
the check. A supervisor who has confirmed that nothing else is attached to the
robot network can accept it with `--allow-nonros-writer TOPIC=COUNT`, for the exact
topic and count listed in the report. A different count still fails, and a writer
belonging to a ROS node is never accepted.

Run the examples for a fixed time, then repeat the check:

```bash
bin="$(ros2 pkg prefix unitree_ros2_example)/lib/unitree_ros2_example"
for example in read_low_state read_motion_state read_wireless_controller; do
  timeout --preserve-status -s INT 15 "$bin/$example" > "go2-telemetry/$example.log" 2>&1
  echo "$example exit=$?"
done
python3 scripts/go2_telemetry_preflight.py --domain 0 --interface ROBOT_INTERFACE \
  --baseline go2-telemetry/preflight-before.json \
  --report go2-telemetry/preflight-after.json
```

Pass the same `--allow-nonros-writer` arguments as before. With `--baseline`,
the report separates command writers that appeared since the first check, each
attributed to an approved example node, another ROS node, a firmware participant
of the baseline or an unidentified participant, from acknowledged writers that
disappeared. Any new writer fails with exit status 1, even when the per-topic
count still matches. Exit status 3 means that acknowledged writers were only
withdrawn, which is firmware graph churn, and it remains a failure to review.
The examples' own processes have exited by the time this check runs, so run a
third check with `--monitor 60 --baseline go2-telemetry/preflight-before.json`
alongside them to record any writer they create. For a host that reaches the
robot through the payload computer, see [go2-routed-telemetry.md](go2-routed-telemetry.md).

The check describes the graph at the time it ran. It cannot prove what a robot
process does with a topic, and any DDS participant, including this one, sends
discovery traffic on the domain.

## Validation matrix and limits

| Function | Automated coverage | Evidence boundary |
| --- | --- | --- |
| `unitree_go`, `unitree_hg`, `unitree_api` | Build and generated-interface serialization tests | Does not qualify every firmware release |
| Existing examples and clients | Build all targets; start subscriber examples | Motion and mode-changing examples are not executed |
| ROS publisher/subscriber transport | Isolated Cyclone DDS tests | ROS-level communication on loopback |
| Native DDS and API envelopes | Plain DDS peer generated from official IDL | Software peer interoperability, not a robot |
| External ROS bags | Explicitly supplied offline Unitree recordings | Dataset-specific field checks and recorded schema incompatibilities |
| Jazzy x86_64 | Native Ubuntu 24.04 CI job | Hosted runners only; no robot deployment is tested |
| Jazzy ARM64 | Native `ubuntu-24.04-arm` CI job | No emulation; no ARM robot deployment is tested |
| Humble x86_64 | Native Ubuntu 22.04 regression job | Uses Humble's own middleware and dependencies |
| Actual robot firmware and control | Separate hardware validation required | Not established by CI or bag replay |

The [native ROS 2 workflow](../.github/workflows/native-ros2.yml) resolves
per-distribution dependencies with rosdep, builds interfaces and examples, gates
the functional regression package, verifies all 24 installed examples and checks
for foreign ROS underlays in CMake caches. It also runs the existing package lint
checks in a separate, nonblocking step and uploads their logs and JUnit artifacts
even when checks fail.
The workflow runs for pull requests, pushes to `master`, and manual dispatch.
A push to a development branch alone does not trigger it.

The functional gate includes the B2 executor ownership and status-only response
regressions and synthetic tests that reject the incompatible recorded-video
layout without changing the official interfaces. On Galactic and later, B2 clients spin only their
response callback groups; applications retain responsibility for spinning other
callbacks on the caller's node. On Foxy, which lacks the callback-group executor
API, a B2 client spins the whole node, as upstream did, and the caller must not
spin that node itself. Their private executor uses bounded waits and an
independent stop flag, so destroying a client immediately after construction
cannot lose a cancellation issued before the worker starts.

Both API clients publish a request only after the request reader and response
writer are matched, within the call's timeout: requests and responses are
volatile, so a message sent before matching is lost. A call whose deadline passes
first returns `UT_ROBOT_TASK_NOT_PUBLISHED` without publishing. Each client keeps one
response subscription and correlates responses by request id and API id; late
responses for timed-out calls, duplicates and other clients' responses are
dropped. The generic client's response reader keeps the latest 1000 samples, which
bounds what is retained while the node is not being spun. Request ids are unique
within the process. Calls time out after five
seconds by default; `SetTimeout()` changes this per client. Calls are not retried
automatically, because API requests (including motion commands) are not
idempotent.

Each client has one call in flight. Concurrent callers wait for it, but only until
their own deadline, which starts when `Call()` is entered: a caller that cannot
start in time gets `UT_ROBOT_TASK_NOT_PUBLISHED` and nothing is published. Waiting
callers are not served in a guaranteed order, so under sustained contention one
caller can time out while others proceed. Use one client per independent caller,
or one call at a time per client, when every call must start.
Generic client copies share the writer, response state, timeout and single-flight
coordination. `SetTimeout()` on one copied handle updates all of those handles.

`UT_ROBOT_TASK_NOT_PUBLISHED` (3102, the official SDK2 send-request error) means
the client never handed the request to DDS, because the deadline passed before
the call could start or before the server was matched. `UT_ROBOT_TASK_TIMEOUT`
(-1) means the request was handed to DDS and no matching response arrived in
time. Treat that as an unknown outcome: the server may have acted on it, may still
receive it, and its response may be lost or late. Code that compared only against
`UT_ROBOT_TASK_TIMEOUT` when no server is running must also handle the new value.
`UT_ROBOT_TASK_TIMEOUT` keeps this package's value; the SDK2 timeout is 3104.

Request writers keep their existing history depth: one sample for the generic
clients, so a later call replaces an unacknowledged request, and ten for the B2
API client and the GO2, B2 and robot-state query and configuration paths, so
unacknowledged requests stay queued in order. All of them have a five-second
lifespan, which only limits how long the writer retains a sample for
retransmission. It is not a delivery or execution deadline: it cannot retract a
packet already delayed in the network or a request the server already received,
it does not follow `SetTimeout()`, and a timeout does not cancel remote execution.
In fault-injection runs against a separate Cyclone DDS 0.10.2 peer, a call with a
one-second timeout returned `UT_ROBOT_TASK_TIMEOUT` and the peer still received
the request after a two-second outage, and after six seconds of network delay.

`SportClient` motion methods use one writer with a depth of one and the same
lifespan, favouring the latest command. These settings reduce stale delivery but
cannot guarantee that an old command is never executed. With a depth of ten, a
three-second outage replayed ten queued commands in order on recovery; with a
depth of one only the latest was retransmitted. The cost is that under packet loss
a command followed closely by another can be replaced before it is delivered, so
space out one-shot actions and confirm the result where that matters. At 20%
simulated loss, 151 of 200 commands sent 20 ms apart arrived (189 with a depth of
ten) and 38 of 40 sent 200 ms apart arrived (40 with a depth of ten). The latest
command arrived in every run. Low-level command topics such as `/lowcmd` keep
their existing publishers and are not affected by these settings.

Matching is observed locally only: on a high-latency link the server may not yet
have matched the client's endpoints, so the first exchange right after discovery
can still be lost and that call times out.

Single-flight calls have no priority or cancellation mechanism. A stop or damp
request can wait behind an unanswered query on the same client. Do not use this
queue as an emergency-stop channel. The prepublication deadline check prevents
sending when expiry is observed, but cannot impose a hard deadline on process
scheduling, serialization or a blocking middleware publication.

Create API clients before their node is spun and destroy them after spinning
has stopped, as the examples do. Jazzy's `rcl` keeps an unlocked per-node
type-description cache: creating endpoints on a node while an executor thread
destroys endpoints of the same type on it can corrupt that cache and crash the
process. Humble has no such cache.

### Serialized input limitation

The packaged Cyclone RMW does not provide complete validation of arbitrary CDR
buffers. Direct deserialization of undersized buffers can terminate a process.
Separate-process loopback experiments also show that publishing truncated
serialized `WirelessController` buffers can deliver zero-valued messages to a
typed subscriber in both Jazzy and Humble. This is reachable through serialized
publisher APIs, even though the bounded experiments did not crash either process.
Typed application callbacks cannot recover the original serialized length.
Successful discovery or callback delivery therefore does not prove payload
validity. These observations do not qualify arbitrary malformed RTPS traffic.

The supplied recording validators and replay verifier check encapsulation,
fields and trailers before invoking RMW. Keep that preflight when using external
serialized recordings. Ordinary telemetry examples use typed subscriptions, and
the recorder serializes typed messages; they do not accept arbitrary buffers
through an application-level deserialization API. Fixing validation inside the
packaged middleware remains outside this repository's message definitions.

The native workflow does not replace the existing Foxy workflow. Foxy compatibility remains a
separate legacy validation target; do not infer it from a Jazzy or Humble run.

To run all existing checks locally, including example lint, use:

```bash
colcon test --executor sequential --event-handlers console_direct+ \
  --return-code-on-test-failure
colcon test-result --all --verbose
```

The full test command retains the existing example lint checks.
The upstream example sources currently have copyright and style violations,
and the bundled-source formatter can exceed its test timeout. These failures
remain visible in CI as a separate nonblocking result. A passing functional CI
gate does not mean the full lint suite is green. This preserves the existing
checks without making unrelated upstream formatting debt a functional gate.
The functional tests in unitree_ros2_test report their results separately.
