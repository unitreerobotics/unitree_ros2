"""Source setup scripts in clean subprocesses, including error cases."""
import os
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_prefix
import pytest


ROOT = Path(__file__).resolve().parents[2]
WORKSPACE = Path(get_package_prefix('unitree_go')).parents[1]


def run(command, workspace=True, **extra):
    env = {key: os.environ[key] for key in ('HOME', 'PATH', 'LANG') if key in os.environ}
    if workspace:
        env['UNITREE_ROS_WS'] = str(WORKSPACE)
    env.update(**extra)
    return subprocess.run(['bash', '--noprofile', '--norc', '-c', command],
                          env=env, text=True, capture_output=True)


def test_local_source_from_other_directory():
    result = run('cd /; source "' + str(ROOT / 'setup_local.sh') +
                 '" ' + os.environ['ROS_DISTRO'] +
                 '; test "$RMW_IMPLEMENTATION" = rmw_cyclonedds_cpp && '
                 'test "$ROS_DOMAIN_ID" = 173 && [[ "$CYCLONEDDS_URI" == *127.0.0.1* ]] && '
                 '[[ "$CYCLONEDDS_URI" == *"<ParticipantIndex>auto</ParticipantIndex>"* ]]')
    assert result.returncode == 0, result.stderr


def test_local_setup_allows_more_than_ten_processes():
    # Unicast discovery with automatic participant indexes only reaches indexes up to
    # MaxAutoParticipantIndex, which Cyclone DDS defaults to 9: the eleventh process on
    # the host cannot create a node. Jazzy's RMW raises it for localhost discovery;
    # Humble's RMW uses the setup script's configuration as given.
    count = 12
    probe = ('import sys, time\n'
             'import rclpy\n'
             'rclpy.init()\n'
             'node = rclpy.create_node("unitree_setup_probe_" + sys.argv[1])\n'
             'print("created", flush=True)\n'
             'time.sleep(6.0)\n')
    result = run('source "' + str(ROOT / 'setup_local.sh') + '" ' + os.environ['ROS_DISTRO'] +
                 ' >/dev/null || exit 9; for i in $(seq ' + str(count) + '); do '
                 'python3 -c "$UNITREE_PROBE" "$i" & done; wait', UNITREE_PROBE=probe)
    assert result.stdout.count('created') == count, result.stdout + result.stderr[-2000:]


def test_reject_mixed_distribution():
    result = run('source "' + str(ROOT / 'setup_local.sh') + '" jazzy', ROS_DISTRO='humble')
    assert result.returncode != 0
    assert 'different ROS distribution' in result.stderr


def test_reject_missing_interface():
    result = run('source "' + str(ROOT / 'setup.sh') + '" missing-interface ' +
                 os.environ['ROS_DISTRO'])
    assert result.returncode != 0
    assert 'existing robot-facing interface' in result.stderr


def test_default_clears_stale_dds_configuration():
    result = run('source "' + str(ROOT / 'setup_default.sh') + '" ' +
                 os.environ['ROS_DISTRO'] + ' && test -z "$CYCLONEDDS_URI"',
                 CYCLONEDDS_URI='file:///nonexistent')
    assert result.returncode == 0, result.stderr


def installed_distributions():
    return sorted(path.parent.name for path in Path('/opt/ros').glob('*/setup.bash'))


def test_argument_free_network_setup_matches_legacy_usage():
    # The legacy README sourced setup.sh with no ROS environment and no arguments.
    # The interface now comes from UNITREE_NETWORK_INTERFACE, the distribution
    # from the single installed one.
    result = run('source "' + str(ROOT / 'setup.sh') + '"; '
                 'test "$ROS_DISTRO" = ' + os.environ['ROS_DISTRO'] + ' && '
                 'test "$RMW_IMPLEMENTATION" = rmw_cyclonedds_cpp && '
                 '[[ "$CYCLONEDDS_URI" == *\'NetworkInterface name="lo"\'* ]] && '
                 'ros2 pkg prefix unitree_go >/dev/null',
                 UNITREE_NETWORK_INTERFACE='lo')
    if len(installed_distributions()) == 1:
        assert result.returncode == 0, result.stdout + result.stderr
    else:
        assert result.returncode != 0
        assert 'Installed distributions' in result.stderr


def test_interface_argument_overrides_variable():
    result = run('source "' + str(ROOT / 'setup.sh') + '" lo ' + os.environ['ROS_DISTRO'] +
                 ' && [[ "$CYCLONEDDS_URI" == *\'name="lo"\'* ]]',
                 UNITREE_NETWORK_INTERFACE='missing-interface')
    assert result.returncode == 0, result.stdout + result.stderr


def test_repository_root_workspace_without_override():
    # docs/jazzy.md builds one workspace at the repository root.
    if WORKSPACE != ROOT or (ROOT / 'cyclonedds_ws/install/local_setup.bash').exists():
        pytest.skip('test package was not built as a repository-root workspace')
    result = run('source "' + str(ROOT / 'setup_local.sh') + '" ' + os.environ['ROS_DISTRO'] +
                 ' && ros2 pkg prefix unitree_ros2_example', workspace=False)
    assert result.returncode == 0, result.stdout + result.stderr
    assert result.stdout.strip().endswith(str(ROOT / 'install/unitree_ros2_example'))
