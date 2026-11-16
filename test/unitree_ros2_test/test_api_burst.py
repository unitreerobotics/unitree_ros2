"""An occupied executor must not lose correlated API replies from a response burst."""
import os
import subprocess


def test_response_burst():
    result = subprocess.run([os.environ['UNITREE_API_BURST_TEST']],
                            capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stdout + result.stderr
