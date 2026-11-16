"""B2 client correlation and full-string JSON parsing through a software peer."""
import os
import subprocess

import pytest


@pytest.mark.parametrize('mode', ['success', 'status', 'timeout', 'malformed',
                                  'empty', 'embedded-null', 'escaped-null',
                                  'status-only', 'status-only-malformed',
                                  'external-executor', 'recreate', 'shared-node',
                                  'rapid-lifecycle'])
def test_b2_api_client(mode):
    result = subprocess.run([os.environ['UNITREE_B2_API_CLIENT_TEST'], mode],
                            capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stdout + result.stderr
