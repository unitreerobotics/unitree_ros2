"""Client correlation, duplicate responses, status codes and timeout."""
import os
import subprocess

import pytest


@pytest.mark.parametrize('mode', ['success', 'status', 'timeout', 'malformed',
                                  'empty-json', 'embedded-null', 'escaped-null',
                                  'status-only', 'unique-ids'])
def test_api_client(mode):
    result = subprocess.run([os.environ['UNITREE_API_CLIENT_TEST'], mode],
                            capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stdout + result.stderr
