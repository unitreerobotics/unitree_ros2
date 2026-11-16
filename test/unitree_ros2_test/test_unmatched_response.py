"""API calls never publish after their deadline when no response writer matched."""
import os
import subprocess

import pytest


@pytest.mark.parametrize('variable', ['UNITREE_UNMATCHED_RESPONSE_TEST',
                                      'UNITREE_B2_UNMATCHED_RESPONSE_TEST'])
def test_call_without_response_writer_is_not_published(variable):
    result = subprocess.run([os.environ[variable]], capture_output=True, text=True, timeout=20)
    assert result.returncode == 0, result.stdout + result.stderr
