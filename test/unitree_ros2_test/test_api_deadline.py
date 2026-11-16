import os
import subprocess
import pytest

@pytest.mark.parametrize("client", ["UNITREE_API_DEADLINE", "UNITREE_B2_API_DEADLINE"])
@pytest.mark.parametrize("mode", ["missing-response", "expired"])
def test_no_publish_after_deadline(client, mode):
    subprocess.run([os.environ[client], mode], check=True, timeout=10)
