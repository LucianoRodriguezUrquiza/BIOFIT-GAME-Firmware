"""Compare complete control-state fingerprints and packets from actual .ino on identical inputs."""
from pathlib import Path
import subprocess
import sys

script = Path(__file__).with_name("run_replay.py")
def replay(*args):
    return subprocess.check_output([sys.executable, str(script), "--equivalence", *args], text=True)
reference = replay("--reference-v229")
candidate = replay()
if candidate != reference:
    raise AssertionError(f"V229:\n{reference}\nV230:\n{candidate}")
print("PASS: exact V229/profile0 event and state equivalence on deterministic replay")
print(candidate.strip())
