"""Own ROS subprocess groups without searching or killing unrelated processes."""

import os
import signal
import subprocess
import time


class OwnedBackend:
    """One launch session, including children created by ros2 launch/run."""

    def __init__(self):
        """Start without an owned process; external processes are never adopted."""
        self.process = None

    def start(self, args, log_path):
        """Create a separate process group and retain its exact identity."""
        if self.process is not None:
            raise RuntimeError("Stop the previous backend before starting another")
        with open(os.path.expanduser(log_path), "ab") as output:
            self.process = subprocess.Popen(
                args, stdout=output, stderr=output, stdin=subprocess.DEVNULL,
                start_new_session=True,
            )
        return self

    def poll(self):
        """Inspect exit status without reaping the group leader or freeing its PID."""
        if self.process is None:
            return 0
        result = os.waitid(os.P_PID, self.process.pid, os.WEXITED | os.WNOHANG | os.WNOWAIT)
        return None if result is None else result.si_status

    def stop(self):
        """Stop and reap only this session, escalating if its children linger."""
        process = self.process
        if process is None:
            return
        # The leader's PID remains reserved until wait()/poll() reaps it. A
        # session group is used so ros2 launch children also receive shutdown.
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                break
            time.sleep(0.05)
        else:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        process.wait(timeout=3.0)
        self.process = None
