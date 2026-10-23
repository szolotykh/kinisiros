"""Verify subprocess ownership without launching ROS or signaling real processes."""

import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kinisirobot import owned_backend as runtime


def test_stop_without_ownership_never_signals(monkeypatch):
    """An externally launched backend must be left alone."""
    signal_group = Mock()
    monkeypatch.setattr(runtime.os, 'killpg', signal_group, raising=False)
    runtime.OwnedBackend().stop()
    signal_group.assert_not_called()


def test_launch_and_stop_target_only_the_owned_session(monkeypatch, tmp_path):
    """Launch a separate session and stop its exact group, including ROS children."""
    process = Mock(pid=4321)
    launch = Mock(return_value=process)
    monkeypatch.setattr(runtime.subprocess, 'Popen', launch)
    signal_group = Mock(side_effect=[None, ProcessLookupError()])
    monkeypatch.setattr(runtime.os, 'killpg', signal_group, raising=False)
    backend = runtime.OwnedBackend()
    backend.start(['ros2', 'launch', 'example'], str(tmp_path / 'backend.log'))
    assert launch.call_args.kwargs['start_new_session'] is True
    backend.stop()
    assert all(call.args[0] == 4321 for call in signal_group.call_args_list)
    process.wait.assert_called_once()
    assert backend.process is None


def test_poll_keeps_leader_pid_reserved_until_stop(monkeypatch):
    """Do not reap the leader while owned children could still be running."""
    for name, value in [('P_PID', 1), ('WEXITED', 4), ('WNOHANG', 1), ('WNOWAIT', 8)]:
        monkeypatch.setattr(runtime.os, name, value, raising=False)
    wait = Mock(side_effect=[None, SimpleNamespace(si_status=1)])
    monkeypatch.setattr(runtime.os, 'waitid', wait, raising=False)
    backend = runtime.OwnedBackend()
    backend.process = Mock(pid=4321)
    assert backend.poll() is None
    assert backend.poll() == 1
    assert wait.call_args.args[2] & runtime.os.WNOWAIT
    backend.process.poll.assert_not_called()


def test_lingering_children_are_killed_only_in_owned_group(monkeypatch):
    """A stuck ROS child cannot survive replacement or affect another session."""
    backend = runtime.OwnedBackend()
    backend.process = Mock(pid=4321)
    signal_group = Mock()
    monkeypatch.setattr(runtime.os, 'killpg', signal_group, raising=False)
    monkeypatch.setattr(runtime.signal, 'SIGKILL', 9, raising=False)
    monkeypatch.setattr(runtime.time, 'monotonic', Mock(side_effect=[0, 4]))
    backend.stop()
    assert signal_group.call_args_list[-1].args == (4321, 9)
    assert backend.process is None
