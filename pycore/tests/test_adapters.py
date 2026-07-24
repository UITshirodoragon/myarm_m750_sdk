import json
from pathlib import Path

from myarm_m750_core.adapters import MockRobotAdapter, ReplayRobotAdapter
from myarm_m750_core.domain.models import JointTarget


def test_mock_adapter_updates_measured_state() -> None:
    adapter = MockRobotAdapter([0.0] * 6)
    adapter.connect()
    target = JointTarget((0.1, -0.1, 0.2, 0.0, 0.0, 0.0))
    assert adapter.write_joint_target(target).succeeded
    assert adapter.read_state().position_rad == target.position_rad
    adapter.disconnect()


def test_replay_adapter_is_read_only(tmp_path: Path) -> None:
    replay_path = tmp_path / "replay.jsonl"
    replay_path.write_text(
        json.dumps({"position_rad": [0.0] * 6, "sequence": 1}) + "\n",
        encoding="utf-8",
    )
    adapter = ReplayRobotAdapter(str(replay_path))
    adapter.connect()
    assert adapter.read_state().sequence == 1
    assert not adapter.write_joint_target(JointTarget((0.0,) * 6)).succeeded
