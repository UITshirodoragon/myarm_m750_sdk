from pathlib import Path


def test_requested_python_core_layout(repository_root: Path) -> None:
    expected = {
        "api",
        "application",
        "domain",
        "ports",
        "adapters",
        "runtime",
        "diagnostics",
    }
    source_root = repository_root / "pycore/src"
    assert expected.issubset({path.name for path in source_root.iterdir() if path.is_dir()})
    assert not (source_root / "myarm_m750_core").exists()


def test_requested_ros2_layout(repository_root: Path) -> None:
    expected = {
        "myarm_m750_description",
        "myarm_m750_driver",
        "myarm_m750_bringup",
        "myarm_m750_visualization",
        "myarm_m750_camera",
        "myarm_m750_moveit_config",
        "myarm_m750_gazebo",
        "myarm_m750_msgs",
    }
    source_root = repository_root / "ros2/src"
    assert expected == {path.name for path in source_root.iterdir() if path.is_dir()}
