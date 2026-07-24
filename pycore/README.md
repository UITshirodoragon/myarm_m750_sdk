# myarm-m750-core 0.1.1

ROS-optional Python package for MyArm M750 control, PoE FK/IK/Jacobian, safety,
trajectory execution, diagnostics, hardware adapters, and standalone camera
capture.

The physical source layout is intentionally short:

```text
src/api
src/application
src/domain
src/ports
src/adapters
src/runtime
src/diagnostics
```

Setuptools installs these as the stable namespace `myarm_m750_core.*`.

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r ../requirements/dev.txt
python3 -m pip install -e .
pytest
```

Camera support is optional:

```bash
python3 -m pip install -r ../requirements/camera.txt
```

The default robot YAML uses `MockRobotAdapter`; the standalone camera test uses
`MockCameraAdapter`. Neither path requires ROS 2 or physical hardware.
