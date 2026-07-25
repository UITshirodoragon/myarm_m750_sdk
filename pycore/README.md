# myarm-m750-core

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
src/resources
```

Setuptools installs these as the stable namespace `myarm_m750_core.*`.

```bash
../tools/bootstrap_core.sh
../tools/test_core.sh
```

Camera support is optional:

```bash
../.venv-core/bin/python -m pip install \
  --constraint ../requirements/constraints-py38.txt \
  --editable '.[camera-host]'
```

The default robot YAML uses `MockRobotAdapter`; the standalone camera test uses
`MockCameraAdapter`. Neither path requires ROS 2 or physical hardware.
