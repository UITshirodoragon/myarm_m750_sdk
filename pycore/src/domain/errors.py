"""Error taxonomy for the MyArm M750 SDK.

The public API never exposes an unexplained firmware ``-1``. Hardware and
protocol failures are converted to typed exceptions at the adapter boundary.
"""


class MyArmSdkError(Exception):
    """Base class for all SDK errors."""


class ConfigurationError(MyArmSdkError):
    """Raised when a YAML configuration is missing or invalid."""


class HardwareConnectionError(MyArmSdkError):
    """Raised when a robot adapter cannot connect to its backend."""


class HardwareTimeoutError(MyArmSdkError):
    """Raised when a hardware operation exceeds its bounded timeout."""


class ProtocolError(MyArmSdkError):
    """Raised when firmware returns malformed, stale, or explicit error data."""


class InvalidDriverStateError(MyArmSdkError):
    """Raised when a driver operation is not valid in the current state."""


class SafetyError(MyArmSdkError):
    """Raised when a command violates a configured safety constraint."""


class KinematicsError(MyArmSdkError):
    """Raised when the kinematic model cannot be loaded or evaluated."""


class IkConvergenceError(KinematicsError):
    """Raised when inverse kinematics does not converge to the requested pose."""
