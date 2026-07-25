"""Error taxonomy for the MyArm M750 SDK.

The public API never exposes an unexplained firmware ``-1``. Hardware and
protocol failures are converted to typed exceptions at the adapter boundary.
"""


class MyArmSdkError(Exception):
    """Base class for all SDK errors."""


class ConfigurationError(MyArmSdkError):
    """Raised when a YAML configuration is missing or invalid."""


class ConfigurationMigrationError(ConfigurationError):
    """Raised when a pre-v0.2 configuration requires explicit migration."""


class HardwareConnectionError(MyArmSdkError):
    """Raised when a robot adapter cannot connect to its backend."""


class HardwareTimeoutError(MyArmSdkError):
    """Raised when a hardware operation exceeds its bounded timeout."""


class HardwareStopError(MyArmSdkError):
    """Raised after an active-command stop fails during bounded shutdown."""

    def __init__(
        self,
        message: str,
        command_id: str,
        error_code: str,
    ) -> None:
        super().__init__(message)
        self.command_id = command_id
        self.error_code = error_code


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


class CapabilityError(MyArmSdkError):
    """Raised when an operation has not been verified for the active firmware."""


class CommandCanceledError(MyArmSdkError):
    """Raised internally when an accepted command is canceled."""


class CameraError(MyArmSdkError):
    """Base class for camera capture and lifecycle errors."""


class CameraTimeoutError(CameraError):
    """Raised when no frame arrives before the acquisition deadline."""


class CameraCaptureError(CameraError):
    """Raised when a camera returns malformed or failed capture data."""
