import pytest

from myarm_m750_core.domain.errors import InvalidDriverStateError
from myarm_m750_core.runtime import DriverState, DriverStateMachine


def test_state_machine_rejects_hidden_transition() -> None:
    machine = DriverStateMachine()
    with pytest.raises(InvalidDriverStateError):
        machine.transition_to(DriverState.EXECUTING)
    machine.transition_to(DriverState.CONNECTING)
    machine.transition_to(DriverState.IDLE)
    machine.transition_to(DriverState.EXECUTING)
    machine.transition_to(DriverState.IDLE)
