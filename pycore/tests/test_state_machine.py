import pytest
from myarm_m750_core.domain.errors import InvalidDriverStateError
from myarm_m750_core.runtime import DriverEvent, DriverState, DriverStateMachine


def test_state_machine_requires_events_reasons_and_records_context() -> None:
    machine = DriverStateMachine()
    with pytest.raises(InvalidDriverStateError):
        machine.apply(DriverEvent.EXECUTION_ACCEPTED, reason="invalid")
    machine.apply(DriverEvent.CONNECT_REQUESTED, reason="test connect")
    machine.apply(DriverEvent.CONNECT_SUCCEEDED, reason="test connected")
    machine.apply(
        DriverEvent.EXECUTION_ACCEPTED,
        reason="validated",
        command_id="command-1",
    )
    machine.apply(
        DriverEvent.EXECUTION_SUCCEEDED,
        reason="complete",
        command_id="command-1",
    )
    assert machine.state is DriverState.IDLE
    assert machine.history[-1].command_id == "command-1"
