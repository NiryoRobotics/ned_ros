from ..CustomModbusSlaveContext import CustomModbusSlaveContext

slave_context = CustomModbusSlaveContext()


def load_entries():
    from . import coils
    from . import discrete_inputs
    from . import holding_registers
    from . import input_registers


load_entries()
