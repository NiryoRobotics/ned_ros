# Niryo robot modbus

## Quick Modbus informations

Here are the basics information you need to know about modbus in order to contribute to this package.

It is protocol based on registers ; everything is a read or a write at specific addresses.
These addresses are stored in 4 different registers:
  - Coils (1 bit values, Read/Write)
  - Discrete Inputs (1 bit values, Read only)
  - Holding Registers (16 bits values, Read/Write)
  - Input Registers (16 bits values, Read only)

To summarize, we use coils and discrete inputs to store boolean values, and holding registers and input registers for anything else (int, float, str...)
If the data can be modified (e.g: the learning mode), we store it in a read/write registry, otherwise we store it in a read only registry.

As the registers use 16 bits to store their values, if we want to store bigger values such as 32 bits floats, we will have to store them on multiple addresses.

## How it works

This package heavily rely on the [pymodbus](https://pymodbus.readthedocs.io/) library.
A modbus server with pymodbus use a slave context in order to know how to behave with an incoming request. We created our custom slave context (`CustomModbusSlaveContext`), which will mimic a real modbus registry.

To know what is stored in each address, we all the classes declared in `mapping`. These class inherits from `ABCRegisterEntries` or `ABCRegisterEntry`. It's these classes which will allow the creation of a modbus entry.
`ABCRegisterEntries` is used to declare entries which will be on a range (e.g: the joints positions), and `ABCRegisterEntry` is used for single entries such as the learning mode.

To know in what register the entry must belong, we add a decorator on top of each class.
The decorators are `CustomModbusSlaveContext` functions, which will simply register the entries.

There is some properties and methods to overwrite to create your own entry:

| name               | type                        | default value | description                                                                                       | required                        |
|--------------------|-----------------------------|---------------|---------------------------------------------------------------------------------------------------|---------------------------------|
| data_type          | type                        | None          | The data type of the entry (must be one of SupportedType)                                         | True                            |
| reserved_addresses | int                         | 0             | The number of addresses reserved for this entry. Useful if the number will increase in the future | False (ABCRegisterEntries only) |
| get_address_count  | NiryoRobotRosWrapper -> int | None          | Returns the number of addresses                                                                   | True (ABCRegisterEntries only)  |
| get                | None -> SupportedType       | None          | Core function to access the value stored at this address                                          | True                            |
| set                | SupportedType -> None       | None          | Core function to edit the value stored at this address                                            | False                           |

### Address attribution
The addresses are attributed dynamically; we don't set them ourselves. But we can influence a bit on it by modifying the order in the mapping files (first declared, first addressed), and set the `reserved_addresses` property.
For example, the DIOs all have this property to 50. This means that there will always have 50 addresses reserved for the DIOs no matter how much there really is.

The major pro of this method is that we don't have to bother about the number of addresses used by each entry (e.g: a joint position takes 12 addresses because each joint is encoded on 32 bits (=2 addresses))