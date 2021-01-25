from enum import Enum, unique


@unique
class LanguageEnum(Enum):
    # Runnable
    PYTHON2 = 1
    PYTHON3 = 2

    # Need interpretation
    BLOCKLY = 66
