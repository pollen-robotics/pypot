from pypot.dynamixel.io.abstract_io import AbstractDxlIO, _DxlAccess, _DxlControl
from pypot.dynamixel.protocol import v2 as v2
import pypot.dynamixel.conversion as conv


class DxlXM430IO(AbstractDxlIO):
    _protocol = v2


def baudrate_to_dxl(value, model):
    current_baudrates = {0: 9600.0, 1: 57600.0, 2: 115200.0, 3: 1000000.0, 4: 2000000.0}

    for k, v in current_baudrates.items():
        if (abs(v - value) / float(value)) < 0.05:
            return k
    raise ValueError(
        "incorrect baudrate {} (possible values {})".format(
            value, list(current_baudrates.values())
        )
    )


controls = {
    # EEPROM
    "model": {
        "address": 0x00,
        "access": _DxlAccess.readonly,
        "dxl_to_si": conv.dxl_to_model,
    },
    "id": {
        "address": 0x07,
        "length": 1,
        "access": _DxlAccess.writeonly,
        "setter_name": "change_id",
    },
    "baudrate": {
        "address": 0x08,
        "length": 1,
        "access": _DxlAccess.writeonly,
        "setter_name": "change_baudrate",
        "si_to_dxl": baudrate_to_dxl,
    },
    "return delay time": {
        "address": 0x09,
        "length": 1,
        "dxl_to_si": conv.dxl_to_rdt,
        "si_to_dxl": conv.rdt_to_dxl,
    },
    "drive mode": {
        "address": 10,
        "length": 1,
    },
    "operating mode": {
        "address": 11,
        "length": 1,
    },
    "highest temperature limit": {
        "address": 31,
        "length": 1,
        "dxl_to_si": conv.dxl_to_temperature,
        "si_to_dxl": conv.temperature_to_dxl,
    },
    "max voltage limit": {
        "address": 32,
        "length": 2,
        "dxl_to_si": conv.dxl_to_voltage,
        "si_to_dxl": conv.voltage_to_dxl,
    },
    "min voltage limit": {
        "address": 34,
        "length": 2,
        "dxl_to_si": conv.dxl_to_voltage,
        "si_to_dxl": conv.voltage_to_dxl,
    },

    "current limit": {
        "address": 38,
        "length": 2,
    },
    "max angle limit": {
        "address": 48,
        "length": 4,
        "dxl_to_si": conv.dxl_to_degree,
        "si_to_dxl": conv.degree_to_dxl,
    },
    "min angle limit": {
        "address": 52,
        "length": 4,
        "dxl_to_si": conv.dxl_to_degree,
        "si_to_dxl": conv.degree_to_dxl,
    },

    "pwm slope": {
        "address": 0x3E,
        "length": 1,
    },

    # RAM
    "torque_enable": {
        "address": 64,
        "length": 1,
        "dxl_to_si": conv.dxl_to_bool,
        "si_to_dxl": conv.bool_to_dxl,
        "getter_name": "is_torque_enabled",
        "setter_name": "_set_torque_enable",
    },
    "LED": {
        "address": 65,
        "length": 1,
        "dxl_to_si": conv.dxl_to_bool,
        "si_to_dxl": conv.bool_to_dxl,
        "setter_name": "_set_LED",
        "getter_name": "is_led_on",
    },
    "dip gain": {
        "address": 80,
        "length": 2,
        "nb_elem": 3,
        # "dxl_to_si": conv.dxl_to_pid,
        # "si_to_dxl": conv.pid_to_dxl,
    },
    "goal position": {
        "address": 116,
        "length": 4,
        "dxl_to_si": conv.dxl_to_degree,
        "si_to_dxl": conv.degree_to_dxl,
    },
    "present velocity": {
        "address": 128,
        "length": 4,
        "access": _DxlAccess.readonly,
    },
    "present position": {
        "address": 132,
        "length": 4,
        "access": _DxlAccess.readonly,
        "dxl_to_si": conv.dxl_to_degree,
    },
    "present current": {
        "address": 126,
        "length": 2,
        "access": _DxlAccess.readonly,
    },
    "goal current": {
        "address": 102,
        "length": 2,
        "access": _DxlAccess.readonly,
    },
    "present temperature": {
        "address": 146,
        "length": 1,
        "access": _DxlAccess.readonly,
    },
}


def _add_control(
    name,
    address,
    length=2,
    nb_elem=1,
    access=_DxlAccess.readwrite,
    models=[
        "XM-430",
    ],
    dxl_to_si=lambda val, model: val,
    si_to_dxl=lambda val, model: val,
    getter_name=None,
    setter_name=None,
):
    control = _DxlControl(
        name,
        address,
        length,
        nb_elem,
        access,
        models,
        dxl_to_si,
        si_to_dxl,
        getter_name,
        setter_name,
    )

    DxlXM430IO._generate_accessors(control)


for name, args in controls.items():
    _add_control(name, **args)
