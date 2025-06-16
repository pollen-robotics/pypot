import itertools

from ..dynamixel.io.abstract_io import AbstractDxlIO, _DxlAccess, _DxlControl, DxlCommunicationError, DxlTimeoutError
from ..dynamixel.protocol import v1
from ..dynamixel import conversion as dxl_mx_conv
from ..dynamixel.conversion import dxl_decode_all


class FeetechSTS3215IO(AbstractDxlIO):
    _protocol = v1
    _protocol.DxlInstruction.SYNC_READ = 130

    def _get_control_value(self, control, ids, **kwargs):
        if not ids:
            return ()

        error_handler = kwargs['error_handler'] if ('error_handler' in kwargs) else self._error_handler
        convert = kwargs['convert'] if ('convert' in kwargs) else self._convert

        if self._sync_read and len(ids) > 1:
            rp = self._protocol.DxlSyncReadPacket(ids, control.address,
                                                  control.length * control.nb_elem)

            with self._serial_lock:
                sp = self._send_packet(rp,
                                       error_handler=error_handler,
                                       _force_lock=True)
                if not sp:
                    return ()

                if True:
                    values = list(sp.parameters)
                    for i in range(len(ids) - 1):
                        try:
                            sp = self._real_read(rp, _force_lock=True)
                        except (DxlTimeoutError, DxlCommunicationError):
                            return ()
                        values.extend(sp.parameters)

                    if len(values) < len(ids):
                        return ()

        else:
            values = []
            for motor_id in ids:
                rp = self._protocol.DxlReadDataPacket(motor_id, control.address, control.length * control.nb_elem)
                sp = self._send_packet(rp, error_handler=error_handler)

                if not sp:
                    return ()

                values.extend(sp.parameters)

        values = list(zip(*([iter(values)] * control.length * control.nb_elem)))
        values = [dxl_decode_all(value, control.nb_elem) for value in values]

        if not values:
            return ()

        # when using SYNC_READ instead of getting a timeout
        # a non existing motor will "return" the maximum value
        if self._sync_read and self._protocol.name == 'v1':
            max_val = 2 ** (8 * control.length) - 1
            if max_val in (itertools.chain(*values) if control.nb_elem > 1 else values):
                lost_ids = []
                for i, v in enumerate(itertools.chain(*values) if control.nb_elem > 1 else values):
                    if v == max_val:
                        lost_ids.append(ids[i // control.nb_elem])
                e = DxlTimeoutError(self, rp, list(set(lost_ids)))
                if self._error_handler:
                    self._error_handler.handle_timeout(e)
                    return ()
                else:
                    raise e

        if convert:
            models = self.get_model(ids)
            if not models:
                return ()
            values = [control.dxl_to_si(v, m) for v, m in zip(values, models)]

        return tuple(values)



controls = {
    # EEPROM
    "model": {
        "address": 3,
        "length": 2,
        "access": _DxlAccess.readonly,
    },
    "id": {
        "address": 5,
        "length": 1,
        "access": _DxlAccess.readwrite,
        "setter_name": "change_id",
    },
    "baudrate": {
        "address": 6,
        "length": 1,
        "access": _DxlAccess.readwrite,
        "setter_name": "change_baudrate",
    },
    "return delay time": {
        "address": 7,
        "length": 1,
        "access": _DxlAccess.readwrite,
    },
    "response status level": {
        "address": 8,
        "length": 1,
        "access": _DxlAccess.readwrite,
    },
    "min angle limit": {
        "address": 9,
        "length": 2,
        "access": _DxlAccess.readwrite,
        "dxl_to_si": dxl_mx_conv.dxl_to_degree,
        "si_to_dxl": dxl_mx_conv.degree_to_dxl,
    },
    "max angle limit": {
        "address": 11,
        "length": 2,
        "access": _DxlAccess.readwrite,
        "dxl_to_si": dxl_mx_conv.dxl_to_degree,
        "si_to_dxl": dxl_mx_conv.degree_to_dxl,
    },
    "max temperature limit": {
        "address": 13,
        "length": 1,
        "access": _DxlAccess.readwrite,
    },
    "max voltage limit": {
        "address": 14,
        "length": 1,
        "access": _DxlAccess.readwrite,
    },
    "min voltage limit": {
        "address": 15,
        "length": 1,
        "access": _DxlAccess.readwrite,
    },
    "max torque limit": {
        "address": 16,
        "length": 2,
        "access": _DxlAccess.readwrite,
    },
    "phase": {
        "address": 18,
        "length": 1,
        "access": _DxlAccess.readwrite,
    },
    "unloading condition": {
        "address": 19,
        "length": 1,
        "access": _DxlAccess.readwrite,
    },
    "LED alarm condition": {
        "address": 20,
        "length": 1,
        "access": _DxlAccess.readwrite,
    },
    "P coefficient": {
        "address": 21,
        "length": 1,
        "access": _DxlAccess.readwrite,
    },
    "D coefficient": {
        "address": 22,
        "length": 1,
        "access": _DxlAccess.readwrite,
    },
    "I coefficient": {
        "address": 23,
        "length": 1,
        "access": _DxlAccess.readwrite,
    },
    "minimum startup force": {
        "address": 24,
        "length": 2,
        "access": _DxlAccess.readwrite,
    },
    "CW dead zone": {
        "address": 26,
        "length": 1,
        "access": _DxlAccess.readwrite,
    },
    "CCW dead zone": {
        "address": 27,
        "length": 1,
        "access": _DxlAccess.readwrite,
    },
    "protection current": {
        "address": 28,
        "length": 2,
        "access": _DxlAccess.readwrite,
    },
    "angular resolution": {
        "address": 30,
        "length": 1,
        "access": _DxlAccess.readwrite,
    },
    "offset": {
        "address": 31,
        "length": 2,
        "access": _DxlAccess.readwrite,
        "dxl_to_si": dxl_mx_conv.dxl_to_degree,
        "si_to_dxl": dxl_mx_conv.degree_to_dxl,
    },
    "mode": {
        "address": 33,
        "length": 1,
        "access": _DxlAccess.readwrite,
    },
    "protective torque": {
        "address": 34,
        "length": 1,
        "access": _DxlAccess.readwrite,
    },
    "protection time": {
        "address": 35,
        "length": 1,
        "access": _DxlAccess.readwrite,
    },
    "overload torque": {
        "address": 36,
        "length": 1,
        "access": _DxlAccess.readwrite,
    },
    "speed closed loop P proportional coefficient": {
        "address": 37,
        "length": 1,
        "access": _DxlAccess.readwrite,
    },
    "over current protection time": {
        "address": 38,
        "length": 1,
        "access": _DxlAccess.readwrite,
    },
    "velocity closed loop I integral coefficient": {
        "address": 39,
        "length": 1,
        "access": _DxlAccess.readwrite,
    },
    "torque enable": {
        "address": 40,
        "length": 1,
        "access": _DxlAccess.readwrite,
        "getter_name": "is_torque_enabled",
        "setter_name": "_set_torque_enable",
    },
    "acceleration": {
        "address": 41,
        "length": 1,
        "access": _DxlAccess.readwrite,
    },
    "goal position": {
        "address": 42,
        "length": 2,
        "access": _DxlAccess.readwrite,
        "dxl_to_si": dxl_mx_conv.dxl_to_degree,
        "si_to_dxl": dxl_mx_conv.degree_to_dxl,
    },
    "goal time": {
        "address": 44,
        "length": 2,
        "access": _DxlAccess.readwrite,
    },
    "goal speed": {
        "address": 46,
        "length": 2,
        "access": _DxlAccess.readwrite,
    },
    "torque limit": {
        "address": 48,
        "length": 2,
        "access": _DxlAccess.readwrite,
    },
    "lock": {
        "address": 55,
        "length": 1,
        "access": _DxlAccess.readwrite,
    },
    "present position": {
        "address": 56,
        "length": 2,
        "access": _DxlAccess.readonly,
        "dxl_to_si": dxl_mx_conv.dxl_to_degree,
        "si_to_dxl": dxl_mx_conv.degree_to_dxl,
    },
    "present position raw": {
        "address": 56,
        "length": 2,
        "access": _DxlAccess.readonly,
    },
    "present speed": {
        "address": 58,
        "length": 2,
        "access": _DxlAccess.readonly,
        "dxl_to_si": dxl_mx_conv.feetech_to_speed,
        "si_to_dxl": dxl_mx_conv.speed_to_feetech,
    },
    "present load": {
        "address": 60,
        "length": 2,
        "access": _DxlAccess.readonly,
    },
    "present voltage": {
        "address": 62,
        "length": 1,
        "access": _DxlAccess.readonly,
    },
    "present temperature": {
        "address": 63,
        "length": 1,
        "access": _DxlAccess.readonly,
    },
    "status": {
        "address": 65,
        "length": 1,
        "access": _DxlAccess.readonly,
    },
    "moving": {
        "address": 66,
        "length": 1,
        "access": _DxlAccess.readonly,
    },
    "present current": {
        "address": 69,
        "length": 2,
        "access": _DxlAccess.readonly,
    },
    "vfk": {
        "address": 77,
        "length": 1,
        "access": _DxlAccess.readwrite,
    },
    "vmin": {
        "address": 80,
        "length": 1,
        "access": _DxlAccess.readwrite,
    },
    "vk": {
        "address": 83,
        "length": 1,
        "access": _DxlAccess.readwrite,
    },
    "maximum velocity": {
        "address": 84,
        "length": 1,
        "access": _DxlAccess.readwrite,
    },
    # Not in the Memory Table
    "maximum acceleration": {
        "address": 85,
        "length": 1,
        "access": _DxlAccess.readwrite,
    },
}


def _add_control(
    name,
    address,
    length=2,
    nb_elem=1,
    access=_DxlAccess.readwrite,
    models=[
        "FEETECH-STS3115",
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

    FeetechSTS3215IO._generate_accessors(control)


for name, args in controls.items():
    _add_control(name, **args)
