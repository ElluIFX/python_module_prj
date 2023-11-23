from .interface import request_interface


class i2c(object):
    def __init__(self, address=0x3C):
        self._cmd_mode = 0x00
        self._data_mode = 0x40

        try:
            self._addr = int(str(address), 0)
        except ValueError:
            raise ValueError(f"I2C device address invalid: {address}")

        self._bus = request_interface("i2c","luma.core", address)
        self._managed = True

    def command(self, *cmd):
        self._bus.write_reg_data(self._cmd_mode, list(cmd))

    def data(self, data):
        # block size is the maximum data payload that will be tolerated.
        # The managed i2c will transfer blocks of upto 4K (using i2c_rdwr)
        # whereas we must use the default 32 byte block size when unmanaged
        block_size = self._bus.max_transfer_size
        if len(data) > block_size:
            i = 0
            n = len(data)
            msgs = []
            while i < n:
                msgs.append(
                    self._bus.new_msg.write(
                        [self._data_mode] + list(data[i : i + block_size])
                    )
                )
                i += block_size
            self._bus.transfer_msg(msgs)
        else:
            self._bus.write_reg_data(self._data_mode, data)

    def cleanup(self):
        pass
