import time

import ADD_PATH  # noqa: F401

from drivers import register_interface, request_interface

register_interface("ch347", "uart", 0)

ser = request_interface("uart", "test", 115200)
io = ser.textio_wrapper()
ser.write(b"Hello World!\n")
print(ser.readline())
