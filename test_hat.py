import datetime
import time

from zero_hat import ZeroHat, logger

hat = ZeroHat()
# hat.start_listen_serial(print_state=True, baudrate=921600)
hat.start_listen_i2c(print_state=True)
hat.wait_for_connection()


def key_callback(key_name, key_act):
    logger.info(f"Key: {key_name}, {key_act}")


def cursor_callback(x, y):
    logger.info(f"Cursor: {x}, {y}")


def gesture_callback(gesture):
    logger.info(f"Gesture: {gesture}")


def uart_callback(data):
    logger.info(f"UART: {data}")


hat.register_key_callback(key_callback)
hat.register_cursor_callback(cursor_callback)
hat.register_gesture_callback(gesture_callback)
hat.register_uart_callback(uart_callback)
hat.set_key(complex_event=False)
# hat.set_imu(True, 20)
hat.set_gesture(True)
# hat.set_rtc(datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))))
# hat.start_uart()
hat.ws2812.init(8 * 8)
h, s, v = 0, 255, 255
while True:
    # h = (h + 0.5) % 360
    hat.ws2812.set_color_range(hat.ws2812.hsv_to_rgb(h, s, v), 0, 8 * 8 - 1)
    # hat.set_hsv_led(h, s, v)
    # time.sleep(0.02)
