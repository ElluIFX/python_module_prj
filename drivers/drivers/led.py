import os
from functools import cache


class Led(object):
    def __init__(self, ledname="radxa-zero:state"):
        self._devpath = os.path.join("/sys/class/leds", ledname)

    @property
    def brightness(self):
        with open(os.path.join(self._devpath, "brightness"), "r") as f:
            return int(f.read())

    @brightness.setter
    def brightness(self, value):
        assert 0 <= value <= self.max_brightness, "Invalid brightness value"
        with open(os.path.join(self._devpath, "brightness"), "w") as f:
            f.write(str(int(value)))

    @property
    @cache
    def max_brightness(self):
        with open(os.path.join(self._devpath, "max_brightness"), "r") as f:
            return int(f.read())

    @property
    @cache
    def avail_triggers(self):
        with open(os.path.join(self._devpath, "trigger"), "r") as f:
            triggers = f.read().strip().split(" ")
        triggers = [t.replace("[", "").replace("]", "") for t in triggers]
        return triggers

    @property
    def trigger(self):
        with open(os.path.join(self._devpath, "trigger"), "r") as f:
            triggers = f.read().strip().split(" ")
        for t in triggers:
            if t.startswith("["):
                return t.replace("[", "").replace("]", "")
        raise RuntimeError("No active trigger")

    @trigger.setter
    def trigger(self, value):
        assert value in self.avail_triggers, "Invalid trigger"
        with open(os.path.join(self._devpath, "trigger"), "w") as f:
            f.write(value)

    def on(self):
        self.brightness = self.max_brightness

    def off(self):
        self.brightness = 0

    def toggle(self):
        if self.brightness == 0:
            self.brightness = self.max_brightness
        else:
            self.brightness = 0


if __name__ == "__main__":
    import time

    led = Led()
    while True:
        time.sleep(0.5)
        led.toggle()
