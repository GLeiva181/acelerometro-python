import gpiod
from gpiod.line import Direction, Edge, Bias

class GPIOInterrupt:
    def __init__(self, chip="/dev/gpiochip0", pin=25):
        self.chip = chip
        self.pin = pin
        self.gpio = gpiod.request_lines(
            chip,
            consumer="adxl355-int",
            config={
                pin: gpiod.LineSettings(
                    direction = Direction.INPUT,
                    edge_detection = Edge.FALLING,   # Activo en bajo → flanco descendente
                    bias = Bias.PULL_UP    # Pull-up interno
                )
            }
        )

    def wait_event(self, timeout=None):
        """Bloquea hasta que ocurra un evento en el pin"""
        # gpiod timeout is in nanoseconds
        ready = self.gpio.wait_edge_events(timeout=timeout)
        if ready:
            # Lee todos los eventos que se hayan acumulado
            return self.gpio.read_edge_events()
        else:
            return []