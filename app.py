import threading
from flask import Flask, jsonify
from adxl355 import ADXL355
from interrupt import GPIOInterrupt

app = Flask(__name__)

# Instancias
sensor = ADXL355()
irq = GPIOInterrupt(pin=25)

last_data = None  # Guarda la última lectura del sensor

def irq_handler():
    global last_data
    while True:
        events = irq.wait_event(timeout=1.0)
        if events == []:
            data = sensor.read_fifo(32)
        else: 
            for ev in events:
                if ev.line_offset == irq.pin:
                    # Leer FIFO completo cada vez que hay interrupción
                    data = sensor.read_data(0x06)
                    print(f"Data1: {data}")
                    data = sensor.read_data(0x07)
                    print(f"Data2: {data}")
                    data = sensor.get_temperature()
                    if data:
                        last_data = data
                        print(f"INTERRUPCIÓN: {data}")

@app.route("/data", methods=["GET"])
def get_data():
    """Devuelve última muestra válida leída del FIFO"""
    if last_data:
        return jsonify(last_data)
    else:
        return jsonify({"error": "No data yet"}), 404

if __name__ == "__main__":
    threading.Thread(target=irq_handler, daemon=True).start()
    app.run(host="0.0.0.0", port=5000)
