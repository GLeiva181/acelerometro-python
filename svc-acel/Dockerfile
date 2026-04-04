FROM arm64v8/python:3.9-slim

RUN apt update && apt install -y \
    python3 \
    python3-pip \
    python3-dev \
    build-essential \
    libgpiod-dev \
    python3-rpi.gpio \
    python3-libgpiod \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install gpiod pigpio lgpio

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

CMD ["python3", "app.py"]