FROM python:3.11

# Instalar Python, pip y herramientas para compilar gpiod
RUN apt update 
RUN apt install -y python3 python3-pip python3-dev
RUN apt install -y build-essential
RUN apt install -y libgpiod-dev
# RUN apt install -y python3-gpiod 
RUN apt install -y python3-libgpiod
RUN rm -rf /var/lib/apt/lists/*

# # Instalar el binding Python moderno de gpiod
RUN pip3 install gpiod

WORKDIR /app

# Copiamos requirements primero para aprovechar cache
COPY requirements.txt .

# Instalamos dependencias de Python (incluye Flask)
RUN pip install --no-cache-dir -r requirements.txt

# Copiamos el código de la app
COPY . .

# Exponemos el puerto de Flask
EXPOSE 5000

CMD ["python3", "app.py"]
