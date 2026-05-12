# Математичне моделювання та оптимізація маршрутів доставки для мінімізації витрат палива (VRP)
# Свиргунов Максим АІ-235

FROM python:3.10-slim

WORKDIR /app

COPY main.py .

CMD ["python", "main.py"]
