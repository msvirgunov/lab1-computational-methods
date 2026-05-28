
# Лабораторна робота №7 — Хмарне розгортання DevOps-сервісу

<!--

Модель: Математичне моделювання та оптимізація маршрутів доставки для мінімізації витрат палива (VRP)

Автор: Свиргунов Максим Віталійович, група АІ-235

-->

## Автор

**Свиргунов Максим Віталійович**, група АІ-235

## Модель

Математичне моделювання та оптимізація маршрутів доставки для мінімізації витрат палива (VRP)

## Платформа deployment

[Render](https://render.com) — безкоштовний free-tier

## Публічний URL

https://devops-235-svirgunov.onrender.com

## Тип deployment

Auto Docker deployment через GitHub Actions → Render Deploy Hook

---

## Endpoints

### GET `/`

```bash

curl https://devops-235-svirgunov.onrender.com/

```

### GET `/health`

Health check для Render.

### POST `/calculate`

```bash

curl -X POST https://devops-235-svirgunov.onrender.com/calculate \

  -H "Content-Type: application/json" \

  -d '{

    "distance_matrix": [[0,10,20],[10,0,15],[20,15,0]],

    "demands": [0, 5, 8],

    "vehicle_capacities": [15, 15],

    "num_vehicles": 2,

    "depot": 0

  }'

```

---

## Безпека

- DEBUG=false у production

- SECRET_KEY генерується Render автоматично

- Секрети не публікуються в GitHub

- Контейнер запускається від non-root користувача

- .env файли виключені з репозиторію

## Environment Variables

| Змінна | Опис | За замовчуванням |

|--------|------|-----------------|

| DEBUG | Debug mode Flask | false |

| PORT | Порт сервісу | 5000 |

| SECRET_KEY | Flask secret key | генерується Render |

---

## Локальний запуск

```bash

docker build -t devops-235-svirgunov .

docker run -p 5000:5000 -e DEBUG=false devops-235-svirgunov

```

