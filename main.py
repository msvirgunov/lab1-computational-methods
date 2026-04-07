# Модель: Математичне моделювання та оптимізація маршрутів доставки (VRP)
# Автор: Свиргунов Максим Віталійович, група АІ-235

import pandas as pd
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

class VehicleRoutingModel:
    def __init__(self, data):
        # Словник з матрицею відстаней, попитом, автопарком
        self.data = data 
        self.manager = None
        self.routing = None
        self.solution = None

    def build_model(self):
        # 1. Створення RoutingIndexManager та RoutingModel
        self.manager = pywrapcp.RoutingIndexManager(
            len(self.data['distance_matrix']),
            self.data['num_vehicles'], 
            self.data['depot']
        )
        self.routing = pywrapcp.RoutingModel(self.manager)
        
        # Тут будемо додавати обмеження вантажопідйомності (Qk) та відстаней (Dij)
        print("Модель успішно ініціалізована.")

    def solve_model(self):
        # Налаштування пошуку (First Solution: PATH_CHEAPEST_ARC)
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )
        print("Пошук розв'язку запущено...")

if __name__ == "__main__":
    # Тестові дані для Сценарію 1 (Базовий)
    data = {
        'distance_matrix': [[0, 10], [10, 0]], 
        'num_vehicles': 2,
        'depot': 0
    }
    model = VehicleRoutingModel(data)
    model.build_model()
    model.solve_model()
