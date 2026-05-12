# Модель: Математичне моделювання та оптимізація маршрутів доставки (VRP)
# Автор: Свиргунов Максим Віталійович, група АІ-235

import os
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

class VehicleRoutingModel:
    def __init__(self, data):
        self.data = data
        self.manager = pywrapcp.RoutingIndexManager(
            len(data['distance_matrix']),
            data['num_vehicles'],
            data['depot']
        )
        self.routing = pywrapcp.RoutingModel(self.manager)

    def distance_callback(self, from_index, to_index):
        from_node = self.manager.IndexToNode(from_index)
        to_node = self.manager.IndexToNode(to_index)
        return self.data['distance_matrix'][from_node][to_node]

    def demand_callback(self, from_index):
        from_node = self.manager.IndexToNode(from_index)
        return self.data['demands'][from_node]

    def build_model(self):
        transit_callback_index = self.routing.RegisterTransitCallback(self.distance_callback)
        self.routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
        self.routing.AddDimension(transit_callback_index, 0, 3000, True, 'Distance')

        demand_callback_index = self.routing.RegisterUnaryTransitCallback(self.demand_callback)
        self.routing.AddDimensionWithVehicleCapacity(
            demand_callback_index,
            0,
            self.data['vehicle_capacities'],
            True,
            'Capacity'
        )

    def solve_model(self):
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )
        search_parameters.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
        )
        search_parameters.time_limit.seconds = 5

        self.solution = self.routing.SolveWithParameters(search_parameters)
        if self.solution:
            self.print_solution()
        else:
            print("Error: No solution found.")

    def print_solution(self):
        # Виведення інформації про запуск із змінних середовища
        student = os.getenv("STUDENT_NAME", "Unknown")
        group = os.getenv("GROUP", "Unknown")
        mode = os.getenv("MODE", "standard")

        print("\n" + "="*45)
        print(f"    RUNTIME CONTEXT: {student} ({group})")
        print(f"    OPERATING MODE: {mode.upper()}")
        print("="*45)
        print("          VRP OPTIMIZATION RESULTS          ")
        print("="*45)
        print(f"TOTAL DISTANCE: {self.solution.ObjectiveValue()} km")
        
        for vehicle_id in range(self.data['num_vehicles']):
            index = self.routing.Start(vehicle_id)
            print(f"\nVehicle #{vehicle_id}:")
            route_distance = 0
            nodes_path = []
            while not self.routing.IsEnd(index):
                node_index = self.manager.IndexToNode(index)
                nodes_path.append(str(node_index))
                previous_index = index
                index = self.solution.Value(self.routing.NextVar(index))
                route_distance += self.routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
            nodes_path.append(str(self.manager.IndexToNode(index)))
            print(f"  Path: " + " -> ".join(nodes_path))
            print(f"  Dist: {route_distance} km")
        print("="*45 + "\n")

if __name__ == "__main__":
    payload = {
        'distance_matrix': [
            [0, 15, 10, 20, 25], [15, 0, 35, 25, 10],
            [10, 35, 0, 30, 15], [20, 25, 30, 0, 20],
            [25, 10, 15, 20, 0],
        ],
        'demands': [0, 5, 8, 4, 6],
        'vehicle_capacities': [15, 15],
        'num_vehicles': 2,
        'depot': 0
    }
    model = VehicleRoutingModel(payload)
    model.build_model()
    model.solve_model()
