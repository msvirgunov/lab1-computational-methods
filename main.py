# Модель: Математичне моделювання та оптимізація маршрутів доставки (VRP)
# Автор: Свиргунов Максим Віталійович, група АІ-235

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

class VehicleRoutingModel:
    def __init__(self, data):
        self.data = data
        # Initialize Routing Index Manager and Model
        self.manager = pywrapcp.RoutingIndexManager(
            len(data['distance_matrix']),
            data['num_vehicles'],
            data['depot']
        )
        self.routing = pywrapcp.RoutingModel(self.manager)

    def distance_callback(self, from_index, to_index):
        """Returns the distance between the two nodes."""
        from_node = self.manager.IndexToNode(from_index)
        to_node = self.manager.IndexToNode(to_index)
        return self.data['distance_matrix'][from_node][to_node]

    def demand_callback(self, from_index):
        """Returns the demand of the node."""
        from_node = self.manager.IndexToNode(from_index)
        return self.data['demands'][from_node]

    def build_model(self):
        # 1. Distance constraints (Objective Function)
        transit_callback_index = self.routing.RegisterTransitCallback(self.distance_callback)
        self.routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
        self.routing.AddDimension(transit_callback_index, 0, 3000, True, 'Distance')

        # 2. Capacity constraints (Qk)
        demand_callback_index = self.routing.RegisterUnaryTransitCallback(self.demand_callback)
        self.routing.AddDimensionWithVehicleCapacity(
            demand_callback_index,
            0,  # null capacity slack
            self.data['vehicle_capacities'],  # vehicle maximum capacities
            True,  # start cumul to zero
            'Capacity'
        )
        print(">>> Model built successfully: Distance and Capacity constraints applied.")

    def solve_model(self):
        # Search parameters
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )
        # Metaheuristic for optimization
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
        print("\n" + "="*45)
        print("          VRP OPTIMIZATION RESULTS          ")
        print("="*45)
        print(f"TOTAL DISTANCE: {self.solution.ObjectiveValue()} km")
        
        total_distance = 0
        total_load = 0

        for vehicle_id in range(self.data['num_vehicles']):
            index = self.routing.Start(vehicle_id)
            print(f"\nVehicle #{vehicle_id} (Cap: {self.data['vehicle_capacities'][vehicle_id]}):")
            
            route_distance = 0
            route_load = 0
            nodes_path = []

            while not self.routing.IsEnd(index):
                node_index = self.manager.IndexToNode(index)
                nodes_path.append(str(node_index))
                route_load += self.data['demands'][node_index]
                
                previous_index = index
                index = self.solution.Value(self.routing.NextVar(index))
                route_distance += self.routing.GetArcCostForVehicle(previous_index, index, vehicle_id)

            nodes_path.append(str(self.manager.IndexToNode(index)))
            
            print(f"  Path:  " + " -> ".join(nodes_path))
            print(f"  Load:  {route_load} units")
            print(f"  Dist:  {route_distance} km")
            
            total_distance += route_distance
            total_load += route_load

        print("\n" + "="*45)
        print(f"SUMMARY: Total {total_distance} km | Load {total_load} units")
        print("="*45 + "\n")

if __name__ == "__main__":
    # Scenario: 5 locations, 2 vehicles, capacity constraints
    payload = {
        'distance_matrix': [
            [0, 15, 10, 20, 25], # 0 - Depot
            [15, 0, 35, 25, 10], # 1 - Client
            [10, 35, 0, 30, 15], # 2 - Client
            [20, 25, 30, 0, 20], # 3 - Client
            [25, 10, 15, 20, 0], # 4 - Client
        ],
        'demands': [0, 5, 8, 4, 6],
        'vehicle_capacities': [15, 15],
        'num_vehicles': 2,
        'depot': 0
    }
    
    model = VehicleRoutingModel(payload)
    model.build_model()
    model.solve_model()
