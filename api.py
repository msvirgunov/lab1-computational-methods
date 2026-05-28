# Модель: Математичне моделювання та оптимізація маршрутів доставки (VRP, 5 семестр)
# Автор: Свиргунов Максим Віталійович, група АІ-235

import os
from flask import Flask, request, jsonify
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

app = Flask(__name__)

DEBUG_MODE = os.environ.get("DEBUG", "false").lower() == "true"
PORT = int(os.environ.get("PORT", 5000))
SECRET_KEY = os.environ.get("SECRET_KEY", "change-me-in-production")

app.secret_key = SECRET_KEY


def solve_vrp(distance_matrix, demands, vehicle_capacities, num_vehicles, depot):
    data = {
        'distance_matrix': distance_matrix,
        'demands': demands,
        'vehicle_capacities': vehicle_capacities,
        'num_vehicles': num_vehicles,
        'depot': depot
    }

    manager = pywrapcp.RoutingIndexManager(
        len(data['distance_matrix']),
        data['num_vehicles'],
        data['depot']
    )
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        return data['distance_matrix'][manager.IndexToNode(from_index)][manager.IndexToNode(to_index)]

    def demand_callback(from_index):
        return data['demands'][manager.IndexToNode(from_index)]

    transit_idx = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_idx)
    routing.AddDimension(transit_idx, 0, 3000, True, 'Distance')

    demand_idx = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(demand_idx, 0, data['vehicle_capacities'], True, 'Capacity')

    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    params.time_limit.seconds = 5

    solution = routing.SolveWithParameters(params)

    if not solution:
        return None

    routes = []
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        path = []
        route_distance = 0
        while not routing.IsEnd(index):
            path.append(manager.IndexToNode(index))
            prev = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(prev, index, vehicle_id)
        path.append(manager.IndexToNode(index))
        routes.append({"vehicle": vehicle_id, "path": path, "distance_km": route_distance})

    return {"total_distance_km": solution.ObjectiveValue(), "routes": routes}


@app.route('/calculate', methods=['POST'])
def calculate():
    body = request.get_json()
    if not body:
        return jsonify({"error": "JSON body required"}), 400

    result = solve_vrp(
        distance_matrix=body.get('distance_matrix'),
        demands=body.get('demands'),
        vehicle_capacities=body.get('vehicle_capacities'),
        num_vehicles=body.get('num_vehicles', 2),
        depot=body.get('depot', 0)
    )

    if result is None:
        return jsonify({"error": "No solution found"}), 500

    return jsonify(result)


@app.route('/', methods=['GET'])
def index():
    return jsonify({
        "status": "ok",
        "model": "VRP Optimization",
        "author": "Свиргунов Максим, АІ-235",
        "endpoints": ["/calculate (POST)", "/ (GET)"]
    })


@app.route('/health', methods=['GET'])
def health():
    return jsonify({"status": "healthy"}), 200


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=PORT, debug=DEBUG_MODE)
