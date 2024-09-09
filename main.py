from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
import json

# Define the data model
def create_data_model():
    data = {
        'distance_matrix': [
            [0, 68.262, 58.493, 234.233, 157.275, 230.117, 656.132, 108.679],
            [68.262, 0, 24.302, 166.294, 95.279, 162.127, 703.504, 68.121],
            [58.493, 24.302, 0, 178.583, 98.81, 174.631, 679.801, 55.353],
            [234.233, 166.294, 178.583, 0, 90.373, 4.425, 819.063, 153.952],
            [157.275, 95.279, 98.81, 90.373, 0, 87.391, 731.915, 63.876],
            [230.117, 162.127, 174.631, 4.425, 87.391, 0, 816.933, 150.779],
            [656.132, 703.504, 679.801, 819.063, 731.915, 816.933, 0, 673.805],
            [108.679, 68.121, 55.353, 153.952, 63.876, 150.779, 673.805, 0],
        ],
        'demands': {
            'weights': [0, 0.147, 0.050, 0.225, 0.037, 0.076, 0.258, 0.068],
            'volumes': [0, 270.175, 88.474, 567.91, 59.511, 123.723, 544.0, 149.786]
        },
        'vehicle_capacities': {'weights': [5500, 5000, 4500, 3500, 1200, 600],
                               'volumes': [777, 706, 635, 425, 265, 106]},
        'fixed_costs': [10000, 14000, 16000, 20000, 25000, 40000],
        'per_km_costs': [37.3333, 25.6667, 29.1667, 15.1667, 22.1667, 32.6667],
        'num_vehicles': 6,
        'depot': 0
    }
    return data

# Solve the VRP using OR-Tools
def solve_vrp():
    data = create_data_model()
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']), data['num_vehicles'], data['depot'])

    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return int(data['distance_matrix'][from_node][to_node] * 1000)  # Convert to integer for OR-Tools

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    def weight_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return int(data['demands']['weights'][from_node] * 1000)  # Convert to integer for OR-Tools

    weight_callback_index = routing.RegisterUnaryTransitCallback(weight_callback)
    routing.AddDimensionWithVehicleCapacity(
        weight_callback_index,
        0,
        [int(capacity * 1000) for capacity in data['vehicle_capacities']['weights']],  # vehicle maximum capacities
        True,
        'Weight'
    )

    def volume_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return int(data['demands']['volumes'][from_node] * 1000)  # Convert to integer for OR-Tools

    volume_callback_index = routing.RegisterUnaryTransitCallback(volume_callback)
    routing.AddDimensionWithVehicleCapacity(
        volume_callback_index,
        0,
        [int(capacity * 1000) for capacity in data['vehicle_capacities']['volumes']],  # vehicle maximum capacities
        True,
        'Volume'
    )

    # Set search parameters
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_parameters.time_limit.seconds = 30

    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)

    # Print the solution in JSON format
    if solution:
        solution_json = generate_solution_json(data, manager, routing, solution)
        print(json.dumps(solution_json, indent=4))
    else:
        print("No solution found!")

# Function to generate the solution in JSON format
def generate_solution_json(data, manager, routing, solution):
    solution_json = []
    total_distance = 0  # Initialize total distance

    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        route_weight = 0
        route_volume = 0
        route_distance = 0
        route = []
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_weight += data['demands']['weights'][node_index]
            route_volume += data['demands']['volumes'][node_index]
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
            route.append(node_index)

        # Include depot at the end
        route.append(manager.IndexToNode(index))

        # Calculate the total cost for the route
        route_cost = data['fixed_costs'][vehicle_id] + (route_distance / 1000) * data['per_km_costs'][vehicle_id]
        total_distance += route_distance

        # Create JSON entry
        solution_json.append({
            "route_id": vehicle_id + 1,
            "route_weight": route_weight,
            "route_volume": route_volume,
            "vehicle_id": vehicle_id,
            "vehicle_max_weight": data['vehicle_capacities']['weights'][vehicle_id],
            "vehicle_max_volume": data['vehicle_capacities']['volumes'][vehicle_id],
            "route_cost": route_cost
        })

    # Print the total distance of all routes
    print(f"Total distance of all routes: {total_distance / 1000:.2f} units")  # Convert to original units
    return solution_json

# Run the VRP solver
solve_vrp()
