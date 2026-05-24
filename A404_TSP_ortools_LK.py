from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use("QtAgg")

# 1. Generate test city coordinates
np.random.seed(4)
city_count = 150
cities = np.random.randint(0, 100, size=(city_count, 2))


# 2. Create distance matrix
def create_distance_matrix(points):
    dist_matrix = []
    for i in range(len(points)):
        row = []
        for j in range(len(points)):
            dist = np.hypot(points[i][0] - points[j][0], points[i][1] - points[j][1])
            row.append(int(dist))
        dist_matrix.append(row)
    return dist_matrix


distance_matrix = create_distance_matrix(cities)


# 3. Create and solve TSP model
def solve_tsp(dist_matrix):
    size = len(dist_matrix)
    # Number of vehicles and starting point
    num_vehicles = 1
    start_index = 0
    manager = pywrapcp.RoutingIndexManager(size, num_vehicles, start_index)
    routing = pywrapcp.RoutingModel(manager)

    # Distance callback function
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return dist_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Set search strategy LK
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    solution = routing.SolveWithParameters(search_parameters)

    # Extract optimal path
    path = []
    index = routing.Start(0)
    while not routing.IsEnd(index):
        path.append(manager.IndexToNode(index))
        index = solution.Value(routing.NextVar(index))
    path.append(manager.IndexToNode(routing.End(0)))

    total_distance = solution.ObjectiveValue()
    return path, total_distance


# 4. Plot the route
def plot_route(path, city_locations, title):
    plt.figure(figsize=(6, 5))
    x = [city_locations[p][0] for p in path]
    y = [city_locations[p][1] for p in path]
    plt.plot(x, y, 'o-', color='#2E86AB')
    plt.title(title)
    plt.show()


# 5. Run solver
optimal_path, total_travel_distance = solve_tsp(distance_matrix)

# Display results
print("=== OR-Tools TSP Result ===")
print(f"Total travel distance: {total_travel_distance}")
print(f"Optimal path order: {optimal_path}")
plot_route(optimal_path, cities, "OR-Tools Optimized Route")