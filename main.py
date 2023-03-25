from flask import *
import heapq

app = Flask(__name__)

graph = {
    "a": {"b": 1, "f": 1},
    "b": {"a": 1, "c": 2},
    "c": {"b":2, "d": 2},
    "d": {"c": 2, "e": 3},
    "e": {"d": 3, "f": 2},
    "f": {"e": 2, "a": 1},
}


def get_optimal_order(graph, nodes):
    # Find the minimum distances from each node in the list to all other nodes
    distances = {}
    for node in nodes:
        distances[node] = dijkstra(graph, node)

    # Construct a minimum spanning tree of the graph using Prim's algorithm
    visited = set()
    edges = []
    start_node = nodes[0]
    visited.add(start_node)
    for _ in range(len(nodes) - 1):
        min_edge = None
        for node in visited:
            for neighbor, distance in distances[node].items():
                if neighbor not in visited:
                    if min_edge is None or distance < min_edge[2]:
                        min_edge = (node, neighbor, distance)
        edges.append(min_edge)
        visited.add(min_edge[1])

    # Perform a depth-first traversal of the tree to obtain the optimal order
    optimal_order = []

    def dfs(node):
        optimal_order.append(node)
        for edge in edges:
            if edge[0] == node:
                dfs(edge[1])

    dfs(start_node)
    return optimal_order

def dijkstra(graph, start):
    # Create a dictionary to store the minimum distance to each node
    distances = {node: float("inf") for node in graph}
    distances[start] = 0

    # Create a priority queue to store nodes to visit, ordered by distance
    pq = [(0, start)]

    while pq:
        # Extract the node with the smallest distance from the priority queue
        current_distance, current_node = heapq.heappop(pq)

        # If the smallest distance to the next node is already known, skip this node
        if current_distance > distances[current_node]:
            continue

        # For each neighboring node, calculate the distance to that node and update the minimum distance if it's smaller
        for neighbor, distance in graph[current_node].items():
            total_distance = current_distance + distance
            if total_distance < distances[neighbor]:
                distances[neighbor] = total_distance
                heapq.heappush(pq, (total_distance, neighbor))

    return distances

# base route
@app.route('/', methods=['GET'])
def home_page():
    return ""


@app.route('/PATH', methods=['GET'])
def get_shortest_paths():
    #Get the list of nodes to visit from the query parameters
    nodes = request.args.getlist("node")
    # Call Dijkstra's algorithm to find the minimum distances to all other nodes from each node in the list
    distances = {}
    for node in nodes:
        distances[node] = dijkstra(graph, node)
    # Use the optimal order function to find the best order to visit the nodes
    optimal_order = get_optimal_order(graph, nodes)
    # Return the optimal order as a JSON object
    return jsonify(optimal_order)





if __name__ == '__main__':
    app.run(port=8080)
