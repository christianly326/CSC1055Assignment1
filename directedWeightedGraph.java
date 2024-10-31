import java.util.*;

class Edge {
    int target;
    int weight;

    public Edge(int target, int weight) {
        this.target = target;
        this.weight = weight;
    }
}

class Graph {
    private Map<Integer, List<Edge>> adjacencyList;
    private int[][] allPairsShortestPath;
    private int vertexCount;
    private static final int INF = Integer.MAX_VALUE;

    public Graph(int vertexCount) {
        this.vertexCount = vertexCount;
        this.adjacencyList = new HashMap<>();
        this.allPairsShortestPath = new int[vertexCount][vertexCount];

        // Initialize all-pairs shortest path matrix
        for (int i = 0; i < vertexCount; i++) {
            Arrays.fill(allPairsShortestPath[i], INF);
            allPairsShortestPath[i][i] = 0; // Distance to itself is zero
        }
    }

    // Add a new vertex
    public void addVertex(int vertex) {
        adjacencyList.putIfAbsent(vertex, new ArrayList<>());
    }

    // Add a directed arc with a specified weight
    public void addArc(int source, int target, int weight) {
        adjacencyList.computeIfAbsent(source, k -> new ArrayList<>()).add(new Edge(target, weight));
        allPairsShortestPath[source][target] = weight; // Update for Floyd-Warshall
    }

    // Dijkstra's algorithm for shortest path between two vertices
    public List<Integer> shortestPath(int source, int target) {
        Map<Integer, Integer> distances = new HashMap<>();
        Map<Integer, Integer> previousNodes = new HashMap<>();
        PriorityQueue<int[]> priorityQueue = new PriorityQueue<>(Comparator.comparingInt(a -> a[1]));

        for (int vertex : adjacencyList.keySet()) {
            distances.put(vertex, INF);
        }
        distances.put(source, 0);
        priorityQueue.add(new int[]{source, 0});

        while (!priorityQueue.isEmpty()) {
            int[] current = priorityQueue.poll();
            int currentVertex = current[0];
            int currentDistance = current[1];

            if (currentVertex == target) break;

            if (currentDistance > distances.get(currentVertex)) continue;

            for (Edge edge : adjacencyList.getOrDefault(currentVertex, Collections.emptyList())) {
                int neighbor = edge.target;
                int newDist = distances.get(currentVertex) + edge.weight;

                if (newDist < distances.get(neighbor)) {
                    distances.put(neighbor, newDist);
                    previousNodes.put(neighbor, currentVertex);
                    priorityQueue.add(new int[]{neighbor, newDist});
                }
            }
        }

        List<Integer> path = new ArrayList<>();
        if (distances.get(target) == INF) {
            System.out.println("No path found.");
            return path;
        }

        for (Integer at = target; at != null; at = previousNodes.get(at)) {
            path.add(at);
        }
        Collections.reverse(path);
        return path;
    }

    // Floyd-Warshall algorithm for all-pairs shortest paths
    public void allShortestPaths() {
        int[][] dist = new int[vertexCount][vertexCount];
        
        // Initialize distances based on adjacency matrix for Floyd-Warshall
        for (int i = 0; i < vertexCount; i++) {
            System.arraycopy(allPairsShortestPath[i], 0, dist[i], 0, vertexCount);
        }

        // Floyd-Warshall Algorithm
        for (int k = 0; k < vertexCount; k++) {
            for (int i = 0; i < vertexCount; i++) {
                for (int j = 0; j < vertexCount; j++) {
                    if (dist[i][k] != INF && dist[k][j] != INF && dist[i][k] + dist[k][j] < dist[i][j]) {
                        dist[i][j] = dist[i][k] + dist[k][j];
                    }
                }
            }
        }

        // Print all shortest paths
        for (int i = 0; i < vertexCount; i++) {
            for (int j = 0; j < vertexCount; j++) {
                if (dist[i][j] == INF) {
                    System.out.println("(" + i + "," + j + "): No path");
                } else {
                    System.out.println("(" + i + "," + j + "): " + dist[i][j]);
                }
            }
        }
    }

    public static void main(String[] args) {
        Graph graph = new Graph(4);
        graph.addArc(0, 1, 3);
        graph.addArc(0, 2, 10);
        graph.addArc(1, 2, 1);
        graph.addArc(2, 3, 2);

        System.out.println("Shortest Path from 0 to 3: " + graph.shortestPath(0, 3));
        System.out.println("All Shortest Paths:");
        graph.allShortestPaths();
    }
}
