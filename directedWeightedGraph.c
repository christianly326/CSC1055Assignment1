#include <stdio.h>
#include <limits.h>
#include <stdbool.h>

#define MAX_VERTICES 100
#define INF INT_MAX

int adjMatrix[MAX_VERTICES][MAX_VERTICES];
int vertexCount;

// Initialize the adjacency matrix
void initGraph(int vertices) {
    vertexCount = vertices;
    for (int i = 0; i < vertexCount; i++) {
        for (int j = 0; j < vertexCount; j++) {
            adjMatrix[i][j] = (i == j) ? 0 : INF;  // 0 for self-loops, INF for other edges
        }
    }
}

// Add a directed arc with a specified weight
void addArc(int source, int target, int weight) {
    adjMatrix[source][target] = weight;
}

// Find the vertex with minimum distance that is not yet processed
int minDistance(int dist[], bool sptSet[]) {
    int min = INF, minIndex = -1;
    for (int v = 0; v < vertexCount; v++) {
        if (!sptSet[v] && dist[v] <= min) {
            min = dist[v];
            minIndex = v;
        }
    }
    return minIndex;
}

// Dijkstra's algorithm for shortest path from a source to a target
void shortestPath(int source, int target) {
    int dist[vertexCount];
    bool sptSet[vertexCount];
    int parent[vertexCount];

    for (int i = 0; i < vertexCount; i++) {
        dist[i] = INF;
        sptSet[i] = false;
        parent[i] = -1;
    }
    dist[source] = 0;

    for (int count = 0; count < vertexCount - 1; count++) {
        int u = minDistance(dist, sptSet);
        sptSet[u] = true;

        for (int v = 0; v < vertexCount; v++) {
            if (!sptSet[v] && adjMatrix[u][v] != INF && dist[u] != INF && 
                dist[u] + adjMatrix[u][v] < dist[v]) {
                dist[v] = dist[u] + adjMatrix[u][v];
                parent[v] = u;
            }
        }
    }

    // Print the shortest path from source to target
    if (dist[target] == INF) {
        printf("No path found from %d to %d\n", source, target);
    } else {
        printf("Shortest path from %d to %d: ", source, target);
        int path[MAX_VERTICES];
        int pathIndex = 0;
        for (int v = target; v != -1; v = parent[v]) {
            path[pathIndex++] = v;
        }
        for (int i = pathIndex - 1; i >= 0; i--) {
            printf("%d ", path[i]);
        }
        printf("\n");
    }
}

// Floyd-Warshall algorithm for all-pairs shortest paths
void allShortestPaths() {
    int dist[vertexCount][vertexCount];

    // Initialize distances based on adjacency matrix
    for (int i = 0; i < vertexCount; i++) {
        for (int j = 0; j < vertexCount; j++) {
            dist[i][j] = adjMatrix[i][j];
        }
    }

    // Floyd-Warshall Algorithm
    for (int k = 0; k < vertexCount; k++) {
        for (int i = 0; i < vertexCount; i++) {
            for (int j = 0; j < vertexCount; j++) {
                if (dist[i][k] != INF && dist[k][j] != INF && 
                    dist[i][k] + dist[k][j] < dist[i][j]) {
                    dist[i][j] = dist[i][k] + dist[k][j];
                }
            }
        }
    }

    // Print all-pairs shortest paths
    printf("All pairs shortest paths:\n");
    for (int i = 0; i < vertexCount; i++) {
        for (int j = 0; j < vertexCount; j++) {
            if (dist[i][j] == INF) {
                printf("(%d, %d): No path\n", i, j);
            } else {
                printf("(%d, %d): %d\n", i, j, dist[i][j]);
            }
        }
    }
}

int main() {
    int vertices = 4;
    initGraph(vertices);

    // Adding arcs with weights
    addArc(0, 1, 3);
    addArc(0, 2, 10);
    addArc(1, 2, 1);
    addArc(2, 3, 2);

    // Find the shortest path from vertex 0 to vertex 3
    printf("Shortest Path from 0 to 3:\n");
    shortestPath(0, 3);

    // Print all shortest paths between all pairs
    printf("\nAll Shortest Paths:\n");
    allShortestPaths();

    return 0;
}
