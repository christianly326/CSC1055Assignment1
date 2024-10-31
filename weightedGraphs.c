#include <stdio.h>
#include <limits.h>
#include <stdbool.h>

#define MAX_VERTICES 100
#define INF INT_MAX

int adjMatrix[MAX_VERTICES][MAX_VERTICES];
int vertexCount;

void initGraph(int vertices) {
    vertexCount = vertices;
    for (int i = 0; i < vertexCount; i++) {
        for (int j = 0; j < vertexCount; j++) {
            adjMatrix[i][j] = INF;
        }
    }
}

void addArc(int src, int dest, int weight) {
    adjMatrix[src][dest] = weight;
}

int minDistance(int dist[], bool sptSet[]) {
    int min = INF, minIndex;
    for (int v = 0; v < vertexCount; v++) {
        if (!sptSet[v] && dist[v] <= min) {
            min = dist[v], minIndex = v;
        }
    }
    return minIndex;
}

void dijkstra(int src, int target) {
    int dist[vertexCount];
    bool sptSet[vertexCount];
    int parent[vertexCount];

    for (int i = 0; i < vertexCount; i++) {
        dist[i] = INF;
        sptSet[i] = false;
        parent[i] = -1;
    }

    dist[src] = 0;

    for (int count = 0; count < vertexCount - 1; count++) {
        int u = minDistance(dist, sptSet);
        sptSet[u] = true;

        for (int v = 0; v < vertexCount; v++) {
            if (!sptSet[v] && adjMatrix[u][v] != INF &&
                dist[u] != INF && dist[u] + adjMatrix[u][v] < dist[v]) {
                dist[v] = dist[u] + adjMatrix[u][v];
                parent[v] = u;
            }
        }
    }

    if (dist[target] == INF) {
        printf("No path found.\n");
    } else {
        printf("Shortest path from %d to %d is: ", src, target);
        int v = target;
        int path[MAX_VERTICES];
        int path_index = 0;
        while (v != -1) {
            path[path_index++] = v;
            v = parent[v];
        }
        for (int i = path_index - 1; i >= 0; i--) {
            printf("%d ", path[i]);
        }
        printf("\n");
    }
}

int main() {
    int vertices = 5;
    initGraph(vertices);
    addArc(0, 1, 10);
    addArc(0, 2, 3);
    addArc(1, 2, 1);
    addArc(1, 3, 2);
    addArc(2, 1, 4);
    addArc(2, 3, 8);
    addArc(2, 4, 2);
    addArc(3, 4, 7);
    addArc(4, 3, 9);

    int src = 0;
    int target = 3;
    dijkstra(src, target);

    return 0;
}
