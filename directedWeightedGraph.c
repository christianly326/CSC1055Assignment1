#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <stdbool.h>

#define INF INT_MAX

// Structure for an edge in the adjacency list
typedef struct Edge {
    int destination;
    int weight;
    struct Edge* next;
} Edge;

// Structure for a vertex in the adjacency list
typedef struct Vertex {
    Edge* head;
} Vertex;

// Structure for the graph
typedef struct Graph {
    int vertexCount;
    Vertex* vertices;
} Graph;

// Function to create a new edge
Edge* createEdge(int destination, int weight) {
    Edge* newEdge = (Edge*)malloc(sizeof(Edge));
    newEdge->destination = destination;
    newEdge->weight = weight;
    newEdge->next = NULL;
    return newEdge;
}

// Function to create a graph with a given initial number of vertices
Graph* createGraph(int vertexCount) {
    Graph* graph = (Graph*)malloc(sizeof(Graph));
    graph->vertexCount = vertexCount;
    graph->vertices = (Vertex*)malloc(vertexCount * sizeof(Vertex));
    for (int i = 0; i < vertexCount; i++) {
        graph->vertices[i].head = NULL;
    }
    return graph;
}

// Function to add a vertex to the graph
void addVertex(Graph* graph) {
    graph->vertices = realloc(graph->vertices, (graph->vertexCount + 1) * sizeof(Vertex));
    graph->vertices[graph->vertexCount].head = NULL;
    graph->vertexCount++;
}

// Function to add a directed edge to the graph
void addArc(Graph* graph, int source, int destination, int weight) {
    Edge* newEdge = createEdge(destination, weight);
    newEdge->next = graph->vertices[source].head;
    graph->vertices[source].head = newEdge;
}

// Function to delete an edge from the graph
void deleteArc(Graph* graph, int source, int destination) {
    Edge* current = graph->vertices[source].head;
    Edge* prev = NULL;

    while (current != NULL) {
        if (current->destination == destination) {
            if (prev == NULL) {
                graph->vertices[source].head = current->next;
            } else {
                prev->next = current->next;
            }
            free(current);
            return;
        }
        prev = current;
        current = current->next;
    }
}

// Function to delete a vertex and all associated edges
void deleteVertex(Graph* graph, int vertex) {
    for (int i = 0; i < graph->vertexCount; i++) {
        deleteArc(graph, i, vertex);
    }
    free(graph->vertices[vertex].head);
    graph->vertices[vertex].head = NULL;
}

// Helper function to detect cycles using DFS
bool dfsCycle(Graph* graph, int vertex, bool visited[], bool recStack[]) {
    if (!visited[vertex]) {
        visited[vertex] = true;
        recStack[vertex] = true;

        Edge* edge = graph->vertices[vertex].head;
        while (edge != NULL) {
            if (!visited[edge->destination] && dfsCycle(graph, edge->destination, visited, recStack)) {
                return true;
            } else if (recStack[edge->destination]) {
                return true;
            }
            edge = edge->next;
        }
    }
    recStack[vertex] = false;
    return false;
}

// Function to check if the graph contains a cycle
bool cyclic(Graph* graph) {
    bool* visited = (bool*)malloc(graph->vertexCount * sizeof(bool));
    bool* recStack = (bool*)malloc(graph->vertexCount * sizeof(bool));

    for (int i = 0; i < graph->vertexCount; i++) {
        visited[i] = false;
        recStack[i] = false;
    }

    for (int i = 0; i < graph->vertexCount; i++) {
        if (dfsCycle(graph, i, visited, recStack)) {
            free(visited);
            free(recStack);
            return true;
        }
    }

    free(visited);
    free(recStack);
    return false;
}

// Function to perform Dijkstra's algorithm for the shortest path between two vertices
void shortestPath(Graph* graph, int start, int end) {
    int* dist = (int*)malloc(graph->vertexCount * sizeof(int));
    bool* visited = (bool*)malloc(graph->vertexCount * sizeof(bool));
    int* prev = (int*)malloc(graph->vertexCount * sizeof(int));

    for (int i = 0; i < graph->vertexCount; i++) {
        dist[i] = INF;
        visited[i] = false;
        prev[i] = -1;
    }

    dist[start] = 0;

    for (int i = 0; i < graph->vertexCount - 1; i++) {
        int minDist = INF;
        int u = -1;

        for (int j = 0; j < graph->vertexCount; j++) {
            if (!visited[j] && dist[j] <= minDist) {
                minDist = dist[j];
                u = j;
            }
        }

        if (u == -1) break;
        visited[u] = true;

        Edge* edge = graph->vertices[u].head;
        while (edge != NULL) {
            int v = edge->destination;
            int weight = edge->weight;

            if (!visited[v] && dist[u] != INF && dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                prev[v] = u;
            }
            edge = edge->next;
        }
    }

    if (dist[end] == INF) {
        printf("No path from %d to %d\n", start, end);
    } else {
        printf("Shortest path from %d to %d: ", start, end);
        for (int at = end; at != -1; at = prev[at]) {
            printf("%d ", at);
        }
        printf("\nCost: %d\n", dist[end]);
    }

    free(dist);
    free(visited);
    free(prev);
}

// Floyd-Warshall algorithm for all-pairs shortest paths
// Add a helper function to print the path between two vertices
void printPath(int** next, int u, int v) {
    if (next[u][v] == -1) {
        printf(" No paths");
        return;
    }
    printf(" %d", u + 1); // Adjust to 1-based indexing
    while (u != v) {
        u = next[u][v];
        printf(",%d", u + 1); 
    }
}

// Modify the Floyd-Warshall function to track paths
void allShortestPaths(Graph* graph) {
    int** dist = (int**)malloc(graph->vertexCount * sizeof(int*));
    int** next = (int**)malloc(graph->vertexCount * sizeof(int*));

    for (int i = 0; i < graph->vertexCount; i++) {
        dist[i] = (int*)malloc(graph->vertexCount * sizeof(int));
        next[i] = (int*)malloc(graph->vertexCount * sizeof(int));
        
        for (int j = 0; j < graph->vertexCount; j++) {
            dist[i][j] = (i == j) ? 0 : INF;
            next[i][j] = -1; // Initialize next array to -1
        }

        Edge* edge = graph->vertices[i].head;
        while (edge != NULL) {
            dist[i][edge->destination] = edge->weight;
            next[i][edge->destination] = edge->destination; // Set the next node in the path
            edge = edge->next;
        }
    }

    for (int k = 0; k < graph->vertexCount; k++) {
        for (int i = 0; i < graph->vertexCount; i++) {
            for (int j = 0; j < graph->vertexCount; j++) {
                if (dist[i][k] != INF && dist[k][j] != INF && dist[i][k] + dist[k][j] < dist[i][j]) {
                    dist[i][j] = dist[i][k] + dist[k][j];
                    next[i][j] = next[i][k]; // Update the path to use the intermediate vertex
                }
            }
        }
    }

    printf("All pairs shortest paths:\n");
    for (int i = 0; i < graph->vertexCount; i++) {
        for (int j = 0; j < graph->vertexCount; j++) {
            if (dist[i][j] == INF) {
                printf("(%d, %d): No paths\n", i + 1, j + 1);
            } else if (i != j) {
                printf("(%d, %d):", i + 1, j + 1);
                printPath(next, i, j);
                printf("\n");
            }
        }
    }

    for (int i = 0; i < graph->vertexCount; i++) {
        free(dist[i]);
        free(next[i]);
    }
    free(dist);
    free(next);
}


// Function to print the adjacency list of the graph
void printGraph(Graph* graph) {
    for (int i = 0; i < graph->vertexCount; i++) {
        Edge* edge = graph->vertices[i].head;
        printf("Vertex %d:", i + 1); // Adjusting to 1-based for readability
        while (edge != NULL) {
            printf(" -> %d(weight=%d)", edge->destination + 1, edge->weight);
            edge = edge->next;
        }
        printf("\n");
    }
}

// Function to free the memory allocated for the graph
void freeGraph(Graph* graph) {
    for (int i = 0; i < graph->vertexCount; i++) {
        Edge* edge = graph->vertices[i].head;
        while (edge != NULL) {
            Edge* temp = edge;
            edge = edge->next;
            free(temp);
        }
    }
    free(graph->vertices);
    free(graph);
}


int main() {
    // Initialize a graph with 5 vertices
    int vertices = 5;
    Graph* graph = createGraph(vertices);

    // Adding arcs with weights as specified
    printf("Adding edges to the graph:\n");
    addArc(graph, 0, 3, 30);  // Edge from vertex 1 to 4 with weight 30
    addArc(graph, 0, 4, 100); // Edge from vertex 1 to 5 with weight 100
    addArc(graph, 0, 1, 10);  // Edge from vertex 1 to 2 with weight 10
    addArc(graph, 1, 2, 50);  // Edge from vertex 2 to 3 with weight 50
    addArc(graph, 2, 4, 10);  // Edge from vertex 3 to 5 with weight 10
    addArc(graph, 3, 2, 20);  // Edge from vertex 4 to 3 with weight 20
    addArc(graph, 3, 4, 60);  // Edge from vertex 4 to 5 with weight 60

    // Display the adjacency list representation of the graph
    printf("Graph Adjacency List:\n");
    printGraph(graph);

    // Test Floyd-Warshall algorithm for all-pairs shortest paths
    printf("\nAll Shortest Paths using Floyd-Warshall:\n");
    allShortestPaths(graph);

    // Test Dijkstra's algorithm for shortest path between two vertices
    int start = 0; // Vertex 1 in 0-based indexing
    int end = 4;   // Vertex 5 in 0-based indexing
    printf("\nShortest path from %d to %d using Dijkstra's algorithm:\n", start + 1, end + 1);
    shortestPath(graph, start, end);

    // Add a new vertex and an edge
    printf("\nAdding a new vertex and edge:\n");
    addVertex(graph);
    addArc(graph, 5, 3, 15); // Edge from new vertex 6 to vertex 4 with weight 15
    printGraph(graph);

    // Test deletion of an arc
    printf("\nDeleting the edge from vertex 4 to 3:\n");
    deleteArc(graph, 3, 2); // Deleting edge from vertex 4 to 3
    printGraph(graph);

    // Test deletion of a vertex
    printf("\nDeleting vertex 3 and its edges:\n");
    deleteVertex(graph, 2); // Deleting vertex 3
    printGraph(graph);

    // Check if the graph has a cycle
    printf("\nChecking for cycles in the graph:\n");
    if (cyclic(graph)) {
        printf("The graph contains a cycle.\n");
    } else {
        printf("The graph does not contain a cycle.\n");
    }

    // Free the graph memory
    freeGraph(graph);

    return 0;
}

