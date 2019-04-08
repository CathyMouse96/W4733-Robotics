from collections import defaultdict
from heapq import heapify, heappush, heappop
import matplotlib.pyplot as plt

def manhattan_dist(p1, p2):
    """ Returns the manhattan distance between two points """
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** .5

def dijkstra(start, goal, adj_list):
    visited, dist, prev, heap = set(), dict(), dict(), []
    for vertex in adj_list.keys():
        if vertex == start:
            dist[vertex] = 0
            prev[vertex] = None
            heap.append((0, vertex))
        else:
            dist[vertex] = float('inf')
            prev[vertex] = None
            heap.append((float('inf'), vertex))
    heapify(heap)

    while heap:
        _, vertex = heappop(heap)
        if vertex in visited:
            continue
        elif vertex == goal: # found min distance to goal, no need to continue
            break
        visited.add(vertex)
        for neighbor, edge_len in adj_list[vertex]:
            if neighbor in visited:
                continue
            elif dist[vertex] + edge_len < dist[neighbor]:
                dist[neighbor] = dist[vertex] + edge_len
                prev[neighbor] = vertex
                heappush(heap, (dist[neighbor], neighbor))
    
    # Recover the path
    path, curr = [], goal
    while curr:
        path.append(curr)
        curr = prev[curr]
    
    return path[::-1], dist[goal]

def shortest_path():
    """ Return the shortest path from start to goal """
    edges = [((1, 2), (3, 4)), ((3, 4), (2, 4)), ((1, 2), (5, 6)), ((2, 4), (5, 6))]
    start, goal = (1, 2), (5, 6)
    
    # Compute adjacency list
    adj_list = defaultdict(list)
    for edge in edges:
        edge_len = manhattan_dist(edge[0], edge[1])
        adj_list[edge[0]].append((edge[1], edge_len))
        adj_list[edge[1]].append((edge[0], edge_len))
    # print(adj_list)
    
    return dijkstra(start, goal, adj_list)

def main():
    points, path_len = shortest_path()
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    plt.plot(xs, ys, 'b--', lw=2)
    plt.show()

if __name__ == "__main__":
    main()
