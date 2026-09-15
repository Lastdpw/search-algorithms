from collections import deque
import heapq
from itertools import count


graph = {
    "Үй": [
        ("Дүкен", 3),
        ("Аурухана", 1),
        ("Автовокзал", 2)
    ],

    "Дүкен": [
        ("Полиция", 4),
        ("Мектеп", 2)
    ],

    "Аурухана": [
        ("Мектеп", 2),
        ("Саябақ", 6)
    ],

    "Автовокзал": [
        ("Қойма", 2),
        ("Метро", 5)
    ],

    "Полиция": [
        ("Көпір", 5)
    ],

    "Мектеп": [
        ("Қойма", 2),
        ("Зертхана", 4)
    ],

    "Саябақ": [
        ("Метро", 2)
    ],

    "Қойма": [
        ("Көпір", 2),
        ("Зертхана", 2)
    ],

    "Метро": [
        ("Көпір", 1)
    ],

    "Көпір": [
        ("Бункер", 2)
    ],

    "Зертхана": [
        ("Бункер", 1)
    ],

    "Бункер": []
}


# A* үшін мақсатқа дейінгі болжамды құн
heuristic = {
    "Үй": 7,
    "Дүкен": 7,
    "Аурухана": 7,
    "Автовокзал": 5,
    "Полиция": 7,
    "Мектеп": 5,
    "Саябақ": 5,
    "Қойма": 3,
    "Метро": 3,
    "Көпір": 2,
    "Зертхана": 1,
    "Бункер": 0
}


# DFS
def dfs(graph, current, goal, visited=None, path=None):
    if visited is None:
        visited = []

    if path is None:
        path = []

    visited.append(current)
    path.append(current)

    if current == goal:
        return path, visited

    for neighbor, cost in graph[current]:
        if neighbor not in visited:
            result = dfs(
                graph,
                neighbor,
                goal,
                visited,
                path.copy()
            )

            if result[0] is not None:
                return result

    return None, visited


# BFS
def bfs(graph, start, goal):
    queue = deque([(start, [start])])
    visited = {start}
    visit_order = []

    while queue:
        current, path = queue.popleft()
        visit_order.append(current)

        if current == goal:
            return path, visit_order

        for neighbor, cost in graph[current]:
            if neighbor not in visited:
                visited.add(neighbor)
                queue.append(
                    (neighbor, path + [neighbor])
                )

    return None, visit_order


# UCS
def ucs(graph, start, goal):
    queue = []
    order = count()

    heapq.heappush(
        queue,
        (0, next(order), start, [start])
    )

    best_cost = {start: 0}
    visit_order = []

    while queue:
        cost, _, current, path = heapq.heappop(queue)

        if cost > best_cost[current]:
            continue

        visit_order.append(current)

        if current == goal:
            return path, cost, visit_order

        for neighbor, edge_cost in graph[current]:
            new_cost = cost + edge_cost

            if (neighbor not in best_cost or
                    new_cost < best_cost[neighbor]):

                best_cost[neighbor] = new_cost

                heapq.heappush(
                    queue,
                    (
                        new_cost,
                        next(order),
                        neighbor,
                        path + [neighbor]
                    )
                )

    return None, float("inf"), visit_order


# A*
def astar(graph, heuristic, start, goal):
    queue = []
    order = count()

    heapq.heappush(
        queue,
        (heuristic[start], 0, next(order), start, [start])
    )

    best_cost = {start: 0}
    visit_order = []

    while queue:
        f, g, _, current, path = heapq.heappop(queue)

        if g > best_cost[current]:
            continue

        visit_order.append(current)

        if current == goal:
            return path, g, visit_order

        for neighbor, edge_cost in graph[current]:
            new_g = g + edge_cost
            new_f = new_g + heuristic[neighbor]

            if (neighbor not in best_cost or
                    new_g < best_cost[neighbor]):

                best_cost[neighbor] = new_g

                heapq.heappush(
                    queue,
                    (
                        new_f,
                        new_g,
                        next(order),
                        neighbor,
                        path + [neighbor]
                    )
                )

    return None, float("inf"), visit_order


# Алгоритмдерді іске қосу
start = "Үй"
goal = "Бункер"


dfs_path, dfs_visited = dfs(graph, start, goal)
bfs_path, bfs_visited = bfs(graph, start, goal)
ucs_path, ucs_cost, ucs_visited = ucs(graph, start, goal)

astar_path, astar_cost, astar_visited = astar(
    graph,
    heuristic,
    start,
    goal
)


print("DFS")
print("Қаралу реті:", " → ".join(dfs_visited))
print("Табылған жол:", " → ".join(dfs_path))


print("\n BFS")
print("Қаралу реті:", " → ".join(bfs_visited))
print("Табылған жол:", " → ".join(bfs_path))
print("Қадам саны:", len(bfs_path) - 1)


print("\n UCS ")
print("Қаралу реті:", " → ".join(ucs_visited))
print("Табылған жол:", " → ".join(ucs_path))
print("Қауіп құны:", ucs_cost)


print("\n A* ")
print("Қаралу реті:", " → ".join(astar_visited))
print("Табылған жол:", " → ".join(astar_path))
print("Қауіп құны:", astar_cost)
