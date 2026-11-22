import heapq
import math

# Pakistan Inter-City Route Planner
# Cities and road distances in kilometers

roads = {
    "Lahore": {"Islamabad": 375, "Multan": 340, "Faisalabad": 180, "Gujranwala": 80},
    "Islamabad": {"Lahore": 375, "Peshawar": 185, "Rawalpindi": 20, "Abbottabad": 120},
    "Rawalpindi": {"Islamabad": 20, "Abbottabad": 115, "Murree": 65},
    "Peshawar": {"Islamabad": 185, "Mardan": 65, "Kohat": 70},
    "Mardan": {"Peshawar": 65},
    "Kohat": {"Peshawar": 70},
    "Abbottabad": {"Islamabad": 120, "Rawalpindi": 115, "Murree": 65},
    "Murree": {"Rawalpindi": 65, "Abbottabad": 65},
    "Multan": {"Lahore": 340, "Bahawalpur": 100, "Faisalabad": 245, "Sukkur": 430},
    "Faisalabad": {"Lahore": 180, "Multan": 245, "Sargodha": 95, "Gujranwala": 150},
    "Gujranwala": {"Lahore": 80, "Faisalabad": 150, "Sialkot": 55},
    "Sialkot": {"Gujranwala": 55},
    "Sargodha": {"Faisalabad": 95},
    "Bahawalpur": {"Multan": 100, "Sukkur": 300},
    "Sukkur": {"Multan": 430, "Bahawalpur": 300, "Hyderabad": 330},
    "Hyderabad": {"Sukkur": 330, "Karachi": 165},
    "Karachi": {"Hyderabad": 165}
}


# Uniform Cost Search
def ucs(start, goal):
    queue = [(0, start, [start])]
    visited = set()

    while queue:
        cost, city, path = heapq.heappop(queue)

        if city == goal:
            return path, cost

        if city in visited:
            continue

        visited.add(city)

        for neighbor, distance in roads.get(city, {}).items():
            if neighbor not in visited:
                heapq.heappush(
                    queue,
                    (cost + distance, neighbor, path + [neighbor])
                )

    return None, float("inf")


# Greedy Best First Search
def greedy(start, goal):
    queue = [(0, start, [start])]
    visited = set()

    while queue:
        _, city, path = heapq.heappop(queue)

        if city == goal:
            total_distance = calculate_distance(path)
            return path, total_distance

        if city in visited:
            continue

        visited.add(city)

        for neighbor, distance in roads.get(city, {}).items():
            if neighbor not in visited:
                heapq.heappush(
                    queue,
                    (distance, neighbor, path + [neighbor])
                )

    return None, float("inf")


# Calculate distance of a route
def calculate_distance(path):
    total = 0

    for i in range(len(path) - 1):
        total += roads[path[i]][path[i + 1]]

    return total


# A* Search
def astar(start, goal):
    queue = [(0, 0, start, [start])]
    visited = set()

    while queue:
        f_cost, g_cost, city, path = heapq.heappop(queue)

        if city == goal:
            return path, g_cost

        if city in visited:
            continue

        visited.add(city)

        for neighbor, distance in roads.get(city, {}).items():
            if neighbor not in visited:
                new_g = g_cost + distance

                # Simple heuristic
                h = 0

                new_f = new_g + h

                heapq.heappush(
                    queue,
                    (new_f, new_g, neighbor, path + [neighbor])
                )

    return None, float("inf")


# Display route
def show_route(name, path, distance):
    print("\n", name)
    print("-" * 40)

    if path:
        print("Route:", " -> ".join(path))
        print("Total Distance:", distance, "km")
    else:
        print("No route found.")


# Main Program
def main():
    print("=" * 50)
    print("   PAKISTAN INTER-CITY ROUTE PLANNER")
    print("=" * 50)

    cities = sorted(roads.keys())

    print("\nAvailable Cities:")
    for city in cities:
        print(city)

    start = input("\nEnter starting city: ").strip().title()
    goal = input("Enter destination city: ").strip().title()

    if start not in roads or goal not in roads:
        print("\nInvalid city name!")
        return

    if start == goal:
        print("\nStart and destination are the same city.")
        print("Distance: 0 km")
        return

    # Run algorithms
    ucs_path, ucs_distance = ucs(start, goal)
    greedy_path, greedy_distance = greedy(start, goal)
    astar_path, astar_distance = astar(start, goal)

    # Display results
    show_route("Uniform Cost Search", ucs_path, ucs_distance)
    show_route("Greedy Best First Search", greedy_path, greedy_distance)
    show_route("A* Search", astar_path, astar_distance)

    # Compare results
    print("\n" + "=" * 50)
    print("           ROUTE COMPARISON")
    print("=" * 50)

    print("UCS Distance:", ucs_distance, "km")
    print("Greedy Distance:", greedy_distance, "km")
    print("A* Distance:", astar_distance, "km")

    if ucs_path:
        print("\nShortest Route Found:")
        print(" -> ".join(ucs_path))
        print("Distance:", ucs_distance, "km")


if __name__ == "__main__":
    main()