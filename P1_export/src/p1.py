from p1_support import load_level, show_level, save_level_costs
from math import inf, sqrt
from heapq import heappop, heappush


def dijkstras_shortest_path(initial_position, destination, graph, adj):
    """ Searches for a minimal cost path through a graph using Dijkstra's algorithm.

    Args:
        initial_position: The initial cell from which the path extends.
        destination: The end location for the path.
        graph: A loaded level, containing walls, spaces, and waypoints.
        adj: An adjacency function returning cells adjacent to a given cell as well as their respective edge costs.

    Returns:
        If a path exits, return a list containing all cells from initial_position to destination.
        Otherwise, return None.
    """
    adject = adj(graph, initial_position)
    dij_dict = {initial_position:0}
    qheap = []
    for c in adject:
        heappush(qheap, c)
        dij_dict[c[0]] = c[1]
    while qheap:
        cell = heappop(qheap)
        if cell[0] == destination:
            break
        if cell[0] in graph['walls']:
            continue
        costs = adj(graph, cell[0])
        temp_dict = {(c[0], c[1] + dij_dict[cell[0]]) for c in costs if not c[0] in dij_dict or dij_dict[c[0]] > c[1] + dij_dict[cell[0]]}
        for val in temp_dict:
            dij_dict[val[0]] = val[1]
            heappush(qheap, val)
    if not destination in dij_dict:
        return
    curr = destination
    path = [destination]
    check = curr
    found = check
    while curr != initial_position:
        for y in range(3):
            for x in range (3):
                check = (curr[0] + (x-1), curr[1] + (y-1))
                if check in dij_dict and dij_dict[check] < dij_dict[curr] and dij_dict[check] < dij_dict[found]:
                    found = check
        path.insert(0,found)
        curr = found
    path.insert(0, initial_position)
    return path


def dijkstras_shortest_path_to_all(initial_position, graph, adj):
    """ Calculates the minimum cost to every reachable cell in a graph from the initial_position.

    Args:
        initial_position: The initial cell from which the path extends.
        graph: A loaded level, containing walls, spaces, and waypoints.
        adj: An adjacency function returning cells adjacent to a given cell as well as their respective edge costs.

    Returns:
        A dictionary, mapping destination cells to the cost of a path from the initial_position.
    """

    adject = adj(graph, initial_position)
    dij_dict = {initial_position:0}
    qheap = []
    for c in adject:
        heappush(qheap, c)
        dij_dict[c[0]] = c[1]
    while qheap:
        cell = heappop(qheap)
        if cell[0] in graph['walls']:
            continue
        costs = adj(graph, cell[0])
        temp_dict = {(c[0], c[1] + dij_dict[cell[0]]) for c in costs if not c[0] in dij_dict or dij_dict[c[0]] > c[1] + dij_dict[cell[0]]}
        for val in temp_dict:
            dij_dict[val[0]] = val[1]
            heappush(qheap, val)

    return dij_dict


def navigation_edges(level, cell):
    """ Provides a list of adjacent cells and their respective costs from the given cell.

    Args:
        level: A loaded level, containing walls, spaces, and waypoints.
        cell: A target location.

    Returns:
        A list of tuples containing an adjacent cell's coordinates and the cost of the edge joining it and the
        originating cell.

        E.g. from (0,0):
            [((0,1), 1),
             ((1,0), 1),
             ((1,1), 1.4142135623730951),
             ... ]
    """
    near_cells = [(x,y) for x in [cell[0] - 1,cell[0],cell[0] + 1] for y in [cell[1] - 1,cell[1],cell[1] + 1] if (x,y) != cell]
    near_cost = []
    sqrt2 = sqrt(2)*.5
    for c in near_cells:
        if c in level['spaces']:
            if c[0] != cell[0] and c[1] != cell[1]:
                cost = level['spaces'][cell]*sqrt2 + level['spaces'][c]*sqrt2
                near_cost.append((c, cost))
            else:
                cost = level['spaces'][cell]*0.5 + level['spaces'][c]*0.5
                near_cost.append((c, cost))
        elif c in level['walls']:
            cost = inf
            near_cost.append((c, cost))
    return near_cost



def test_route(filename, src_waypoint, dst_waypoint):
    """ Loads a level, searches for a path between the given waypoints, and displays the result.

    Args:
        filename: The name of the text file containing the level.
        src_waypoint: The character associated with the initial waypoint.
        dst_waypoint: The character associated with the destination waypoint.

    """

    # Load and display the level.
    level = load_level(filename)
    show_level(level)

    # Retrieve the source and destination coordinates from the level.
    src = level['waypoints'][src_waypoint]
    dst = level['waypoints'][dst_waypoint]

    # Search for and display the path from src to dst.
    path = dijkstras_shortest_path(src, dst, level, navigation_edges)
    if path:
        show_level(level, path)
    else:
        print("No path possible!")


def cost_to_all_cells(filename, src_waypoint, output_filename):
    """ Loads a level, calculates the cost to all reachable cells from
    src_waypoint, then saves the result in a csv file with name output_filename.

    Args:
        filename: The name of the text file containing the level.
        src_waypoint: The character associated with the initial waypoint.
        output_filename: The filename for the output csv file.

    """

    # Load and display the level.
    level = load_level(filename)
    show_level(level)

    # Retrieve the source coordinates from the level.
    src = level['waypoints'][src_waypoint]

    # Calculate the cost to all reachable cells from src and save to a csv file.
    costs_to_all_cells = dijkstras_shortest_path_to_all(src, level, navigation_edges)
    save_level_costs(level, costs_to_all_cells, output_filename)


if __name__ == '__main__':
    filename, src_waypoint, dst_waypoint = 'example.txt', 'a','d'

    # Use this function call to find the route between two waypoints.
    test_route(filename, src_waypoint, dst_waypoint)

    # Use this function to calculate the cost to all reachable cells from an origin point.
    #cost_to_all_cells(filename, src_waypoint, 'my_costs.csv')