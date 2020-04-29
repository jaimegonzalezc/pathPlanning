import path_planning as pp
import math

def children(point,grid):
    """
        Calculates the children of a given node over a grid.
        Inputs:
            - point: node for which to calculate children.
            - grid: grid over which to calculate children.
        Outputs:
            - list of children for the given node.
    """
    x,y = point.grid_point
    if x > 0 and x < len(grid) - 1:
        if y > 0 and y < len(grid[0]) - 1:
            links = [grid[d[0]][d[1]] for d in\
                     [(x-1, y),(x,y - 1),(x,y + 1),(x+1,y),\
                      (x-1, y-1), (x-1, y+1), (x+1, y-1),\
                      (x+1, y+1)]]
        elif y > 0:
            links = [grid[d[0]][d[1]] for d in\
                     [(x-1, y),(x,y - 1),(x+1,y),\
                      (x-1, y-1), (x+1, y-1)]]
        else:
            links = [grid[d[0]][d[1]] for d in\
                     [(x-1, y),(x,y + 1),(x+1,y),\
                      (x-1, y+1), (x+1, y+1)]]
    elif x > 0:
        if y > 0 and y < len(grid[0]) - 1:
            links = [grid[d[0]][d[1]] for d in\
                     [(x-1, y),(x,y - 1),(x,y + 1),\
                      (x-1, y-1), (x-1, y+1)]]
        elif y > 0:
            links = [grid[d[0]][d[1]] for d in\
                     [(x-1, y),(x,y - 1),(x-1, y-1)]]
        else:
            links = [grid[d[0]][d[1]] for d in\
                     [(x-1, y), (x,y + 1), (x-1, y+1)]]
    else:
        if y > 0 and y < len(grid[0]) - 1:
            links = [grid[d[0]][d[1]] for d in\
                     [(x+1, y),(x,y - 1),(x,y + 1),\
                      (x+1, y-1), (x+1, y+1)]]
        elif y > 0:
            links = [grid[d[0]][d[1]] for d in\
                     [(x+1, y),(x,y - 1),(x+1, y-1)]]
        else:
            links = [grid[d[0]][d[1]] for d in\
                     [(x+1, y), (x,y + 1), (x+1, y+1)]]
    return [link for link in links if link.value != 9]

openset = set()

def thetaStar(start, goal, grid, heuristic):
    """
    start.G = 0
    start.H = pp.heuristic[heuristic](start,goal)
    start.parent = start
    abiertos = set()
    abiertos.add(start, start.G + start.H)
    """
    closedset = set()
    current = start #!!!!!
    openset.add(start)
    while openset:
        current = min(openset, key=lambda o: o.G + o.H)
        pp.expanded_nodes += 1
        if current == goal:
            path = []
            while current.parent:
                path.append(current)
                current = current.parent
            path.append(current)
            return path[::-1]
        openset.remove(current)
        closedset.add(current)
        for node in children(current, grid):
            if node in closedset:
                continue
            if node in openset:
                if lineOfSight(current, node, grid):
                    new_g = current.parent.G + current.parent.move_cost(node)
                    if node.G > new_g:
                        node.G = new_g
                        node.parent = current.parent
                else:
                    new_g = current.G + current.move_cost(node)
                    if node.G > new_g:
                        node.G = new_g
                        node.parent = current
            else:
                node.G = current.G + current.move_cost(node)
                node.H = pp.heuristic[heuristic](node, goal)
                node.parent = current
                openset.add(node)
    raise ValueError('No Path Found')

pp.register_search_method('Theta*', thetaStar)

def lineOfSight(current, node, grid):
    x0, y0 = current.grid_point
    x1, y1 = node.grid_point
    dy = y1 - y0
    dx = x1 - x0
    sx = 0
    sy = 0
    f = 0
    if dy < 0:
        dy = -dy
        sy = -1
    else:
        sy = 1
    if dx < 0:
        dx = -dx
        sx = -1
    else:
        sx = 1
    if dx >= dy:
        while x0 != x1:
            f += dy
            if f >= dx:
                if grid(x0 + ((sx - 1) / 2), y0 + ((sy - 1) / 2)):
                    return False
                y0 += sy
                f -= dx
            if f != 0 and grid(x0 + ((sx - 1) / 2), y0 + ((sy - 1) / 2)):
                return False
            if dy == 0 and grid(x0 + ((sx - 1) / 2), y0) and grid(x0 + ((sx - 1) / 2), y0 - 1):
                return False
            x0 += sx
    else:
        while y0 != y1:
            f += dx
            if f >= dy:
                if grid(x0 + ((sx - 1) / 2), y0 + ((sy - 1) / 2)):
                    return False
                x0 += sx
                f -= dy
            if f != 0 and grid(x0 + ((sx - 1) / 2), y0 + ((sy - 1) / 2)):
                return False
            if dx == 0 and grid(x0 + ((sx - 1) / 2), y0) and grid(x0 + ((sx - 1) / 2), y0 - 1):
                return False
            y0 += sy
    return True

def euclideanDistance(p1, p2):
    x1, y1 = p1.grid_point
    x2, y2 = p2.grid_point
    return math.sqrt(pow(x2-x1, 2) + pow(y2-y1, 2))

