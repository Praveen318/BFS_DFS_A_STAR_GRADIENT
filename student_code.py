from expand import expand
from queue import PriorityQueue

# def a_star_search(dis_map, time_map, start, end):
#     frontier = PriorityQueue()
#     frontier.put((0, 0, start))
#     came_from = {}
#     cost_so_far = {}
#     came_from[start] = None
#     cost_so_far[start] = 0
#     count = 0

#     while not frontier.empty():
#         current = frontier.get()[2]

#         if current == end:
#             break

#         for next in expand(current, time_map):
#             new_cost = cost_so_far[current] + time_map[current][next]
#             if next not in cost_so_far or new_cost < cost_so_far[next]:
#                 cost_so_far[next] = new_cost
#                 priority = new_cost + dis_map[next][end]
#                 count += 1
#                 frontier.put((priority, count, next))
#                 came_from[next] = current

#     path = []
#     node = end
#     while node != start:
#         path.append(node)
#         node = came_from[node]
#     path.append(start)
#     path.reverse()
#     return path

from queue import PriorityQueue
def a_star_search(dis_map, time_map, start, end):
    frontier = PriorityQueue()
    frontier.put((0, start))
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    count = 1
    while not frontier.empty():
        current = frontier.get()[1]
        if end == current:
            break
        counter = False
        children = expand(current, time_map)
        for next in children:
            new_cost = cost_so_far[current] + time_map[current][next]
            if (next not in cost_so_far or new_cost < cost_so_far[next]): # or (new_cost == cost_so_far[next] and dis_map[next][end] < hn):
                cost_so_far[next] = new_cost
                priority = new_cost + dis_map[next][end]
                frontier.put((priority, next))
                came_from[next] = current

    path = []
    node = end
    while node != start:
        path.append(node)
        node = came_from[node]
    path.append(start)
    path.reverse()
    # print(path)
    return path

# def a_star_search(dis_map, time_map, start, end):
#     frontier = [(0, start)]
#     came_from = {}
#     cost_so_far = {}
#     came_from[start] = None
#     cost_so_far[start] = 0

    # while len(frontier) > 0:
    #     current = min(frontier, key=lambda x: x[0])[1]
    #     print(current)
    #     frontier.remove((0,current))

    #     if current == end:
    #         break

    #     for next in expand(current, time_map):
    #         new_cost = cost_so_far[current] + time_map[current][next]
    #         if next not in cost_so_far or new_cost < cost_so_far[next]:
    #             cost_so_far[next] = new_cost
    #             priority = new_cost + dis_map[next][end]
    #             frontier.append((priority, next))
    #             came_from[next] = current

    # path = []
    # node = end
    # while node != start:
    #     path.append(node)
    #     node = came_from[node]
    # path.append(start)
    # path.reverse()
    # return path
	

# Right to Left Path
def depth_first_search(time_map, start, end):
    found = False
    stack = [start]
    parents = {}
    
    while stack:
        node = stack.pop()
        if node == end:
            found = True
            break
        
        children = expand(node, time_map)
            
        for child in reversed(children):
            stack.append(child)
            parents[child] = node
    
    path = [end]
    
    if found:
        # Traverse back through the parent nodes for the path
        while node != start:
            path.insert(0, parents[node])
            node = parents[node]
	    
    return path

# Left to Right path
# def depth_first_search(time_map, start, end):
#     found = False
#     stack = [start]
#     parents = {}
    
#     while stack:
#         node = stack.pop()
#         if node == end:
#             found = True
#             break
        
#         children = expand(node, time_map)
            
#         for child in children:
#             stack.append(child)
#             parents[child] = node
    
#     path = [end]
    
#     if found:
#         # Traverse back through the parent nodes for the path
#         while node != start:
#             path.insert(0, parents[node])
#             node = parents[node]
#     return path

def breadth_first_search(time_map, start, end):
	found = False
	visited = []
	queue = [start]
	parents = {}
	while queue:
		node = queue.pop(0)
		if node == end:
			found = True
			break
		if node not in visited:
			visited.append(node)
			children = expand(node, time_map)
			for child in children:
				if child not in visited:
					queue.append(child)
					parents[child] = node
	path = [end]
	if found == True:
		node = end
		# Traverse back through the parent nodes for the path
		while (node != start):
			path.insert(0, parents[node])
			node = parents[node]
	return path	
