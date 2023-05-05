class Anode():
    """A Anode class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end Anode
    start_Anode = Anode(None, start)
    start_Anode.g = start_Anode.h = start_Anode.f = 0
    end_Anode = Anode(None, end)
    end_Anode.g = end_Anode.h = end_Anode.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start Anode
    open_list.append(start_Anode)

    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current Anode
        current_Anode = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_Anode.f:
                current_Anode = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_Anode)

        # Found the goal
        if current_Anode == end_Anode:
            path = []
            current = current_Anode
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

            # Get Anode position
            Anode_position = (current_Anode.position[0] + new_position[0], current_Anode.position[1] + new_position[1])

            # Make sure within range
            if Anode_position[0] > (len(maze) - 1) or Anode_position[0] < 0 or Anode_position[1] > (len(maze[len(maze)-1]) -1) or Anode_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[Anode_position[0]][Anode_position[1]] != 0:
                continue

            # Create new Anode
            new_Anode = Anode(current_Anode, Anode_position)

            # Append
            children.append(new_Anode)

        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            child.g = current_Anode.g + 1
            child.h = ((child.position[0] - end_Anode.position[0]) ** 2) + ((child.position[1] - end_Anode.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            for open_Anode in open_list:
                if child == open_Anode and child.g > open_Anode.g:
                    continue

            # Add the child to the open list
            open_list.append(child)


def main():

    maze = [[0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

    start = (0, 0)
    end = (7, 6)

    path = astar(maze, start, end)
    print(path)


if __name__ == '__main__':
    main()