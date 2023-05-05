import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        self.start_pose = PoseStamped()
        self.goal_pose = PoseStamped()
        self.get_logger().info('Path planner node started')

        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.map_ = None
        
        self.tf_buffer = Buffer(self.get_clock())
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10)
        self.publisher = self.create_publisher(Path, '/path', 10)

    def goal_callback(self, msg):
        self.goal_pose = msg
        path_msg = self.compute_path()
        if path_msg = None:
            self.get_logger().info("Couldn't generate a path...")
        else:
            self.publisher.publish(path_msg)

    def map_callback(self, msg):
        self.map_ = msg  

    def get_map(self):
        while self.map_ is None:
            self.get_logger().info('Waiting for map...')
        return self.map_


    def compute_path(self):
        for i in range(10):
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map', 
                    'base_link',       
                    self.get_clock().now(), 
                    timeout=rclpy.duration.Duration(seconds=1.0)) 
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn('Failed to get transform from map to base_link: {}'.format(e))
            
            return

        
        self.start_pose.pose.position.x = transform.transform.translation.x
        self.start_pose.pose.position.y = transform.transform.translation.y
        self.start_pose.pose.orientation = transform.transform.rotation

        
        start = (self.start_pose.pose.position.x, self.start_pose.pose.position.y)
        goal = (self.goal_pose.pose.position.x, self.goal_pose.pose.position.y)

        
        path = astar_path(start, goal, self.get_map())


        if path = None:
            return None

            
        if len(path) < 2:
            return None

        
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        for p in path:
            pose = PoseStamped()
            pose.header.stamp = path_msg.header.stamp
            pose.header.frame_id = path_msg.header.frame_id
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)

        return path_msg        


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

def get_pose(map, pose):
    o_x = abs(map.info.origin.position.x)
    o_y = abs(map.info.origin.position.y)

    height = map.info.height
    width = map.info.width
    resol = map.info.resolution
    i = height - 1 - int((o_y + pose[1])/resol)
    j = int((o_x + pose[0])/resol)
    return (i,j)


def generate_maze(map):
    height = map.info.height
    width = map.info.width
    data = map.data
    thresh = 50
    maze = []
    for i in range(height):
        l = []
        for j in range(i*width, width*(i+1)):
            if data[j] == -1 or data[j] >= thresh :
                l.append(1)
            else:
                l.append(0)
        maze.append(l)
    return maze

def astar_path(start,goal,map):
    maze = generate_maze(map)

    return astar(maze,get_pose(map, start),get_pose(map, goal))


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

    i = 0

    # Loop until you find the end
    while len(open_list) > 0 and i <= 10000:

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

        i += 1


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()