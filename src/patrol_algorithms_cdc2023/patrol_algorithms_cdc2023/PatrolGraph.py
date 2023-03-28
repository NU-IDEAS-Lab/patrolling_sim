import networkx as nx

class PatrolGraph():
    ''' This reads a graph file of the format provided by
        https://github.com/davidbsp/patrolling_sim '''

    def __init__(self, filepath):
        self.graph = nx.Graph()
        self.loadFromFile(filepath)

    def loadFromFile(self, filepath: str):
        with open(filepath, "r") as file:
            self.graphDimension = int(file.readline())
            self.widthPixels = int(file.readline())
            self.heightPixels = int(file.readline())
            self.resolution = float(file.readline())
            self.offsetX = float(file.readline())
            self.offsetY = float(file.readline())

            for _ in range(self.graphDimension):
                file.readline()
                
                # Create the node.
                i = int(file.readline())
                self.graph.add_node(i,
                    pos = (int(file.readline()) * self.resolution + self.offsetX,
                        int(file.readline()) * self.resolution + self.offsetY)
                )
                
                # Create edges.
                numEdges = int(file.readline())
                for _ in range(numEdges):
                    j = int(file.readline())
                    direction = str(file.readline()) # not useful!
                    cost = int(file.readline())
                    self.graph.add_edge(i, j, weight = cost)