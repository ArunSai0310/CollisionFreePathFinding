"""
****Arun Sai Sangawar Vijay**
*********800890154***********
"""
__author__ = 'ArunSai'
import sys

# Specifies the attributes for a vertex in the graph.
class Vertex(object):

    def __init__(self,nm,pt):
        self.name = nm
        self.AdjVertices = []
        self.previousVertex = None
        self.point = pt
        self.dist = sys.maxint

    def reset(self):
        self.dist = sys.maxint
        self.previousVertex = None


# Graph class for evaluating shortest paths.
class Graph(object):

    def __init__(self):
        self.vertexMap = {}
        self.path = []

    # Adds a new edge to the graph.
    def addEdge(self,source,srcpoint,destination,destpoint):
        v = self.getVertex(source,srcpoint)
        w = self.getVertex(destination,destpoint)
        v.AdjVertices.append(w)

    # If vertex i.e. point is not present, add it to vertexMap and return the Vertex.
    def getVertex(self,vertexName,point):
        if not self.vertexMap.has_key(point):
            v = Vertex(vertexName,point)
            self.vertexMap[point] = v
            return v
        v = self.vertexMap[point]
        return v


    # It calls recursive routine to find shortest path from
    # destination Node to source after a shortest path method has run.
    def printShortPath(self,point):

        if not self.vertexMap.has_key(point):
            print("Destination Vertex Not Found")
            return
        v = self.vertexMap[point]
        if v.dist == sys.maxint:
            print("%r is not reachable "%v.point)
            return
        else:
            # print("Distence is %r"%v.dist)
            return self.printPath(v)


    # Recursive routine called by the printShortPath method
    def printPath(self,destVertex):

        if destVertex.previousVertex != None:
            self.printPath(destVertex.previousVertex)

        self.path.append(destVertex.point)

        return self.path

    # Initializes the vertex output info prior to executing the shortest path method.
    def clearAll(self):
        for vertex in self.vertexMap.values():
            vertex.reset()

    # Single-source shortest-path method to find shortest path from source to every vertex.
    def shortestPath(self, startPoint):
        self.clearAll()

        if not self.vertexMap.has_key(startPoint):
            print("Start vertex not found")
            return

        start = self.vertexMap[startPoint]

        q = []
        start.dist = 0
        q.append(start)

        while len(q)!= 0:
            v = q.pop(0)

            for w in v.AdjVertices:

                if w.dist == sys.maxint:
                    w.dist = v.dist + 1
                    w.previousVertex = v
                    q.append(w)

