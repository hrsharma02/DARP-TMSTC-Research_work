import numpy as np
from Edges import Edge

class CalculateTrajectories():
    def __init__(self, r, c, MST):
        self.MAX_NODES = 4 * r * c
        self.PathSequence = []
        self.rows = r
        self.cols = c
        self.MSTvector = MST
        self.MSTedges = len(self.MSTvector)
        self.allEdges = set()
        self.nodes = {node: set() for node in range(self.MAX_NODES)}

    def initializeGraph(self, A, connect4):
        for i in range(2 * self.rows):
            for j in range(2 * self.cols):
                if A[i, j]:
                    index = i * 2 * self.cols + j
                    if i > 0 and A[i - 1, j]:
                        self.AddToAllEdges(index, (i - 1) * 2 * self.cols + j, 1)
                    if i < 2 * self.rows - 1 and A[i + 1, j]:
                        self.AddToAllEdges(index, (i + 1) * 2 * self.cols + j, 1)
                    if j > 0 and A[i, j - 1]:
                        self.AddToAllEdges(index, i * 2 * self.cols + j - 1, 1)
                    if j < 2 * self.cols - 1 and A[i, j + 1]:
                        self.AddToAllEdges(index, i * 2 * self.cols + j + 1, 1)
                    if not connect4:
                        if i > 0 and j > 0 and A[i - 1, j - 1]:
                            self.AddToAllEdges(index, (i - 1) * 2 * self.cols + j - 1, 1)
                        if i < 2 * self.rows - 1 and j < 2 * self.cols - 1 and A[i + 1, j + 1]:
                            self.AddToAllEdges(index, (i + 1) * 2 * self.cols + j + 1, 1)
                        if i < 2 * self.rows - 1 and j > 0 and A[i + 1, j - 1]:
                            self.AddToAllEdges(index, (i + 1) * 2 * self.cols + j - 1, 1)
                        if i > 0 and j < 2 * self.cols - 1 and A[i - 1, j + 1]:
                            self.AddToAllEdges(index, (i - 1) * 2 * self.cols + j + 1, 1)

    def AddToAllEdges(self, _from: int, to: int, cost):
        edge = Edge(_from, to, cost)
        self.allEdges.add(edge)
        self.nodes[_from].add(to)
        self.nodes[to].add(_from)

    def RemoveTheAppropriateEdges(self):
        for e in self.MSTvector:
            maxN = max(e.src, e.dst)
            minN = min(e.src, e.dst)
            if abs(e.src - e.dst) == 1:
                alpha = (4 * minN + 3) - 2 * (maxN % self.cols)
                remove = [
                    Edge(alpha, alpha + 2 * self.cols, 1),
                    Edge(alpha + 2 * self.cols, alpha, 1),
                    Edge(alpha + 1, alpha + 1 + 2 * self.cols, 1),
                    Edge(alpha + 1 + 2 * self.cols, alpha + 1, 1)
                ]
            else:
                alpha = (4 * minN + 2 * self.cols) - 2 * (maxN % self.cols)
                remove = [
                    Edge(alpha, alpha + 1, 1),
                    Edge(alpha + 1, alpha, 1),
                    Edge(alpha + 2 * self.cols, alpha + 1 + 2 * self.cols, 1),
                    Edge(alpha + 1 + 2 * self.cols, alpha + 2 * self.cols, 1)
                ]
            for edge in remove:
                if edge in self.allEdges:
                    self.SafeRemoveEdge(edge)

    def SafeRemoveEdge(self, curEdge):
        try:
            self.allEdges.remove(curEdge)
            self.nodes[curEdge.src].discard(curEdge.dst)
            self.nodes[curEdge.dst].discard(curEdge.src)
        except KeyError:
            print("Edge removal failed: ", curEdge)
            import sys
            sys.exit(1)

    def CalculatePathsSequence(self, StartingNode):
        movement = [2 * self.cols, -1, -2 * self.cols, 1]  # down, left, up, right
        currentNode = StartingNode
        prevNode = None
        PathSequence = []
        Visited = set()

        # Try to find first valid move
        for d in movement:
            neighbor = currentNode + d
            if neighbor in self.nodes[currentNode]:
                prevNode = currentNode
                currentNode = neighbor
                break
        else:
            print("No initial neighbor found for traversal.")
            self.PathSequence = []
            return

        while True:
            if currentNode != StartingNode:
                Visited.add(currentNode)
            PathSequence.append(self._node_pair_to_coords(prevNode, currentNode))

            found = False
            for d in movement:
                next_node = currentNode + d
                if next_node in self.nodes[currentNode] and next_node not in Visited:
                    prevNode, currentNode = currentNode, next_node
                    found = True
                    break

            if not found:
                break

        self.PathSequence = PathSequence

    def _node_pair_to_coords(self, prev, curr):
        previ = prev // (2 * self.cols)
        prevj = prev % (2 * self.cols)
        i = curr // (2 * self.cols)
        j = curr % (2 * self.cols)
        return (previ, prevj, i, j)
