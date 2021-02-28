import math
import sys


class NodeCost(object):
    def __init__(self, node):
        self.node = node
        self.known = False
        self.cost = math.inf
        self.path = None


class Edge(object):
    def __init__(self, node, distance):
        self.__node = node
        self.__distance = distance
        self.__edge = None

    def addEdge(self, edge):
        self.__edge = edge

    def getNode(self):
        return self.__node

    def getDistance(self):
        return self.__distance

    def setDistance(self, distance):
        self.__distance = distance

    def getEdge(self):
        return self.__edge


class Graph(object):
    def __init__(self):
        self.adjList = []
        self.__shortestRoute = None
        self.__minDistance = 0
        self.__nodeCost = []
        self.__src = None
        self.__dest = None
        self.__path = []

    def addEdge(self, node1, node2, distance):
        found1 = False
        found2 = False

        if not node1 or not node2 or distance < 0:
            return

        for element in self.adjList:
            if node1 == element.getNode():
                self.__appendElement__(element, node2, distance)
                found1 = True
            elif node2 == element.getNode():
                self.__appendElement__(element, node1, distance)
                found2 = True
            else:
                continue
        if not found1:
            index = len(self.adjList)
            self.adjList.append(Edge(node1, 0))
            self.adjList[index].addEdge(Edge(node2, distance))
            self.__nodeCost.append(NodeCost(node1))
        if not found2:
            index = len(self.adjList)
            self.adjList.append(Edge(node2, 0))
            self.adjList[index].addEdge(Edge(node1, distance))
            self.__nodeCost.append(NodeCost(node2))

    #         print('Node List after adding : ', node1,node2 , self.__nodeCost)

    def __appendElement__(self, element, node, distance):
        if element.getNode() == node:
            element.setDistance(distance)
        elif element.getEdge():
            self.__appendElement__(element.getEdge(), node, distance)
        else:
            element.addEdge(Edge(node, distance))

    def findShortestPath(self, node1, node2):
        self.__src = node1
        self.__dest = node2
        self.__getAllShortestRoute(node1)
        self.__findFinalPath(node2)

    def __getAllShortestRoute(self, source):
        sourceNode = None
        nodeCost = math.inf
        #         print(len(self.adjList))
        for i in range(len(self.adjList)):
            # print('node info :', source, i, self.__nodeCost[i].node,self.adjList[i].getNode())
            if self.adjList[i].getNode() == source:
                print('node info :', source, i, self.__nodeCost[i].node, self.adjList[i].getNode())
                sourceNode = self.adjList[i]
                self.__nodeCost[i].known = True
                if self.__src == source:
                    self.__nodeCost[i].cost = 0
                nodeCost = self.__nodeCost[i].cost
                break
        if sourceNode is None:
            raise Exception("Not a valid Source :", sourceNode)
        self.__updateAlldjacentNode(sourceNode, nodeCost)
        nextNode = self.__findNextNodeToTravel()
        if nextNode:
            #             print(nextNode)
            self.__getAllShortestRoute(nextNode)

    #         for node in self.__nodeCost:
    #             print('nodeCost info : ', node.node, node.cost, node.path, node.known)

    def __updateAlldjacentNode(self, edge, nodeCost):
        source = edge.getNode()
        temp = edge.getEdge()
        while temp:
            self.__updateCost(source, temp, nodeCost)
            temp = temp.getEdge()

    def __updateCost(self, source, edge, nodeCost):
        for node in self.__nodeCost:
            if not node.known and node.node == edge.getNode() and node.cost > nodeCost + edge.getDistance():
                node.cost = nodeCost + edge.getDistance()
                node.path = source

    def __findNextNodeToTravel(self):
        nextNode = None
        minCost = math.inf
        for node in self.__nodeCost:
            if not node.known and node.cost < minCost:
                minCost = node.cost
                nextNode = node.node
        return nextNode

    def __findFinalPath(self, dest):
        tempDest = None
        for node in self.__nodeCost:
            #             print('path traversal : ', node.node)
            if node.node == dest:
                if self.__dest == dest:
                    self.__minDistance = node.cost
                self.__path.append(dest)
                tempDest = node.path
        if tempDest:

            #         print('src and dest : ', self.__src, tempDest)
            if self.__src == tempDest:
                self.__path.append(tempDest)
                return
            self.__findFinalPath(tempDest)
        else:
            raise Exception("Not a valid Destination :", dest)

    def getFinalPath(self):
        self.__path.reverse()
        return self.__path

    def getMinCost(self):
        return self.__minDistance

    def printAdjList(self):
        print('--------------------------------')
        print('----ADJ LIST--------------------')
        print('--------------------------------')
        for edge in self.adjList:
            print("adj list for :", edge.getNode())
            tempEdge = edge
            while tempEdge.getEdge():
                tempEdge = tempEdge.getEdge()
                print('Nodes of list and distance is : ', tempEdge.getNode(), tempEdge.getDistance())
        print('--------------------------------')
        print('----ADJ LIST--------------------')
        print('--------------------------------')


def convert_hrs_to_minutes(hrs):
    min = hrs * 60
    before, after = str(min).split('.')
    return "%s:%s" % (before, after)


if __name__ == '__main__':
    graphWeightedNodes = []
    srcNode = ""
    dstNode = ""
    speed = 0
    minDistance = 0;
    travelTime = 0;

    try:
        # trying to open a file in read mode
        with open("inputPS4.txt", "r") as infile:
            for line in infile:
                line = line.replace(" ", "")
                if line.rstrip() and line.replace(" ", "") and " ".join(line.split()):
                    if line.__contains__("Hospital") and line.__contains__("A"):
                        srcNode = (line.strip().split(":")[1]).strip()
                    elif line.__contains__("Hospital") and line.__contains__("B"):
                        dstNode = (line.strip().split(":")[1]).strip()
                    elif line.__contains__("Speed") or line.__contains__("Ambulance"):
                        speed = int((line.strip().split(":")[1]).strip())
                    elif line.__contains__("/"):
                        graphWeightedNodes.append(line.strip().split("/"))
                    else:
                        print("discarding invalid Input")
                        pass

        print(graphWeightedNodes)
        print("Source Node A ->", srcNode)
        print("Destination Node B ->", dstNode)
        print("Ambulance Speed ->", speed)

        #     graph = Graph()
        #     graph.addEdge('A', 'B', 8)
        #     graph.addEdge('A', 'C', 2)
        #     graph.addEdge('A', 'D', 5)
        #     graph.addEdge('B', 'A', 8)
        #     graph.addEdge('B', 'D', 2)
        #     graph.addEdge('B', 'F', 11)
        #     graph.addEdge('C', 'A', 2)
        #     graph.addEdge('C', 'D', 2)
        #     graph.addEdge('C', 'E', 3)
        #     graph.addEdge('C', 'A', 5)
        #     graph.addEdge('D', 'B', 2)
        #     graph.addEdge('D', 'C', 2)
        #     graph.addEdge('D', 'E', 1)
        #     graph.addEdge('D', 'F', 6)
        #     graph.addEdge('D', 'G', 3)
        #     graph.addEdge('E', 'C', 3)
        #     graph.addEdge('E', 'D', 1)
        #     graph.addEdge('E', 'G', 1)
        #     graph.addEdge('F', 'C', 11)
        #     graph.addEdge('F', 'E', 6)
        #     graph.addEdge('F', 'G', 2)
        #     graph.addEdge('F', 'H', 3)
        #     graph.addEdge('G', 'D', 3)
        #     graph.addEdge('G', 'E', 1)
        #     graph.addEdge('G', 'F', 2)
        #     graph.addEdge('G', 'H', 6)
        #     graph.addEdge('H', 'G', 6)
        #     graph.addEdge('H', 'F', 3)
        #     graph.addEdge('I',None,2)

        #     graph.printAdjList()

        # graph.addEdge('A', 'C', 1)
        # graph.addEdge('B', 'C', 2)
        # graph.addEdge('B', 'D', 3)
        # graph.addEdge('C', 'D', 9)
        # graph.findShortestPath('A', 'C')
        # ////////////////Graph Construction ////////////////////
        graph = Graph()
        src = ''
        dst = ''
        weight = 0
        for node in graphWeightedNodes:
            print("node is ->", node)
            if len(node) != 3:
                raise Exception(" input is not well formatted");
                exit(0)
            src = node[0].strip()
            dst = node[1].strip()
            weight = int(node[2].strip())
            # graph.addEdge(node[0], node[1], int(node[2]))
            graph.addEdge(src, dst, weight)
            print("Graph Node are ", node[0], node[1], int(node[2]))
        graph.printAdjList()
        # identify shrortest path
        if srcNode is "" or dstNode is "":
            raise Exception(" input is not well formatted");
            exit(0)
        graph.findShortestPath(srcNode, dstNode)
        #     graph.findShortestPath('A','I')
        minDistance = graph.getMinCost()
        shortestPath = graph.getFinalPath()
        print('Min Distance =>', minDistance)
        #     print('min Distance:',shortestDistance)
        #     print(*shortestPath,sep="->" )
        travelTime = (minDistance / speed)
        print('Time taken in Minutes ->', convert_hrs_to_minutes(travelTime))

        lineWithNodes = (
            "Shortest route from the Hospital A (Node {src}) to reach the Hospital B (Node {dst}) is \n".format(
                src=srcNode,
                dst=dstNode));
        lineWithShortestpath = ""
        for i in shortestPath:
            print(i)
            lineWithShortestpath = lineWithShortestpath + "-" + i

        lineWithShortestpath = lineWithShortestpath.lstrip("-")
        print(lineWithShortestpath)

        lineWithTravelDistance = "and it has minimum travel distance {KM}km \n".format(KM=minDistance);
        lineWithTime = "it will take {time} minutes for the ambulance to reach the Hospital B at a speed of {velocity} " \
                       "kmph. \n".format(time=convert_hrs_to_minutes(travelTime), velocity=speed);

        with open("outputPS4.txt", "w") as outfile:
            outfile.write(lineWithNodes);
            outfile.write('[' + lineWithShortestpath + ']' + '\n');
            outfile.write(lineWithTravelDistance);
            outfile.write(lineWithTime);

    except FileNotFoundError:
        print("Input file inputPS4.txt does not exist")
    except:
        print("Other error", sys.exc_info())
