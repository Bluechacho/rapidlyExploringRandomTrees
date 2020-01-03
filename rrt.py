import sys

import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import numpy as np

# Set up matplotlib to create a plot with an empty square
def setupPlot():
    fig = plt.figure(num=None, figsize=(5, 5), dpi=120, facecolor='w', edgecolor='k')
    ax = fig.subplots()
    ax.set_axisbelow(True)
    ax.set_ylim(-1, 11)
    ax.set_xlim(-1, 11)
    ax.grid(which='minor', linestyle=':', alpha=0.2)
    ax.grid(which='major', linestyle=':', alpha=0.5)
    return fig, ax


# Make a patch for a single polygon 
def createPolygonPatch(polygon, color):
    verts = []
    codes= []
    for v in range(0, len(polygon)):
        xy = polygon[v]
        verts.append((xy[0], xy[1]))
        if v == 0:
            codes.append(Path.MOVETO)
        else:
            codes.append(Path.LINETO)
    verts.append(verts[0])
    codes.append(Path.CLOSEPOLY)
    path = Path(verts, codes)
    patch = patches.PathPatch(path, facecolor=color, lw=1)

    return patch
    

# Render the problem  
def drawProblem(robotStart, robotGoal, polygons):
    _, ax = setupPlot()
    patch = createPolygonPatch(robotStart, 'green')
    ax.add_patch(patch)    
    patch = createPolygonPatch(robotGoal, 'red')
    ax.add_patch(patch)    
    for p in range(0, len(polygons)):
        patch = createPolygonPatch(polygons[p], 'gray')
        ax.add_patch(patch)    
    plt.show()

# Add given edge to given tree
def addPoint(tree, points, pointDetails, newIndex):
    # If pointDetails[1] connects to an existing point:
    if (pointDetails[1] != -1):
        # Connect pointDetails[1] to pointDetails[0] and vice versa
        if (pointDetails[1] in tree):
                tree[pointDetails[1]].append(pointDetails[0])
        else:
            tree.update({pointDetails[1]:pointDetails[0]})
            
        if (pointDetails[0] in tree):
            tree[pointDetails[0]].append(pointDetails[1])
        else:
            tree.update({pointDetails[0]:[pointDetails[1]]})
        
        # No new point
        newPoint = None
            
    # If pointDetails[1] creates a new point:
    else:
        # Create a new point, store in points, newPoints
        points[newIndex] = pointDetails[2]
        newPoint = pointDetails[2]
                
        # Disconnect edge points pointDetails[4] and pointDetails[5]               
        temp = tree[pointDetails[4]]
        temp.remove(pointDetails[5])
        tree[pointDetails[4]] = temp
        
        temp = tree[pointDetails[5]]
        temp.remove(pointDetails[4])
        tree[pointDetails[5]] = temp
            
        # Connect pointDetails[4], pointDetails[5] to new point and vice versa
        tree[pointDetails[4]].append(newIndex)
        tree[pointDetails[5]].append(newIndex)
                
        tree[newIndex] = [pointDetails[4]]
        tree[newIndex].append(pointDetails[5])
               
        # Connect pointDetails[1] to new point and vice versa
        if newIndex in tree:
            tree[newIndex].append(pointDetails[0])
        else:
            tree[newIndex] = [pointDetails[0]]

        if pointDetails[0] in tree:
            tree[pointDetails[0]].append(newIndex)
        else:
            tree[pointDetails[0]] = [newIndex]
           
    return tree, newPoint, points

# Pair of functions to determine if two lines intersect
def counterClockwise(pointA,pointB,pointC):
    return (pointC[1]-pointA[1]) * (pointB[0]-pointA[0]) > (pointB[1]-pointA[1]) * (pointC[0]-pointA[0])

def intersect(A,B,C,D):
    return (counterClockwise(A,C,D) != counterClockwise(B,C,D) and counterClockwise(A,B,C) != counterClockwise(A,B,D))

# Find distance of two points
def distance(a, b):
    x1 = a[0]
    y1 = a[1]
    x2 = b[0]
    y2 = b[1]
    return np.sqrt(np.square(x1-x2) + np.square(y1-y2))

# Find the closest line between the passed point to the passed edge
def findIntersection(x1,y1,x2,y2,x3,y3):
    
    # Create a perpendicular line, find intersection
    # of the given line and the created line
    if (y1==y2): 
        return [(x3,y1), 0]
    if (x1==x2): 
        return [(x1,y3), 0]
    else:
        m1 = (y2-y1)/(x2-x1)
        m2 = -1/m1
        x = (m1*x1-m2*x3+y3-y1) / (m1-m2)
        y = m2*(x-x3)+y3
    
    # Check that our answer is part of the edge: if not,
    # return the closest endpoint of the edge
    # 0 = closest point is new point
    # 1 = closest point is (x1, y1)
    # 2 = closest point is (x2, y2)
   
    if round((distance((x,y),(x1,y1)) + distance((x,y),(x2,y2)))/(distance((x1, y1),(x2,y2))), 2) == 1.00:
        return [(x,y), 0]
    else:
        if (distance((x,y),(x1,y1)) < distance((x,y),(x2,y2))):
            return [(x1,y1), 1]
        
        else:
            return [(x2,y2), 2]


# Grow a simple RRT 
def growSimpleRRT(points):
    adjListMap = {}
    minDist = 10
    nextFreeVertex = len(points) + 1
    root = (5,5)
    points[0] = root
    vertexCount = len(points)
    newPoints = {0 : root}
    
    for i in range(1, vertexCount):
        # If there are no edges, create the first edge between root and points[i]
        if not adjListMap:
            if (points[i] != root):
                adjListMap[i] = [0]
                adjListMap[0] = [i]
                
        # Otherwise, add points[i] to the closest edge
        else:
            # Find the closest edge details, recorded in this format:
            # minDist[new edge created, distance of intersection point, existing edge being connected onto]
            minDist = [-1, -1, (99,99), 999, -1, -1]
            for j in adjListMap:
                for k in adjListMap.get(j):
                    v1 = points[j]
                    v2 = points[k]
                    v3 = points[i]
                    intPoint = findIntersection(v1[0],v1[1],v2[0],v2[1],v3[0],v3[1])
                    if (distance((intPoint[0]),(v3)) < minDist[3]):
                        minDist[0] = i
                        if (intPoint[1] == 1):
                            minDist[1] = j
                        if (intPoint[1] == 2):
                            minDist[1] = k
                        if (intPoint[1] == 0):
                            minDist[1] = -1
                        minDist[2] = intPoint[0]
                        minDist[3] = distance((intPoint[0]),(v3))
                        minDist[4] = j
                        minDist[5] = k
            
            # minDist is the next addition to adjListMap
            adjListMap, newestPoint, points = addPoint(adjListMap, points, minDist, nextFreeVertex)
            if (newestPoint is not None):
                newPoints[nextFreeVertex] = newestPoint
                nextFreeVertex = nextFreeVertex + 1
        newPoints[i] = points[i]

    return newPoints, adjListMap


# Perform basic search 
def basicSearch(tree, start, goal):
    path = []
    nodeQueue = []

    # Initialize search
    currentNode = [start]
    

    # Iterate through tree from start to goal    
    while(True):
        path.append(currentNode[-1])        
        # Populate nodeQueue with current node neighbor paths
        for x in tree[currentNode[-1]]:
            if x not in path:
                temp = currentNode.copy()
                temp.append(x)
                nodeQueue.append(temp)
        
        if nodeQueue:
            currentNode = nodeQueue.pop(0)
        else:
            return None
        
        # When shortest path is found, prepare path and pathLength
        if (currentNode[-1] == goal):
            path = currentNode
            break
        
    return path


# Display the RRT and Path
def displayRRTandPath(points, adjListMap, path, robotStart=None, robotGoal=None, polygons=None):
   
    # Draw the problem (iff polygons is given)
    _, ax = setupPlot()
    if (polygons is not None):    
        patch = createPolygonPatch(robotStart, 'green')
        ax.add_patch(patch)    
        patch = createPolygonPatch(robotGoal, 'red')
        ax.add_patch(patch)    
        for p in range(0, len(polygons)):
            patch = createPolygonPatch(polygons[p], 'gray')
            ax.add_patch(patch)    
    
    # Draw the RRT
    for a in adjListMap:
            for b in adjListMap[a]:
                ax.plot([points[a][0], points[b][0]], [points[a][1], points[b][1]], 'black')        
    
    # Draw the path if it exists
    if path is not None:
        for a in path:
            # If index isn't last
            if a == path[0]:
                last = a
            else:
                ax.plot([points[a][0], points[last][0]], [points[a][1], points[last][1]], 'orange')    
                last = a
            
    plt.show()
    
    return

# Collision checking
def isCollisionFree(robot, point, obstacles):

    updatedRobot = []
        
    # Move robot to point
    for pointR in range(0, len(robot)):
        updatedRobot.append(tuple(map(lambda x, y: x + y, robot[pointR], point)))
    robot = updatedRobot
    
    # Part 1 - Local Coordinate Collision Check
    # Before more advanced checks, we want to know if any edge of Robot intersects
    # any obstacle edge. If so, we return False.
    
    
    # For each robot boundary, check if it intersects any obstacle bounds    
    
    for pointR in range(0, len(robot)):
        # Generate a robotEdge
        if (pointR != (len(robot)-1)):
            robotEdge = [robot[pointR], robot[pointR+1]]
        else:
            robotEdge = [robot[pointR], robot[0]]
            
        # Iterate through all obstacle edges
        for objO in range(0,len(obstacles)):
            for pointO in range(0,len(obstacles[objO])):
                # Generate a robotEdge
                if (pointO != (len(obstacles[objO])-1)):
                    obstacleEdge = [obstacles[objO][pointO], obstacles[objO][pointO+1]]
                else:
                    obstacleEdge = [obstacles[objO][pointO], obstacles[objO][0]]
                
                # If robotEdge and obstacleEdge intersect, collision is not free
                if(intersect(robotEdge[0], robotEdge[1], obstacleEdge[0], obstacleEdge[1])):
                    return False
        
    # No robotEdge collided with any obstacleEdge, so we advance to Part 2
    
    # Part 2 - Global Coordinate Collision Check
    # We now want to see if there is collision in the global
    # coordinate space - are we inside a polygon?
    
    # Convert each obstacles[] into a Path object EXCEPT the outer boundary
    for poly in obstacles:
        isBoundary = False
        temp = poly
        for coord in temp:
            if (10.0) in coord:
                isBoundary = True
        
        # poly is not an outer boundary, so create a Path
        # and test robot against that Path using contains_points
        if isBoundary is False:
            polyPath = Path(temp)
            if True in (polyPath.contains_points(robot)):
                return False    
    

    # We made it all the way here, so there is no collision!
    return True


# The full RRT algorithm
def RRT(robot, obstacles, startPoint, goalPoint):

    pointCount = 50
    points = dict()
    root = [5,5]
    points[0] = root
    spIndex = pointCount + 1
    gpIndex = pointCount + 2
    points[spIndex] = startPoint
    points[gpIndex] = goalPoint
    tree = dict()
    path = []
    newPoints = {0 : root, spIndex : startPoint, gpIndex : goalPoint}
    isValid = []
    testPt = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    
    # Generate (arbitrary amount) 500 points and note if one
    # of those points is startPoint or goalPoint
    x = np.random.uniform(0.01, 9.0, size=pointCount).tolist()
    y = np.random.uniform(0.01, 9.0, size=pointCount).tolist()
    
    for i in range(0,pointCount):
        points[i+1] = [round(x[i],5), round(y[i],5)]

    # Cull repeats
    for i in range(0,pointCount):
        if (list(points).count(points[i]) > 1):
            list(points).remove(points[i])
        if (points[i] is startPoint):
            spIndex = i
        if (points[i] is goalPoint):
            gpIndex = i
    
    nextFreeVertex = len(points) + 1
    # Connect point points[nextPt] to tree if the 
    # created edge passes collision checks
    
    for nextPt in range(1, len(points)):
        # Initialize tree with first valid point
        if not tree:
            isValid = True
            # Create 17 points along the potential edge
            testPt[0] = [points[nextPt][0], points[nextPt][1]]
            testPt[16] = [points[0][0], points[0][1]]
            testPt[8] = [(testPt[0][0] + testPt[16][0])/2,(testPt[0][1] + testPt[16][1])/2]            
            testPt[4] = [(testPt[0][0] + testPt[8][0])/2,(testPt[0][1] + testPt[8][1])/2]
            testPt[12] = [(testPt[8][0] + testPt[16][0])/2,(testPt[8][1] + testPt[16][1])/2]            
            testPt[2] = [(testPt[0][0] + testPt[4][0])/2,(testPt[0][1] + testPt[4][1])/2]
            testPt[6] = [(testPt[4][0] + testPt[8][0])/2,(testPt[4][1] + testPt[8][1])/2]
            testPt[10] = [(testPt[8][0] + testPt[12][0])/2,(testPt[8][1] + testPt[12][1])/2]
            testPt[14] = [(testPt[12][0] + testPt[16][0])/2,(testPt[12][1] + testPt[16][1])/2]
            testPt[1] = [(testPt[0][0] + testPt[2][0])/2,(testPt[0][1] + testPt[2][1])/2]
            testPt[3] = [(testPt[2][0] + testPt[4][0])/2,(testPt[2][1] + testPt[4][1])/2]
            testPt[5] = [(testPt[4][0] + testPt[6][0])/2,(testPt[4][1] + testPt[6][1])/2]
            testPt[7] = [(testPt[6][0] + testPt[8][0])/2,(testPt[6][1] + testPt[8][1])/2]
            testPt[9] = [(testPt[8][0] + testPt[10][0])/2,(testPt[8][1] + testPt[10][1])/2]
            testPt[11] = [(testPt[10][0] + testPt[12][0])/2,(testPt[10][1] + testPt[12][1])/2]
            testPt[13] = [(testPt[12][0] + testPt[14][0])/2,(testPt[12][1] + testPt[14][1])/2]
            testPt[15] = [(testPt[14][0] + testPt[16][0])/2,(testPt[14][1] + testPt[16][1])/2]
            
            for currentPt in testPt:
                if (isValid is True):
                    isValid = isCollisionFree(robot, currentPt, obstacles)
                
            # Add next point to the tree if valid
            if (isValid is True):
                tree[nextPt] = [0]
                tree[0] = [nextPt]
        
        # Add next valid points[nextPt] to the closest edge
        else:
            # Find the closest edge details, recorded in this format:
            # minDist[new edge created, distance of intersection point, existing edge being connected onto]
            minDist = [-1, -1, (99,99), 999, -1, -1]
            for j in tree:
                for k in tree.get(j):
                    v1 = points[j]
                    v2 = points[k]
                    v3 = points[nextPt]
                    intPoint = findIntersection(v1[0],v1[1],v2[0],v2[1],v3[0],v3[1])
                    if (distance((intPoint[0]),(v3)) < minDist[3]):
                        minDist[0] = nextPt
                        if (intPoint[1] == 1):
                            minDist[1] = j
                        if (intPoint[1] == 2):
                            minDist[1] = k
                        if (intPoint[1] == 0):
                            minDist[1] = -1
                        minDist[2] = intPoint[0]
                        minDist[3] = distance((intPoint[0]),(v3))
                        minDist[4] = j
                        minDist[5] = k
        
            # minDist is the potential next addition to 
            # adjListMap, but muss pass a collision check 
            isValid = True
            if (minDist[1] == -1):
                edge = [points[minDist[0]][0], points[minDist[0]][1], minDist[2][0], minDist[2][1]]
            else:
                edge = [points[minDist[0]][0], points[minDist[0]][1], points[minDist[1]][0], points[minDist[1]][1]]
            
            
            # Create 17 points along the potential edge            
            testPt[0] = [edge[0], edge[1]]
            testPt[16] = [edge[2], edge[3]]
            testPt[8] = [(testPt[0][0] + testPt[16][0])/2,(testPt[0][1] + testPt[16][1])/2]            
            testPt[4] = [(testPt[0][0] + testPt[8][0])/2,(testPt[0][1] + testPt[8][1])/2]
            testPt[12] = [(testPt[8][0] + testPt[16][0])/2,(testPt[8][1] + testPt[16][1])/2]            
            testPt[2] = [(testPt[0][0] + testPt[4][0])/2,(testPt[0][1] + testPt[4][1])/2]
            testPt[6] = [(testPt[4][0] + testPt[8][0])/2,(testPt[4][1] + testPt[8][1])/2]
            testPt[10] = [(testPt[8][0] + testPt[12][0])/2,(testPt[8][1] + testPt[12][1])/2]
            testPt[14] = [(testPt[12][0] + testPt[16][0])/2,(testPt[12][1] + testPt[16][1])/2]
            testPt[1] = [(testPt[0][0] + testPt[2][0])/2,(testPt[0][1] + testPt[2][1])/2]
            testPt[3] = [(testPt[2][0] + testPt[4][0])/2,(testPt[2][1] + testPt[4][1])/2]
            testPt[5] = [(testPt[4][0] + testPt[6][0])/2,(testPt[4][1] + testPt[6][1])/2]
            testPt[7] = [(testPt[6][0] + testPt[8][0])/2,(testPt[6][1] + testPt[8][1])/2]
            testPt[9] = [(testPt[8][0] + testPt[10][0])/2,(testPt[8][1] + testPt[10][1])/2]
            testPt[11] = [(testPt[10][0] + testPt[12][0])/2,(testPt[10][1] + testPt[12][1])/2]
            testPt[13] = [(testPt[12][0] + testPt[14][0])/2,(testPt[12][1] + testPt[14][1])/2]
            testPt[15] = [(testPt[14][0] + testPt[16][0])/2,(testPt[14][1] + testPt[16][1])/2]
            
            for currentPt in testPt:
                if (isValid is True):
                    isValid = isCollisionFree(robot, currentPt, obstacles)

            # Add next point to the tree if valid
            if (isValid is True):
                tree, newestPoint, points = addPoint(tree, points, minDist, nextFreeVertex)
                if (newestPoint is not None):
                    newPoints[nextFreeVertex] = newestPoint
                    nextFreeVertex = nextFreeVertex + 1
                newPoints[nextPt] = points[nextPt]
                
    # Attach the startPoint and goalPoint to the RRT,
    # if they were not in the generated points

    # Create path between spIndex and gpIndex
    path = basicSearch(tree, spIndex, gpIndex)

    return points, tree, path


def main(filename, x1, y1, x2, y2, display=''):
    # Read data and parse polygons
    lines = [line.rstrip('\n') for line in open(filename)]
    robot = []
    obstacles = []
    for line in range(0, len(lines)):
        xys = lines[line].split(';')
        polygon = []
        for p in range(0, len(xys)):
            xy = xys[p].split(',')
            polygon.append((float(xy[0]), float(xy[1])))
        if line == 0 :
            robot = polygon
        else:
            obstacles.append(polygon)

    # Print out the data
    print("Robot:")
    print(str(robot))
    print("Pologonal obstacles:")
    for p in range(0, len(obstacles)):
        print(str(obstacles[p]))
    print("")

    # Visualize initial map
    if display == 'display':
        robotStart = [(x + x1, y + y1) for x, y in robot]
        robotGoal = [(x + x2, y + y2) for x, y in robot]
        drawProblem(robotStart, robotGoal, obstacles)
    
    # Solve a generated RRT problem
    points, adjListMap, path = RRT(robot, obstacles, (x1, y1), (x2, y2))
    
    # Visualization code 
    if display == 'display':
        displayRRTandPath(points, adjListMap, path, robotStart, robotGoal, obstacles) 


if __name__ == "__main__":
    # Retrive file name for input data
    if(len(sys.argv) < 6):
        print("Five arguments required: python rrt.py [env-file] [x1] [y1] [x2] [y2]")
        exit()
    
    filename = sys.argv[1]
    x1 = float(sys.argv[2])
    y1 = float(sys.argv[3])
    x2 = float(sys.argv[4])
    y2 = float(sys.argv[5])
    display = ''
    if(len(sys.argv) == 7):
        display = sys.argv[6]

    main(filename, x1, y1, x2, y2, display)