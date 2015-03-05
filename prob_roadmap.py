
from math import copysign, sqrt, pi
from random import randint
from time import sleep
import numpy as np
import cairo

class Vector():
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.str = self.__repr__
    
    def __repr__(self):
        return "(" + str(self.x) + ", " + str(self.y) + ")"

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __ne__(self, other):
        return not __eq__(self, other)

    def __hash__(self):
        return self.x * 100 + self.y

class Rect():
    def __init__(self, xMin, xMax, yMin, yMax):
        self.xMin = xMin
        self.xMax = xMax
        self.yMin = yMin
        self.yMax = yMax

def Clear(rects, pt):
    for r in rects:
        if pt.x >= r.xMin and pt.x <= r.xMax and \
           pt.y >= r.yMin and pt.y <= r.yMax:
            return False
    return True

def CrossProd(vRhs, vLhs):
    return vRhs.x * vLhs.y - vRhs.y * vLhs.x

def Intersects(rect, src, dest):
    pathRay = Vector(dest.x - src.x, dest.y - src.y)
    rectEdges = [(Vector(rect.xMin, rect.yMin), Vector(rect.xMin, rect.yMax)),
                 (Vector(rect.xMin, rect.yMin), Vector(rect.xMax, rect.yMin)),
                 (Vector(rect.xMin, rect.yMax), Vector(rect.xMax, rect.yMax)),
                 (Vector(rect.xMax, rect.yMin), Vector(rect.xMax, rect.yMax))]
    for edge in rectEdges:
        # First check if the path-ray intersects the edge
        edgeVtx1 = Vector(edge[0].x - src.x, edge[0].y - src.y)
        edgeVtx2 = Vector(edge[1].x - src.x, edge[1].y - src.y)
        sign1 = copysign(1, CrossProd(pathRay, edgeVtx1))
        sign2 = copysign(1, CrossProd(pathRay, edgeVtx2))
        if sign1 != sign2:
            # Now check if the edge-ray intersects the path
            edgeRay = Vector(edge[1].x - edge[0].x, edge[1].y - edge[0].y)
            pathVtx1 = Vector(src.x - edge[0].x, src.y - edge[0].y)
            pathVtx2 = Vector(dest.x - edge[0].x, dest.y - edge[0].y)
            sign1 = copysign(1, CrossProd(edgeRay, pathVtx1))
            sign2 = copysign(1, CrossProd(edgeRay, pathVtx2))
            if sign1 != sign2:
                return True
    return False

def Link(rects, src, dest):
    if not Clear(rects, src) or not Clear(rects, dest):
        return False
    for r in rects:
        if Intersects(r, src, dest):
            return False
    return True

def distance(p1, p2):
    dx = p1.x - p2.x
    dy = p1.y - p2.y
    return sqrt(dx * dx + dy * dy)

def createCairoImg(rects, src, dest, xMax, yMax):
    imScale = 32
    imXDim = imScale * xMax
    imYDim = imScale * yMax
    rmBmp = np.zeros((imXDim, imYDim, 4), dtype=np.uint8)
    rmSurf = cairo.ImageSurface.create_for_data(rmBmp,
                                                cairo.FORMAT_ARGB32,
                                                imXDim, imYDim)
    rmCtx = cairo.Context(rmSurf)
    rmCtx.set_source_rgb(1.0, 1.0, 1.0)
    rmCtx.translate(0, imYDim)
    rmCtx.scale(imScale, -imScale)
    rmCtx.paint()
    rmCtx.set_line_width(0.1)
    for r in rects:
        rmCtx.set_source_rgb(0.0, 0.0, 0.0)
        rmCtx.rectangle(r.xMin, r.yMin, r.xMax - r.xMin, r.yMax - r.yMin)
        rmCtx.stroke()
        rmCtx.set_source_rgb(0.4, 0.4, 0.4)
        rmCtx.rectangle(r.xMin, r.yMin, r.xMax - r.xMin, r.yMax - r.yMin)
        rmCtx.fill()
    return (rmSurf, rmCtx)

def drawLine(ctx, pt1, pt2, rgbVal):
    ctx.set_source_rgb(rgbVal[0], rgbVal[1], rgbVal[2])
    ctx.move_to(pt1.x, pt1.y)
    ctx.line_to(pt2.x, pt2.y)
    ctx.stroke()

def drawPt(ctx, pt, rgbVal):
    ctx.set_source_rgb(rgbVal[0], rgbVal[1], rgbVal[2])
    ctx.arc(pt.x, pt.y, 0.5, 0, 2 * pi)
    ctx.stroke()

def drawShortestPath(ctx, graphTC, src, dest):
    if not dest in graphTC[src].keys():
        return
    # To compute the shortest path given a transitive closure,
    # just look at all neighboring points and choose the one
    # which has the same distance to the destination
    # as the current point minus the distance to that point
    curPt = src
    epsilon = 10E-3
    while not curPt == dest:
        curDist, _ = graphTC[curPt][dest]
        for pt in graphTC[curPt].keys():
            if dest in graphTC[pt].keys():
                dist1, conn = graphTC[curPt][pt]
                dist2, _ = graphTC[pt][dest]
                totDist = dist1 + dist2
                if conn and abs(curDist - totDist) < epsilon:
                    drawLine(ctx, curPt, pt, (0.0, 0.8, 0.3))
                    curPt = pt

def addPoint(ctx, rects, graphTC, newPt):
    graphTC[newPt] = {}
    # First find the points it can connect to
    for pt in graphTC.keys():
        test = Link(rects, pt, newPt)
        if test:
            drawLine(ctx, pt, newPt, (0.0, 0.0, 0.0))
            dist = distance(pt, newPt)
            graphTC[pt][newPt] = (dist, True)
            graphTC[newPt][pt] = (dist, True)
            # Then give it all of the points it can connect to
            for pt2 in graphTC[pt].keys():
                dist2, _ = graphTC[pt][pt2]
                if pt2 in graphTC[newPt].keys():
                    prevDist, _ = graphTC[newPt][pt2]
                    if prevDist > dist + dist2:
                        graphTC[newPt][pt2] = (dist + dist2,
                                               False)
                        graphTC[pt2][newPt] = (dist + dist2,
                                               False)
                else:
                    graphTC[newPt][pt2] = (dist + dist2, False)
                    graphTC[pt2][newPt] = (dist + dist2, False)
    # Now update all of the other points
    # All of the points that can connect to the new point are
    # already in the list of points, so don't bother with any others
    for pt in graphTC[newPt].keys():
        for pt2 in graphTC[newPt].keys():
            dist1, _ = graphTC[pt][newPt]
            dist2, _ = graphTC[newPt][pt2]
            dist = dist1 + dist2
            try:
                prevDist, _ = graphTC[pt][pt2]
                if dist < prevDist:
                    graphTC[pt][pt2] = (dist, False)
                    graphTC[pt2][pt] = (dist, False)
            except KeyError:
                graphTC[pt][pt2] = (dist, False)
                graphTC[pt2][pt] = (dist, False)

def buildPRM(rects, src, dest, xMax, yMax, numPts):
    inf = float('inf')
    # Incremental algorithm which updates a transitive closure graph
    # Runs in O(numPts^3) time
    # I did it this way because I initially wanted
    # something that terminated as soon as it found a solution
    # Then I read the assignment :)
    graphTC = {}
    graphTC[src] = {src: (0, False)}
    graphTC[dest] = {dest: (0, False)}
    cairoObjs = createCairoImg(rects, src, dest, xMax, yMax)
    if Link(rects, src, dest):
        dist = distance(src, dest)
        graphTC[src][dest] = dist
        graphTC[dest][src] = dist
    while len(graphTC) < numPts:
        newPt = Vector(randint(1, xMax - 1), randint(1, yMax - 1))
        if (not Clear(rects, newPt)) or newPt in graphTC.keys():
            continue
    for pt in graphTC.keys():
        drawPt(cairoObjs[1], pt, (1.0, 0.0, 0.0))
    return (cairoObjs, graphTC)

def runTest():
    rects1 = [Rect(6, 13, 14, 22), Rect(4, 12, 0, 8),
              Rect(14, 22, 4, 10)]
    src1 = Vector(2, 2)
    dest1 = Vector(14, 21)
    rects2 = [Rect(4, 16, 0, 12), Rect(6, 13, 14, 22),
              Rect(14, 22, 4, 10)]
    src2 = Vector(2, 2)
    dest2 = Vector(14, 21)
    problems = [("Test_1", src1, dest1, rects1),
                ("Test_2", src2, dest2, rects2)]
    for name, src, dest, rects in problems:
        print(name)
        for n in range(1, 128):
            avgDist = 0
            numTrials = 10
            for i in range(numTrials):
                cairoObjs, graphTC = buildPRM(rects, src, dest, 22, 22, n)
                dist = float('inf')
                if dest in graphTC[src].keys():
                    dist, _ = graphTC[src][dest]
                drawShortestPath(cairoObjs[1], graphTC, src, dest)
                cairoObjs[0].write_to_png("Images/" + name + "_" + str(n) +
                                          "_" + str(i) + ".png")
                avgDist += dist
            avgDist /= numTrials
            print(str(n) + ", " + str(avgDist))

if __name__ == "__main__":
    runTest()
