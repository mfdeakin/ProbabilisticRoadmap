
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
    rmCtx.set_line_width(0.2)
    rmCtx.set_source_rgb(0.0, 1.0, 0.0)
    rmCtx.arc(dest.x, dest.y, 0.5, 0, 2 * pi)
    rmCtx.stroke()
    rmCtx.set_source_rgb(0.0, 0.0, 1.0)
    rmCtx.arc(src.x, src.y, 0.5, 0, 2 * pi)
    rmCtx.stroke()
    rmCtx.set_source_rgb(0.0, 0.0, 0.0)
    for r in rects:
        rmCtx.rectangle(r.xMin, r.yMin, r.xMax - r.xMin, r.yMax - r.yMin)
        rmCtx.stroke()
    return (rmBmp, rmSurf, rmCtx)

def drawCairoLine(cairoObj, pt1, pt2):
    rmCtx = cairoObj[2]
    rmCtx.set_source_rgb(0.0, 0.0, 0.0)
    rmCtx.move_to(pt1.x, pt1.y)
    rmCtx.line_to(pt2.x, pt2.y)
    rmCtx.stroke()

def drawCairoPt(cairoObj, pt):
    rmCtx = cairoObj[2]
    rmCtx.set_source_rgb(1.0, 0.0, 0.0)
    rmCtx.arc(pt.x, pt.y, 0.5, 0, 2 * pi)
    rmCtx.stroke()

def buildPRM(rects, src, dest, xMax, yMax, numPts):
    inf = float('inf')
    # Incremental algorithm which updates a transitive closure graph
    # Runs in O(numPts^3) time
    # I largely did it this way because I initially wanted
    # something that terminated as soon as it found a solution
    graphTC = {}
    graphTC[src] = {src: 0}
    graphTC[dest] = {dest: 0}
    cairoObj = createCairoImg(rects, src, dest, xMax, yMax)
    if Link(rects, src, dest):
        dist = distance(src, dest)
        graphTC[src][dest] = dist
        graphTC[dest][src] = dist
    while len(graphTC) < numPts:
        newPt = Vector(randint(1, xMax - 1), randint(1, yMax - 1))
        if not Clear(rects, newPt) or newPt in graphTC.keys():
            continue
        graphTC[newPt] = {}
        # First find the points it can connect to
        for pt in graphTC.keys():
            test = Link(rects, pt, newPt)
            if test:
                drawCairoLine(cairoObj, pt, newPt)
                dist = distance(pt, newPt)
                graphTC[pt][newPt] = dist
                graphTC[newPt][pt] = dist
                # Then give it all of the points it can connect to
                for pt2 in graphTC[pt].keys():
                    try:
                        prevDist = graphTC[newPt][pt2]
                        if prevDist > dist + graphTC[pt][pt2]:
                            graphTC[newPt][pt2] = dist + graphTC[pt][pt2]
                            graphTC[pt2][newPt] = dist + graphTC[pt][pt2]
                    except KeyError:
                        graphTC[newPt][pt2] = dist + graphTC[pt][pt2]
                        graphTC[pt2][newPt] = dist + graphTC[pt][pt2]
        # Now update all of the other points
        # All of the points that can connect to the new point are
        # already in the list of points, so don't bother with any others
        for pt in graphTC[newPt].keys():
            for pt2 in graphTC[newPt].keys():
                dist = graphTC[pt][newPt] + graphTC[newPt][pt2]
                try:
                    prevDist = graphTC[pt][pt2]
                    if dist < prevDist:
                        graphTC[pt][pt2] = dist
                except KeyError:
                    graphTC[pt][pt2] = dist
    for pt in graphTC.keys():
        drawCairoPt(cairoObj, pt)
    if dest in graphTC[src].keys():
        return (graphTC[src][dest], cairoObj[1])
    else:
        return (inf, cairoObj[1])

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
                dist, img = buildPRM(rects, src, dest, 22, 22, n)
                img.write_to_png(name + "_" + str(n) + "_" + str(i) + ".png")
                avgDist += dist
            avgDist /= numTrials
            print(str(n) + " Points Average Path Length: " + str(avgDist))

if __name__ == "__main__":
    runTest()
