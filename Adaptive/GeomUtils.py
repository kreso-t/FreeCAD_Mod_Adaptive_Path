import pyclipper
import math

of = pyclipper.PyclipperOffset()

def anyValidPath(paths):
    for pth in paths:
        if len(pth) > 0:
            return True
    return False


def closePath(path):
    if len(path) == 0:
        return path
    lastPtIndex = len(path)-1
    if path[lastPtIndex][0] != path[0][0] or path[lastPtIndex][1] != path[0][1]:
        path.append([path[0][0], path[0][1]])
    return path


def genToolGeometry(toolRadiusScaled):
    of = pyclipper.PyclipperOffset()
    of.AddPath([[0, 0], [0, 0]], pyclipper.JT_ROUND, pyclipper.ET_OPENROUND)
    geo = of.Execute(toolRadiusScaled)[0]
    return pyclipper.CleanPolygon(geo)


def translatePath(path, pt):
    output = []
    for p in path:
        output.append([p[0]+pt[0], p[1] + pt[1]])
    return output


def translatePaths(paths, pt):
    outpaths = []
    for path in paths:
        output = []
        for p in path:
            output.append([p[0] + pt[0], p[1] + pt[1]])
        outpaths.append(output)
    return outpaths

# calculates center of polygon
def centroid(path):
    c = [0, 0]
    sa = 0
    x0 = 0
    y0 = 0
    x1 = 0
    y1 = 0
    a = 0
    cnt = len(path)
    path = closePath(path)
    for i in range(0, cnt):
        x0 = path[i][0]
        y0 = path[i][1]
        x1 = path[(i+1) % cnt][0]
        y1 = path[(i+1 % cnt)][1]
        a = x0*y1-x1*y0
        sa = sa + a
        c[0] = c[0] + (x0+x1)*a
        c[1] = c[1] + (y0+y1)*a
    sa = 3*sa
    c[0] = c[0] / sa
    c[1] = c[1] / sa
    return c


def getDirectionVAt(path, index):
    p1i = index-1
    p2i = index
    p3i = index+1
    p4i = index+2
    if p3i > len(path)-1:
        p3i = p3i-(len(path)-1)
    if p4i > len(path)-1:
        p4i = p4i - (len(path)-1)

    #find delta vectors between points
    pt1 = path[p1i]
    pt2 = path[p2i]
    pt3 = path[p3i]
    pt4 = path[p4i]

    v1 = [pt2[0]-pt1[0], pt2[1]-pt1[1]]
    v2 = [pt3[0]-pt2[0], pt3[1]-pt2[1]]
    v3 = [pt4[0]-pt3[0], pt4[1]-pt3[1]]
    #add those two vectors - for smoothing of angles at corners
    v = [v1[0]+v2[0]+v3[0], v1[1]+v2[1]+v3[1]]
    #v =[pt2[0]-pt1[0],pt2[1]-pt1[1]]
    #print pt2,pt1
    #print v
    #normalize
    d = math.sqrt(v[0]*v[0] + v[1]*v[1])
    return [v[0]/d, v[1]/d]


def normalize(v):
    d = math.sqrt(v[0]*v[0] + v[1]*v[1])
    return [v[0]/d, v[1]/d]


def getDirectionV(pt1, pt2):
    #find delta vector between points
    v = [pt2[0]-pt1[0], pt2[1]-pt1[1]]
    #normalize
    d = math.sqrt(v[0]*v[0] + v[1]*v[1])
    return [v[0]/d, v[1]/d]


def getAngle(v):
    return math.atan2(v[1], v[0])


def magnitude(v):
    return math.sqrt(v[0]*v[0] + v[1]*v[1])

#get angle between two vectors


def getAngle2v(v1, v2):
    try:
        d = (v1[0]*v2[0] + v1[1]*v2[1])
        m = (math.sqrt(v1[0]*v1[0] + v1[1]*v1[1])) * \
            (math.sqrt(v2[0]*v2[0] + v2[1]*v2[1]))
        if m != 0:
            return math.acos(d/m)
        else:
            return math.pi/32
    except:
        #print "matherror",v1,v2
        return math.pi/4


def sub2v(v1, v2):
    return [v1[0] - v2[0], v1[1] - v2[1]]


def sumv(path):
    res = [0, 0]
    for pt in path:
        res[0] = res[0] + pt[0]
        res[1] = res[1] + pt[1]
    return res


def getIntersectionPointLWP(lineSegment, paths):
    l1 = lineSegment  # first line segment
    for pth in paths:
        if len(pth) > 1:
            for i in range(0, len(pth)):
                l2 = [pth[i-1], pth[i]]  # second line segment (path line)
                d = (l1[1][1]-l1[0][1])*(l2[1][0]-l2[0][0]) - \
                    (l2[1][1]-l2[0][1])*(l1[1][0]-l1[0][0])
                if d == 0:  # lines are parallel
                    continue
                p1d = (l2[1][1]-l2[0][1])*(l1[0][0]-l2[0][0]) - \
                    (l2[1][0]-l2[0][0])*(l1[0][1]-l2[0][1])
                p2d = (l1[1][0]-l1[0][0])*(l2[0][1]-l1[0][1]) - \
                    (l1[1][1]-l1[0][1])*(l2[0][0]-l1[0][0])
                #clamp
                if d < 0:
                    if (p1d < d or p1d > 0):
                        continue  # not inside segment
                    if (p2d < d or p2d > 0):
                        continue  # not inside segment
                else:
                    if (p1d < 0 or p1d > d):
                        continue  # not inside segment
                    if (p2d < 0 or p2d > d):
                        continue  # not inside segment
                return [l1[0][0] + (l1[1][0]-l1[0][0])*p1d/d, l1[0][1] + (l1[1][1]-l1[0][1])*p1d/d]
    #nothing found, return None
    return None


def rotate(v, rad):
    c = math.cos(rad)
    s = math.sin(rad)
    return [c*v[0] - s*v[1], s*v[0] + c*v[1]]


# get closest point on path to point
def getClosestPointOnPaths(paths, pt):
    closestPathIndex = 0
    closestPtIndex = 0
    minDistSq = 1000000000000
    closestPt = []
    for pthi in range(0, len(paths)):
        path = paths[pthi]
        for i in range(0, len(path)):
            #line segment
            p1 = path[i-1]
            p2 = path[i]
            #length between points on the line segment
            lsq = (p2[0] - p1[0]) * (p2[0] - p1[0]) + \
                (p2[1] - p1[1]) * (p2[1] - p1[1])
            if lsq == 0:  # segment is very short take the distance to one of end points
                distSq = (pt[0] - p1[0]) * (pt[0] - p1[0]) + \
                    (pt[1] - p1[1]) * (pt[1] - p1[1])
                clp = p1
            else:
                #((point.x - this.start.x) * (this.end.x - this.start.x) + (point.y - this.start.y) * (this.end.y - this.start.y))
                #parameter of the closest point
                t = (((pt[0] - p1[0]) * (p2[0] - p1[0]) +
                      (pt[1] - p1[1]) * (p2[1] - p1[1])))
                #clamp it
                if t > lsq:
                    t = lsq
                if t < 0:
                    t = 0
                #point on line at t
                clp = [p1[0] + t*(p2[0]-p1[0])/lsq, p1[1] +
                       t*(p2[1]-p1[1])/lsq]
                distSq = (pt[0]-clp[0])*(pt[0]-clp[0]) + \
                    (pt[1]-clp[1])*(pt[1]-clp[1])
            if distSq < minDistSq:
                closestPtIndex = i
                closestPathIndex = pthi
                minDistSq = distSq
                closestPt = clp
    return closestPt, math.sqrt(minDistSq)

# check if point is within tolerance distance to one of points in the array
def closeToOneOfPoints(pt, points, toleranceScaled):
    for p2 in points:
        if magnitude(sub2v(p2, pt)) <= toleranceScaled:
            return True
    return False
