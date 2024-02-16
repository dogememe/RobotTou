import pathplanner

path = list(pathplanner.main())

grid = [[1, 2, 3, 4], #1
        [5, 6, 7, 8], #2
        [9, 10,11,12], #3
        [13,14,15,16]] #4

def getXY(indX):
    X = (1+(indX-1)%4)/2-0.25
    Y = 2-(indX-indX%4)/8-.25

    return (X,Y)

XtrajList = []
YtrajList = []

for point in path:
    pos = getXY(point)
    XtrajList.append(pos[0])
    YtrajList.append(pos[1])
print(len(XtrajList)-1)
print(XtrajList[1:])
print(YtrajList[1:])