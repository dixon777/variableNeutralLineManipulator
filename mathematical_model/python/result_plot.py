import numpy as np
import matplotlib.pyplot as plt
import matplotlib.widgets as pltW
import math

import snake_joint_2d.solvers as its
import snake_joint_2d.helper_functions as hf


axesMap = {}

records = {} # 3 layers (each big iteration, each joint, each samll)
            # in future, extends to 4 layers (each tension sets, ....)

def drawEndPiece2d(curvatureAngle, curvatureRadius, length):
    pass


    
class EventController:
    def __init__(self, fig, func=None, isScroll=False):
        self.curIndex = 0
        self.fig = fig
        self.func = func
        if func is not None:
            func(fig, self.curIndex)
        if isScroll:
            fig.canvas.mpl_connect('scroll_event', self)
            
    def setFunc(self, func):
        self.func = func
        self.func(self.fig, self.curIndex)
        
    def __call__(self, event):
        if event.button == "up":
            self.prev(event)
        elif event.button == "down":
            self.next(event)
        else:
            print("Gesture not implemented")
            
    def next(self, event):
        if self.func is not None:
            if self.func(event.canvas.figure, self.curIndex+1):
                self.curIndex = self.curIndex + 1
             
    def prev(self, event):
        if self.func is not None:
            if self.func(event.canvas.figure, self.curIndex-1):
                self.curIndex = self.curIndex - 1
                
  

def refreshAxes(resDict):
        axesMap["freeBodyAxe"].clear()
        for v in resDict.values():
            if isinstance(v, tuple):
                if not hf.allWithin(v[0], [0,0], threshold=0.001):
                    axesMap["freeBodyAxe"].quiver(v[1][0], v[1][1], v[0][0],v[0][1])
        axesMap["freeBodyAxe"].axis('equal')

def defineEndPieceIteractiveFunc_plot():
    for index, (i,j,k,res) in enumerate(its.computeIterator(tensionLeft=1, tensionRight=0.6, curvatureRadius=2, curvatureAngle=math.pi/2,
              numJoints=2, length=3, frictionCoefficient=0.5, initJointBendingAngles=[0, 0])):
        print(index, i, j, k)
        if records.get(i) is None:
            records[i] = {}
        if records[i].get(j) is None:
            records[i][j] = {}
                
        records[i][j][k] = res # prone to errors (inconssitency of iterations)
        
    plt.ion()
    mainFig, axes = plt.subplots(2,1)
    axesMap["freeBodyAxe"] = axes[0]
    axesMap["convergenceAxe"] = axes[1]
    plt.subplots_adjust(bottom=0.2)
    
    smallIteController = EventController(mainFig)   
    ringIteController = EventController(mainFig)
    bigIteController = EventController(mainFig)
    
    def __onSmallIteUpdate(fig, index):
        if index < 0 or index >= len(records[bigIteController.curIndex][ringIteController.curIndex]):
            return False
        refreshAxes(records[bigIteController.curIndex][ringIteController.curIndex][index])
        print(records[bigIteController.curIndex][ringIteController.curIndex][index])
        return True
    
    def __onRingIteUpdate(fig, index):
        if index < 0 or index >= len(records[bigIteController.curIndex]):
            return False
        refreshAxes(records[bigIteController.curIndex][index][smallIteController.curIndex])  
        print(records[bigIteController.curIndex][index][smallIteController.curIndex])
        return True
        
    def __onBigIteUpdate(fig, index):
        if index < 0 or index >= len(records):
            return False
        refreshAxes(records[index][ringIteController.curIndex][smallIteController.curIndex])  
        return True
    
    smallIteController.setFunc(__onSmallIteUpdate)
    ringIteController.setFunc(__onRingIteUpdate)
    bigIteController.setFunc(__onBigIteUpdate) 
        # axesMap["convergenceAxe"].clear()
        # x = []
        # y = [] 
        # for (i,d) in enumerate(iterationDictList):
        #     x.append(i)
        #     y.append(d["result"])
        # axesMap["convergenceAxe"].plot(x,y)
        

    bprev = pltW.Button(plt.axes([0.75, 0.05, 0.1, 0.075]), 'Prev')
    bprev.on_clicked(smallIteController.prev)
    bnext = pltW.Button(plt.axes([0.85, 0.05, 0.1, 0.075]), 'Next')
    bnext.on_clicked(smallIteController.next)
    
    bPrevRing = pltW.Button(plt.axes([0.45, 0.05, 0.1, 0.075]), 'Prev ring')
    bPrevRing.on_clicked(ringIteController.prev)
    bNextRing = pltW.Button(plt.axes([0.55, 0.05, 0.1, 0.075]), 'Next ring')
    bNextRing.on_clicked(ringIteController.next)
    
    bPrevIte = pltW.Button(plt.axes([0.15, 0.05, 0.1, 0.075]), 'Prev ite')
    bPrevIte.on_clicked(bigIteController.prev)
    bNextIte = pltW.Button(plt.axes([0.25, 0.05, 0.1, 0.075]), 'Next ite')
    bNextIte.on_clicked(bigIteController.next)
    plt.show(block=True)  
    
    
if __name__ == "__main__":
    defineEndPieceIteractiveFunc_plot()