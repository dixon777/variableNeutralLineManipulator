
# Must run on Python version 3.6.6 or below
import sys, math
sys.path.append("C:\Program Files\FreeCAD 0.18\\bin")
print(sys.path)

import FreeCAD, Part
from FreeCAD import Base

class Box:
    def __init__(self, obj):
        '''Add some custom properties to our box feature'''
        obj.addProperty("App::PropertyLength","Length","Box","Length of the box").Length=1.0
        obj.addProperty("App::PropertyLength","Width","Box","Width of the box").Width=1.0
        obj.addProperty("App::PropertyLength","Height","Box", "Height of the box").Height=1.0
        obj.Proxy = self
   
    def onChanged(self, fp, prop):
        '''Do something when a property has changed'''
        FreeCAD.Console.PrintMessage("Change property: " + str(prop) + "\n")
 
    def execute(self, fp):
        '''Do something when doing a recomputation, this method is mandatory'''
        FreeCAD.Console.PrintMessage("Recompute Python Box feature\n")

def main():
    doc = FreeCAD.newDocument()
    obj = doc.addObject("Part::FeaturePython", "A")
    Box(obj)

if __name__ == "__main__":
    main()
    