import sys
sys.path.append("C:\\Program Files\\FreeCAD 0.18\\bin")
import math
import os

import FreeCAD as App 
import FreeCADGui
import FreeCAD.Part as Part
import FreeCAD.Sketcher as Sketcher 

from .entities import *

from ..common import *

_featureClassMapping = {
    "Body": 'PartDesign::Body',
    "Sketch": "Sketcher::SketchObject",
    "Pad": "PartDesign::Pad",
    "Pocket": "PartDesign::Pocket",
    "Plane": "PartDesign::Plane",
    "Fillet": "PartDesign::Fillet",
    "Cylinder": "Part::Cylinder",
    "Cut": "Part::Cut",
}

def _createCylinder(doc, obj, cylindricalRadius, length, hasTopCurve, hasBottomCurve, addition=0.001):
    sketch = obj.newObject(_featureClassMapping["Sketch"], "cylinderSketch")
    sketch.Support = (doc.XY_Plane, [''])
    sketch.MapMode = "FlatFace"
    
    sketch.addGeometry(Part.Circle(
        App.Vector(0,0,0), 
        App.Vector(0,0,1), 
        cylindricalRadius), False)
    
    # Extrude face
    pad = obj.newObject(_featureClassMapping["Pad"], "cylinderPad")
    pad.Profile = sketch
    pad.Type = 4
    pad.Length = length/2+ (addition if hasTopCurve else 0)
    pad.Length2 = length/2+ (addition if hasBottomCurve else 0)
    pad.UpToFace = None
    pad.Reversed = 0
    pad.Midplane = 0
    pad.Offset = 0.0
    
    
def _cutBottomCurve(doc, obj, length, curveRadius):
     # Create bottom curve sketch
    bottomSketch = obj.newObject(_featureClassMapping["Sketch"], "bottomSketch")
    bottomSketch.Support = (doc.YZ_Plane, '')
    bottomSketch.MapMode = "FlatFace"
    
    centerY = curveRadius - length/2
    bottomEdgeY = -(length/2+1)
    bottomSketch.addGeometry(Part.ArcOfCircle(Part.Circle(
        App.Vector(0, centerY, 0), 
        App.Vector(0,0,1), 
        curveRadius
    ), -math.pi,0), False) # (in radian)
    
    points = [
        (-curveRadius,centerY),
        (-curveRadius, bottomEdgeY),
        (curveRadius, bottomEdgeY),
        (curveRadius,centerY)
    ]
    for i in range(len(points)-1):
        bottomSketch.addGeometry(Part.LineSegment(
            App.Vector(*(points[i])),
            App.Vector(*(points[i+1]))
        ))
        
    bottomSketch.addConstraint(Sketcher.Constraint("Vertical",1))
    bottomSketch.addConstraint(Sketcher.Constraint("Horizontal",2))
    bottomSketch.addConstraint(Sketcher.Constraint("Vertical",3))  
    
    coincidentPairs = (
        (0, 1, 1, 1),
        (0, 2, 3, 2),
        (1, 2, 2, 1),
        (2, 2, 3, 1)
    )
    for p in coincidentPairs:
        bottomSketch.addConstraint(Sketcher.Constraint("Coincident", *p))
        
    bottomSketch.addConstraint(Sketcher.Constraint("DistanceY", -1,1,0,3,centerY))
    bottomSketch.addConstraint(Sketcher.Constraint("DistanceY", -1,-1,1,2,bottomEdgeY))
    
    # Cut curve
    pocket = obj.newObject(_featureClassMapping["Pocket"], "bottomCurvePocket")
    pocket.Profile = bottomSketch
    pocket.Length = 5
    pocket.Type = 1
    pocket.UpToFace = None
    pocket.Reversed = 0
    pocket.Midplane = 1
    pocket.Offset = 0
    
    
def _cutTopCurve(doc, obj, length, orientationBF, curveRadius):
    topCurvePlane = obj.newObject(_featureClassMapping["Plane"], "topCurvePlane")
    topCurvePlane.AttachmentOffset = App.Placement(
        App.Vector(0,0,0),
        App.Rotation(0,math.degrees(orientationBF),0) # Yaw = x, Pitch = z, Roll = y (in deg)
    )
    topCurvePlane.MapReversed = False
    topCurvePlane.Support = [(doc.YZ_Plane, '')]
    topCurvePlane.MapMode = "FlatFace"
    
     # Create bottom curve sketch
    topSketch = obj.newObject(_featureClassMapping["Sketch"], "topSketch")
    topSketch.Support = (topCurvePlane, '')
    topSketch.MapMode = "FlatFace"
    
    centerY = length/2 - curveRadius # one
    topEdgeY = length/2+1
    topSketch.addGeometry(Part.ArcOfCircle(Part.Circle(
        App.Vector(0, centerY, 0), 
        App.Vector(0,0,1), 
        curveRadius
    ), 0, math.pi), False) # (in radian)
    
    points = (
        (curveRadius,centerY),
        (curveRadius, topEdgeY),
        (-curveRadius, topEdgeY),
        (-curveRadius,centerY)
    )
    for i in range(len(points)-1):
        topSketch.addGeometry(Part.LineSegment(
            App.Vector(*(points[i])),
            App.Vector(*(points[i+1]))
        ))
        
    topSketch.addConstraint(Sketcher.Constraint("Vertical",1))
    topSketch.addConstraint(Sketcher.Constraint("Horizontal",2))
    topSketch.addConstraint(Sketcher.Constraint("Vertical",3))  
    
    coincidentPairs = (
        (0, 1, 1, 1),
        (0, 2, 3, 2),
        (1, 2, 2, 1),
        (2, 2, 3, 1)
    )
    for p in coincidentPairs:
        topSketch.addConstraint(Sketcher.Constraint("Coincident", *p))
    
    topSketch.addConstraint(Sketcher.Constraint("DistanceY", -1,1,0,3,centerY))
    topSketch.addConstraint(Sketcher.Constraint("DistanceY", -1,-1,1,2,topEdgeY))
    
    # Cut curve
    pocket = obj.newObject(_featureClassMapping["Pocket"], "topCurvePocket")
    pocket.Profile = topSketch
    pocket.Length = 5
    pocket.Type = 1
    pocket.UpToFace = None
    pocket.Reversed = 0
    pocket.Midplane = 1
    pocket.Offset = 0
   
   
def _cutThroughHole(doc, obj, orientation, distFromAxis, holeRadius):
    x = distFromAxis*math.cos(orientation)
    y = distFromAxis*math.sin(orientation)
    
    sketch = obj.newObject(_featureClassMapping["Sketch"], "cylinderSketch")
    sketch.Support = (doc.XY_Plane, [''])
    sketch.MapMode = "FlatFace"
    
    holeCircle = sketch.addGeometry(Part.Circle(
        App.Vector(0,0),
        App.Vector(0,0,1),
        1
    ))
    sketch.addConstraint(Sketcher.Constraint('Radius', 0, holeRadius))
    sketch.addConstraint(Sketcher.Constraint('DistanceX',-1, 1, 0, 3, x)) 
    sketch.addConstraint(Sketcher.Constraint('DistanceY',-1, 1, 0, 3, y)) 
    
    pocket = obj.newObject(_featureClassMapping["Pocket"], "holeCut")
    pocket.Profile = sketch
    pocket.Type = 1
    pocket.Midplane = 1
    pocket.Offset = 0
    
    
def _applyFilletToHoles(doc, obj, radius):
    doc.recompute() # Recompute to generate faces
    lastFeature = obj.Group[-1]
        
    fillet = obj.newObject(_featureClassMapping["Fillet"], "holeFillets")
    fillet.Radius = radius
    # Before fillets, Face1 = Cylindrical face, Face2 = Top surface, Face3 = Bottom surface, Face(4-end) = Cylindrical surface of drilled holes
    fillet.Base = (lastFeature, [f"Face{i}" for i in range(3, len(lastFeature.Shape.Faces)+1)])    
    
def _cutCenterHole(doc, obj, radius):
    """
        Generate a center hole
        Must be called after _applyFilletToHoles() to avoid face renaming issue of FreeCAD (originated from Opencascade)
    """
    _cutThroughHole(doc, obj, 0.0, 0.0, radius)
    

def _cutKnob(doc, obj, orientation, distFromAxis, radius, outerRadius, baseToCenter):
    """
        ringLength = to bypass the app
    """
    knobPlane = obj.newObject(_featureClassMapping["Plane"], "knobPlane")
    knobPlane.AttachmentOffset = App.Placement(
        App.Vector(0,0,baseToCenter),
        App.Rotation(0,0,0)
    )
    knobPlane.MapReversed = False
    knobPlane.Support = [(doc.XY_Plane, '')]
    knobPlane.MapMode = "FlatFace"
    
    sketch = obj.newObject(_featureClassMapping["Sketch"], "knobSketch")
    sketch.Support = (knobPlane, '')
    sketch.MapMode = "FlatFace"
    
    sketch.addGeometry(Part.ArcOfCircle(Part.Circle(
        App.Vector(0,0),
        App.Vector(0,0,1),
        1
    ), math.pi/2, 3*math.pi/2), False)
    
    sketch.addConstraint(Sketcher.Constraint('Radius',0, radius)) 
    
    # Enforce 180 deg span of arc
    sketch.addGeometry(Part.LineSegment(App.Vector(0.0,0.0),App.Vector(1.0,0.0)),True)
    sketch.addConstraint(Sketcher.Constraint('Coincident',1,1,0,1)) 
    sketch.addConstraint(Sketcher.Constraint('Coincident',1,2,0,3)) 
    
    sketch.addGeometry(Part.LineSegment(App.Vector(0.0,0.0),App.Vector(1.0,0.0)),True)
    sketch.addConstraint(Sketcher.Constraint('Coincident',2,1,0,3)) 
    sketch.addConstraint(Sketcher.Constraint('Coincident',2,2,0,2)) 
    sketch.addConstraint(Sketcher.Constraint('Angle',1,2,2,1,math.pi)) 
    
    # Complete shape
    sketch.addGeometry(Part.LineSegment(App.Vector(0,radius),App.Vector(1.0, radius)),False) # 3
    sketch.addGeometry(Part.LineSegment(App.Vector(0.0,-radius),App.Vector(1.0,-radius)),False) # 4
    sketch.addConstraint(Sketcher.Constraint('Tangent',3,1,0,1)) 
    sketch.addConstraint(Sketcher.Constraint('Tangent',4,1,0,2)) 
    sketch.addConstraint(Sketcher.Constraint('Equal',3,4)) 
    sketch.addConstraint(Sketcher.Constraint('Distance',3,outerRadius)) 
    
    sketch.addGeometry(Part.LineSegment(App.Vector(1.0, radius),App.Vector(1.0, -radius)),False) # 5
    sketch.addConstraint(Sketcher.Constraint('Coincident',5,1,3,2)) 
    sketch.addConstraint(Sketcher.Constraint('Coincident',5,2,4,2)) 
    
    # Enforce orientation
    sketch.addGeometry(Part.LineSegment(App.Vector(0.0,radius),App.Vector(1.0, radius)),True) # 6
    sketch.addConstraint(Sketcher.Constraint('Horizontal',6)) 
    sketch.addConstraint(Sketcher.Constraint('Distance',6,radius)) 
    sketch.addConstraint(Sketcher.Constraint('Coincident',6,1,0,1)) 
    sketch.addConstraint(Sketcher.Constraint('Angle',6,1,3,1,orientation)) 
    
    # Enforce disp
    x = distFromAxis*math.cos(orientation)
    y = distFromAxis*math.sin(orientation)
    sketch.addConstraint(Sketcher.Constraint('DistanceX',-1,1,0,3,x)) 
    sketch.addConstraint(Sketcher.Constraint('DistanceY',-1,1,0,3,y)) 
    
    pocket = obj.newObject(_featureClassMapping["Pocket"],"knobCut")
    pocket.Profile = sketch
    pocket.Length = 10.0
    pocket.Length2 = 100.0
    pocket.Type = 1
    pocket.UpToFace = None
    pocket.Reversed = 1
    pocket.Midplane = 0
    pocket.Offset = 0.000000
    
    # doc.removeObject("knobPlane")
    
    
def _generateRingObj(doc,
               ringGeometry,
               bodyName="ring"):
    rg = ringGeometry
    obj = doc.addObject(_featureClassMapping["Body"], bodyName)
    
    _createCylinder(doc, obj, 
                   rg.cylindricalRadius, 
                   rg.length, 
                   rg.topCurveRadius is not None, 
                   rg.topCurveRadius is not None)
    
    if rg.bottomCurveRadius:
        _cutBottomCurve(doc, obj, rg.length, rg.bottomCurveRadius)
    
    if rg.topCurveRadius:
        _cutTopCurve(doc, obj, rg.length, rg.topOrientationBF, rg.topCurveRadius)
    
    for tg in rg.tendonGuideGeometries:
        _cutThroughHole(doc, obj, tg.orientationRF(rg.orientationBF), tg.distFromAxis, tg.radius)
        if isinstance(tg, TendonKnobGuideGeometry): 
            # (-0.0001) is for avoiding the tendon guide perimeter intersect with the knob's circular edge. 
            # Smaller value is still applicable for generating a valid Object file (.obj) but will lead to missing faces of the body if the source file (.FCStd) is opened in FreeCAD GUI
            _cutKnob(doc, obj, tg.orientationRF(rg.orientationBF), tg.distFromAxis - tg.radius + tg.knobSlotRadius-0.0001, tg.knobSlotRadius, rg.cylindricalRadius, tg.knobLength - rg.length/2)
    
    # _applyFilletToHoles(doc, obj, rg.tendonGuideFilletRadius)
    
    if rg.centerHoleRadius:
        _cutCenterHole(doc, obj, rg.centerHoleRadius)
        
    doc.recompute()
    return obj
    


def _saveSrc(doc, path):
    from datetime import datetime
    path = ensurePath(path, ".FCStd")
    doc.saveAs(path)
    return path
    
def _saveAsMesh(objs, path):
    from datetime import datetime
    path = ensurePath(path, ".obj")
    import Mesh
    Mesh.export(objs, path)
    return path

# def generateRings(ringGeometries, savedSrcDirPath=None, exportObjDirPath=None):    
#     for i, rg in enumerate(ringGeometries):
#         s = f"ring{i}"
#         doc = App.newDocument(s)
#         obj = _generateRingObj(doc, rg, s)
#         print(f"Object Generation \'{s}\' is completed")
#         if savedSrcDirPath:
#             path = _saveSrc(doc, savedSrcDirPath, s)
#             print(f"Source file \'{s}\' has been output to {path}")
#         if exportObjDirPath:
#             path = _saveAsMesh([obj,], exportObjDirPath, s)
#             print(f"Obj file \'{s}\' has been output to {path}")
        
        
        
class RingCAD():
    def __init__(self, ringGeomtry, name=""):
        self.name = name
        self.ringGeomtry = ringGeomtry
        self.doc = None
        self.obj = None
        
    def generate(self):
        self.doc = App.newDocument(self.name)
        self.obj = _generateRingObj(self.doc, self.ringGeomtry, "body")
        
    def saveSrc(self, path=None):
        if not path or not os.path.basename(path):
            path = os.path.join(os.path.dirname(path) if path else "", self.name)
        if self.doc:
           return  _saveSrc(self.doc, path)
        
    def saveAsObj(self, path=None):
        if not path or not os.path.basename(path):
            path = os.path.join(os.path.dirname(path) if path else "", self.name)
        if self.obj:
            return _saveAsMesh([self.obj,], path)