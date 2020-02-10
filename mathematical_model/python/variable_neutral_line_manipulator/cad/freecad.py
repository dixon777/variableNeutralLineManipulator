import sys
sys.path.append("C:\\Program Files\\FreeCAD 0.18\\bin")
import math
import os

import FreeCAD as App 
import FreeCADGui
import FreeCAD.Part as Part
import FreeCAD.Sketcher as Sketcher 



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
    topPlane = obj.newObject(_featureClassMapping["Plane"], "topPlane")
    topPlane.AttachmentOffset = App.Placement(
        App.Vector(0,0,0),
        App.Rotation(0,math.degrees(orientationBF),0) # Yaw = x, Pitch = z, Roll = y (in deg)
    )
    topPlane.MapReversed = False
    topPlane.Support = [(doc.YZ_Plane, '')]
    topPlane.MapMode = "FlatFace"
    
     # Create bottom curve sketch
    topSketch = obj.newObject(_featureClassMapping["Sketch"], "topSketch")
    topSketch.Support = (topPlane, '')
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
    sketch = obj.newObject(_featureClassMapping["Sketch"], "cylinderSketch")
    sketch.Support = (doc.XY_Plane, [''])
    sketch.MapMode = "FlatFace"
    
    constructLine = sketch.addGeometry(Part.LineSegment(
        App.Vector(0, 0),
        App.Vector(0, 1)
    ))
    sketch.addConstraint(Sketcher.Constraint('Distance', 0, distFromAxis)) 
    sketch.addConstraint(Sketcher.Constraint("Coincident", -1, 1, 0, 1))
    sketch.addConstraint(Sketcher.Constraint("Angle", -1, 1, 0, 1, orientation))
    sketch.toggleConstruction(0)
    
    holeCircle = sketch.addGeometry(Part.Circle(
        App.Vector(0,0),
        App.Vector(0,0,1),
        1
    ))
    sketch.addConstraint(Sketcher.Constraint('Radius', 1, holeRadius))
    sketch.addConstraint(Sketcher.Constraint('Coincident',0, 2, 1, 3)) 
    
    
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
    fillet.Base = (lastFeature, [f"Face{i}" for i in range(3, len(lastFeature.Shape.Faces))])    
    
def _cutCenterHole(doc, obj, radius, length):
    """
        Generate a center hole
        Must be called after _applyFilletToHoles() to avoid face renaming issue of FreeCAD (originated from Opencascade)
    """
    cylinder = doc.addObject(_featureClassMapping["Cylinder"], "centerHoleCylinder")
    cylinder.Radius = f"{radius} mm"
    cylinder.Height = f"{length} mm"
    cylinder.Placement = App.Placement(
        App.Vector(0,0,-length/2),
        App.Rotation(App.Vector(0,0,1),0),
        App.Vector(0,0,1)
    )
    cut = doc.addObject(_featureClassMapping["Cut"], "centerHoleCut")
    cut.Base = obj
    cut.Tool = cylinder
    
    
def _generateRingObj(doc,
               ringGeometry,
               bodyName="ring"):
    
    obj = doc.addObject(_featureClassMapping["Body"], bodyName)
    
    _createCylinder(doc, obj, 
                   ringGeometry.cylindricalRadius, 
                   ringGeometry.length, 
                   ringGeometry.topCurveRadius is not None, 
                   ringGeometry.topCurveRadius is not None)
    
    if ringGeometry.bottomCurveRadius:
        _cutBottomCurve(doc, obj, ringGeometry.length, ringGeometry.bottomCurveRadius)
    
    if ringGeometry.topCurveRadius:
        _cutTopCurve(doc, obj, ringGeometry.length, ringGeometry.topOrientationBF, ringGeometry.topCurveRadius)
    
    for tg in ringGeometry.tendonGuideGeometries:
        _cutThroughHole(doc, obj, tg.orientationRF(ringGeometry.orientationBF), tg.distFromAxis, tg.radius)

    _applyFilletToHoles(doc, obj, ringGeometry.tendonGuideFilletRadius)
    
    if ringGeometry.centerHoleRadius:
        _cutCenterHole(doc, obj, ringGeometry.centerHoleRadius, ringGeometry.length)
        
    doc.recompute()
    
    


def _saveDoc(doc, dirName, fileName):
    fileName = fileName.split(".")[0]+".FCStd"
    path = os.path.join(dirName, fileName)
    if(os.path.exists(path)):
        os.remove(path)
    doc.saveAs(path)
    return path
    
def _exportAsMesh(objs, dirName, fileName):
    path = os.path.join(dirName, fileName.split(".")[0]+".obj")
    if(os.path.exists(path)):
        os.remove(path)
    if objs:
        import Mesh
        Mesh.export(objs, path)
    return path

def generateRings(ringGeometries, savedSrcDirPath=None, exportObjDirPath=None):    
    for i, rg in enumerate(ringGeometries):
        s = f"ring{i}"
        doc = App.newDocument(s)
        _generateRingObj(doc, rg, s)
        obj = doc.Objects[-1]
        print(f"Object Generation \'{s}\' is completed")
        if savedSrcDirPath:
            path = _saveDoc(doc, savedSrcDirPath, s)
            print(f"Source file \'{s}\' has been output to {path}")
        if exportObjDirPath:
            path = _exportAsMesh([obj,], exportObjDirPath, s)
            print(f"Obj file \'{s}\' has been output to {path}")
        
        
        
    
    
        
def main():
    from entities import RingGeometry, TendonGuideGeometry
    rg = RingGeometry(
        length = 5,
        cylindricalRadius = 2.5,
        orientationBF = 0,
        topOrientationRF = math.pi/2,
        bottomCurveRadius = 3,
        topCurveRadius = 3.5,
        tendonGuideFilletRadius=0.08,
        centerHoleRadius=0.3,
        tendonGuideGeometries=[
            TendonGuideGeometry(
                distFromAxis=1.8,
                orientationBF=i*math.pi/4,
                radius=0.5,                
            ) for i in range(8)
        ],
    )
    dirName =  os.path.dirname(__file__)
    generateRings([rg]*2, dirName, dirName)
    
    
if __name__ == "__main__":
    main()