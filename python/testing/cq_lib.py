from math import *
import os
import cadquery as cq

class DiskType:
    TOP = "top"
    
    BOTTOM = "bottom"

class Disk:
    def __init__(self, t:DiskType, D, L, ctr, cto, cbr, cbo, chd, gd, gn, gdfc):
        super().__init__()
        self.t = t
        self.D = D
        self.L = L
        self.ctr = ctr
        self.cto = cto
        self.cbr = cbr
        self.cbo = cbo
        
        self.chd = chd
        self.gd = gd
        self.gn = gn
        self.gdfc = gdfc
    
    @staticmethod
    def Intermediate(self, D, L, ctr, cto, cbr, cbo, chd, gd, gn, gdfc):
        return 
        


def export(shape, svg_path=None, stl_path=None, step_path=None, dir_path=None):
    if not svg_path and not stl_path and not step_path:
        return

    if not os.path.exists(dir_path):
        os.mkdir(dir_path)
        
    if svg_path:
        svg_path = os.path.join(dir_path, svg_path) if dir_path else svg_path
        cq.exporters.exportSVG(shape, svg_path)
        
    if stl_path:
        stl_path = os.path.join(dir_path, stl_path) if dir_path else stl_path
        with open(stl_path, "w") as f:
            cq.exporters.exportShape(shape, cq.exporters.ExportTypes.STL, f)
        
    if step_path:
        step_path = os.path.join(dir_path, step_path) if dir_path else step_path
        with open(step_path, "w") as f:
            cq.exporters.exportShape(shape, cq.exporters.ExportTypes.STEP, f)

def extrude_base(s, R, r, L):
    s = s.copyWorkplane(cq.Workplane('XY')).circle(R).circle(r).extrude(L/2, both=True)
    return s

def cut_top_curve(s, cr, co, R, L):
    L2 = L/2
    yDispCurveEdge = sqrt((L2)**2-cr**2)
    s = s.copyWorkplane(cq.Workplane('YZ')).transformed(rotate=(0, co, 0)).moveTo(-R, yDispCurveEdge).threePointArc((0, L2), (R, yDispCurveEdge))
    s = s.lineTo(R, L2+10).lineTo(-R, L2+10).close().cutThruAll()
    return s

def cut_bottom_curve(s, cr, co, R, L):
    L2 = L/2
    yDispCurveEdge = -sqrt((L2)**2-cr**2)
    s = s.copyWorkplane(cq.Workplane('YZ')).transformed(rotate=(0, co, 0)).moveTo(-R, yDispCurveEdge).threePointArc((0, -L2), (R, yDispCurveEdge))
    s = s.lineTo(R, -L2-10).lineTo(-R, -L2-10).close().cutThruAll()
    return s

def cut_guide_holes(s, r, nG, dfG):
    s = s.copyWorkplane(cq.Workplane('XY')).polygon(nG, dfG*2, forConstruction=True).vertices().circle(r).cutThruAll()
    return s

def _generate_disk_shape(d:Disk, top, bottom):
    shape = cq.Workplane('XY')
    shape = extrude_base(shape, d.D/2, d.chd/2, d.L)
    if top:
        shape = cut_top_curve(shape, d.ctr, d.cto, d.D/2, d.L)
    if bottom:
        shape = cut_bottom_curve(shape, d.btr, d.bto, d.D/2, d.L)
    shape = cut_guide_holes(shape, d.gd/2, d.gn, d.gdfc)
    return shape

def generate_intermediate_disk_shape(d:Disk):
    return _generate_disk_shape(d, top=True, bottom=True)

def generate_top_disk_shape(d:Disk):
    return _generate_disk_shape(d, top=True, bottom=False)

def generate_bottom_disk_shape(d:Disk):
    return _generate_disk_shape(d, top=False, bottom=True)

    



if __name__ == "__main__":
    import time
    start = time.time()
    D = 5
    d = 1.5
    L = 5
    gd = 1
    tcr = 2
    bcr = 2
    tco = 70
    bco = 0
    dG = 1
    nG = 8
    dfG = 1.5

    shape = cq.Workplane('XY')
    shape = extrude_base(shape, D/2, d/2, L)
    shape = cut_top_curve(shape, tcr, tco, D/2, L)
    shape = cut_bottom_curve(shape, bcr, bco, D/2, L)
    shape = cut_guide_holes(shape, dG/2, nG, dfG)
    
    generate_end_time = time.time()

    export(shape, svg_path="a.svg", stl_path="a.stl", dir_path=os.path.join(os.path.curdir, "tmp"))
    
    export_end_time = time.time()
    
    print(f"Generate period: {round(generate_end_time-start,3)}s. Export period: {round(export_end_time - generate_end_time,3)}s.")

