import numpy as np

        
class Range1d(object):
    def __init__(self):
        self.min = None
        self.max = None
        
    @property
    def bound(self):
        if self.min is None or self.max is None:
            return None
        return (self.min, self.max)
        
    @property
    def total(self):
        if self.min is None:
            return self.max
        elif self.max is None:
            return self.min
        return self.max + self.min
    
    @property
    def avg(self):
        total = self.total
        return None if total is None else total/2
            
    
    @property
    def diff(self):
        if self.min is None:
            return self.max
        elif self.max is None:
            return -self.min
        return self.max - self.min
    
    def update(self, vals):
        if not isinstance(vals, (list, tuple)):
            vals = (vals,)
        for v in vals:
            if self.min is None or self.min > v:
                self.min = v
            elif self.max is None or self.max < v:
                self.max = v
                
    def __repr__(self):
        return f"bound({self.min}, {self.max})"
                
class Range3d(object):
    def __init__(self):
        self.ranges = (Range1d(), Range1d(), Range1d())
    
    @property
    def x(self):
        return self.ranges[0]

    @property
    def y(self):
        return self.ranges[1]

    @property
    def z(self):
        return self.ranges[2]
        
    def update(self, x=(), y=(), z=()):
        self.updateX(x)
        self.updateY(y)
        self.updateZ(z)
        
    def updateX(self, vals):
        self.ranges[0].update(vals)
        
    def updateY(self, vals):
        self.ranges[1].update(vals)
        
    def updateZ(self, vals):
        self.ranges[2].update(vals)
        
    def __repr__(self):
        return f"range3d[x: {self.x}, y: {self.y}, z: {self.z}]"
    
    
def enforceRange(ax, range3d: Range3d):
    maxDiff = 0.5* max(r.diff for r in range3d.ranges)
    g = np.mgrid[-1:2:2, -1:2:2, -1:2:2]
    
    for i, j, k in zip(maxDiff*g[0].flatten() + range3d.x.avg, 
                       maxDiff*g[1].flatten() + range3d.y.avg, 
                       maxDiff*g[2].flatten() + range3d.z.avg):
        print(i,j,k)
        ax.plot((i,), (j,), (k,), 'w')