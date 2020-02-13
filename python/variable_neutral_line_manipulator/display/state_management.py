import rx
from rx import operators as ops
from rx.subject import Subject

from .helper import Singleton

from .repo import Repo

class StateManagement(metaclass=Singleton):
    def __init__(self):
        self.addSegmentConfigSrc = Subject()
        self.addSegmentConfigSink = self.addSegmentConfigSrc.pipe(
            ops.map(lambda _: Repo().addSegment())
        )
        self.removeSegmentConfigSrc = Subject()
        self.removeSegmentConfigSink = self.removeSegmentConfigSrc.pipe()
        self.removeSegmentConfigSrc.subscribe(lambda key: Repo().removeSegment(key))
        
        self.updateSegmentSrc = Subject()
        self.updateSegmentSrc.subscribe(Repo().updateSegment) 
        
        self.generateSegmentsSrc = Subject()
        self.generateSegmentsSrc.subscribe(Repo().generateSegments)
        self.retriveKnobTendonModels = Subject()
        
        self.updateTensionsSrc = Subject()
        self.updateTensionsSrc.subscribe(Repo().updateTensions)
        
        self.computeTensionsSrc = Subject()
        self.computeTensionsSink = self.computeTensionsSrc.pipe(
            ops.map(lambda _: Repo().computeTensions())
        )
        
    def __del__(self):
        for v in self.__dict__.values():
            if isinstance(v, Subject):        
                v.dispose()
                
