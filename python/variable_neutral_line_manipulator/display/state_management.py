import rx
from rx import operators as ops
from rx.subject import Subject

from ..common import Singleton
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
        self.updateSegmentSink = self.updateSegmentSrc.pipe(
            ops.map(Repo().updateSegment)
        )
        # self.updateSegmentSink.subscribe(lambda _: _a) 
        
        self.generateSegmentsSrc = Subject()
        self.retriveKnobTendonModels = self.generateSegmentsSrc.pipe(
            ops.map(Repo().generateSegments)
        )
        self.retriveKnobTendonModels.subscribe(lambda _: None)
        
        self.updateTensionsSrc = Subject()
        self.updateTensionsSrc.subscribe(Repo().updateTensions)
        
        self.computeTensionsSrc = Subject()
        self.computeTensionsSink = self.computeTensionsSrc.pipe(
            ops.map(lambda _: Repo().computeTensions())
        )
        
        self.graphResizeUpdateSrc = Subject()
        self.graphResizeUpdateSink = self.graphResizeUpdateSrc.pipe(
            ops.debounce(2.0)
        )
        
        
    def __del__(self):
        for v in self.__dict__.values():
            if isinstance(v, Subject):        
                v.dispose()
                
