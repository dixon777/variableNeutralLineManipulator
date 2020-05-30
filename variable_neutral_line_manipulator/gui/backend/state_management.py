from math import degrees

import numpy as np

import rx
from rx import operators as ops, Observable
from rx.subject import Subject

from ..gui_common import *
from .repo import Repo

def round_val(val):
    return np.round(val, 5)

def format_manipulator(manipulator_state:ManipulatorState):
    s = "Bottom joint angles (deg):\n"
    for i, disk_state in enumerate(manipulator_state.disk_states):
        s += f" {i+1}: {round_val(degrees(disk_state.bottom_joint_angle))}\n"
        
    s += "\nTransformation matrices (Base-disk-orientation):\n"
    for i in range(len(manipulator_state.disk_states)+1):
        s+=f"{i}:\nBottom:\n"
        s+=str(round_val(manipulator_state.get_TF(i,"b","bd")))
        s+="\nTop:\n"
        s+=str(round_val(manipulator_state.get_TF(i,"t","bd")))
        s+="\n"
        
    s += "\nTransformation matrices (Bottom-curvature-orientation):\n"
    for i in range(len(manipulator_state.disk_states)+1):
        s+=f"{i}:\nBottom:\n"
        s+=str(round_val(manipulator_state.get_TF(i,"b","bc")))
        s+="\nTop:\n"
        s+=str(round_val(manipulator_state.get_TF(i,"t","bc")))
        s+="\n"
    return s

class StateManagement(metaclass=Singleton):
    def __init__(self):
        self.initRequest = Subject()
        self.addSegmentConfigRequest = Subject()
        self.removeSegmentConfigRequest = Subject()
        self.updateSegmentConfigRequest = Subject()
        self.generateManipulatorRequest = Subject()
        self.updateTensionsRequest = Subject()
        self.computeStateRequest = Subject()
        
        segment_config_repo_op = ops.merge(
            self.initRequest.pipe(
                ops.map(Repo().publish_init_segments_config)
            ),
            self.addSegmentConfigRequest.pipe(
                ops.map(Repo().add_segment)
            ),
            self.removeSegmentConfigRequest.pipe(
                ops.map(Repo().remove_segment)
            ), 
            self.updateSegmentConfigRequest.pipe(
                ops.map(Repo().update_segment_config)
            )
        )
        self._segment_configs_stream = Observable().pipe(
            segment_config_repo_op,
        )
        
        # TODO
        self._segment_configs_err_stream = Observable().pipe(
            segment_config_repo_op,
            ops.flat_map(lambda x: Observable.just(Repo().get_error()))
        )
        
        self._tension_inputs_stream = Observable().pipe(
            ops.merge(
                self.generateManipulatorRequest.pipe(
                    ops.map(Repo().generate_manipulator)
                ),
                self.updateTensionsRequest.pipe(
                    ops.map(Repo().updateTensions)
                ),
            )
        )
        
        compute_state_result = self.computeStateRequest.pipe(
            ops.map(Repo().computeTensions)
        )
        
        self._text_result_stream = compute_state_result.pipe(
            ops.map(format_manipulator)
        )
        
        self._graph_stream = compute_state_result.pipe(
        )
        
    def request_init_segment_configs(self):
        self.initRequest.on_next(0)
        
    def request_add_segment_config(self):
        self.addSegmentConfigRequest.on_next(0)
        
    def request_remove_segment_config(self, key):
        self.removeSegmentConfigRequest.on_next(key)
        
    def request_update_segment_config(self, config):
        self.updateSegmentConfigRequest.on_next(config)
        
    def request_generate_manipulator(self):
        self.generateManipulatorRequest.on_next(0)
        
    def request_update_tensions(self, tensions):
        self.updateTensionsRequest.on_next(tensions)
        
    def request_compute_state(self):
        self.computeStateRequest.on_next(0)
    
    @property
    def segment_configs_stream(self):
        return self._segment_configs_stream
    
    @property
    def segment_configs_err_stream(self):
        return self._segment_configs_err_stream
    
    @property
    def tension_inputs_stream(self):
        return self._tension_inputs_stream
    
    @property
    def graph_stream(self):
        return self._graph_stream
    
    @property
    def text_result_stream(self):
        return self._text_result_stream
        
    def __del__(self):
        for v in self.__dict__.values():
            if isinstance(v, Subject):        
                v.dispose()
                
