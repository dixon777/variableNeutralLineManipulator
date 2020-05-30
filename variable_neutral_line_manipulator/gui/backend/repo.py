from math import degrees

from uuid import uuid4
from copy import deepcopy

from ..gui_common import *

class Repo(metaclass=Singleton):
    def __init__(self):
        super().__init__()
        self._manipulator_config: ManipulatorConfigDisplayModel = ManipulatorConfigDisplayModel(
            base_disk_length=5,
            outer_diameter=5,
        )
        self._manipulator_model = ManipulatorMathModel()
        self._tension_inputs = TensionInputListDisplayModel()
        
    def publish_init_segments_config(self, *args):
        Logger.D(f"Publish init segments")
        return self._manipulator_config

    def add_segment(self, *args):
        Logger.D(f"Add segment")
        key = uuid4()
        self._manipulator_config.segment_models[key] = SegmentConfigDisplayModel(
            key=key,
            n_joints=1,
            is_2_DoF=False,
            disk_length=5,
            orientationBF=0,
            curve_radius=3,
            tendon_dist_from_axis=1,
            end_disk_length=5,
        )
        return self._manipulator_config

    def remove_segment(self, key):
        Logger.D(f"Remove segment: {key}")
        del self._manipulator_config.segment_models[key]
        return self._manipulator_config

    def update_segment_config(self, config):
        Logger.D(f"Update config: {config}")
        self._manipulator_config = config
        # self._manipulator_config.errors.clear()
        return self._manipulator_config

    def generate_manipulator(self, *args):
        Logger.D("Generate segments")
        self._manipulator_model.update(outer_diameter=self._manipulator_config.outer_diameter,
                                       base_disk_length=self._manipulator_config.base_disk_length,
                                       segment_configs=list(self._manipulator_config.segment_models.values()))
        
        if not self._manipulator_model.generate_models():
            return ErrorDict(self._manipulator_model.error_dict)

        self._tension_inputs.values = [
            [TensionInputDisplayModel(t.orientationBF, 0.0) for t in kt] 
            for _, kt in self._manipulator_model.disk_knobbed_tendons_iterator if kt
        ]
        return self._tension_inputs

    def updateTensions(self, tension_inputs):
        Logger.D(f"Update tensions")
        self._tension_inputs = tension_inputs
        return self._tension_inputs

    def computeTensions(self, *args):
        Logger.D("Compute tensions")
        manipulator_state = eval_manipulator_state(self._manipulator_model, self._tension_inputs.to_pure_values(), solver_type=SolverType.DIRECT)
        return manipulator_state
    