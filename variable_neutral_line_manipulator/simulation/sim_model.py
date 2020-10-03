import os
import json
from math import degrees
from collections.abc import Iterable
from typing import List,  Union, Dict

from .entities import *
from ..util import Logger, normalise_angle
from ..cad.cadquery_disk_generator import generate_disk_CAD, export_CAD
from ..math_model.calculation import eval_tendon_guide_top_end_disp, eval_tendon_guide_bottom_end_disp
from .adam_middleware import *


class _CadCacheManager:
    """
        Manages the retrival and storage of CAD cache 
        For faster reimport of CAD into Adams View by caching them into parasolid format from STEP format
    """
    CONSTANT_KEY_SUB_PATH = "sub_path"
    CONSTANT_KEY_GEOMETRY = "geometry"

    def __init__(self, dir_path, cache_details_file_basename="cad_details.json"):
        super().__init__()
        self._dir_path = os.path.abspath(dir_path)
        os.makedirs(self._dir_path, exist_ok=True)
        self._cache_details_path = self._generate_path(
            cache_details_file_basename)

    def _generate_path(self, *subpaths):
        return os.path.join(self._dir_path, *subpaths)

    @property
    def details(self):
        if not (os.path.exists(self._cache_details_path)):
            return []
        with open(self._cache_details_path, "r") as f:
            content = f.read()
            return json.loads(content, cls=BaseDataClass.Decoder) if content else []

    def store_details(self, details):
        with open(self._cache_details_path, "w") as f:
            return json.dump(details, f, cls=BaseDataClass.Encoder, indent=4)

    def search_path(self, geometry: SimDiskGeometry, check_existance=True):
        cad_details = self.details
        for i, cad_item in enumerate(cad_details):
            if cad_item[self.CONSTANT_KEY_GEOMETRY] == geometry:
                path = self._generate_path(
                    cad_item[self.CONSTANT_KEY_SUB_PATH])
                if not check_existance or os.path.exists(path):
                    return self._generate_path(path)

                # If the identical geometry is in the record but the path does not exist
                Logger.W(
                    f"Error: CAD is marked cached in the file [{self._cache_details_path}] but the path [{path}] does not exist\n"
                    "Entry is removed")

                del cad_details[i]
                self.store_details(cad_details)
                break
        return None

    def _generate_new_entry(self, geometry: SimDiskGeometry):
        cad_details = self.details
        largest_index = -1
        for cad_item in cad_details:
            largest_index = max(
                (largest_index, int(os.path.basename(cad_item[self.CONSTANT_KEY_SUB_PATH]).split(".")[0])))
            if cad_item[self.CONSTANT_KEY_GEOMETRY] == geometry:
                Logger.W("Has duplicate. Terminate caching new model")

        return {
            self.CONSTANT_KEY_SUB_PATH: f"{largest_index+1}.xmt_txt",
            self.CONSTANT_KEY_GEOMETRY: dict(geometry)
        }

    def add_entry(self, geometry: SimDiskGeometry):
        cad_details = self.details
        cad_details.append(self._generate_new_entry(geometry))
        self.store_details(cad_details)

    def remove_entry(self, geometry: SimDiskGeometry):
        filtered_details = [
            cad_item for cad_item in self.CAD_details if cad_item["geometry"] != geometry]
        self.store_details(filtered_details)


class _ManipulatorAdamSimNameGenerator():
    '''Generator which creates meaningful names for all items within Adams View'''

    @staticmethod
    def _convert_angle(angle_in_rad):
        return int(degrees(normalise_angle(angle_in_rad)))

    # Part
    @staticmethod
    def disk_part_name(index):
        return f"disk{index}"

    # Body
    @classmethod
    def disk_body_name(cls, index):
        return f"body_{cls.disk_part_name(index)}"

    # Marker
    ground_base_disk_bottom_marker_name = "ground_base_disk_bottom"
    base_disk_bottom_marker_name = "ma_base_disk_bottom"
    
    @classmethod
    def disk_center_marker_name(cls, index):
        return f"ma_{cls.disk_part_name(index)}_center"

    @classmethod
    def tendon_guide_end_marker_name(cls, index, orientationMF, dist_from_axis, is_top):
        return f"ma_{cls.disk_part_name(index)}_o{cls._convert_angle(orientationMF)}_d{dist_from_axis}_{'top' if is_top else 'bottom'}"

    @classmethod
    def base_tendon_guide_end_floating_marker_name(cls, orientationMF, dist_from_axis):
        return f"ma_base_o{cls._convert_angle(orientationMF)}_d{dist_from_axis}"

    # Var
    var_name_tension_avg = "v_tension_avg"
    var_name_base_duration = "v_base_duration"
    var_name_final_duration = "v_final_duration"

    @classmethod
    def final_tension_mag_var_name(cls, orientationMF, dist_from_axis):
        return f"v_tension_mag_o{cls._convert_angle(orientationMF)}_d{dist_from_axis}"

    # Constraints
    ground_constraint_name = "ground_constraint"

    # Forces
    @classmethod
    def force_contact_name(cls, index):
        return f"f_contact{index}"

    @classmethod
    def force_tension_base_bottom_tendon_guide_end_name(cls, orientationMF, dist_from_axis):
        return f"f_base_tension_o{cls._convert_angle(orientationMF)}_d{dist_from_axis}"

    @classmethod
    def force_tension_between_tendon_guide_ends_name(cls, joint_index, orientationMF, dist_from_axis):
        return f"f_joint{joint_index}_tension_o{cls._convert_angle(orientationMF)}_d{dist_from_axis}"

    # Measurements
    @classmethod
    def measurement_joint_angle_name(cls, joint_index, orientationMF=None, dist_from_axis=None):
        return (f"mm_joint{joint_index}_angle" +
                (f'_o{cls._convert_angle(orientationMF)}' if orientationMF is not None else '') +
                (f"_d{int(dist_from_axis)}" if dist_from_axis is not None else ''))

    # @classmethod
    # def measurement_joint_angle_validation_name(cls, joint_index, orientationMF, dist_from_axis):
    #     return f"mm_joint{joint_index}_angle_validation_o{cls._convert_angle(orientationMF)}_d{dist_from_axis}"

    @classmethod
    def measurement_contact_component_name(cls, disk_index, is_top_surface, item):
        return f"mm_{cls.disk_part_name(disk_index)}_{'top' if is_top_surface else 'bottom'}_contact_{item}"

    @classmethod
    def measurement_all_contact_component_names(cls, disk_index, is_top_surface):
        return [cls.measurement_contact_component_name(disk_index, is_top_surface, item) for item in ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]]

    @classmethod
    def measurement_tension_component_name(cls, disk_index, orientationMF, dist_from_axis, is_top_surface, item):
        return f"mm_{cls.disk_part_name(disk_index)}_tension_o{cls._convert_angle(orientationMF)}_d{dist_from_axis}_{'top' if is_top_surface else 'bottom'}_{item}"

    @classmethod
    def measurement_all_tension_component_names(cls, disk_index, orientationMF, dist_from_axis, is_top_surface):
        return [cls.measurement_tension_component_name(disk_index, orientationMF, dist_from_axis, is_top_surface, item) for item in ["Fx", "Fy", "Fz"]]

    measurement_base_reaaction_names = [
        "base_Fx", "base_Fy", "base_Fz", "base_Tx", "base_Ty", "base_Tz"
    ]


class SimManipulatorAdamModel:
    """
    It contains methods for building and running simulation, extracting info from Adam Views with custom config

    Params:
     - manipulator_model: Manipulator model
     - port: Port listened by Adams View command server
     - cache_dir_path: Temporary file which holds all caches required for simulation
     - configs: All model related configs (Currently it has no effect) TODO
    """
    ADAM_INFO_BUFFER_FILE_PREFIX = ".return_result"
    CAD_CACHE_SUB_DIR = "cad_cache"     # Sub directory for CAD cache
    CONSTANT_MODEL_NAME_IN_ADAM = "MANIPULATOR"

    # Configs
    # Body
    CONFIG_DISK_DENSITY = 0.000008  # kg/mm^3 (Typical steel)

    # Contact
    # Stable for 10 joint 5*10**7, 2.8, 5*10**4, 0.025, 2000, 0.1, 60, 100, 200
    CONFIG_CONTACT_STIFFNESS = 1*10**4
    CONFIG_CONTACT_FORCE_EXPONENT = 1.5
    CONFIG_CONTACT_DAMPING = 1*10**2
    CONFIG_CONTACT_PENETRATION_DEPTH = 0.05

    CONFIG_CONTACT_FRICTION_COEF = 1*10**8
    CONFIG_CONTACT_FRICTION_VEL = 1

    # DEFAULT_OVERLAPPING_LENGTH = 0.005
    DEFAULT_OVERLAPPING_LENGTH = 0.01
    MARKER_OFFSET_FROM_CURVE = 0.011

    # Sim
    CONFIG_SCRIPT_NAME = "iterative_solver_script"

    def __init__(self,
                 manipulator_model: ManipulatorModel,
                 port=5002,  # Should be always 5002
                 cache_dir_path="./.manipulator_adam_cache",
                 configs={}  # TODO
                 ):
        self.manipulator_model = manipulator_model
        self.disk_models: List[DiskModel] = manipulator_model.get_disk_models(
            include_base=True)
        self.tendon_models: List[TendonModel] = manipulator_model.tendon_models
        self.model_name = self.CONSTANT_MODEL_NAME_IN_ADAM
        self._cache_dir_path = os.path.abspath(cache_dir_path)
        os.makedirs(self._cache_dir_path, exist_ok=True)
        self.configs = configs  # TODO

        self.socket: AdamViewSocket = AdamViewSocket(
            port, self._generate_path(self.ADAM_INFO_BUFFER_FILE_PREFIX))

        self.cad_cache = _CadCacheManager(
            self._generate_path(self.CAD_CACHE_SUB_DIR))
        self.name_gen = _ManipulatorAdamSimNameGenerator()

    def _generate_path(self, *subpaths):
        return os.path.join(self._cache_dir_path, *subpaths)

    def _import_CADs(self, disk_models):
        success = True
        for i, model in enumerate(disk_models):
            part_name = self.name_gen.disk_part_name(i)
            disk_geometry = SimDiskGeometry.from_base_geometry(
                model.disk_geometry, model.tendon_models)

            # search from cache
            cad_path = self.cad_cache.search_path(
                disk_geometry, check_existance=True)

            if cad_path is not None:
                success &= self.socket.import_parasolid(
                    path=cad_path, part_name=part_name)

            # Import and cache if it does not exist
            else:
                disk_cad_obj = generate_disk_CAD(length=disk_geometry.length,
                                                 outer_diameter=disk_geometry.outer_diameter,
                                                 bottom_curve_radius=disk_geometry.bottom_curve_radius,
                                                 top_curve_radius=disk_geometry.top_curve_radius,
                                                 top_orientationDF=disk_geometry.top_orientationDF)
                temp_step_file_path = self._generate_path(".temp.step")
                export_CAD(disk_cad_obj, path=temp_step_file_path)

                if(self.socket.create_part_rigid_body(part_name=part_name)):
                    if(self.socket.import_geometry(temp_step_file_path, part_name=part_name)):
                        self.cad_cache.add_entry(disk_geometry)
                        if not (self.socket.export_parasolid(self.cad_cache.search_path(disk_geometry, check_existance=False), part_name)):
                            self.cad_cache.remove_entry(disk_geometry)

                        os.remove(temp_step_file_path)

            if not success:
                return False

            self._modify_geometry_solid_name(i)

        return success

    def _modify_geometry_solid_name(self, index):
        new_name = self.name_gen.disk_body_name(index)

        for j in range(100):  # may lead to error
            if self.socket.rename_entity(f"SOLID{j}", new_name):
                break
        else:
            Logger.E(f"Error: Fail to capture the solid name for disk{index}")
            return False

        return True

    def _create_markers(self, disk_models):
        for i, model in enumerate(disk_models):
            disk_geometry = model.disk_geometry
            part_name = self.name_gen.disk_part_name(i)

            # Redirect path name
            self.socket.modify_part_rigid_body(part_name)

            # Create disk's center marker (may differ from center of mass marker(auto generated))
            self.socket.create_marker(
                self.name_gen.disk_center_marker_name(i),
            )

            for tendon_model in model.tendon_models:
                # Create marker at tendon guide bottom end (including base disk)
                disp = eval_tendon_guide_bottom_end_disp(
                    disk_geometry.length,
                    disk_geometry.bottom_curve_radius,
                    tendon_model.dist_from_axis,
                    tendon_model.orientation - model.bottom_orientationMF)
                # Offset the marker away from the curve surface
                disp[2] += self.MARKER_OFFSET_FROM_CURVE
                self.socket.create_marker(
                    self.name_gen.tendon_guide_end_marker_name(
                        i,
                        tendon_model.orientation,
                        tendon_model.dist_from_axis,
                        is_top=False),
                    location=disp
                )

                # Create marker at tendon guide top end
                if disk_geometry.top_curve_radius:
                    disp = eval_tendon_guide_top_end_disp(
                        disk_geometry.length, disk_geometry.top_curve_radius,
                        tendon_model.dist_from_axis,
                        tendon_model.orientation - model.bottom_orientationMF,
                        disk_geometry.top_orientationDF)
                    # Offset the marker away from the curve surface
                    disp[2] -= self.MARKER_OFFSET_FROM_CURVE
                    self.socket.create_marker(
                        self.name_gen.tendon_guide_end_marker_name(
                            i,
                            tendon_model.orientation,
                            tendon_model.dist_from_axis,
                            is_top=True),
                        location=disp,
                        orientation=(0, 0, disk_geometry.top_orientationDF)
                    )
                    
        # Create base disk bottom marker
        self.socket.modify_part_rigid_body(self.name_gen.disk_part_name(0))
        self.socket.create_marker(self.name_gen.base_disk_bottom_marker_name, 
                                    location=(0,0,-disk_models[0].disk_geometry.length/2),
                                  )
        

    def _move_disks_to_pos(self, disk_init_loc_orientations):
        for i, pos in enumerate(disk_init_loc_orientations):
            self.socket.modify_part_rigid_body(
                self.name_gen.disk_part_name(i), **pos)

    def _define_mass_properties(self, disk_models: List[DiskModel]):
        for i in range(len(disk_models)):
            self.socket.create_part_rigid_body_mass_properties(
                self.name_gen.disk_part_name(i), self.CONFIG_DISK_DENSITY)

    def _generate_parametric_variables(self, tendon_models: List[TendonModel]):
        self.socket.create_variable(self.name_gen.var_name_base_duration, 0.0)
        self.socket.create_variable(self.name_gen.var_name_final_duration, 1.0)
        self.socket.create_variable(self.name_gen.var_name_tension_avg, 1.0)
        for tm in tendon_models:
            self.socket.create_variable(
                self.name_gen.final_tension_mag_var_name(tm.orientation, tm.dist_from_axis), 0.1)

    def _connect_forces_between_tendon_guide_ends(self, disk_models: List[DiskModel]):
        for i, distal_disk_model in enumerate(disk_models[1:]):
            for tm in distal_disk_model.tendon_models:
                self.socket.create_single_component_force(
                    self.name_gen.force_tension_between_tendon_guide_ends_name(
                        i, tm.orientation, tm.dist_from_axis),
                    self.name_gen.tendon_guide_end_marker_name(
                        i+1, tm.orientation, tm.dist_from_axis, is_top=False),
                    self.name_gen.tendon_guide_end_marker_name(
                        i, tm.orientation, tm.dist_from_axis, is_top=True),

                    function=f"-({self.name_gen.var_name_tension_avg}+"
                    f"({self.name_gen.final_tension_mag_var_name(tm.orientation, tm.dist_from_axis)}-{self.name_gen.var_name_tension_avg})*"
                    # f"STEP(TIME, {self.name_gen.var_name_base_duration}, 0, {self.name_gen.var_name_final_duration}, 1))"
                    f"MIN(MAX((TIME-{self.name_gen.var_name_base_duration})/{self.name_gen.var_name_final_duration},0), 1))"
                )
        for tm in distal_disk_model.tendon_models:
            floating_marker_name = self.name_gen.base_tendon_guide_end_floating_marker_name(
                tm.orientation, tm.dist_from_axis
            )
            tendon_guide_end_marker_name = self.name_gen.tendon_guide_end_marker_name(
                0, tm.orientation, tm.dist_from_axis, False
            )
            self.socket.modify_part_rigid_body("ground")
            self.socket.create_vector_force(
                self.name_gen.force_tension_base_bottom_tendon_guide_end_name(
                    tm.orientation, tm.dist_from_axis),
                tendon_guide_end_marker_name,
                "ground",
                tendon_guide_end_marker_name,
                z_force_function=f"-({self.name_gen.var_name_tension_avg}+"
                    f"({self.name_gen.final_tension_mag_var_name(tm.orientation, tm.dist_from_axis)}-{self.name_gen.var_name_tension_avg})*"
                    # f"STEP(TIME, {self.name_gen.var_name_base_duration}, 0, {self.name_gen.var_name_final_duration}, 1))"
                    f"MIN(MAX((TIME-{self.name_gen.var_name_base_duration})/{self.name_gen.var_name_final_duration},0), 1))"
            )

    def _enforce_base_disk_ground_constraint(self):
        self.socket.modify_part_rigid_body("ground")
        self.socket.create_marker(self.name_gen.ground_base_disk_bottom_marker_name,
                                  location=(0,0,-self.disk_models[0].disk_geometry.length/2),
                                  reference_marker_name=self.name_gen.disk_center_marker_name(0))
        self.socket.create_constraint_fixed(self.name_gen.ground_constraint_name,
                                            i_marker_name=self.name_gen.base_disk_bottom_marker_name,
                                            j_marker_name=self.name_gen.ground_base_disk_bottom_marker_name)

    def _generate_contacts(self, disk_models: List[DiskModel]):
        for i in range(len(disk_models)-1):
            self.socket.create_contact(
                self.name_gen.force_contact_name(i),
                self.name_gen.disk_body_name(i+1),
                self.name_gen.disk_body_name(i),
                self.CONFIG_CONTACT_STIFFNESS,
                self.CONFIG_CONTACT_DAMPING,
                self.CONFIG_CONTACT_PENETRATION_DEPTH,
                self.CONFIG_CONTACT_FORCE_EXPONENT,
                self.CONFIG_CONTACT_FRICTION_COEF,
                self.CONFIG_CONTACT_FRICTION_COEF,
                self.CONFIG_CONTACT_FRICTION_VEL,
                self.CONFIG_CONTACT_FRICTION_VEL,
            )

    def _generate_measurement_joint_angles(self, disk_models: List[DiskModel]):
        for i, distal_disk_model in enumerate(disk_models[1:]):
            for j, tendon_model in enumerate(distal_disk_model.tendon_models):
                proximal_top_marker_name = self.name_gen.tendon_guide_end_marker_name(
                    i, tendon_model.orientation, tendon_model.dist_from_axis, True)
                distal_bottom_marker_name = self.name_gen.tendon_guide_end_marker_name(
                    i+1, tendon_model.orientation, tendon_model.dist_from_axis, False)
                self.socket.create_measure_function(
                    self.name_gen.measurement_joint_angle_name(
                        i, tendon_model.orientation, tendon_model.dist_from_axis),
                    function=f"AX({distal_bottom_marker_name}, {proximal_top_marker_name})",
                    unit="angle",
                )

                if j == 0:
                    self.socket.create_measure_function(
                        self.name_gen.measurement_joint_angle_name(i),
                        function=f"AX({distal_bottom_marker_name}, {proximal_top_marker_name})",
                        unit="angle",
                    )

    def _generate_measurement_contact_force(self, disk_models: List[DiskModel]):
        for i in range(len(disk_models)-1):
            # Top surface of i-th disk
            for force_type_index, measurement_name in zip((2, 3, 4, 6, 7, 8), self.name_gen.measurement_all_contact_component_names(i, True)):
                self.socket.create_measure_function(
                    measurement_name,
                    function=f"CONTACT({self.name_gen.force_contact_name(i)}, 1, {force_type_index}, {self.name_gen.disk_center_marker_name(i)})",
                )

            # Bottom surface of (i+1)-th disk
            for force_type_index, measurement_name in zip((2, 3, 4, 6, 7, 8), self.name_gen.measurement_all_contact_component_names(i+1, False)):
                self.socket.create_measure_function(
                    measurement_name,
                    function=f"CONTACT({self.name_gen.force_contact_name(i)}, 0, {force_type_index}, {self.name_gen.disk_center_marker_name(i+1)})",
                )

    def _generate_measurement_tension_vec(self, disk_models: List[DiskModel]):
        for i, model in enumerate(disk_models):
            for ts in model.continuous_tendon_models:
                # Top tensions of i-th disk
                for force_type_index, measurement_name in zip((2, 3, 4), self.name_gen.measurement_all_tension_component_names(i, ts.orientation, ts.dist_from_axis, True)):
                    self.socket.create_measure_function(
                        measurement_name,
                        function=f"SFORCE({self.name_gen.force_tension_between_tendon_guide_ends_name(i, ts.orientation, ts.dist_from_axis)}, 1, {force_type_index}, {self.name_gen.disk_center_marker_name(i)})",
                    )

            if i > 0:
                for ts in model.tendon_models:
                    # Bottom tensions of i-th disk
                    for force_type_index, measurement_name in zip((2, 3, 4), self.name_gen.measurement_all_tension_component_names(i, ts.orientation, ts.dist_from_axis, False)):
                        self.socket.create_measure_function(
                            measurement_name,
                            function=f"SFORCE({self.name_gen.force_tension_between_tendon_guide_ends_name(i-1, ts.orientation, ts.dist_from_axis)}, 0, {force_type_index}, {self.name_gen.disk_center_marker_name(i)})",
                        )

    def _generate_measurement_base_reaction(self):
        for force_type_index, measurement_name in zip((2, 3, 4, 6, 7, 8),
                                                      self.name_gen.measurement_base_reaaction_names,
                                                      ):
            self.socket.create_measure_function(
                measurement_name,
                f"JOINT({self.name_gen.ground_constraint_name}, 0, {force_type_index}, {self.name_gen.base_disk_bottom_marker_name})"
            )

    def _final_cleanup(self):
        self.socket.modify_part_rigid_body("ground")

    def _disks_init_location_orientation(self, initial_disk_overlap_length=DEFAULT_OVERLAPPING_LENGTH):
        yield {"location": (0, 0, 0), "orientation": (0, 0, self.disk_models[0].bottom_orientationMF)}
        disk_length_accumulate = self.disk_models[0].disk_geometry.length/2
        for model in self.disk_models[1:]:
            disk_length_accumulate += model.disk_geometry.length/2 - initial_disk_overlap_length
            yield {"location": (0, 0, disk_length_accumulate), "orientation": (0, 0, model.bottom_orientationMF)}
            disk_length_accumulate += model.disk_geometry.length/2

    def _eval_tension_mags_at_step(self, final_values, current_step, total_step_until_final):
        res = []
        avg = np.average(final_values)
        for final_val in final_values:
            ratio = current_step / total_step_until_final
            slope = 1.1*(final_val - avg)
            a = slope - 2*final_val + 2*avg
            b = final_val - avg - a
            res.append(a*ratio**3+b*ratio**2+avg)

            # Linear
            # res.append((final_val-avg)*ratio + avg)
        return res

    def _create_debug_measurements(self):
        # debug
        for i in range(len(self.disk_models)-1):
            self.socket.create_measure_function(f"dm_0_{i+1}",
                                                f"DM(disk0_center, disk{i+1}_center)",
                                                #  should_display=True
                                                )
            self.socket.create_measure_function(f"dm_{i}_{i+1}",
                                                f"DM(disk{i}_center, disk{i+1}_center)",
                                                # should_display=True
                                                )

        for i in range(len(self.disk_models)-1):
            self.socket.create_measure_function(
                f"c_f{i}",
                function=f"CONTACT({self.name_gen.force_contact_name(i)}, 0, 1, {self.name_gen.disk_center_marker_name(i+1)})",
                # should_display=True
            )

    def _extract_steady_state_one_component_from_spreadsheet(self, result_set_name, component_name="Q") -> Union[str,List[str]]: 
        """
            component_name: The component name(s) defined in result set. (usually 'TIME' or 'Q' (the value))
        """
        res = self.socket.extract_steady_state_from_spread_sheet(result_set_name)
        if isinstance(component_name, str):
            return res.get(component_name, None)
        elif isinstance(component_name, list):
            return [res.get(c, None) for c in component_name]
        else:
            raise ValueError()

    def _validate_state(self):
        res = True
        for i, distal_disk_model in enumerate(self.disk_models[1:]):
            first_angle = self._extract_steady_state_one_component_from_spreadsheet(
                self.name_gen.measurement_joint_angle_name(i)
            )
            for j, tendon_model in enumerate(distal_disk_model.tendon_models):
                other_angle = self._extract_steady_state_one_component_from_spreadsheet(
                    self.name_gen.measurement_joint_angle_name(i,
                                                               tendon_model.orientation,
                                                               tendon_model.dist_from_axis))
                if first_angle != other_angle:
                    print(
                        f"Angles at joint{i}, orientation {degrees(tendon_model.orientation)} deg, dist {tendon_model.dist_from_axis} are not inconsistent with others")
                    print(f" First angle = {degrees(first_angle)}")
                    print(f" This angle = {degrees(other_angle)}\n")
                    res = False
        return res

    def generate_model(self, initial_disk_overlap_length=DEFAULT_OVERLAPPING_LENGTH):
        """
        Config and set up the simulation model
        """
        # Pick c++ implementation solver
        self.socket.config_sim_general("cplusplus")

        # set default unit
        self.socket.set_default_units(
            force_unit="newton",
            mass_unit="kg",
            length_unit="mm",
            time_unit="second",
            angle_unit="radian")

        self.socket.create_model(self.model_name)

        disk_models = self.disk_models
        tendon_models = self.tendon_models

        if not self._import_CADs(disk_models):
            Logger.E("Fail to import CAD")

        self._define_mass_properties(disk_models)
        self._create_markers(disk_models)
        self._move_disks_to_pos(
            self._disks_init_location_orientation(initial_disk_overlap_length))
        self._generate_parametric_variables(tendon_models)
        self._enforce_base_disk_ground_constraint()
        self._connect_forces_between_tendon_guide_ends(
            self.disk_models)
        self._generate_contacts(disk_models)
        self._generate_measurement_joint_angles(disk_models)
        self._generate_measurement_contact_force(disk_models)
        self._generate_measurement_tension_vec(disk_models)
        self._generate_measurement_base_reaction()

        # self._create_debug_measurements()
        self._final_cleanup()

    def clear_model(self):
        """
        Remove model. Meaningful only when it is followed by generate_model()
        """
        self.socket.delete_model(self.model_name)
        
    def _execute_static_sim(self, duration, num_steps, percentage_per_joint_angle_validation):
        """
            Execute static simulation while keeping track its progress and detecting any failure
            @param
            duration: Total duration between initial state and final state
            num_steps: Total number of steps to reach the final state from the initial state
            percentage_per_joint_angle_validation: Percentage of progress at which it checks whether the orientations of cables belonging to the same joint are the same
        """
        duration_per_step = duration/num_steps
        duration_between_validation = duration*percentage_per_joint_angle_validation
        
        cur_end_time = 0.0
        while cur_end_time < duration:
            next_check_end_time = min(
                cur_end_time + duration_between_validation, duration)
            yield cur_end_time, next_check_end_time # Allow the caller func to display the state updates to user
            
            while cur_end_time < next_check_end_time:
                next_end_time = min(cur_end_time + duration_per_step, next_check_end_time)
                self.socket.run_sim_transient(self.model_name,
                                            end_time=next_end_time,
                                            number_of_steps=1,
                                            solver_type="STATIC",)
                cur_end_time_in_sim = self._extract_steady_state_one_component_from_spreadsheet(self.name_gen.measurement_joint_angle_name(0), component_name="TIME")
                if cur_end_time_in_sim is None or cur_end_time_in_sim <= cur_end_time:
                    raise RuntimeError(f"Static simulation cannot be performed {'at the start' if cur_end_time_in_sim is None else f'from {cur_end_time_in_sim} to {next_end_time}'}."
                                       "\nIt may be solved by increasing max iterations for static simulation, or adjusting other parameters.")
                cur_end_time = next_end_time
                
            if not self._validate_state():
                raise RuntimeError(f"The joint angle of the tendons are incorrect at {next_check_end_time}")
            cur_end_time = next_check_end_time
            
    def run_sim(self,
                input_forces,
                max_iterations_search_eqilibrium=None,
                num_steps=None,
                percentage_per_joint_angle_validation=0.5,
                solver_stability=None,
                solver_translational_limit=None,
                solver_rotational_limit=None,
                solver_static_method=None,
                initial_disk_overlap_length=DEFAULT_OVERLAPPING_LENGTH,):
        """
        Run simulation on the model and extract its final steady state results. 
        generate_model() should have been run.
        """
        duration = 1 # Fixed to 1 time unit 
        
        if len(self.tendon_models) != len(input_forces):
            raise ValueError(
                "Num of tension inputs does not match num of tendons")

        # if bool(num_steps is None) == bool(step_size is None):
        #     raise ValueError("Either and only either \'num_steps\' or \'step_size\' is a positive integer")

        # # Convert num_steps to step_size if the incremental param is given as num_steps
        # if step_size is None:
        #     step_size = duration / num_steps

        # Reset state
        self.socket.run_sim_reset(self.model_name)

        # Increase static equilibrium iteration numbers
        self.socket.set_sim_equilibrium_param(
            self.model_name,
            max_iterations=max_iterations_search_eqilibrium,
            tlimit=solver_translational_limit,
            alimit=solver_rotational_limit,
            stability=solver_stability,
            static_method=solver_static_method)

        # Update variable
        self.socket.set_variable(
            self.name_gen.var_name_base_duration, 0
        )
        self.socket.set_variable(
            self.name_gen.var_name_final_duration, duration
        )
        self.socket.set_variable(
            self.name_gen.var_name_tension_avg, np.average(input_forces)
        )

        for tm, val in zip(self.tendon_models, input_forces):
            self.socket.set_variable(
                self.name_gen.final_tension_mag_var_name(
                    tm.orientation,
                    tm.dist_from_axis),
                val
            )

        # Run simulation
        self.socket.run_sim_equilibrium(self.model_name)

        # Make the simulation generate the records the disks' positions for next iteration
        self.socket.set_auto_plot_param("Last_run", True)

        for cur_end_time, next_end_time in self._execute_static_sim(duration, num_steps, percentage_per_joint_angle_validation):
            print(f"Progress: {cur_end_time/duration*100:.2f}-{next_end_time/duration*100:.2f}%")

        print(f"Progress: Completed")

    def extract_final_state(self):
        if not self._validate_state():
            print(
                "Warning: Joint angles are not consistent, thus the assumption is incorrect")

        # Extract input forces
        input_forces = []
        for tm in self.tendon_models:
            input_forces.append(self.socket.get_variable_real_value(
                self.name_gen.final_tension_mag_var_name(
                    tm.orientation,
                    tm.dist_from_axis)
            ))

        # Extract disk states
        disk_states = []
        for i, model in enumerate(self.disk_models):
            if i < len(self.disk_models) - 1:
                top_force_moment = [self._extract_steady_state_one_component_from_spreadsheet(
                    contact_component,
                ) for contact_component in self.name_gen.measurement_all_contact_component_names(i, True)]

                top_joint_angle = self._extract_steady_state_one_component_from_spreadsheet(
                    self.name_gen.measurement_joint_angle_name(i))

            if i > 0:
                bottom_force_moment = [self._extract_steady_state_one_component_from_spreadsheet(
                    contact_component,
                ) for contact_component in self.name_gen.measurement_all_contact_component_names(i, False)]

                bottom_joint_angle = self._extract_steady_state_one_component_from_spreadsheet(
                    self.name_gen.measurement_joint_angle_name(i-1))
            else:
                bottom_force_moment = [self._extract_steady_state_one_component_from_spreadsheet(
                    component,
                ) for component in self.name_gen.measurement_base_reaaction_names]
                bottom_joint_angle = None

            knobbed_tendon_states = []
            continuous_tendon_states = []
            for tendon_models, tendon_states_container in [(model.knobbed_tendon_models, knobbed_tendon_states),
                                                 (model.continuous_tendon_models, continuous_tendon_states)]:
                for tm in tendon_models:
                    if i > 0:
                        bottom_tension_vec = [self._extract_steady_state_one_component_from_spreadsheet(
                            component,
                        ) for component in self.name_gen.measurement_all_tension_component_names(i, tm.orientation, tm.dist_from_axis, False)]
                    else:
                        bottom_tension_vec = None
                        
                    if i < len(self.disk_models) - 1:
                        top_tension_vec = [self._extract_steady_state_one_component_from_spreadsheet(
                            component,
                        ) for component in self.name_gen.measurement_all_tension_component_names(i, tm.orientation, tm.dist_from_axis, True)]
                    else:
                        top_tension_vec = None
                        
                    tendon_states_container.append(TendonState(
                        tm,
                        bottom_tension_vec,
                        top_tension_vec
                    ))

            disk_states.append(DiskState(
                model,
                bottom_contact_forceDF=bottom_force_moment[:3],
                bottom_contact_pure_momentDF=bottom_force_moment[3:],
                bottom_joint_angle=bottom_joint_angle if i > 0 else None,

                top_contact_forceDF=top_force_moment[:3]
                if i < len(self.disk_models) - 1 else None,
                top_contact_pure_momentDF=top_force_moment[3:]
                if i < len(self.disk_models) - 1 else None,
                top_joint_angle=top_joint_angle
                if i < len(self.disk_models) - 1 else None,
                knobbed_tendon_states=knobbed_tendon_states,
                continuous_tendon_states=continuous_tendon_states,
            ))
        # for i in range(1, len(self.disk_models)):
        #     model = self.disk_models[i]
        #     if i < len(self.disk_models) - 1:
        #         top_force_moment = [self._extract_steady_state_one_component_from_spreadsheet(
        #             contact_component,
        #         ) for contact_component in self.name_gen.measurement_all_contact_component_names(i, True)]

        #         top_joint_angle = self._extract_steady_state_one_component_from_spreadsheet(
        #             self.name_gen.measurement_joint_angle_name(i))

        #     bottom_force_moment = [self._extract_steady_state_one_component_from_spreadsheet(
        #         contact_component,
        #     ) for contact_component in self.name_gen.measurement_all_contact_component_names(i, False)]

        #     bottom_joint_angle = self._extract_steady_state_one_component_from_spreadsheet(
        #         self.name_gen.measurement_joint_angle_name(i-1))

        #     knobbed_tendon_states = []
        #     continuous_tendon_states = []
        #     for tendon_models, tendon_states in [(model.knobbed_tendon_models, knobbed_tendon_states),
        #                                          (model.continuous_tendon_models, continuous_tendon_states)]:
        #         for tm in tendon_models:
        #             bottom_tension_vec = [self._extract_steady_state_one_component_from_spreadsheet(
        #                 component,
        #             ) for component in self.name_gen.measurement_all_tension_component_names(i, tm.orientation, tm.dist_from_axis, False)]
        #             top_tension_vec = [self._extract_steady_state_one_component_from_spreadsheet(
        #                 component,
        #             ) for component in self.name_gen.measurement_all_tension_component_names(i, tm.orientation, tm.dist_from_axis, True)]

        #             tendon_states.append(TendonState(
        #                 tm,
        #                 bottom_tension_vec if all(
        #                     [c is not None for c in bottom_tension_vec]) else None,
        #                 top_tension_vec if all(
        #                     [c is not None for c in top_tension_vec]) else None
        #             ))

        #     disk_states.append(DiskState(
        #         model,
        #         bottom_contact_forceDF=bottom_force_moment[:3] if i > 0 else None,
        #         bottom_contact_pure_momentDF=bottom_force_moment[3:] if i > 0 else None,
        #         bottom_joint_angle=bottom_joint_angle if i > 0 else None,

        #         top_contact_forceDF=top_force_moment[:3]
        #         if i < len(self.disk_models) - 1 else None,
        #         top_contact_pure_momentDF=top_force_moment[3:]
        #         if i < len(self.disk_models) - 1 else None,
        #         top_joint_angle=top_joint_angle
        #         if i < len(self.disk_models) - 1 else None,
        #         knobbed_tendon_states=knobbed_tendon_states,
        #         continuous_tendon_states=continuous_tendon_states,
        #     ))

        return ManipulatorState(self.manipulator_model, input_forces, disk_states)
