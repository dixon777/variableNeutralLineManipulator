import time
import socket
import os
import re
import json
from math import degrees, radians
from collections.abc import Iterable

from .entities import *
from ..cad.cadquery_disk_generator import generate_disk_CAD, export_CAD
from ..common.calculation import normalise_angle
from ..math_model.calculation import eval_tendon_guide_top_end_disp, eval_tendon_guide_bottom_end_disp


def _to_Adam_val(v):
    if isinstance(v, str):
        new_v = v.replace('"', '\'')
        return f"\"{new_v}\""
    elif isinstance(v, bool):
        return "yes" if v else "no"
    elif isinstance(v, (int, float)):
        return f"{v}"
    elif isinstance(v, Iterable):
        return ",".join(map(_to_Adam_val, v))

    raise ValueError(f"Unsupported type for conversion: {type(v)}")


def adam_construct_cmd(init, params={}, without_prefix=False):
    s = f"{'cmd ' if not without_prefix else ''}{init} "
    for p in params:
        if params[p] is None:
            continue
        s += f"{p} = {_to_Adam_val(params[p])} "
    return s


def adam_read_spreadsheet(path):
    import re
    res = {}
    with open(path, "r") as f:
        for i in range(6):
            next(f)

        # read attribute
        attr_line = f.readline()

        lines = f.read()

        # read values at equilibrium
        equilibrium_val_line = lines.splitlines()[-4]

    attrs = [re.search("(?<=.)\w+(?=\")", s).group(0)
             for s in attr_line.split('\t')]
    vals = [float(v) for v in equilibrium_val_line.split('\t')]
    return {attr: val for attr, val in zip(attrs, vals)}


class AdamDetails(BaseDataClass):
    def __init__(self, path):
        with open(path, "r") as f:
            self.content = f.read()

    @property
    def local_attr_keys(self):
        return ["content"]


class AdamViewSocket:
    def __init__(self, port=5002, return_text_prefix="./.return_result"):
        self.port = port
        self.return_text_prefix = os.path.abspath(return_text_prefix)
        self.reset_script_generated = False

    def deal_with_command(self, cmd, params=None):
        if params:
            cmd = adam_construct_cmd(cmd, params)
        # print(f"Send: {str(cmd)}")
        # Must reconnect the server before every command is sent
        self.soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.soc.connect(("localhost", self.port))
        if isinstance(cmd, str):
            cmd = cmd.encode("ascii")
        self.soc.send(cmd)
        data = self.soc.recv(1024)
        success = data[-1] == ord('0')
        if not success:
            print(f"Failure: {cmd}")
        return success

    # Default

    def set_default_units(self, force_unit=None, mass_unit=None, length_unit=None, time_unit=None, angle_unit=None):
        return self.deal_with_command(adam_construct_cmd("default units", {
            "force": force_unit,
            "mass": mass_unit,
            "length": length_unit,
            "time": time_unit,
            "angle": angle_unit,
        }))

    # Entity (for geometry)
    def rename_entity(self, name, new_name):
        return self.deal_with_command(adam_construct_cmd("entity modify", {
            "entity_name": name,
            "new_entity_name": new_name
        }))

    # Models
    def create_model(self, model_name):
        return self.deal_with_command(adam_construct_cmd("model create", {
            "model_name": model_name
        }))

    def delete_model(self, model_name):
        return self.deal_with_command(adam_construct_cmd("model delete", {
            "model_name": model_name
        }))

    # Parts
    def create_part_rigid_body(self, part_name, comments=None, is_ground_part=None):
        return self.deal_with_command(adam_construct_cmd("part create rigid_body name_and_position", {
            "part_name": part_name,
            "comments": comments,
            "ground_part": is_ground_part,
        }))

    def modify_part_rigid_body(self, part_name, location=None, orientation=None, relative_to=None, is_ground_part=None):
        return self.deal_with_command(adam_construct_cmd("part modify rigid_body name_and_position", {
            "part_name": part_name,
            "location": location,
            "orientation": orientation,
            "relative_to": relative_to,
            "ground_part": is_ground_part,
        }))

    def create_part_rigid_body_mass_properties(self, part_name, density):
        return self.deal_with_command(adam_construct_cmd("part create rigid_body mass_properties", {
            "part_name": part_name,
            "density": density,
        }))

    # Markers
    def create_marker(self, marker_name, location=None, orientation=None, relative_to=None, reference_marker_name=None):
        return self.deal_with_command(adam_construct_cmd("marker create", {
            "marker_name": marker_name,
            "location": location,
            "orientation": orientation,
            "relative_to": relative_to,
            "reference_marker_name": reference_marker_name
        }))

    def modify_marker(self, marker_name, new_marker_name=None, location=None, orientation=None, relative_to=None, reference_marker_name=None):
        return self.deal_with_command(adam_construct_cmd("marker modify", {
            "marker_name": marker_name,
            "new_marker_name": new_marker_name,
            "location": location,
            "orientation": orientation,
            "relative_to": relative_to,
            "reference_marker_name": reference_marker_name
        }))

    def create_floating_marker(self, marker_name):
        return self.deal_with_command(adam_construct_cmd("floating_marker create", {
            "marker_name": marker_name,
        }))

    # Forces
    def create_single_component_force(self, force_name, i_marker_name, j_marker_name, function,):
        return self.deal_with_command(adam_construct_cmd("force create direct single_component_force", {
            "single_component_force_name": force_name,
            "i_marker_name": i_marker_name,
            "j_marker_name": j_marker_name,
            "action_only": "off",
            "function": function
        }))

    # Contact
    def create_contact(self, name, i_geometry_name, j_geometry_name, stiffness, damping, max_intrusion, exponent, mu_static, mu_dynamic, friction_transition_velocity, stiction_transition_velocity):
        return self.deal_with_command(adam_construct_cmd("contact create", {
            "contact_name": name,
            "i_geometry_name": i_geometry_name,
            "j_geometry_name": j_geometry_name,
            "stiffness": stiffness,
            "damping": damping,
            "dmax": max_intrusion,
            "exponent": exponent,
            "coulomb_friction": "on",
            "mu_static": mu_static,
            "mu_dynamic": mu_dynamic,
            "friction_transition_velocity": friction_transition_velocity,
            "stiction_transition_velocity": stiction_transition_velocity,
            "augmented_lagrangian_formulation": False,
        }))

    # Constraints
    def create_constraint_fixed(self, name, i_part_name=None, j_part_name=None, i_marker_name=None, j_marker_name=None, location=None, orientation=None, relative_to=None):
        return self.deal_with_command(adam_construct_cmd("constraint create joint fixed", {
            "joint_name": name,
            "i_part_name": i_part_name,
            "j_part_name": j_part_name,
            "i_marker_name": i_marker_name,
            "j_marker_name": j_marker_name,
            "location": location,
            "orientation": orientation,
            "relative_to": relative_to
        }))

    def create_constraint_planar_joint(self, name, i_part_name=None, j_part_name=None, i_marker_name=None, j_marker_name=None, location=None, orientation=None, relative_to=None):
        return self.deal_with_command(adam_construct_cmd("constraint create joint planar", {
            "joint_name": name,
            "i_part_name": i_part_name,
            "j_part_name": j_part_name,
            "i_marker_name": i_marker_name,
            "j_marker_name": j_marker_name,
            "location": location,
            "orientation": orientation,
            "relative_to": relative_to
        }))

    # Variables
    def create_variable(self, name, value, unit=None):
        return self.deal_with_command(adam_construct_cmd("variable create", {
            "variable_name": name,
            "real_value": value,
            "unit": unit,
            "use_range": False,
            "use_allowed_values": False,
        }))

    def set_variable(self, name, value, unit=None):
        return self.deal_with_command(adam_construct_cmd("variable modify", {
            "variable_name": name,
            "real_value": value,
            "unit": unit,
            "use_range": False,
            "use_allowed_values": False,
        }))

    # Measures
    def create_measure_function(self, name, function, unit=None, should_display=False):
        return self.deal_with_command(adam_construct_cmd("measure create function", {
            "measure_name": name,
            "function": function,
            "units": unit,
            "create_measure_display": should_display,
        }))

    # SImulation
    def create_sim_script(self, name, commands):
        return self.deal_with_command("simulation script create", {
            "sim_script_name": name,
            "commands": commands
        })

    def modify_sim_script(self, name, commands):
        return self.deal_with_command("simulation script modify", {
            "sim_script_name": name,
            "commands": commands
        })

    def delete_sim_script(self, name):
        return self.deal_with_command("simulation script delete", {
            "sim_script_name": name,
        })

    def import_sim_script(self, script_name, path):
        return self.deal_with_command("simulation script read_acf", {
            "sim_script_name": script_name,
            "file_name": os.path.abspath(path)
        })

    def export_sim_script(self, script_name, path):
        return self.deal_with_command("simulation script write_acf", {
            "sim_script_name": script_name,
            "file_name": os.path.abspath(path)
        })

    def run_sim_script(self, model_name, script_name):
        return self.deal_with_command(adam_construct_cmd("simulation single_run scripted", {
            "model_name": model_name,
            "sim_script_name": script_name,
            "reset_before_and_after": False,
        }))

    def set_sim_general(self, server_choice=None):
        return self.deal_with_command(adam_construct_cmd("simulation set", {
            "choice_for_solver": server_choice
        }))

    def set_sim_equilibrium_param(self, model_name, max_iterations=None):
        return self.deal_with_command(adam_construct_cmd("executive_control set equilibrium_parameters", {
            "model_name": model_name,
            "maxit": max_iterations
        }))

    def run_sim_equilibrium(self, model_name):
        return self.deal_with_command(adam_construct_cmd("simulation single_run equilibrium", {
            "model_name": model_name,
        }))

    def run_sim_transient(self, model_name, step_size, solver_type="DYNAMIC", end_time=None, duration=None, initial_static=None):
        return self.deal_with_command("simulation single_run transient", {
            "model_name": model_name,
            "initial_static": initial_static,
            "type": solver_type,
            "step_size": step_size,
            "end_time": end_time,
            "duration": duration,
        })

    def run_sim_reset(self, model_name):
        # Adam View does not respond correctly with "simulation single_run reset"
        # thus, the following strategy involves writing a command script to the model
        # and activate it whenever this function gets called
        script_name = "reset_script_AUTO_GENERATED"
        if not self.reset_script_generated:
            self.create_sim_script(script_name, "simulation single_run reset")
            self.reset_script_generated = True
        return self.run_sim_script(model_name, script_name)
        # return self.deal_with_command("simulation single_run reset", {
        # })

    # Imports

    def import_geometry(self, path, part_name=None, type_of_geometry="stp", location=None, orientation=None, relative_to=None):
        return self.deal_with_command(adam_construct_cmd("file geometry read", {
            "type_of_geometry": type_of_geometry,
            "file": os.path.abspath(path),
            # mutually exclusive with part_name
            "model_name": self.current_model_name if not part_name else None,
            "part_name": part_name,
            "location": location,
            "orientation": orientation,
            "relative_to": relative_to,
        }))

    def import_parasolid(self, path, part_name=None, encoding="ascii", location=None, orientation=None, relative_to=None):
        return self.deal_with_command(adam_construct_cmd("file parasolid read", {
            "file_name": os.path.abspath(path),
            "type": encoding,
            "model_name": self.current_model_name if not part_name else None,
            "part_name": part_name,
            "location": location,
            "orientation": orientation,
            "relative_to": relative_to,
        }))

    def import_cmd(self, path):
        return self.deal_with_command("file command read", {
            "file_name": os.path.abspath(path),
        })

    # Exports
    def export_parasolid(self, path, part_name=None, encoding="ascii"):
        return self.deal_with_command(adam_construct_cmd("file parasolid write", {
            "file_name": os.path.abspath(path),
            "type": encoding,
            "model_name": self.current_model_name if not part_name else None,
            "part_name": part_name,
        }))

    def export_spread_sheet(self, path, result_names):
        return self.deal_with_command(adam_construct_cmd("file spread_sheet write", {
            "file_name": os.path.abspath(path),
            "result_set_name": result_names,
        }))

    def extract_spread_sheet(self, result_names):
        path = f"{self.return_text_prefix}.tab"
        if os.path.exists(path):
            os.remove(path)
        return adam_read_spreadsheet(path) if self.export_spread_sheet(path, result_names) else None

    # Plots
    def set_auto_plot_param(self, analysis_name,
                            include_displacements=False,
                            include_velocity=False):
        return self.deal_with_command(adam_construct_cmd("xy_plots auto_plot output_requests", {
            "analysis_name": analysis_name,
            "displacements": "on" if include_displacements else "off",
            "velocities": "on" if include_velocity else "off",
        }))

    def create_plot(self, plot_name,  data_name=None, measurement_name=None):
        return self.deal_with_command("xy_plots curve create", {
            "plot_name": plot_name,
            "vaxis_data": data_name,
            "vmeasure": measurement_name,
        })

    # Info
    def get_info(self, item_name, dictionary):
        path = os.path.exists(f"{self.return_text_prefix}.txt")
        os.remove(path)
        return AdamDetails(path) if self.deal_with_command(adam_construct_cmd(f"list_info {item_name}", {
            "write_to_terminal": "off",
            "brief": "on",
            "file_name": os.path.abspath(path),
            **dictionary
        })) else None

    def get_model_info(self, model_name=None):
        return self.get_info("model", {
            "model_name": model_name,
        })

    def get_part_info(self, part_name=None):
        return self.get_info("part", {
            "model_name": part_name,
        })

    def get_entity_info(self, entity_name=None):
        return self.get_info("entity", {
            "entity_name": entity_name,
        })

    def get_geometry_info(self, geometry_name=None):
        return self.get_info("geometry", {
            "geometry_name": geometry_name,
        })

    def get_database_names(self, entity_name=None):
        return self.get_info("names", {
            "entity_name": entity_name,
        })

    def get_default_settings(self):
        return self.get_info("defaults", {})


class CADCacheManager:
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
            if cad_item["geometry"] == geometry:
                path = self._generate_path(cad_item["path"])
                if not check_existance or os.path.exists(path):
                    return path

                # If the identical geometry is in the record but the path does not exist
                print(
                    f"Error: CAD is marked cached in the file [{self._cache_details_path}] but the path [{path}] does not exist")
                print("Entry is removed")
                del cad_details[i]
                self.store_details(cad_details)
                break
        return None

    def _generate_new_entry(self, geometry: SimDiskGeometry):
        cad_details = self.details
        largest_index = -1
        for cad_item in cad_details:
            largest_index = max(
                (largest_index, int(os.path.basename(cad_item["path"]).split(".")[0])))
            if cad_item["geometry"] == geometry:
                print("Has duplicate. Terminate caching new model")

        return {
            "path": self._generate_path(f"{largest_index+1}.xmt_txt"),
            "geometry": dict(geometry)
        }

    def add_entry(self, geometry: SimDiskGeometry):
        cad_details = self.details
        cad_details.append(self._generate_new_entry(geometry))
        self.store_details(cad_details)

    def remove_entry(self, geometry: SimDiskGeometry):
        filtered_details = [
            cad_item for cad_item in self.CAD_details if cad_item["geometry"] != geometry]
        self.store_details(filtered_details)


class SimManipulatorAdamNameGenerator():

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
        return f"b_{cls.disk_part_name(index)}"

    # Marker
    @classmethod
    def disk_center_marker_name(cls, index):
        return f"ma_{cls.disk_part_name(index)}_center"

    @classmethod
    def tendon_guide_end_marker_name(cls, index, orientationMF, is_top):
        return f"ma_{cls.disk_part_name(index)}_{cls._convert_angle(orientationMF)}_{'top' if is_top else 'bottom'}"

    # Var
    var_name_tension_avg = "v_tension_avg"
    var_name_base_duration = "v_base_duration"
    var_name_final_duration = "v_final_duration"

    @classmethod
    def final_tension_meg_var_name(cls, orientationMF, dist_from_axis=None):
        return (f"v_tension_mag_{cls._convert_angle(orientationMF)}"
                f"{f'_{dist_from_axis}' if dist_from_axis else ''}")

    # Forces
    @classmethod
    def force_contact_name(cls, index):
        return f"contact{index}"

    @classmethod
    def force_tension_between_tendon_guide_ends_name(cls, joint_index, orientationMF, dist_from_axis=None):
        return (f"f_joint{joint_index}_tension_{cls._convert_angle(orientationMF)}"
                f"{f'_{dist_from_axis}' if dist_from_axis else ''}")

    # Measurements
    @classmethod
    def measurement_joint_angle_name(cls, joint_index):
        return f"mm_joint{joint_index}_angle"

    @classmethod
    def measurement_contact_component_name(cls, disk_index, is_top_surface, item):
        return f"mm_{cls.disk_part_name(disk_index)}_{'top' if is_top_surface else 'bottom'}_contact_{item}"

    @classmethod
    def measurement_all_contact_component_names(cls, disk_index, is_top_surface):
        return [cls.measurement_contact_component_name(disk_index, is_top_surface, item) for item in ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]]


class SimManipulatorAdamExecuter:
    ADAM_INFO_BUFFER_FILE_PREFIX = ".return_result"
    CAD_CACHE_SUB_PATH = "cad_cache"
    MARKER_OFFSET_FROM_CURVE = 0.01
    TIME_STEP_PER_ITERATION = 100

    # Configs
    # Body
    CONFIG_DISK_DENSITY = 0.000008

    # Contact
    # Stable for 10 joint 5*10**7, 2.8, 5*10**4, 0.025, 2000, 0.1, 60, 100, 200
    CONFIG_CONTACT_STIFFNESS = 1*10**4
    CONFIG_CONTACT_FORCE_EXPONENT = 1.5
    CONFIG_CONTACT_DAMPING = 1*10**2
    CONFIG_CONTACT_PENETRATION_DEPTH = 0.05

    CONFIG_CONTACT_FRICTION_COEF = 2*10**7
    CONFIG_CONTACT_FRICTION_VEL = 1

    DEFAULT_OVERLAPPING_LENGTH = 0.005

    # Sim
    CONFIG_SCRIPT_NAME = "iterative_solver_script"

    def __init__(self,
                 manipulatorModel: ManipulatorModel,
                 port=5002,
                 model_name="MANIPULATOR",
                 temp_dir_path="./.temp_adam"):
        self.disk_models: List[DiskModel] = manipulatorModel.get_disk_models(
            include_base=True)
        self.tendon_models: List[TendonModel] = manipulatorModel.tendon_models
        self.model_name = model_name
        self.temp_dir_path = os.path.abspath(temp_dir_path)
        os.makedirs(self.temp_dir_path, exist_ok=True)

        self.socket: AdamViewSocket = AdamViewSocket(
            port, self._generate_path(self.ADAM_INFO_BUFFER_FILE_PREFIX))

        self.cad_cache = CADCacheManager(
            self._generate_path(self.CAD_CACHE_SUB_PATH))
        self.name_gen = SimManipulatorAdamNameGenerator()

    def _generate_path(self, *subpaths):
        return os.path.join(self.temp_dir_path, *subpaths)

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
            print(f"Error: Fail to capture the solid name for disk{index}")
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
                # Create marker at tendon guide bottom end
                if disk_geometry.bottom_curve_radius:
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
                            is_top=True),
                        location=disp,
                        orientation=(0, 0, disk_geometry.top_orientationDF)
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
        for tg in tendon_models:
            self.socket.create_variable(
                self.name_gen.final_tension_meg_var_name(tg.orientation), 0.1)

    def _connect_forces_between_tendon_guide_ends(self, disk_models: List[DiskModel]):
        for i, (proximal_disk_model, distal_disk_model) in enumerate(zip(disk_models[:-1], disk_models[1:])):
            for tg in distal_disk_model.tendon_models:
                self.socket.create_single_component_force(
                    self.name_gen.force_tension_between_tendon_guide_ends_name(
                        i, tg.orientation),
                    self.name_gen.tendon_guide_end_marker_name(
                        i+1, tg.orientation, is_top=False),
                    self.name_gen.tendon_guide_end_marker_name(
                        i, tg.orientation, is_top=True),

                    function=f"-({self.name_gen.var_name_tension_avg}+"
                    f"({self.name_gen.final_tension_meg_var_name(tg.orientation)}-{self.name_gen.var_name_tension_avg})*"
                    # f"STEP(TIME, {self.name_gen.var_name_base_duration}, 0, {self.name_gen.var_name_final_duration}, 1))"
                    f"MIN(MAX(((TIME-{self.name_gen.var_name_base_duration})/{self.name_gen.var_name_final_duration}),0), 1))"
                )

    def _enforce_base_disk_ground_constraint(self):
        self.socket.modify_part_rigid_body("ground")
        self.socket.create_marker("ground_center")
        self.socket.create_constraint_fixed("ground_constraint",
                                            i_marker_name=self.name_gen.disk_center_marker_name(0),
                                            j_marker_name="ground_center")

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
            if len(distal_disk_model.tendon_models) > 0:
                tendon_geometryMF = distal_disk_model.tendon_models[0]
                proximal_top_marker_name = self.name_gen.tendon_guide_end_marker_name(
                    i, tendon_geometryMF.orientation, True)
                distal_bottom_marker_name = self.name_gen.tendon_guide_end_marker_name(
                    i+1, tendon_geometryMF.orientation, False)
                self.socket.create_measure_function(
                    self.name_gen.measurement_joint_angle_name(i),
                    function=f"AX({distal_bottom_marker_name}, {proximal_top_marker_name})",
                    unit="angle",

                )

    def _generate_measurement_contact_force(self, disk_models: List[DiskModel]):
        for i in range(len(disk_models)-1):
            # Top surfaceof i-th disk
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
            # a*x**3+b*x**2+c*x+d = y
            # c = 0
            # d = avg
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

    def generate_model(self, initial_disk_overlap_length=DEFAULT_OVERLAPPING_LENGTH):
        # Pick c++ implementation solver
        self.socket.set_sim_general("cplusplus")

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
            print("Fail to import CAD")

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

        # self._create_debug_measurements()
        self._final_cleanup()

    def reset_model(self):
        self.socket.delete_model(self.model_name)
        
        
    def run_sim(self,
                input_tensions,
                total_iterations=10,
                step_size=1,
                duration=0.5,
                max_iterations_search_eqilibrium=30,
                initial_disk_overlap_length=DEFAULT_OVERLAPPING_LENGTH,):
        # Reset disks' positions to neutral state
        self._move_disks_to_pos(
            self._disks_init_location_orientation(initial_disk_overlap_length))

        # Increase static equilibrium iteration numbers
        self.socket.set_sim_equilibrium_param(
            self.model_name,
            max_iterations=max_iterations_search_eqilibrium)

        
        # Update variable
        self.socket.set_variable(
            self.name_gen.var_name_base_duration, 0
        )
        self.socket.set_variable(
            self.name_gen.var_name_final_duration, duration
        )
        self.socket.set_variable(
            self.name_gen.var_name_tension_avg, np.average(input_tensions)
        )

        for tg, val in zip(self.tendon_models, input_tensions):
            self.socket.set_variable(
                self.name_gen.final_tension_meg_var_name(
                    tg.orientation),
                val
            )

        # Run simulation
        self.socket.run_sim_reset(self.model_name)
        self.socket.run_sim_equilibrium(self.model_name)
        self.socket.run_sim_transient(self.model_name, 
                                      step_size=step_size,
                                        solver_type="STATIC", 
                                    duration=duration)

        # Make the simulation generate the records the disks' positions for next iteration
        self.socket.set_auto_plot_param("Last_run", True)

        return self.extract_state()
    
    def extract_state(self):
        disk_states = []
        for i, model in enumerate(self.disk_models[1:]):
            force_moment = []
            for contact_component in self.name_gen.measurement_all_contact_component_names(i+1, False):
                res = self.socket.extract_spread_sheet(contact_component)
                force_moment.append(res["Q"])

            res = self.socket.extract_spread_sheet(
                self.name_gen.measurement_joint_angle_name(i))
            bottom_joint_angle = res["Q"]
            disk_states.append(DiskState(
                model,
                bottom_contact_forceDF=force_moment[:3],
                bottom_contact_pure_momentDF=force_moment[3:],
                bottom_joint_angle=bottom_joint_angle,
            ))
        return disk_states

    # def run_sim(self,
    #             input_tensions,
    #             total_iterations=10,
    #             step_size=1,
    #             seconds_changing=0.5,
    #             seconds_until_end=50,
    #             max_iterations_search_eqilibrium=30,
    #             initial_disk_overlap_length=DEFAULT_OVERLAPPING_LENGTH,):
    #     # Reset disks' positions to neutral state
    #     self._move_disks_to_pos(
    #         self._disks_init_location_orientation(initial_disk_overlap_length))

    #     # Increase static equilibrium iteration numbers
    #     self.socket.set_sim_equilibrium_param(
    #         self.model_name,
    #         max_iterations=max_iterations_search_eqilibrium)

        
    #     # Update variable
    #     self.socket.set_variable(
    #         self.name_gen.var_name_base_duration, 0.0001
    #     )
    #     self.socket.set_variable(
    #         self.name_gen.var_name_final_duration, seconds_until_end
    #     )
    #     self.socket.set_variable(
    #         self.name_gen.var_name_tension_avg, np.average(input_tensions)
    #     )

    #     for tg, val in zip(self.tendon_models, input_tensions):
    #         self.socket.set_variable(
    #             self.name_gen.final_tension_meg_var_name(
    #                 tg.orientation),
    #             val
    #         )

    #     last_displacements = []
    #     for step in range(total_iterations):
    #         actual_seconds_per_iteration = seconds_changing + seconds_until_end
    #         actual_step_size = step_size
    #         print(f"Progress: {step+1}/{total_iterations}")

    #         while True:
    #             # Update tension magnitudes
    #             # for tg, prev_val, val in zip(self.tendon_models,
    #             #                              self._eval_tension_mags_at_step(
    #             #                                  input_tensions, step, total_iterations),
    #             #                              self._eval_tension_mags_at_step(
    #             #                                  input_tensions, step+1, total_iterations)):
    #             #     self.socket.set_variable(
    #             #         self.name_gen.final_tension_meg_var_name(
    #             #             tg.orientation),
    #             #         val
    #             #     )

    #             # Run simulation
    #             self.socket.run_sim_reset(self.model_name)
    #             self.socket.run_sim_equilibrium(self.model_name)
    #             self.socket.run_sim_transient(self.model_name, step_size=actual_step_size,
    #                                           solver_type="STATIC", duration=seconds_changing + seconds_until_end)

    #             # Make the simulation generate the records the disks' positions for next iteration
    #             self.socket.set_auto_plot_param("Last_run", True)

    #             # Do not update initial position at the last step
    #             if step == total_iterations-1:
    #                 break

    #             # Update initial position of the disks [2nd to last] from the auto-generated records
    #             displacements = []
    #             for i in range(1, len(self.disk_models)):
    #                 res = self.socket.extract_spread_sheet(f"disk{i}_XFORM")
    #                 displacements.append(
    #                     [res["X"], res["Y"], res["Z"], res["PSI"], res["THETA"], res["PHI"]])
    #             if displacements == last_displacements:
    #                 print(
    #                     "Final displacements are identical to that in initial setup, which means it fails to find the converged equilibrium solution")
    #                 print(
    #                     f"Increase the time step by 1.5 times from {actual_seconds_per_iteration} to {actual_seconds_per_iteration*1.5}")
    #                 actual_seconds_per_iteration *= 1.5
    #                 actual_step_size /= 1.5
    #                 continue

    #             last_displacements = displacements
    #             for i, disp in enumerate(displacements):
    #                 self.socket.modify_part_rigid_body(
    #                     part_name=self.name_gen.disk_part_name(i+1),
    #                     location=disp[:3],
    #                     orientation=disp[3:],
    #                 )
    #             break

    #     # Extract the final state from Adam
    #     disk_states = []
    #     for i, model in enumerate(self.disk_models[1:]):
    #         force_moment = []
    #         for contact_component in self._generate_measurement_all_contact_components(i+1, False):
    #             res = self.socket.extract_spread_sheet(contact_component)
    #             force_moment.append(res["Q"])

    #         res = self.socket.extract_spread_sheet(
    #             self.name_gen.measurement_joint_angle_name(i))
    #         bottom_joint_angle = res["Q"]
    #         disk_states.insert(0, DiskState(
    #             model,
    #             bottom_contact_forceDF=force_moment[:3],
    #             bottom_contact_pure_momentDF=force_moment[3:],
    #             bottom_joint_angle=bottom_joint_angle,
    #         ))
    #     return disk_states
