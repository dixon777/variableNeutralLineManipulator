import time
import socket
import os
import re
import json
from math import degrees
from collections.abc import Iterable

from ..common.entities import *
from ..cad.cadquery_disk_generator import generate_disk_CAD, export_CAD
from ..common.calculation import normalise_angle
from ..math_model.calculation import eval_tendon_guide_top_end_disp, eval_tendon_guide_bottom_end_disp


def _to_Adam_val(v):
    if isinstance(v, str):
        return f"\"{v}\""
    elif isinstance(v, bool):
        return "yes" if v else "no"
    elif isinstance(v, (int, float)):
        return f"{v}"
    elif isinstance(v, Iterable):
        return ",".join(map(_to_Adam_val, v))

    raise ValueError(f"Unsupported type for conversion: {type(v)}")


def _construct_cmd(init, params, add_question_mark=False):
    s = f"cmd {init} "
    for p in params:
        if params[p] is None:
            continue
        s += f"{p} = {_to_Adam_val(params[p])} "
    if add_question_mark:
        s += "?"
    return s


class AdamDetails(BaseDataClass):
    def __init__(self, path):
        with open(path, "r") as f:
            self.content = f.read()

    @property
    def attr_keys(self):
        return super().attr_keys + ["content"]


class AdamViewSocket:
    def __init__(self, port=5002, return_text_path="./.return_result.txt"):
        self.port = port
        self.return_text_path = os.path.abspath(return_text_path)
        # self.create_model(model_name)

    def deal_with_command(self, cmd):
        print(f"Send: {str(cmd)}")
        # Must reconnect the server before every command is sent
        self.soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.soc.connect(("localhost", self.port))
        if isinstance(cmd, str):
            cmd = cmd.encode("ascii")
        self.soc.send(cmd)
        data = self.soc.recv(1024)
        success = data[-1] == ord('0')
        print(f"{ 'Success' if success else 'Failure'}\n")
        return success
    
    # Entity (for geometry)
    def rename_entity(self, name, new_name):
        return self.deal_with_command(_construct_cmd("entity modify", {
            "entity_name": name,
            "new_entity_name": new_name
        }))

    # Models
    def create_model(self, model_name):
        return self.deal_with_command(_construct_cmd("model create", {
            "model_name": model_name
        }))

    def delete_model(self, model_name):
        return self.deal_with_command(_construct_cmd("model delete", {
            "model_name": model_name
        }))

    # Parts
    def create_part_rigid_body(self, part_name, comments=None, is_ground_part=None):
        return self.deal_with_command(_construct_cmd("part create rigid_body name_and_position", {
            "part_name": part_name,
            "comments": comments,
            "ground_part": is_ground_part,
        }))

    def modify_part_rigid_body(self, part_name, location=None, orientation=None, relative_to=None, is_ground_part=None):
        return self.deal_with_command(_construct_cmd("part modify rigid_body name_and_position", {
            "part_name": part_name,
            "location": location,
            "orientation": orientation,
            "relative_to": relative_to,
            "ground_part": is_ground_part,
        }))

    def create_part_rigid_body_mass_properties(self, part_name, density):
        return self.deal_with_command(_construct_cmd("part create rigid_body mass_properties", {
            "part_name": part_name,
            "density": density,
        }))

    # Markers
    def create_marker(self, marker_name, location=None, orientation=None, relative_to=None, reference_marker_name=None):
        return self.deal_with_command(_construct_cmd("marker create", {
            "marker_name": marker_name,
            "location": location,
            "orientation": orientation,
            "relative_to": relative_to,
            "reference_marker_name": reference_marker_name
        }))

    def modify_marker(self, marker_name, new_marker_name=None, location=None, orientation=None, relative_to=None, reference_marker_name=None):
        return self.deal_with_command(_construct_cmd("marker modify", {
            "marker_name": marker_name,
            "new_marker_name": new_marker_name,
            "location": location,
            "orientation": orientation,
            "relative_to": relative_to,
            "reference_marker_name": reference_marker_name
        }))

    def create_floating_marker(self, marker_name):
        return self.deal_with_command(_construct_cmd("floating_marker create", {
            "marker_name": marker_name,
        }))

    # Forces
    # def create_force_vector(self, vector_name, i_marker_name, ):
    #     raise NotImplementedError()

    def create_single_component_force(self, force_name, i_marker_name, j_marker_name, function,):
        return self.deal_with_command(_construct_cmd("force create direct single_component_force", {
            "single_component_force_name": force_name,
            "i_marker_name": i_marker_name,
            "j_marker_name": j_marker_name,
            "action_only": "off",
            "function": function
        }))

    # Contact
    def create_contact(self, name, i_geometry_name, j_geometry_name, stiffness, damping, max_intrusion, exponent, mu_static, mu_dynamic, friction_transition_velocity, stiction_transition_velocity):
        return self.deal_with_command(_construct_cmd("contact create", {
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
        return self.deal_with_command(_construct_cmd("constraint create joint fixed", {
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
        return self.deal_with_command(_construct_cmd("constraint create joint planar", {
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
    def create_variable(self, variable_name, value, unit=None):
        return self.deal_with_command(_construct_cmd("variable create", {
            "variable_name": variable_name,
            "real_value": value,
            "unit": unit,
            "use_range": False,
            "use_allowed_values": False,
        }))

    # Imports
    def import_geometry(self, path, part_name=None, type_of_geometry="stp", location=None, orientation=None, relative_to=None):
        return self.deal_with_command(_construct_cmd("file geometry read", {
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
        return self.deal_with_command(_construct_cmd("file parasolid read", {
            "file_name": os.path.abspath(path),
            "type": encoding,
            "model_name": self.current_model_name if not part_name else None,
            "part_name": part_name,
            "location": location,
            "orientation": orientation,
            "relative_to": relative_to,
        }))

    # Exports
    def export_parasolid(self, path, part_name=None, encoding="ascii"):
        return self.deal_with_command(_construct_cmd("file parasolid write", {
            "file_name": os.path.abspath(path),
            "type": encoding,
            "model_name": self.current_model_name if not part_name else None,
            "part_name": part_name,
        }))

    # Info
    def get_info(self, item_name, dictionary):
        if os.path.exists(self.return_text_path):
            os.remove(self.return_text_path)
        return AdamDetails(self.return_text_path) if self.deal_with_command(_construct_cmd(f"list_info {item_name}", {
            "write_to_terminal": "off",
            "brief": "on",
            "file_name": os.path.abspath(self.return_text_path),
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


class ManipulatorCreator:
    cache_cad_details_basename = ".cad_details.json"
    marker_offset_from_curve = 0.05
    contact_max_intrusion = marker_offset_from_curve - 0.01

    def __init__(self,
                 port=5002,
                 model_name="MANIPULATOR",
                 temp_dir_path="./.temp_adam"):
        self.model_name = model_name
        self.temp_dir_path = os.path.abspath(temp_dir_path)
        os.makedirs(self.temp_dir_path, exist_ok=True)
        self.socket: AdamViewSocket = AdamViewSocket(
            port, os.path.join(self.temp_dir_path, ".return_result.txt"))

    def _generate_path(self, base_path):
        return os.path.join(self.temp_dir_path, base_path)

    # Name generation
    def _generate_part_name(self, index):
        return f"disk_{index}"

    def _generate_tension_magnitude_variable_name(self, orientationMF):
        return f"tendon_tension_magnitude__{int(degrees(normalise_angle(orientationMF)))}"

    def _generate_tendon_force_between_disk(self, proximal_disk_index, orientationMF):
        return f"tension_force__{proximal_disk_index}_{proximal_disk_index+1}_{int(degrees(normalise_angle(orientationMF)))}"

    def _generate_marker_disk_center_name(self, part_name):
        return f"{part_name}__center"

    def _generate_marker_for_planar_constraint_name(self, part_name, is_top):
        return f"{part_name}__planar_constraint_{'top' if is_top else 'bottom'}"

    def _generate_constraint_planar_name(self, index):
        return f"planar__{index}_{index+1}"

    def _generate_tendon_guide_end_marker_name(self, part_name, orientationMF, is_top):
        return f"{part_name}__{int(degrees(normalise_angle(orientationMF)))}_{'top' if is_top else 'bottom'}"

    def _generate_contact_name(self, index):
        return f"contact__{index}_{index+1}"

    def _generate_solid_geometry_name(self, index):
        return f"disk_solid__{index}"

    # def _get_marker_disk_cm_name(self, part_name):
    #     return f".{self.model_name}.{part_name}.cm"
    
    @ property
    def _cache_details_path(self):
        return self._generate_path(self.cache_cad_details_basename)

    def _cache_get_CAD_details(self):
        if not (os.path.exists(self._cache_details_path)):
            return []
        with open(self._cache_details_path, "r") as f:
            content = f.read()
            return json.loads(content, cls=BaseDataClass.Decoder) if content else []

    def _cache_store_CAD_details(self, details):
        with open(self._cache_details_path, "w") as f:
            return json.dump(details, f, cls=BaseDataClass.Encoder, indent=4)

    def _cache_find_CAD_path(self, geometry: DiskGeometry):
        cad_details = self._cache_get_CAD_details()
        for i, cad_item in enumerate(cad_details):
            if cad_item["geometry"] == geometry:
                path = self._generate_path(cad_item["path"])
                if os.path.exists(path):
                    return path

                # If the identical geometry is in the record but the path does not exist
                print(
                    f"Error: CAD is marked cached in the file [{self._cache_details_path}] but the path [{path}] does not exist")
                print("Entry is removed")
                del cad_details[i]
                self._cache_store_CAD_details(cad_details)
                break
        return None

    def _cache_generate_CAD_details_entry(self, geometry: DiskGeometry):
        cad_details = self._cache_get_CAD_details()
        largest_index = -1
        for cad_item in cad_details:
            largest_index = max(
                (largest_index, int(cad_item["path"].split(".")[0])))
            if cad_item["geometry"] == geometry:
                print("Has duplicate. Terminate caching new model")

        return {
            "path": f"{largest_index+1}.xmt_txt",
            "geometry": dict(geometry)
        }

    def _cache_store_CAD_details_entry(self, entry):
        cad_details = self._cache_get_CAD_details()
        cad_details.append(entry)
        self._cache_store_CAD_details(cad_details)

    def _import_and_cache_CAD(self, disk_geometry: DiskGeometry, part_name: str):
        # search from cache
        cad_path = self._cache_find_CAD_path(disk_geometry)

        success = False
        if cad_path is not None:
            success = self.socket.import_parasolid(
                path=cad_path, part_name=part_name)

        # Import and cache if it does not exist
        else:
            disk_cad_obj = generate_disk_CAD(disk_geometry, True)
            temp_step_file_path = self._generate_path(".temp.step")
            export_CAD(disk_cad_obj, path=temp_step_file_path)

            if(self.socket.create_part_rigid_body(part_name=part_name)):
                if(self.socket.import_geometry(temp_step_file_path, part_name=part_name)):
                    entry = self._cache_generate_CAD_details_entry(
                        disk_geometry)
                    if(self.socket.export_parasolid(self._generate_path(entry["path"]), part_name)):
                        self._cache_store_CAD_details_entry(entry)
                        success = True

                    os.remove(temp_step_file_path)

        return success
    
    def _modify_geometry_solid_name(self, index):
        new_name = self._generate_solid_geometry_name(index)
        for i in range(10):
            if self.socket.rename_entity(f"SOLID{i}", new_name):
                return True
            
        print("Error: Fail to modify solid name")
        return False
    

    def _create_markers_per_disk(self, model: DiskGeometryModel, part_name: str):
        disk_geometry = model.geometry

        # Create disk's center marker (may differ from center of mass marker(auto generated))
        self.socket.create_marker(
            self._generate_marker_disk_center_name(
                part_name),
        )

        for tendon_geometryDF in disk_geometry.tendon_guide_geometriesDF:
            # Create marker at tendon guide bottom end
            if disk_geometry.bottom_curve_radius:
                disp = eval_tendon_guide_bottom_end_disp(
                    disk_geometry.length, disk_geometry.bottom_curve_radius, tendon_geometryDF.dist_from_axis, tendon_geometryDF.orientationDF)
                # Offset the marker away from the curve surface
                disp[2] += self.marker_offset_from_curve
                self.socket.create_marker(
                    self._generate_tendon_guide_end_marker_name(
                        part_name, model.bottom_orientationMF + tendon_geometryDF.orientationDF, is_top=False),
                    location=disp
                )

            # Create marker at tendon guide top end
            if disk_geometry.top_curve_radius:
                disp = eval_tendon_guide_top_end_disp(
                    disk_geometry.length, disk_geometry.top_curve_radius, tendon_geometryDF.dist_from_axis, tendon_geometryDF.orientationDF, disk_geometry.top_orientationDF)
                # Offset the marker away from the curve surface
                disp[2] -= self.marker_offset_from_curve
                self.socket.create_marker(
                    self._generate_tendon_guide_end_marker_name(
                        part_name, model.bottom_orientationMF + tendon_geometryDF.orientationDF, is_top=True),
                    location=disp
                )

    def _define_mass_properties_per_disk(self, model: DiskGeometryModel, part_name: str):
        self.socket.create_part_rigid_body_mass_properties(part_name, 1)
        # Shouldn't rename auto-generated center of mass marker

    def _generate_parametric_variables(self, tendon_guide_geometriesMF: List[TendonGuideGeometryMF]):
        for tg in tendon_guide_geometriesMF:
            self.socket.create_variable(
                self._generate_tension_magnitude_variable_name(tg.orientationMF), 1.0)

    def _connect_forces_between_tendon_guide_ends(self, disk_models: List[DiskGeometryModel]):
        for i, (proximal_disk_model, distal_disk_model) in enumerate(zip(disk_models[:-1], disk_models[1:])):
            for tg in distal_disk_model.tendon_guide_geometriesMF:
                self.socket.create_single_component_force(
                    self._generate_tendon_force_between_disk(
                        i, tg.orientationMF),
                    self._generate_tendon_guide_end_marker_name(
                        self._generate_part_name(i+1), tg.orientationMF, is_top=False),
                    self._generate_tendon_guide_end_marker_name(
                        self._generate_part_name(i), tg.orientationMF, is_top=True),
                    function=f"-{self._generate_tension_magnitude_variable_name(tg.orientationMF)}"
                )

    def _enforce_base_disk_ground_constraint(self):
        self.socket.modify_part_rigid_body("ground")
        self.socket.create_marker("ground_center")
        self.socket.create_constraint_fixed("ground_constraint", i_marker_name=self._generate_marker_disk_center_name(
            self._generate_part_name(0)), j_marker_name="ground_center")

    def _enforce_planar_constraints(self, disk_models: List[DiskGeometryModel]):
        for i, (proximal_disk_model, distal_disk_model) in enumerate(zip(disk_models[:-1], disk_models[1:])):
            # Create new marker at proximal disk's top
            self.socket.modify_part_rigid_body(self._generate_part_name(i))
            self.socket.create_marker(self._generate_marker_for_planar_constraint_name(self._generate_part_name(i), is_top=True),
                                      location=(
                                          0, 0, proximal_disk_model.geometry.length/2),
                                      orientation=(
                                          degrees(proximal_disk_model.geometry.top_orientationDF), 90, 0),
                                      relative_to=self._generate_marker_disk_center_name(self._generate_part_name(i)))

            # Create new marker at distal disk's bottom
            self.socket.modify_part_rigid_body(self._generate_part_name(i+1))
            self.socket.create_marker(self._generate_marker_for_planar_constraint_name(self._generate_part_name(i+1), is_top=False),
                                      location=(
                                          0, 0, -distal_disk_model.geometry.length/2),
                                      orientation=(0, 90, 0),
                                      relative_to=self._generate_marker_disk_center_name(self._generate_part_name(i+1)))

            # Add constraint
            self.socket.create_constraint_planar_joint(self._generate_constraint_planar_name(i),
                                                       i_marker_name=self._generate_marker_for_planar_constraint_name(self._generate_part_name(i+1),
                                                                                                                      is_top=False),
                                                       j_marker_name=self._generate_marker_for_planar_constraint_name(self._generate_part_name(i),
                                                                                                                      is_top=True),)

    def _generate_contacts(self, disk_models: List[DiskGeometryModel]):
        for i in range(len(disk_models)-1):
            self.socket.create_contact(
                self._generate_contact_name(i),
                self._generate_solid_geometry_name(i),
                self._generate_solid_geometry_name(i+1),
                10**6,
                10**4,
                self.contact_max_intrusion,
                5,
                10**6,
                10**6,
                10**4,
                10**4,
            )

    def _final_cleanup(self):
        self.socket.modify_part_rigid_body("ground")

    def generate(self, manipulatorModel: ManipulatorGeometryModel):
        indices_disk_model_pairs = manipulatorModel.generate_indices_disk_model_pairs()
        disk_models = indices_entity_pairs_to_ordered_list(
            indices_disk_model_pairs)
        disk_length_accumulate = 0
        for i, model in enumerate(disk_models):
            part_name = self._generate_part_name(i)
            if(self._import_and_cache_CAD(model.geometry, part_name)):
                self._modify_geometry_solid_name(i)
                self._define_mass_properties_per_disk(model, part_name)
                self._create_markers_per_disk(model, part_name)
                # Move the disk to designated location
                self.socket.modify_part_rigid_body(
                    part_name, orientation=(0, 0, degrees(model.bottom_orientationMF)))
                self.socket.modify_part_rigid_body(
                    part_name, location=(0, 0, disk_length_accumulate))
                disk_length_accumulate += model.geometry.length

        self._generate_parametric_variables(
            manipulatorModel.tendon_guide_geometriesMF)
        self._connect_forces_between_tendon_guide_ends(disk_models)
        self._enforce_base_disk_ground_constraint()
        self._enforce_planar_constraints(disk_models)
        self._generate_contacts(disk_models)
        self._final_cleanup()

    def reset_model(self):
        self.socket.delete_model(self.model_name)
        self.socket.create_model(self.model_name)


# if __name__ == "__main__":
#     s = AdamViewSocket(model_name="MODEL_12")
#     # s.export_parasolid("./bbcb", part_name="PART_2")

#     print(s.get_model_info())
#     # s.create_rigid_body_part("BASE_DISK")
#     # s.import_geometry(os.path.realpath("./base.x_t"), part_name="BASE_DISK", location=(3,3,3))
#     # print(s.get_part_info(part_name="PART_2"))
