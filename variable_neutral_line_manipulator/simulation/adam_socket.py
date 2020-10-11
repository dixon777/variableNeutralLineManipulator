import time
import socket
import os
import re
import json
from math import degrees, radians
from collections.abc import Iterable
from typing import List,  Union, Dict

from ..common.entities import *
from ..util import Logger


def _adams_convert_value(v: Union[str, bool, int, float, Iterable]) -> str:
    '''Convert Python value to Adam View's format'''
    if isinstance(v, str):
        new_v = v.replace('"', '\'')
        return f"\"{new_v}\""
    elif isinstance(v, bool):
        return "yes" if v else "no"
    elif isinstance(v, (int, float)):
        return f"{v}"
    elif isinstance(v, Iterable):
        return ",".join(map(_adams_convert_value, v))

    raise ValueError(f"Unsupported type for conversion: {type(v)}")


def _adams_construct_cmd(init, params={}, without_prefix=False):
    '''Construct Adam command'''
    s = f"{'cmd ' if not without_prefix else ''}{init} "
    for p in params:
        if params[p] is None:
            continue
        s += f"{p} = {_adams_convert_value(params[p])} "
    return s


def adams_read_spreadsheet_steady_state_value(path):
    '''Extract steady state value from the spreadsheet exported by Adams View'''
    res = {}
    with open(path, "r") as f:
        for i in range(6):
            next(f)

        # read attribute
        attr_line = f.readline()

        lines_list = f.read().splitlines()
        if len(lines_list) < 3:
            return {}

        # read values at equilibrium
        equilibrium_val_line = lines_list[-3]

    attrs = [re.search("(?<=.)\w+(?=\")", s).group(0)
             for s in attr_line.split('\t')]
    vals = [float(v) for v in equilibrium_val_line.split('\t')]
    return {attr: val for attr, val in zip(attrs, vals)}


class AdamViewSocket:
    """
    Python interface for communication with Adams View command server via localhost network

    Params:
     - port: Port listened by Adams View command server
     - return_text_prefix: The basename prefix (basename without extension) of the files exported by Adams View
     - reset_script_name: Name of script for resetting simulation to initial config.
        Since such command cannot be properly executed by passing it directly to Adams View command server, thus it is worked around by executing a predefined script.
    """

    def __init__(self,
                 port=5002,  # Should be always 5002
                 return_text_prefix="./.return_result",
                 reset_script_name="AUTO_GENERATED_sim_reset_script"):
        self.port = port
        self.return_text_prefix = os.path.abspath(return_text_prefix)
        self.reset_script_name = reset_script_name
        
        
    def deal_with_cmd(self,
                      cmd: str,
                      params: Union[None,
                                    Dict[str, Union[int,
                                                    float, str, bool, Iterable]]] = None,
                      timeout: float = None,):
        if params:
            cmd = _adams_construct_cmd(cmd, params)
        if isinstance(cmd, str):
            cmd = cmd.encode("ascii")
            
        Logger.D(f"Send cmd: {str(cmd)}")
        # Must reconnect the server before every command is sent
        soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        soc.settimeout(timeout)
        try:
            soc.connect(("localhost", self.port))
        except ConnectionRefusedError as e:
            print(
                f"Connection is refused. Please make sure Adam Views command server is listening to port {self.port}")
            raise e

        soc.send(cmd)
        res = False
        try:
            res = soc.recv(1024)[-1] == ord('0')
        except Exception as e:
            print(e)
        finally:
            soc.close()
        return res
    
    # Default
    def set_default_units(self, 
                          force_unit=None, 
                          mass_unit=None, 
                          length_unit=None, 
                          time_unit=None, 
                          angle_unit=None):
        return self.deal_with_cmd("default units", {
            "force": force_unit,
            "mass": mass_unit,
            "length": length_unit,
            "time": time_unit,
            "angle": angle_unit,
        })

    # Entity (for geometry)
    def rename_entity(self, name, new_name):
        return self.deal_with_cmd("entity modify", {
            "entity_name": name,
            "new_entity_name": new_name
        })

    # Models
    def create_model(self, model_name):
        return self.deal_with_cmd("model create", {
            "model_name": model_name
        })

    def delete_model(self, model_name):
        return self.deal_with_cmd("model delete", {
            "model_name": model_name
        })

    # Parts
    def create_part_rigid_body(self, part_name, comments=None, is_ground_part=None):
        return self.deal_with_cmd("part create rigid_body name_and_position", {
            "part_name": part_name,
            "comments": comments,
            "ground_part": is_ground_part,
        })

    def modify_part_rigid_body(self, part_name, location=None, orientation=None, relative_to=None, is_ground_part=None):
        return self.deal_with_cmd("part modify rigid_body name_and_position", {
            "part_name": part_name,
            "location": location,
            "orientation": orientation,
            "relative_to": relative_to,
            "ground_part": is_ground_part,
        })

    def create_part_rigid_body_mass_properties(self, part_name, density):
        return self.deal_with_cmd("part create rigid_body mass_properties", {
            "part_name": part_name,
            "density": density,
        })

    # Markers
    def create_marker(self, marker_name, location=None, orientation=None, relative_to=None):
        """
            Create a marker named "marker_name", at "location" and it has orientation "orientation" 
            @params
                marker_name: Marker name [str] (Required, Must not exist) 
                location: Position at which the marker is created, offset from "relative_to" marker.
                            [List[3 float]] (Optional:Default [0,0,0])
                orientation: Orientation of the reference frame of the marker, with respect to "relative_to" marker's frame .
                            It is in Z-X-Z Euler angle rotation. [List[3 float]] (Optional:Default [0,0,0]) 
                relative_to: Reference marker [str] (Optional:Default Global coordinate system)
        """
        return self.deal_with_cmd("marker create", {
            "marker_name": marker_name,
            "location": location,
            "orientation": orientation,
            "relative_to": relative_to,
        })

    def modify_marker(self, marker_name, new_marker_name=None, location=None, orientation=None, relative_to=None, reference_marker_name=None):
        return self.deal_with_cmd("marker modify", {
            "marker_name": marker_name,
            "new_marker_name": new_marker_name,
            "location": location,
            "orientation": orientation,
            "relative_to": relative_to,
            "reference_marker_name": reference_marker_name
        })

    def create_floating_marker(self, marker_name):
        return self.deal_with_cmd("floating_marker create", {
            "floating_marker_name": marker_name,
        })

    # Forces
    def create_single_component_force(self, force_name, i_marker_name, j_marker_name, function,):
        return self.deal_with_cmd("force create direct single_component_force", {
            "single_component_force_name": force_name,
            "i_marker_name": i_marker_name,
            "j_marker_name": j_marker_name,
            "action_only": "off",
            "function": function
        })

    def create_vector_force(self, 
                            force_name, 
                            i_marker_name, 
                            j_part_name,
                            ref_marker_name,
                            x_force_function="0", y_force_function="0", z_force_function="0"):
        """
            Create a force acted on the part, to which "i_marker_name" belongs, by "j_part_name" at position "i_marker_name". 
            @params
                force_name: Force name (Required, Must not exist)
                i_marker_name: Position at which the force acts on both parts  (Required, Must exist)
                    The part to which it belongs is the action body
                j_part_name: Reaction body (Required, Must exist)
                ref_marker_name: Reference frame for force vector definition (Required, Must exist)
                x_force_function: Runtime function for x component of force vector (Optional:Default 0)
                y_force_function: Runtime function for y component of force vector (Optional:Default 0)
                z_force_function: Runtime function for z component of force vector (Optional:Default 0)
        """
        return self.deal_with_cmd("force create direct force_vector", {
            "force_vector_name": force_name,
            "i_marker_name": i_marker_name,
            "j_part_name": j_part_name,
            "ref_marker_name": ref_marker_name,
            "x_force_function": x_force_function,
            "y_force_function": y_force_function,
            "z_force_function": z_force_function,
        })
        
    def create_gravity(self, gravity_field_name, x_component_gravity=0.0, y_component_gravity=0.0, z_component_gravity=0.0):
        return self.deal_with_cmd("force create body gravitational", {
            "gravity_field_name": gravity_field_name,
            "x_component_gravity": x_component_gravity,
            "y_component_gravity": y_component_gravity,
            "z_component_gravity": z_component_gravity,
        })

    # Contact
    def create_contact(self, name, i_geometry_name, j_geometry_name, stiffness, damping, max_intrusion, exponent, mu_static, mu_dynamic, friction_transition_velocity, stiction_transition_velocity):
        return self.deal_with_cmd("contact create", {
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
        })

    # Constraints
    def create_constraint_fixed(self, name, i_part_name=None, j_part_name=None, i_marker_name=None, j_marker_name=None, location=None, orientation=None, relative_to=None):
        return self.deal_with_cmd("constraint create joint fixed", {
            "joint_name": name,
            "i_part_name": i_part_name,
            "j_part_name": j_part_name,
            "i_marker_name": i_marker_name,
            "j_marker_name": j_marker_name,
            "location": location,
            "orientation": orientation,
            "relative_to": relative_to
        })

    def create_constraint_planar_joint(self, name, i_part_name=None, j_part_name=None, i_marker_name=None, j_marker_name=None, location=None, orientation=None, relative_to=None):
        return self.deal_with_cmd("constraint create joint planar", {
            "joint_name": name,
            "i_part_name": i_part_name,
            "j_part_name": j_part_name,
            "i_marker_name": i_marker_name,
            "j_marker_name": j_marker_name,
            "location": location,
            "orientation": orientation,
            "relative_to": relative_to
        })

    # Variables
    def create_variable(self, name, value, unit=None):
        return self.deal_with_cmd("variable create", {
            "variable_name": name,
            "real_value": value,
            "unit": unit,
            "use_range": False,
            "use_allowed_values": False,
        })

    def set_variable(self, name, value, unit=None):
        return self.deal_with_cmd("variable modify", {
            "variable_name": name,
            "real_value": value,
            "unit": unit,
            "use_range": False,
            "use_allowed_values": False,
        })

    # Measures
    def create_measure_function(self, name, function, unit=None, should_display=False):
        return self.deal_with_cmd("measure create function", {
            "measure_name": name,
            "function": function,
            "units": unit,
            "create_measure_display": should_display,
        })

    # SImulation
    def create_sim_script(self, name, commands):
        return self.deal_with_cmd("simulation script create", {
            "sim_script_name": name,
            "commands": commands
        })

    def modify_sim_script(self, name, commands):
        return self.deal_with_cmd("simulation script modify", {
            "sim_script_name": name,
            "commands": commands
        })

    def delete_sim_script(self, name):
        return self.deal_with_cmd("simulation script delete", {
            "sim_script_name": name,
        })

    def run_sim_script(self, model_name, script_name):
        return self.deal_with_cmd("simulation single_run scripted", {
            "model_name": model_name,
            "sim_script_name": script_name,
            "reset_before_and_after": False,
        })

    def config_sim_general(self, server_choice=None):
        return self.deal_with_cmd("simulation set", {
            "choice_for_solver": server_choice
        })

    def set_sim_equilibrium_param(self, model_name, max_iterations=None, stability=None, tlimit=None, alimit=None, static_method=None):
        return self.deal_with_cmd("executive_control set equilibrium_parameters", {
            "model_name": model_name,
            "maxit": max_iterations,
            "tlimit": tlimit,
            "alimit": alimit,
            "stability": stability,
            "static_method": static_method,
        })

    def run_sim_equilibrium(self, model_name):
        return self.deal_with_cmd("simulation single_run equilibrium", {
            "model_name": model_name,
        })

    def run_sim_transient(self, model_name, number_of_steps=None, step_size=None, solver_type="DYNAMIC", end_time=None, duration=None, initial_static=None, timeout=None):
        return self.deal_with_cmd("simulation single_run transient", {
            "model_name": model_name,
            "initial_static": initial_static,
            "type": solver_type,
            "number_of_steps": number_of_steps,
            "step_size": step_size,
            "end_time": end_time,
            "duration": duration,
        },
            timeout=timeout)

    def run_sim_reset(self, model_name):
        # Adam View does not respond at all if "simulation single_run reset" command is transferred into it via command server,
        # but the command works when it is executed in the sim script
        # Therefore, the following strategy involves writing a command script to the model
        # and activate it whenever this function gets called
        i = 0
        script_name = self.reset_script_name
        while not self.create_sim_script(script_name, "simulation single_run reset"):
            i += 1
            script_name += f"{i}"

        res = self.run_sim_script(model_name, self.reset_script_name)
        self.delete_sim_script(script_name)
        return res

    # Imports
    def import_geometry(self, path, part_name=None, type_of_geometry="stp", location=None, orientation=None, relative_to=None):
        return self.deal_with_cmd("file geometry read", {
            "type_of_geometry": type_of_geometry,
            "file": os.path.abspath(path),
            # mutually exclusive with part_name
            "model_name": self.current_model_name if not part_name else None,
            "part_name": part_name,
            "location": location,
            "orientation": orientation,
            "relative_to": relative_to,
        })

    def import_parasolid(self, path, part_name=None, encoding="ascii", location=None, orientation=None, relative_to=None):
        return self.deal_with_cmd("file parasolid read", {
            "file_name": os.path.abspath(path),
            "type": encoding,
            "model_name": self.current_model_name if not part_name else None,
            "part_name": part_name,
            "location": location,
            "orientation": orientation,
            "relative_to": relative_to,
        })

    def import_cmd(self, path):
        return self.deal_with_cmd("file command read", {
            "file_name": os.path.abspath(path),
        })

    def import_sim_script(self, script_name, path):
        return self.deal_with_cmd("simulation script read_acf", {
            "sim_script_name": script_name,
            "file_name": os.path.abspath(path)
        })

    # Exports
    def export_parasolid(self, path, part_name=None, encoding="ascii"):
        return self.deal_with_cmd("file parasolid write", {
            "file_name": os.path.abspath(path),
            "type": encoding,
            "model_name": self.current_model_name if not part_name else None,
            "part_name": part_name,
        })

    def export_spread_sheet(self, path, result_names):
        return self.deal_with_cmd("file spread_sheet write", {
            "file_name": os.path.abspath(path),
            "result_set_name": result_names,
        })

    def extract_steady_state_from_spread_sheet(self, result_names):
        path = f"{self.return_text_prefix}.tab"
        if os.path.exists(path):
            os.remove(path)
        return adams_read_spreadsheet_steady_state_value(path) if self.export_spread_sheet(path, result_names) else {}

    def export_sim_script(self, script_name, path):
        return self.deal_with_cmd("simulation script write_acf", {
            "sim_script_name": script_name,
            "file_name": os.path.abspath(path)
        })

    # Plots
    def set_auto_plot_param(self, analysis_name,
                            include_displacements=False,
                            include_velocity=False):
        return self.deal_with_cmd("xy_plots auto_plot output_requests", {
            "analysis_name": analysis_name,
            "displacements": "on" if include_displacements else "off",
            "velocities": "on" if include_velocity else "off",
        })

    def create_plot(self, plot_name,  data_name=None, measurement_name=None):
        return self.deal_with_cmd("xy_plots curve create", {
            "plot_name": plot_name,
            "vaxis_data": data_name,
            "vmeasure": measurement_name,
        })

    # Info
    def get_info(self, item_name, dictionary):
        path = f"{self.return_text_prefix}.txt"
        if os.path.exists(path):
            os.remove(path)
        if self.deal_with_cmd(f"list_info {item_name}", {
            "write_to_terminal": "off",
            "brief": "on",
            "file_name": os.path.abspath(path),
            **dictionary
        }):
            with open(path, "r") as f:
                return f.read()
        return None

    def get_variable_info(self, variable_name):
        return self.get_info("variable", {
            "variable_name": variable_name,
        })

    def get_variable_real_value(self, variable_name):
        res = self.get_variable_info(variable_name)
        if res:
            res = float(
                re.search(r"(?<=Real Value\(s\)\W{5})\d.\d", res).group(0))
        return res

    # def get_model_info(self, model_name=None):
    #     return self.get_info("model", {
    #         "model_name": model_name,
    #     })

    # def get_part_info(self, part_name=None):
    #     return self.get_info("part", {
    #         "model_name": part_name,
    #     })

    # def get_entity_info(self, entity_name=None):
    #     return self.get_info("entity", {
    #         "entity_name": entity_name,
    #     })

    # def get_geometry_info(self, geometry_name=None):
    #     return self.get_info("geometry", {
    #         "geometry_name": geometry_name,
    #     })

    # def get_database_names(self, entity_name=None):
    #     return self.get_info("names", {
    #         "entity_name": entity_name,
    #     })

    # def get_default_settings(self):
    #     return self.get_info("defaults", {})
