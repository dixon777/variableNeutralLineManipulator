import time
import socket
import os
import re
import json
from collections.abc import Iterable  

from ..common.entities import *
from ..cad.cadquery_disk_generator import generate_disk_CAD, export_CAD



def _to_Adam_val(v):
    if isinstance(v, str):
        return f"\"{v}\""
    elif isinstance(v, (int,float)):
        return f"{v}"
    elif isinstance(v, bool):
        return "yes" if v else "no"
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
    def __init__(self, port=5002, model_name=None, return_text_path="./.return_result.txt"):
        self.port = port
        self._model_name = model_name
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

    # Models
    def create_model(self, model_name):
        self._model_name = model_name
        return self.deal_with_command(_construct_cmd("model create", {
            "model_name": model_name
        }))


    # Parts
    def create_rigid_body_part(self, part_name, comments=None):
        return self.deal_with_command(_construct_cmd("part create rigid_body name_and_position", {
            "part_name": part_name,
            "comments": comments,
        }))
        
    def modify_rigid_body_position(self, part_name, location=None, orientation=None, relative_to=None):
        return self.deal_with_command(_construct_cmd("part modify rigid_body name_and_position", {
            "part_name": part_name,
            "location": location,
            "orientation": orientation,
            "relative_to": relative_to,
        }))

    # Imports
    def import_geometry(self, path, part_name=None, type_of_geometry="stp", location=None, orientation=None, relative_to=None):
        return self.deal_with_command(_construct_cmd("file geometry read", {
            "type_of_geometry": type_of_geometry,
            "file": os.path.abspath(path),
            # mutually exclusive with part_name
            "model_name": self.model_name if not part_name else None,
            "part_name": part_name,
            "location": location,
            "orientation": orientation,
            "relative_to": relative_to,
        }))
        
    def import_parasolid(self, path, part_name=None, encoding="ascii", location=None, orientation=None, relative_to=None):
        return self.deal_with_command(_construct_cmd("file parasolid read", {
            "file_name": os.path.abspath(path),
            "type": encoding,
            "model_name": self.model_name if not part_name else None,
            "part_name": part_name,
            "location": location,
            "orientation": orientation,
            "relative_to": relative_to,
        }))
        
    def export_parasolid(self, path, part_name=None, encoding="ascii"):
        return self.deal_with_command(_construct_cmd("file parasolid write", {
            "file_name": os.path.abspath(path),
            "type": encoding,
            "model_name": self.model_name if not part_name else None,
            "part_name": part_name,
        }))

    # Exports
    def get_model_info(self, model_name=None):
        if os.path.exists(self.return_text_path):
            os.remove(self.return_text_path)
        return AdamDetails(self.return_text_path) if self.deal_with_command(_construct_cmd("list_info part", {
            "model_name": f"{model_name}",
            "write_to_terminal": "off",
            "brief": "on",
            "file_name": os.path.abspath(self.return_text_path),
        })) else None
        
    def get_part_info(self, part_name=None):
        if os.path.exists(self.return_text_path):
            os.remove(self.return_text_path)
        return AdamDetails(self.return_text_path) if self.deal_with_command(_construct_cmd("list_info part", {
            "part_name": f"{part_name}" if part_name else None,
            "write_to_terminal": "off",
            "brief": "on",
            "file_name": os.path.abspath(self.return_text_path),
        }, add_question_mark=part_name is None)) else None
        
    def get_database_names(self, entity_name=None):
        if os.path.exists(self.return_text_path):
            os.remove(self.return_text_path)
        return AdamDetails(self.return_text_path) if self.deal_with_command(_construct_cmd("list_info names", {
            "entity_name": entity_name if entity_name else "?",
            "write_to_terminal": "off",
            "brief": "off",
            "file_name": os.path.abspath(self.return_text_path),
        })) else None
        
    def get_default_settings(self):
        if os.path.exists(self.return_text_path):
            os.remove(self.return_text_path)
        return AdamDetails(self.return_text_path) if self.deal_with_command(_construct_cmd("list_info defaults", {
            "write_to_terminal": "off",
            "brief": "on",
            "file_name": os.path.abspath(self.return_text_path),
        })) else None
    
    @property
    def model_name(self):
        if not self._model_name:
            res = self.get_default_settings()
            self._model_name = re.search(r"(?<=,\sobject\sname:\s).+(?=\n)", res.content).group() if res else None
        return self._model_name
        
        
        
class ManipulatorCreator:
    cache_cad_details_basename = ".cad_details.json"
    
    def __init__(self, 
                 port=5002, 
                 model_name=None, 
                 temp_dir_path="./.temp_adam"):
        self.temp_dir_path = os.path.abspath(temp_dir_path)
        os.makedirs(self.temp_dir_path, exist_ok=True)
        self.socket:AdamViewSocket = AdamViewSocket(port, model_name, os.path.join(self.temp_dir_path, ".return_result.txt") )
    
    def _generate_path(self, base_path):
        return os.path.join(self.temp_dir_path, base_path)
    
    @property
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
        
    
    def _cache_find_CAD_path(self, geometry:DiskGeometry):
        cad_details = self._cache_get_CAD_details()
        for i, cad_item in enumerate(cad_details):
            if cad_item["geometry"] == geometry:
                path = self._generate_path(cad_item["path"])
                if os.path.exists(path):
                    return path
                
                # If the identical geometry is in the record but the path does not exist
                print(f"Error: CAD is marked cached in the file [{self._cache_details_path}] but the path [{path}] does not exist")
                print("Entry is removed")
                del cad_details[i]
                self._cache_store_CAD_details(cad_details)
                break
        return None
            
    def _cache_generate_CAD_details_entry(self, geometry:DiskGeometry):
        cad_details = self._cache_get_CAD_details()
        largest_index = -1
        for cad_item in cad_details:
            largest_index = max((largest_index, int(cad_item["path"].split(".")[0])))
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
                
        
        
        
    def _import_and_cache_CAD(self, disk_geometry:DiskGeometry, part_name: str):
        # search from cache
        cad_path = self._cache_find_CAD_path(disk_geometry)
        
        success = False
        if cad_path is not None:
            success = self.socket.import_parasolid(path=cad_path, part_name=part_name)
        
        # Import and cache if it does not exist
        else:
            disk_cad_obj = generate_disk_CAD(disk_geometry, True)
            temp_step_file_path = self._generate_path(".temp.step")
            export_CAD(disk_cad_obj, path=temp_step_file_path)
            
            if(self.socket.create_rigid_body_part(part_name=part_name)):
                if(self.socket.import_geometry(temp_step_file_path, part_name=part_name)):
                    entry = self._cache_generate_CAD_details_entry(disk_geometry)                
                    if(self.socket.export_parasolid(self._generate_path(entry["path"]), part_name)):
                        self._cache_store_CAD_details_entry(entry) 
                        success = True
            
                    os.remove(temp_step_file_path)
                    
        return success
        
    def create_markers(self, model:DiskGeometryModel, part_name:str):
        pass  
    
    def generate_disks(self, indices_disk_model_pairs:Iterable[Tuple[List[int], DiskGeometryModel]]):
        disk_length_accumulate = 0
        for indices, model in indices_disk_model_pairs:
            part_name = f"disk_{'_'.join(str(i) for i in indices)}"
            if(self._import_and_cache_CAD(model.geometry, part_name)):
                self.socket.modify_rigid_body_position(part_name, location=(0,0,disk_length_accumulate))
                disk_length_accumulate += model.geometry.length
                
    
    def generate_parametric_variables(self):
        pass
    
    def generate_constraints(self):
        pass
    
    def generate_forces(self):
        pass
    
    def generate_measures(self):
        pass
            
    
            


# if __name__ == "__main__":
#     s = AdamViewSocket(model_name="MODEL_12")
#     # s.export_parasolid("./bbcb", part_name="PART_2")
    
#     print(s.get_model_info())
#     # s.create_rigid_body_part("BASE_DISK")
#     # s.import_geometry(os.path.realpath("./base.x_t"), part_name="BASE_DISK", location=(3,3,3))
#     # print(s.get_part_info(part_name="PART_2"))
