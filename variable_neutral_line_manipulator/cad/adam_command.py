import time
import socket
import os
import yaml
import re


class AdamPartDetails():
    def __init__(self, path):
        with open(path, "r") as f:
            content = f.read()
        self.id = re.search("(?<=Adams ID\s{8}:\s{2})\d", content).group()
        
    def __str__(self):
        return f"ID: {self.id}"


def to_Adam_val(v):
    if isinstance(v, str):
        return f"\"{v}\""
    elif isinstance(v, bool):
        return "yes" if v else "no"

    raise ValueError("Unsupported type for conversion")


def construct_cmd(init, params):
    s = f"cmd {init} "
    for p in params:
        if params[p] is None:
            continue
        s += f"{p} = {to_Adam_val(params[p])} "
    return s


class AdamViewSocket:
    def __init__(self, port=5002, model_name="MODEL_1"):
        super().__init__()
        self.port = port
        self.model_name = None
        self.create_model(model_name)

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

    def create_model(self, name):
        if self.model_name is not None:
            self.delete_model()
        self.deal_with_command(construct_cmd("model create", {
            "model_name": name
        }))
        self.model_name = name

    def delete_model(self):
        if self.model_name is None:
            return False
        self.deal_with_command(construct_cmd("model delete", {
            "model_name": self.model_name
        }))
        self.model_name = None

    def create_rigid_body_part(self, name, comment=None):
        if self.model_name is None:
            return False
        self.deal_with_command(construct_cmd("part create rigid_body name_and_position", {
            "part_name": name,
            "comment": comment,
        }))

    # import parasolid file
    def import_parasolid(self, path, part_name=None, encoding=None):
        self.deal_with_command(construct_cmd("file parasolid read", {
            "file": path,
            "type": encoding,
            # mutually exclusive with part_name
            "model_name": self.model_name if not part_name else None,
            "part_name": part_name,
        }))

    def get_part_info(self, part_name, res_path="res.txt"):
        if os.path.exists(res_path):
            os.remove(res_path)
        return AdamPartDetails(res_path) if self.deal_with_command(construct_cmd("list_info part", {
            "part_name": part_name,
            "write_to_terminal": "off",
            "brief": "on",
            "file_name": os.path.realpath(res_path),
        })) else None
        
        
class ManipulatorCreator:
    def __init__(self, socket):
        super().__init__()
        self.soc = socket
    
    def addBasei(self):
        return


if __name__ == "__main__":
    s = AdamViewSocket(model_name="MANIPULATOR")
    s.create_rigid_body_part("BASE_DISK")
    s.import_parasolid(os.path.realpath("./base.x_t"), part_name="BASE_DISK")
    print(s.get_part_info(part_name="BASE_DISK"))
