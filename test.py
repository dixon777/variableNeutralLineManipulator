from variable_neutral_line_manipulator.common.entities import *
from variable_neutral_line_manipulator.math_model.models import *
from variable_neutral_line_manipulator.simulation.adam_command import *
import time

def main_generate_CAD():
    m = ManipulatorGeometryModel([
        SegmentMathConfig(
            True, 2, 5, 0, 3, 2, 5
        )

    ],
        base_disk_length=5,
        outer_diameter=5,
        centre_hole_diameter=1,
        tendon_guide_diameter=0.5)
    m.generate_and_export_CAD(".temp_CAD",export_type="step",tolerance=0.0001)
    

def main2():
    s = AdamViewSocket(model_name="MANIPULATOR")
    i = 0
    while True:
        path = os.path.join(".temp_CAD", f"{i}.step")
        if not os.path.exists(path):
            break
        start = time.time()
        part_name = f"DISK_{i}"
        s.create_rigid_body_part(part_name)
        s.import_geometry(os.path.realpath(path), part_name=part_name, location=(0,0, i*5+2.5), type_of_geometry="stp")
        i += 1
        print(time.time() - start)
    # print(s.get_part_info(part_name="BASE_DISK"))


if __name__ == "__main__":
    main_generate_CAD()
    main2()
