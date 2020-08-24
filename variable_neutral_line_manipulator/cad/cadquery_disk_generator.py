from math import sqrt, cos, sin
import cadquery as cq
from ..common.entities import *
    

def generate_disk_CAD(disk_geometry: DiskGeometry, curve_only=False):
    outer_diameter = disk_geometry.outer_diameter
    length = disk_geometry.length
    bottom_curve_radius = disk_geometry.bottom_curve_radius
    top_curve_radius = disk_geometry.top_curve_radius
    top_orientationDF = disk_geometry.top_orientationDF
    centre_hole_diameter = disk_geometry.centre_hole_diameter
    tendon_guide_geometries = disk_geometry.tendon_guide_geometriesDF

    # Validation
    # errors = validate_disk_param(outer_diameter,
    #                              length,
    #                              bottom_curve_radius,
    #                              top_curve_radius,
    #                              top_orientationDF,
    #                              centre_hole_diameter,
    #                              tendon_guide_geometries)
    # if len(errors) > 0:
    #     raise Exception(errors)

    TC = 0.1  # Constant to eliminate numerical errors in computation

    disk_object = (cq.Workplane("XY").circle(outer_diameter/2))

    if curve_only and centre_hole_diameter > 0:
        disk_object = disk_object.circle(centre_hole_diameter/2)


    disk_object = disk_object.extrude(length/2, both=True)



    if top_curve_radius:
        top_curve_object = (cq.Workplane("YZ")
                            .transformed(rotate=(0, top_orientationDF, 0))
                            .moveTo(-(outer_diameter/2+TC), length+TC)
                            .vLineTo(sqrt(top_curve_radius**2-(outer_diameter/2+TC)**2)-top_curve_radius+length/2)
                            .radiusArc((outer_diameter/2+TC, sqrt(top_curve_radius**2-(outer_diameter/2+TC)**2)-top_curve_radius+length/2), top_curve_radius)
                            .vLineTo(length+TC)
                            .close()
                            .extrude(outer_diameter/2+TC, both=True))
        disk_object = disk_object.cut(top_curve_object)
        
    if bottom_curve_radius:
        bottom_curve_object = (cq.Workplane("YZ")
                               .moveTo(-(outer_diameter/2+TC), -length-TC)
                               .vLineTo(-sqrt(bottom_curve_radius**2-(outer_diameter/2+TC)**2)+bottom_curve_radius-length/2)
                               .radiusArc((outer_diameter/2+TC, -sqrt(bottom_curve_radius**2-(outer_diameter/2+TC)**2)+bottom_curve_radius-length/2), -bottom_curve_radius)
                               .vLineTo(-length-TC)
                               .close()
                               .extrude(outer_diameter/2+TC, both=True))
        disk_object = disk_object.cut(bottom_curve_object)

    
    if curve_only:
        return disk_object
    
    # tendon guides
    tendon_guide_objects = cq.Workplane("XY")
    for tg in tendon_guide_geometries:
        print(tg)
        tendon_guide_objects = (tendon_guide_objects.moveTo(tg.dist_from_axis*cos(tg.orientationBF), tg.dist_from_axis*sin(tg.orientationBF))
                       .circle(tg.diameter/2))
    tendon_guide_objects = tendon_guide_objects.extrude(length/2+TC, both=True)
    disk_object = (disk_object.cut(tendon_guide_objects))
    
    return disk_object


def export_CAD(obj, path=None, export_type="step", override=True, tolerance=0.001):
    import os
    print(export_type)
    
    export_type_conversion_dict = {
        "step":cq.exporters.ExportTypes.STEP,
        "stl": cq.exporters.ExportTypes.STL,
    }
    
    if export_type not in export_type_conversion_dict:
        return False
    
    # Generate a meaningful path if None
    if path is None:
        import datetime
        path = f"{datetime.datetime.now().strftime('%m_%d_%Y__%H_%M_%S')}"
    
    # Append proper file extension if it does not exist
    if os.path.basename(path).rfind(".") == -1:
        
        path += f".{export_type.lower()}"
    
    # Avoid override if necessary
    if not override and os.path.exists(path):
        return False
    
    # Write file
    with open(path, "w") as f:
        cq.exporters.exportShape(
            obj, exportType=export_type_conversion_dict[export_type], fileLike=f, tolerance=tolerance)


# if __name__ == "__main__":
#     """
#     Parameters of the geometry
#     """
#     param = {
#         "outerDiameter": 5,
#         "length": 5,
#         "bottomCurvatureRadius": None,
#         "topCurvatureRadius": 3,
#         "topCurvatureRelativeOrientation": 90,
#         "centreHoleDiameter": 1,
#     }

#     export(generate_disk_CAD(DiskGeometry(
#         5, 5, 0, None, 0, 3, 1, []
#     ))
#         [0], filename="standalone.step", exportType=cq.exporters.ExportTypes.STEP)
