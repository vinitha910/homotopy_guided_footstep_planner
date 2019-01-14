import os
import bpy
def deselect_all():
    for obj in bpy.data.objects:
        obj.select = False

def export():
    deselect_all()
    curr_path = bpy.path.abspath('//')
    for group in bpy.data.groups.keys():
        objects = bpy.data.groups[group].objects.keys()
        if len(objects) == 0:
            continue
        for object in objects:
            bpy.data.objects[object].select = True
        fpath = curr_path + group + ".stl"
        if group.find("stepping") != -1:
            fpath = curr_path + "stepping_surfaces/" + group + ".stl"
        elif group.find("surface") != -1:
            fpath = curr_path + "surfaces/" + group + ".stl"
        bpy.ops.export_mesh.stl(filepath=fpath, use_selection=True)
        deselect_all()

export()
