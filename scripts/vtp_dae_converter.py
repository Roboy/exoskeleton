import os
import vtk
import sys


def vtp_to_dae():
    """This will combine the vtp to stl translation and the stl to dae conversion"""
    # first, convert all vtps to stls
    for dirpath, dirnames, filenames in os.walk(
            "/Users/Kevin/Documents/Uni/RCI/Roboy/git_repos/exoskeleton/output/meshes/visual"):
        for filename in [f for f in filenames if f.endswith(".vtp")]:
            # call the convert script
            vtp_to_stl(os.path.join(dirpath, filename),
                       "/Users/Kevin/Documents/Uni/RCI/Roboy/git_repos/exoskeleton/output/meshes/stl/" +
                       filename.split(".")[0] + ".stl")

    # second, convert all stls to daes
    os.system('/Applications/Blender/blender.app/Contents/MacOS/blender --background --python '
              '/Users/Kevin/Documents/Uni/RCI/Roboy/git_repos/common_utilities/python/reduce-stl-dae-bpy.py -- '
              '/Users/Kevin/Documents/Uni/RCI/Roboy/git_repos/exoskeleton/output/meshes/stl '
              '/Users/Kevin/Documents/Uni/RCI/Roboy/git_repos/exoskeleton/output/meshes/ '
              '1 '
              '1')


def vtp_to_stl(vtp_file, stl_file):
    reader = vtk.vtkXMLPolyDataReader()
    reader.SetFileName(vtp_file)
    surface_filter = vtk.vtkDataSetSurfaceFilter()
    surface_filter.SetInputConnection(reader.GetOutputPort())
    triangle_filter = vtk.vtkTriangleFilter()
    triangle_filter.SetInputConnection(surface_filter.GetOutputPort())
    writer = vtk.vtkSTLWriter()
    writer.SetFileName(stl_file)
    writer.SetInputConnection(triangle_filter.GetOutputPort())
    writer.SetFileTypeToBinary()
    writer.Write()


if __name__ == "__main__":
    vtp_to_dae()
