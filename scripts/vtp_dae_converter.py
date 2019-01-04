import os
import vtk
from shutil import rmtree


def vtp_to_dae(vtp_dir, dae_output_dir):
    """
    This will combine the vtp to stl translation and the stl to dae conversion
    :param vtp_dir: the directory that contains vtp files
    :param dae_output_dir: the output directory that will get the visual and the collisions folder
    with the according files
    """
    stl_dir = "/Users/Kevin/Documents/Uni/RCI/Roboy/git_repos/exoskeleton/output/meshes/stl"

    # create stl directory if it doesn't exist
    try:
        os.makedirs(stl_dir)
    except OSError:
        # it just says that the path already exists
        print "meshes/visual already exists"

    # first, convert all vtps to stls
    for dirpath, dirnames, filenames in os.walk(vtp_dir):
        for filename in [f for f in filenames if f.endswith(".vtp")]:
            # call the convert script

            vtp_to_stl(os.path.join(dirpath, filename),
                       stl_dir + "/" +
                       filename.split(".")[0] + ".stl")

    # second, convert all stls to daes
    os.system('/Applications/Blender/blender.app/Contents/MacOS/blender --background --python '
              '/Users/Kevin/Documents/Uni/RCI/Roboy/git_repos/common_utilities/python/reduce-stl-dae-bpy.py -- ' +
              stl_dir + ' ' +
              dae_output_dir + ' '
              '1 '
              '1')

    # in the end, clean up after ourselfs and remove the intermediate stl folder
    rmtree(stl_dir)


def vtp_to_stl(vtp_file, stl_file):
    """
    Converts a vtp file to a stl file by using the vtk library
    :param vtp_file: path to the vtp file
    :param stl_file: path to the resulting stl file
    """
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
    vtp_to_dae("/Users/Kevin/Documents/Uni/RCI/Roboy/git_repos/exoskeleton/output/meshes/visual",
               '/Users/Kevin/Documents/Uni/RCI/Roboy/git_repos/exoskeleton/output/meshes/')
