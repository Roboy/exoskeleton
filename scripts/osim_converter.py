import xml.etree.ElementTree as ET
from xml.dom import minidom
import os
from shutil import copy2, rmtree
import vtp_dae_converter as vtp_conv


def prettify(elem):
    """Return a pretty-printed XML string for the Element.
    """
    rough_string = ET.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")


def forceset_adjustment(forceset):
    # forceset adjustement
    for muscle in forceset.iter("Thelen2003Muscle"):
        del_list = ["isDisabled", "min_control", "max_control", "optimal_force", "pennation_angle_at_optimal",
                    "max_contraction_velocity", "activation_time_constant", "deactivation_time_constant",
                    "FmaxTendonStrain", "FmaxMuscleStrain", "KshapeActive", "KshapePassive", "Af", "Flen"]
        remove_nodes(del_list, muscle)

    for geometry in forceset.iter("GeometryPath"):
        del_list = ["VisibleObject", "PathWrapSet"]
        remove_nodes(del_list, geometry)


def remove_nodes(del_list, parent):
    for to_del in del_list:
        for node in parent.findall(to_del):
            parent.remove(node)


def bodyset_adjustment(bodyset):
    bodyset_objects = [objects for objects in bodyset.iter("objects")][0]

    for body in bodyset_objects.findall("Body"):
        # if no WrapObjectSet delete
        if body.find("WrapObjectSet") is None:
            bodyset_objects.remove(body)
        # else delte everything that's not needed
        else:
            del_list = ["mass", "mass_center", "inertia_xx", "inertia_yy", "inertia_zz", "inertia_xy",
                        "inertia_xz", "inertia_yz", "Joint", "VisibleObject"]
            remove_nodes(del_list, body)


def future_shizzle(bodyset):
    """ for the future when i don't just delete but also modify and order"""
    # bodyset shizzle
    new_bodyset = ET.Element("BodySet")
    objects_node = ET.SubElement(new_bodyset, "objects")
    for body in bodyset[0].iter("Body"):
        new_body = ET.SubElement(objects_node, "Body")
        new_body.set("name", body.get("name"))


def create_osim(file_path):
    model_name = file_path.split("/")[-1].split(".")[0]
    # read the osim xml file and transform it to a XML ElementTree
    tree = ET.parse(file_path)
    root = tree.getroot()

    # get the Body and the ForceSet from the original file
    bodyset = [body for body in root.iter("BodySet")]
    forceset = [force for force in root.iter("ForceSet")]

    # make some adjustment like deleting some nodes
    forceset_adjustment(forceset[0])
    bodyset_adjustment(bodyset[0])

    # create a new node root node that will get the adjusted body and foceset as child nodes
    new_osim = ET.Element(root.tag)
    new_osim.set("Version", root.get("Version"))
    model = ET.SubElement(new_osim, "Model")
    model.append(forceset[0])
    model.append(bodyset[0])

    # add according new lines and tabs, and the xml version line
    pretty_string = prettify(new_osim)

    # write it into a file
    with open("/Users/Kevin/Documents/Uni/RCI/Roboy/git_repos/exoskeleton/output/" + model_name + "/muscles.osim", "w") as output_file:
        output_file.write(pretty_string)


def create_sdf(file_path):
    model_name = file_path.split("/")[-1].split(".")[0]
    # read the osim xml file to a ElementTree
    osim_tree = ET.parse(file_path)
    osim_root = osim_tree.getroot()

    # start the sdf tree
    sdf_node = ET.Element("sdf")
    sdf_node.set("version", "1.5")

    # get the model name
    osim_model = [model for model in osim_root.iter("Model")][0]

    # create a sdf model node with the same name
    sdf_model = ET.SubElement(sdf_node, "model", {"name": osim_model.get("name")})

    # add the osim partner as "muscles"
    muscle_node = ET.SubElement(sdf_model, "muscles")
    muscle_node.text = 'model://' + model_name + '/muscles.osim'

    # start adding body parts as links
    # first get the bodyset
    bodyset = [body for body in osim_root.iter("BodySet")][0]
    body_to_link(bodyset, sdf_model)

    # operation joint per joint
    bodyset_objects = [objects for objects in bodyset.iter("objects")][0]
    joints(bodyset_objects, sdf_model)

    # add the plugin that converts the muscles
    ET.SubElement(sdf_model, "plugin", {"filename": "libgazebo_ros_muscle_interface.so",
                                                "name": "muscle_interface_plugin"})
    pretty_string =  prettify(sdf_node)

    with open("/Users/Kevin/Documents/Uni/RCI/Roboy/git_repos/exoskeleton/output/" + model_name + "/model.sdf", "w") as output_file:
        output_file.write(pretty_string)


def joints(bodyset_objects, sdf_model):
    for body in bodyset_objects.findall("Body"):
        for osim_joint in body.iter("Joint"):
            if osim_joint.find("WeldJoint") is not None:
                sdf_joint = ET.SubElement(sdf_model, "joint")
                weld_joint = [temp for temp in osim_joint.iter("WeldJoint")][0]
                sdf_joint.set("name", weld_joint.get("name"))
                sdf_joint.set("type", "fixed")
                parent = ET.SubElement(sdf_joint, "parent")
                possible_parent = [temp for temp in weld_joint.iter("parent_body")][0].text
                if possible_parent == "ground":
                    possible_parent = "world"
                parent.text = possible_parent

                # the pose of the joint being stuffed together from location_in_parent & orientation_in_parent
                # location = [temp for temp in weld_joint.iter("location_in_parent")][0].text
                # orientation = [temp for temp in weld_joint.iter("orientation_in_parent")][0].text
                # pose = ET.SubElement(sdf_joint, "pose")
                # pose.text = location + " " + orientation

                # parent is written down, child is the body that is used right now
                child = ET.SubElement(sdf_joint, "child")
                child.text = body.get("name")

            elif osim_joint.find("CustomJoint") is not None:
                sdf_joint = ET.SubElement(sdf_model, "joint")
                custom_joint = [temp for temp in osim_joint.iter("CustomJoint")][0]
                sdf_joint.set("name", custom_joint.get("name"))
                sdf_joint.set("type", "ball")
                parent = ET.SubElement(sdf_joint, "parent")
                parent.text = [temp for temp in custom_joint.iter("parent_body")][0].text

                # the pose of the joint being stuffed together from location_in_parent & orientation_in_parent
                # location = [temp for temp in custom_joint.iter("location_in_parent")][0].text
                # orientation = [temp for temp in custom_joint.iter("orientation_in_parent")][0].text
                # pose = ET.SubElement(sdf_joint, "pose")
                # pose.text = location + orientation

                # parent is written down, child is the body that is used right now
                child = ET.SubElement(sdf_joint, "child")
                child.text = body.get("name")


def body_to_link(bodyset, sdf_model):
    bodyset_objects = [objects for objects in bodyset.iter("objects")][0]
    pose_offset = [0.0, 0.0, 0.0, 1.5708, 0.0, 0.0]
    for body in bodyset_objects.findall("Body"):
        # ignore the ground body, it's not needed in gazebo
        if body.get("name") == "ground":
            continue

        # create new link that is the equivalence to a body
        link = ET.SubElement(sdf_model, "link", {"name": body.get("name")})

        # the pose is derived from the joint location and orientation of the previous joints
        # and the ones from this joint
        osim_joint = [temp for temp in body.iter("Joint")][0]
        location = None
        orientation = None

        if osim_joint.find("WeldJoint") is not None:
            weld_joint = [temp for temp in osim_joint.iter("WeldJoint")][0]

            # the pose of the joint being stuffed together from location_in_parent & orientation_in_parent
            location = [temp for temp in weld_joint.iter("location_in_parent")][0].text
            orientation = [temp for temp in weld_joint.iter("orientation_in_parent")][0].text

        elif osim_joint.find("CustomJoint") is not None:
            custom_joint = [temp for temp in osim_joint.iter("CustomJoint")][0]

            # the pose of the joint being stuffed together from location_in_parent & orientation_in_parent
            location = [temp for temp in custom_joint.iter("location_in_parent")][0].text
            orientation = [temp for temp in custom_joint.iter("orientation_in_parent")][0].text

        pose = ET.SubElement(link, "pose", {"frame": ""})
        try:
            pose_offset[0] += float(location.strip().split(" ")[0])
            pose_offset[1] -= float(location.strip().split(" ")[2])
            pose_offset[2] += float(location.strip().split(" ")[1])

            pose_offset[3] += float(orientation.strip().split(" ")[0])
            pose_offset[4] += float(orientation.strip().split(" ")[1])
            pose_offset[5] += float(orientation.strip().split(" ")[2])
        except ValueError:
            print "location ", location.strip().split(" ")
            print "orientation ", orientation.strip().split(" ")
            exit(1)

        pose.text = str(pose_offset[0]) + ' ' + str(pose_offset[1]) + ' ' + str(pose_offset[2]) + ' ' + \
                    str(pose_offset[3]) + ' ' + str(pose_offset[4]) + ' ' + str(pose_offset[5])

        # add the inertial part
        inertial = ET.SubElement(link, "inertial")

        # and its subcomponents
        mass = ET.SubElement(inertial, "mass")
        mass.text = [body_mass for body_mass in body.iter("mass")][0].text
        if mass.text == "0":
            mass.text = "0.0001"
        inertial_pose = ET.SubElement(inertial, "pose")
        inertial_pose.text = '0.0 0.0 0.0 0.0 0.0 0.0'

        inertia_sub = ET.SubElement(inertial, "inertia")
        ixx = ET.SubElement(inertia_sub, "ixx")
        ixx.text = [inertia_xx for inertia_xx in body.iter("inertia_xx")][0].text

        ixy = ET.SubElement(inertia_sub, "ixy")
        ixy.text = [inertia_xy for inertia_xy in body.iter("inertia_xy")][0].text

        ixz = ET.SubElement(inertia_sub, "ixz")
        ixz.text = [inertia_xz for inertia_xz in body.iter("inertia_xz")][0].text

        iyy = ET.SubElement(inertia_sub, "iyy")
        iyy.text = [inertia_yy for inertia_yy in body.iter("inertia_yy")][0].text

        iyz = ET.SubElement(inertia_sub, "iyz")
        iyz.text = [inertia_yz for inertia_yz in body.iter("inertia_yz")][0].text

        izz = ET.SubElement(inertia_sub, "izz")
        izz.text = [inertia_zz for inertia_zz in body.iter("inertia_zz")][0].text

        add_sdf_visuals(body, link, sdf_model.get("name"))


def add_sdf_visuals(body, link, model_name):
    if body.find("VisibleObject") is not None:
        # the visual part that is the counterpart to the GeometrySet
        # find the geometry set in the VisibleObject
        visible_object = [temp for temp in body.iter("VisibleObject")][0]
        geometry_set = [temp for temp in visible_object.iter("GeometrySet")][0]
        geometry_set_object = [temp for temp in geometry_set.iter("objects")][0]

        # the intermediate directory that contains the vtp files
        vtp_dir = "/Users/Kevin/Documents/Uni/RCI/Roboy/git_repos/exoskeleton/output/" + model_name + "/meshes/vtp"

        # create meshes directory
        try:
            os.makedirs(vtp_dir)
        except OSError:
            # it just says that the path already exists
            print "meshes/visual already exists"

        for geometry in geometry_set_object.findall("DisplayGeometry"):
            # find the file because its content is needed first
            geometry_file = [temp for temp in geometry.iter("geometry_file")][0]

            # with the file info, create the visual object for the sdf
            visual = ET.SubElement(link, "visual", {"name": geometry_file.text.split(".")[0]})

            # add the necessary visual parts
            # pose equals transform
            visual_pose = ET.SubElement(visual, "pose")
            visual_pose.text = [temp for temp in geometry.iter("transform")][0].text

            # geometry contains the info about the file and the scale factor
            visual_geometry = ET.SubElement(visual, "geometry")
            mesh = ET.SubElement(visual_geometry, "mesh")
            uri = ET.SubElement(mesh, "uri")

            # we already point to the resulting dae file, that will exist after the vtp to dae conversion
            uri.text = 'model://' + model_name + '/meshes/visuals/' + geometry_file.text.split('.')[0] + ".dae"
            scale = ET.SubElement(mesh, "scale")
            scale.text = [temp for temp in geometry.iter("scale_factors")][0].text

            # copy the file into meshes/visual
            for dirpath, dirnames, filenames in os.walk(
                    "/Applications/OpenSim 4.0-2018-08-27-ae111a4/OpenSim 4.0-2018-08-27-ae111a4.app/Contents"):
                for filename in [f for f in filenames if f.endswith(geometry_file.text)]:
                    # copy that file into the dedicated place
                    copy2(os.path.join(dirpath, filename), vtp_dir + "/")

        # after this step the vtp files are in the dedicated folder, so we can start the conversion
        vtp_conv.vtp_to_dae(vtp_dir, "/Users/Kevin/Documents/Uni/RCI/Roboy/git_repos/exoskeleton/output/" + model_name
                            + "/meshes/")
        # leftover,  we have a vtp folder that we don't need any more
        rmtree(vtp_dir)


def create_config(model_name):
    # create model.config
    model = ET.Element("model")

    name = ET.SubElement(model, "name")
    name.text = model_name

    version = ET.SubElement(model, "version")
    version.text = "1.0"

    sdf = ET.SubElement(model, "sdf")
    sdf.set("version", "1.5")
    sdf.text = "model.sdf"

    author = ET.SubElement(model, "author")
    author_name = ET.SubElement(author, "name")
    author_name.text = "Kevin Just"
    author_email = ET.SubElement(author, "email")
    author_email.text = "kevinjust87@gmail.com"

    description = ET.SubElement(model, "description")
    description.text = "Gazebo Model of the " + model_name + " OpenSim Model"

    pretty_string = prettify(model)

    with open("/Users/Kevin/Documents/Uni/RCI/Roboy/git_repos/exoskeleton/output/" + model_name + "/model.config", "w")\
            as output_file:
        output_file.write(pretty_string)


def convert_osim(osim_file):
    model_name = osim_file.split("/")[-1].split(".")[0]

    try:
        os.makedirs("/Users/Kevin/Documents/Uni/RCI/Roboy/git_repos/exoskeleton/output/" + model_name)
    except OSError:
        # it just says that the path already exists
        print model_name + " already exists"

    try:
        os.makedirs("/Users/Kevin/Documents/Uni/RCI/Roboy/git_repos/exoskeleton/output/" + model_name + "/meshes")
    except OSError:
        # it just says that the path already exists
        print model_name + "/meshes already exists"

    create_osim(osim_file)
    create_sdf(osim_file)
    create_config(model_name)


if __name__ == "__main__":
    #convert_osim('/Users/Kevin/Documents/Uni/RCI/Roboy/git_repos/exoskeleton/input/arm26.osim')
    convert_osim("/Users/Kevin/Documents/Uni/RCI/Roboy/git_repos/exoskeleton/input/leg6dof9musc.osim")
