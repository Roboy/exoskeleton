import xml.etree.ElementTree as ET
from xml.dom import minidom
import os
from osim_converter import prettify


def join_models(base_model_file_path, attached_model_file_path):
    # get the the model trees
    base_tree = ET.parse(base_model_file_path)
    attached_tree = ET.parse(attached_model_file_path)

    # extract the model nodes
    base_model = [model for model in base_tree.getroot().iter("model")][0]
    attached_model = [model for model in attached_tree.getroot().iter("model")][0]

    # add the new pose to the models
    base_pose = ET.SubElement(base_model, "pose")
    base_pose.set("frame", '')
    base_pose.text = "0.007648 0.001248 0.100007 0 -0 0"

    attached_pose = ET.SubElement(attached_model, "pose")
    attached_pose.set("frame", '')
    attached_pose.text = "-0.007648 -0.001248 -0.100006 0 -0 0"

    # create root model that contains the two models and the joints
    sdf_node = ET.Element("sdf", {"version": "1.6"})
    root_model = ET.SubElement(sdf_node, "model", {"name": "ExoSkeleton"})

    # add the two models
    root_model.append(attached_model)
    root_model.append(base_model)

    # add the joints that link the two models
    append_fixed_joint(root_model, "base_JOINT_3", "arm26::base", "ExoSuit::shoulder")
    append_fixed_joint(root_model, "r_humerus_JOINT_4", "arm26::r_humerus", "ExoSuit::upperArm")
    append_fixed_joint(root_model, "r_ulna_radius_hand_JOINT_5", "arm26::r_ulna_radius_hand",
                       "ExoSuit::lowerArm")

    # add the remaining to tags
    static = ET.SubElement(root_model, "static")
    static.text = "0"

    disable = ET.SubElement(root_model, "allow_auto_disable")
    disable.text = "1"

    return prettify(sdf_node)


def append_fixed_joint(parent_node, joint_name, parent_name, child_name):
    """
    appends a joint that fixes the parent link to the child link, to the parent_node

    :param parent_node: the node that gets the joint
    :param joint_name:  the name of the joint
    :param parent_name: the name of the parent link
    :param child_name:  the name of the child link
    """
    # creating the root element of the joint
    joint = ET.SubElement(parent_node, "joint", {"name": joint_name, "type": "fixed"})

    # adding the parent and the child node
    joint_parent = ET.SubElement(joint, "parent")
    joint_parent.text = parent_name

    joint_child = ET.SubElement(joint, "child")
    joint_child.text = child_name

    # everything from here on is common to all fixed links of this type
    joint_pose = ET.SubElement(joint, "pose", {"frame": ''})
    joint_pose.text = "0 0 0 0 -0 0"

    joint_physics = ET.SubElement(joint, "physics")
    joint_physics_ode = ET.SubElement(joint_physics, "ode")
    joint_physics_ode_limit = ET.SubElement(joint_physics_ode, "limit")

    joint_physics_ode_limit_cfm = ET.SubElement(joint_physics_ode_limit, "cfm")
    joint_physics_ode_limit_cfm.text = "0"

    joint_physics_ode_limit_erp = ET.SubElement(joint_physics_ode_limit, "erp")
    joint_physics_ode_limit_erp.text = "0.2"

    joint_physics_ode_suspension = ET.SubElement(joint_physics_ode, "suspension")

    joint_physics_ode_suspension_cfm = ET.SubElement(joint_physics_ode_suspension, "cfm")
    joint_physics_ode_suspension_cfm.text = "0"

    joint_physics_ode_suspension_erp = ET.SubElement(joint_physics_ode_suspension, "erp")
    joint_physics_ode_suspension_erp.text = "0.2"


def create_sdf(file_path, input):
    with open(file_path, "w") as sdf_file:
        sdf_file.write(input)


def create_config(file_path, model_name):
    # create model.config
    model = ET.Element("model")

    name = ET.SubElement(model, "name")
    name.text = model_name

    version = ET.SubElement(model, "version")
    version.text = "1.0"

    sdf = ET.SubElement(model, "sdf")
    sdf.set("version", "1.6")
    sdf.text = "model.sdf"

    author = ET.SubElement(model, "author")
    author_name = ET.SubElement(author, "name")
    author_email = ET.SubElement(author, "email")

    description = ET.SubElement(model, "description")

    pretty_string = prettify(model)

    with open(file_path, "w") as output_file:
        output_file.write(pretty_string)


def create_model(model_name, base_sdf,
                 attached_sdf):
    try:
        os.makedirs("/Users/Kevin/Documents/Uni/RCI/Roboy/git_repos/exoskeleton/output/" + model_name)
    except OSError:
        # it just says that the path already exists
        print model_name + " already exists"

    create_sdf("/Users/Kevin/Documents/Uni/RCI/Roboy/git_repos/exoskeleton/output/" + model_name + "/model.sdf",
               join_models(base_sdf,
                           attached_sdf)
               )
    create_config("/Users/Kevin/Documents/Uni/RCI/Roboy/git_repos/exoskeleton/output/" + model_name + "/model.config",
                  model_name)


if __name__ == "__main__":
    create_model("ExoSkeleton", "/Users/Kevin/Documents/Uni/RCI/Roboy/git_repos/exoskeleton/output/arm26/model.sdf",
                 "/Users/Kevin/Documents/Uni/RCI/Roboy/git_repos/exoskeleton/models/ExoSuit/model.sdf")
