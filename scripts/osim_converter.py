import xml.etree.ElementTree as ET
from xml.dom import minidom


def prettify(elem):
    """Return a pretty-printed XML string for the Element.
    """
    rough_string = ET.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")


def forceset_adjustment(forceset):
    # forceset adjustement
    for muscle in forceset.iter("Thelen2003Muscle"):
        for node in muscle.findall("isDisabled"):
            muscle.remove(node)
        for node in muscle.findall("min_control"):
            muscle.remove(node)
        for node in muscle.findall("max_control"):
            muscle.remove(node)
        for node in muscle.findall("optimal_force"):
            muscle.remove(node)
        for node in muscle.findall("pennation_angle_at_optimal"):
            muscle.remove(node)
        for node in muscle.findall("max_contraction_velocity"):
            muscle.remove(node)
        for node in muscle.findall("activation_time_constant"):
            muscle.remove(node)
        for node in muscle.findall("deactivation_time_constant"):
            muscle.remove(node)
        for node in muscle.findall("FmaxTendonStrain"):
            muscle.remove(node)
        for node in muscle.findall("FmaxMuscleStrain"):
            muscle.remove(node)
        for node in muscle.findall("KshapeActive"):
            muscle.remove(node)
        for node in muscle.findall("KshapePassive"):
            muscle.remove(node)
        for node in muscle.findall("Af"):
            muscle.remove(node)
        for node in muscle.findall("Flen"):
            muscle.remove(node)

    for geometry in forceset.iter("GeometryPath"):
        for node in geometry.findall("VisibleObject"):
            geometry.remove(node)
        for node in geometry.findall("PathWrapSet"):
            geometry.remove(node)


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
            for to_del in del_list:
                for node in body.findall(to_del):
                    body.remove(node)


def future_shizzle(bodyset):
    """ for the future when i don't just delete but also modify and order"""
    # bodyset shizzle
    new_bodyset = ET.Element("BodySet")
    objects_node = ET.SubElement(new_bodyset, "objects")
    for body in bodyset[0].iter("Body"):
        new_body = ET.SubElement(objects_node, "Body")
        new_body.set("name", body.get("name"))


def create_osim(file_path):
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
    with open("/Users/Kevin/Documents/Uni/RCI/Roboy/git_repos/exoskeleton/output/muscles.osim", "w") as output_file:
        output_file.write(pretty_string)


if __name__ == "__main__":
    create_osim('/Users/Kevin/Documents/Uni/RCI/Roboy/git_repos/exoskeleton/input/arm26.osim')