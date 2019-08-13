import xml.etree.ElementTree as ET
from xml.dom import minidom
from osim_converter import prettify
from float_to_vp import vp_position, shoulder_height, shoulder_radius, upper_arm_radius, upper_arm_first_height
from float_to_vp import m0_0_default_distance, m1_0_default_distance, m2_0_default_distance, m3_0_default_distance, \
    m0_1_default_distance, m1_1_default_distance, m2_1_default_distance, m3_1_default_distance


def update_osim(old_osim_path, via_points):
    # read old csv file
    old_osim_tree = ET.parse(old_osim_path)
    osim_root = old_osim_tree.getroot()
    osim_force_set = [force_set for force_set in osim_root.iter("ForceSet")][0]
    osim_muscles = [muscle for muscle in osim_force_set.iter("Thelen2003Muscle")]
    for muscle in osim_muscles:
        # print muscle.get("name")
        path_points = [point for point in muscle.iter("PathPoint")]
        for point in path_points:
            location = [loc for loc in point.iter("location")][0]
            # print "  ", point.get("name"), location.text
            if point.get("name") == "muscle0_node0":
                location.text = str(via_points[0][0][0]) + " " + str(via_points[0][0][1]) + " " + str(
                    via_points[0][0][2])
            if point.get("name") == "muscle0_node1":
                location.text = str(via_points[0][1][0]) + " " + str(via_points[0][1][1]) + " " + str(
                    via_points[0][1][2])
            if point.get("name") == "muscle1_node0":
                location.text = str(via_points[1][0][0]) + " " + str(via_points[1][0][1]) + " " + str(
                    via_points[1][0][2])
            if point.get("name") == "muscle1_node1":
                location.text = str(via_points[1][1][0]) + " " + str(via_points[1][1][1]) + " " + str(
                    via_points[1][1][2])
            if point.get("name") == "muscle2_node0":
                location.text = str(via_points[2][0][0]) + " " + str(via_points[2][0][1]) + " " + str(
                    via_points[2][0][2])
            if point.get("name") == "muscle2_node1":
                location.text = str(via_points[2][1][0]) + " " + str(via_points[2][1][1]) + " " + str(
                    via_points[2][1][2])
            if point.get("name") == "muscle3_node0":
                location.text = str(via_points[3][0][0]) + " " + str(via_points[3][0][1]) + " " + str(
                    via_points[3][0][2])
            if point.get("name") == "muscle3_node1":
                location.text = str(via_points[3][1][0]) + " " + str(via_points[3][1][1]) + " " + str(
                    via_points[3][1][2])

    with open(old_osim_path, "w") as temp_file:
        temp_file.write(minidom.parseString(ET.tostring(osim_root)).toxml())


def floats_to_path_points(float_list):
    path_points = []
    for i in range(4):
        path_points.append([])

    path_points[0].append(vp_position(float_list[0], shoulder_radius, shoulder_height))
    path_points[1].append(vp_position(float_list[1], shoulder_radius, shoulder_height))
    path_points[2].append(vp_position(float_list[2], shoulder_radius, shoulder_height, True))
    path_points[3].append(vp_position(float_list[3], shoulder_radius, shoulder_height, True))

    path_points[0].append(vp_position(float_list[4], upper_arm_radius, upper_arm_first_height))
    path_points[1].append(vp_position(float_list[5], upper_arm_radius, upper_arm_first_height))
    path_points[2].append(vp_position(float_list[6], upper_arm_radius, upper_arm_first_height, True))
    path_points[3].append(vp_position(float_list[7], upper_arm_radius, upper_arm_first_height, True))

    return path_points


if __name__ == "__main__":
    path_points = floats_to_path_points(
        [m0_0_default_distance, m1_0_default_distance, m2_0_default_distance, m3_0_default_distance,
         m0_1_default_distance, m1_1_default_distance, m2_1_default_distance, m3_1_default_distance])

    update_osim("temp.osim", path_points)
