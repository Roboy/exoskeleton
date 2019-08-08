import numpy as np
import xml.etree.ElementTree as ET
from xml.dom import minidom
from osim_converter import prettify

# the heights or z values of the via points are constants, so we handle them like that
shoulder_height = -0.014766129883181094
upper_arm_first_height = 0.08614385644926131
upper_arm_sec_height = -0.07385614355073884
lower_arm_first_height = 0.06816106739960062
lower_arm_sec_height = -0.06766913996516934

# values are given in meters since gazebo and the rest of the sim environment is using this unit
shoulder_radius = 0.05
upper_arm_radius = 0.028
lower_arm_radius = 0.035

m0_0_default_distance = 0.025
m3_0_default_distance = 0.075
m1_0_default_distance = 0.075
m2_0_default_distance = 0.025

m0_1_default_distance = 0.014
m1_1_default_distance = 0.042
m2_1_default_distance = 0.014
m3_1_default_distance = 0.042

m4_0_default_distance = 0.014
m5_0_default_distance = 0.042
m6_0_default_distance = 0.014
m7_0_default_distance = 0.042

m4_1_default_distance = 0.0175
m5_1_default_distance = 0.0525
m6_1_default_distance = 0.0175
m7_1_default_distance = 0.0525

m4_2_default_distance = 0.0175
m5_2_default_distance = 0.0525
m6_2_default_distance = 0.0175
m7_2_default_distance = 0.0525


def y_from_d(distance, radius):
    """
    simply solved pythagoras for y. Origin: r^2 = x^2 + y^2
    :param distance: the distance from the right border of the circle
    :param radius: the radius of the circle
    :return:
    """
    return np.sqrt((radius * radius) - ((radius - distance) * (radius - distance)))


def vp_position(distance, radius, height, back=False):
    """
    The final via point position consisting of the derived y value from the distance and the radius
    :param distance: distance of the vp from the right border
    :param radius: radius of the circle describing the cylinder
    :param height: default height of vps of this class
    :param back: boolean value if vp is on the back side, if then y is inverted
    :return: an array of x, y and z that can be directly pasted into cardsflow.xml
    """
    x = radius - distance
    y = y_from_d(distance, radius)
    if back:
        y = -y
    z = height
    return y, x, z


def vps_to_xml(motor_vp):
    cardsflow_xml = ET.Element("cardsflow")
    for i in range(0, 8):
        motor = ET.SubElement(cardsflow_xml, "myoMuscle")
        motor.set("name", "motor" + str(i))
        if i in range(0, 4):
            shoulder = ET.SubElement(motor, "link")
            shoulder.set("name", "shoulder")
            s_vp = ET.SubElement(shoulder, "viaPoint", {"type": "FIXPOINT"})
            s_vp.text = str(motor_vp[i][0][0]) + " " + str(motor_vp[i][0][1]) + " " + str(motor_vp[i][0][2])

            upper_arm = ET.SubElement(motor, "link")
            upper_arm.set("name", "upperArm")
            u_vp = ET.SubElement(upper_arm, "viaPoint", {"type": "FIXPOINT"})
            u_vp.text = str(motor_vp[i][1][0]) + " " + str(motor_vp[i][1][1]) + " " + str(motor_vp[i][1][2])
        elif i in range(4, 8):
            upper_arm = ET.SubElement(motor, "link")
            upper_arm.set("name", "upperArm")
            u_vp = ET.SubElement(upper_arm, "viaPoint", {"type": "FIXPOINT"})
            u_vp.text = str(motor_vp[i][0][0]) + " " + str(motor_vp[i][0][1]) + " " + str(motor_vp[i][0][2])

            lower_arm = ET.SubElement(motor, "link")
            lower_arm.set("name", "lowerArm")
            l_vp_1 = ET.SubElement(lower_arm, "viaPoint", {"type": "FIXPOINT"})
            l_vp_1.text = str(motor_vp[i][1][0]) + " " + str(motor_vp[i][1][1]) + " " + str(motor_vp[i][1][2])

    pretty_string = prettify(cardsflow_xml)
    with open("test_cardsflow.xml", "w") as output_file:
        output_file.write(pretty_string)


def distances_to_xml(distances):
    motor_vp = []

    for i in range(8):
        motor_vp.append([])

    motor_vp[0].append(vp_position(distances[0], shoulder_radius, shoulder_height))
    motor_vp[1].append(vp_position(distances[1], shoulder_radius, shoulder_height))
    motor_vp[2].append(vp_position(distances[2], shoulder_radius, shoulder_height, True))
    motor_vp[3].append(vp_position(distances[3], shoulder_radius, shoulder_height, True))

    motor_vp[0].append(vp_position(distances[4], upper_arm_radius, upper_arm_first_height))
    motor_vp[1].append(vp_position(distances[5], upper_arm_radius, upper_arm_first_height))
    motor_vp[2].append(vp_position(distances[6], upper_arm_radius, upper_arm_first_height, True))
    motor_vp[3].append(vp_position(distances[7], upper_arm_radius, upper_arm_first_height, True))

    motor_vp[4].append(vp_position(distances[8], upper_arm_radius, upper_arm_sec_height))
    motor_vp[5].append(vp_position(distances[9], upper_arm_radius, upper_arm_sec_height))
    motor_vp[6].append(vp_position(distances[10], upper_arm_radius, upper_arm_sec_height, True))
    motor_vp[7].append(vp_position(distances[11], upper_arm_radius, upper_arm_sec_height, True))

    motor_vp[4].append(vp_position(distances[12], lower_arm_radius, lower_arm_first_height))
    motor_vp[5].append(vp_position(distances[13], lower_arm_radius, lower_arm_first_height))
    motor_vp[6].append(vp_position(distances[14], lower_arm_radius, lower_arm_first_height, True))
    motor_vp[7].append(vp_position(distances[15], lower_arm_radius, lower_arm_first_height, True))

    vps_to_xml(motor_vp)


if __name__ == "__main__":
    distances = [m0_0_default_distance, m3_0_default_distance, m1_0_default_distance, m2_0_default_distance, m0_1_default_distance, m1_1_default_distance,
                 m2_1_default_distance, m3_1_default_distance, m4_0_default_distance, m5_0_default_distance, m6_0_default_distance, m7_0_default_distance,
                 m4_1_default_distance, m5_1_default_distance, m6_1_default_distance, m7_1_default_distance, m4_2_default_distance, m5_2_default_distance,
                 m6_2_default_distance, m7_2_default_distance]

    distances_to_xml(distances)
