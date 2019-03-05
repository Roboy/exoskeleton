# Author-
# Description-

import adsk.core
import adsk.fusion
import adsk.cam
import traceback
from collections import defaultdict


def run(context):
    ui = None

    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        # ui.messageBox('Hello script')
        # get active design
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        all_components = design.allComponents
        rootOcc = design.rootComponent.occurrences
        with open("/Users/Kevin/Documents/Uni/RCI/Roboy/git_repos/exoskeleton/geometry.txt", "w") as geometry_file:
            for com in all_components:
                if com is not None:

                    all_joints = com.asBuiltJoints

                    for joi in all_joints:
                        if joi is not None:
                            vec = adsk.fusion.RevoluteJointMotion.cast(joi.jointMotion).rotationAxisVector.asPoint()

                            # ui.messageBox("Component: " + com.name +
                            #               "\n As-Built Joint: " + joi.name +
                            #               "\n rotationalAxisVector.asPoint() :  " + str(vec.asArray()) +
                            #               "\n geometry : " + str(joi.geometry.origin.asArray()))
                            # geometry_file.write(joi.name + " geometry : " + str(joi.geometry.origin.asArray()) + "\n")

                    all_rigids = com.rigidGroups

                    # for rigid in all_rigids:
                    #     if rigid is not None:
                    #         for occ in rigid.occurrences:
                    #
                    #             ui.messageBox("Component: " + com.name +
                    #                           "\n RigidGroup: " + rigid.name +
                    #                           "\n Occurence: " + occ.name +
                    #                           "\n transform Matrix: " + str(occ.transform.translation.asArray()))

            for occ in rootOcc:
                if occ is not None:
                    ui.messageBox("Occurrence: " + occ.name +
                                  "\n x: " + str(occ.transform.translation.x * 0.01) +
                                  "\n y: " + str(occ.transform.translation.y * 0.01) +
                                  "\n z: " + str(occ.transform.translation.z * 0.01))



    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
