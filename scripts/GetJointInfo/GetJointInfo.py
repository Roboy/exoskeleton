#Author-
#Description-

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
        #ui.messageBox('Hello script')
        # get active design
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        all_components = design.allComponents
        allRigidGroups = design.rootComponent.allRigidGroups
        ui.messageBox('trying to get the components')
        with open("/Users/Kevin/Documents/Uni/RCI/Roboy/git_repos/exoskeleton/geometry.txt", "w") as geometry_file:
            for com in all_components:
                if com is not None:

                    all_joints = com.asBuiltJoints

                    for joi in all_joints:
                        if joi is not None:
                            vec = adsk.fusion.RevoluteJointMotion.cast(joi.jointMotion).rotationAxisVector.asPoint()

                            ui.messageBox("As-Built Joint: " + joi.name +
                                          "\n rotationalAxisVector.asPoint() :  " + str(vec.asArray()) +
                                          "\n geometry : " + str(joi.geometry.origin.asArray()))
                            ui.messageBox("attributes: " + str(joi.objectType))
                            geometry_file.write(joi.name + " geometry : " + str(joi.geometry.origin.asArray()) + "\n")


    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
