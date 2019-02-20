#Author-
#Description-

import adsk.core
import adsk.fusion
import adsk.cam
import traceback


def run(context):
    ui = None
    
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        ui.messageBox('Hello script')
        # get active design
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        all_components = design.allComponents
        ui.messageBox('trying to get the components')
        for com in all_components:
            if com is not None:
                ui.messageBox("Component: " + com.name)

                all_joints = com.asBuiltJoints

                for joi in all_joints:
                    if joi is not None:
                        ui.messageBox("As-Built Joint: " + joi.name)

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
