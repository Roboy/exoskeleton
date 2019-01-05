# Exoskeleton structure optimization using muskuloskeletal actuators
"The best Idea ever" - Albert Einstein, pretty smart dude

## notes
### stl to dae command

    blender --background --python reduce-stl-dae-bpy.py -- /Users/Kevin/Documents/Uni/RCI/Roboy/git_repos/exoskeleton/output/meshes/stl /Users/Kevin/Documents/Uni/RCI/Roboy/git_repos/exoskeleton/output/meshes/ 1 1

### possible sdf joint configurations
[sdf joint specification](http://sdformat.org/spec?elem=joint)

### gazebo hacks
if you want gazebo to work properly, you need to activate the OpenSim engine by 
calling gazebo accordingly 

    gazebo -e opensim

#### notes for enhancing the converter
- in pose the second an the third entry of the translation need to be exchanged
- the poses need to be added up starting from the parents parent and so on
- you can see how they were added up by comparing arm26.osim and model.sdf from amr26
