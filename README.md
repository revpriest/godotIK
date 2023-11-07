# Inverse Kinematics For Godot 4

Moving the bones in a Godot skeleton such that the
hands are placed on a target node.

Like this:

https://github.com/revpriest/godotIK/raw/main/demo.wemb?raw=true

[![](demo.webm)](demo.webm)


The built-in system is apparently depreciated in V4, it's not clear if it'll be replaced.

Maybe this should be user-land code anyway.

This is mostly cribbed from https://github.com/GainAw/GodotCSharpProjectTests/
but updated for Godot4 and to suit my own needs better.
The magnet being a node not a vector, look-at node and 
things like that.

## Two Bones Only

Well, three if you include the wrist at the end.

Maybe this can be extended to more, I dunno how,
maybe I'll even have to one day but not today.
Let me know if you do one.

## Use

Attach a node to the target, or anywhere really.

Set the parameters:

* Target Node = Where the rig should point
* manget Node = Where the elbow should point if it can (Null for none)
* Look At = Null for keeping bone rotation, a node to look at otherwise
* initBoneNames = strings for root, middle, end bones that will be manipupated.
     Or fill in the IDs in the boneIds array.
* boneRestRotset = Set of rotations to apply to bone rest-pose,
                   can be useful if the wrist is backwards or wrong z-axis or whatever. 

# Demo Scene

All you will need in your project is the IKSolver script to
attach to your target nodes.

Everything else in this project is just a demo scene.