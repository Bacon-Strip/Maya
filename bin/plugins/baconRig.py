"""
// ==========================================================================
// www.Bacon-Strip.com
// Luis Alonso
// Creative Commons 2017
// Bacon-Strip Node ID Block are: 0x0012a940 - 0x0012aa3f
// Registered for Bacon-Strip [Luis Alonso]
//
// ==========================================================================

////////////////////////////////////////////////////////////////////////
// DESCRIPTION:
// Produces the dependency graph node "baconRig".
// This plug-in is part of a rigging package known as The "Bacon-Strip"
//
// ID: 
// 0x0012a940
//
////////////////////////////////////////////////////////////////////////
"""


import math, sys
import maya.OpenMaya as OpenMaya
import maya.OpenMayaMPx as OpenMayaMPx

kPluginNodeTypeName = "baconRig"
baconRigId = OpenMaya.MTypeId(0x0012a940)


class baconRig(OpenMayaMPx.MPxNode):

    rigName = OpenMaya.MObject()
    rigType = OpenMaya.MObject()
    pickerName = OpenMaya.MObject()
    baconBits = OpenMaya.MObject()
    lodMeshes = OpenMaya.MObject()

    def __init__(self):
        OpenMayaMPx.MPxNode.__init__(self)

    def compute(self, plug, dataBlock):
        # INPUTS
        rootDagHandle = dataBlock.inputValue(self.rootDag)


def nodeCreator():
    return OpenMayaMPx.asMPxPtr( baconRig() )

def nodeInitializer():
    self = baconRig

    # Name
    rigNameAttr = OpenMaya.MFnTypedAttribute()
    rigNameAttrStr = OpenMaya.MFnStringData().create("Bacon01")
    self.rigName = rigNameAttr.create("rigName", "rn", OpenMaya.MFnStringData.kString, rigNameAttrStr)
    rigNameAttr.setStorable(True)
    rigNameAttr.setKeyable(False)
    self.addAttribute( self.rigName )

    # Rig Type
    rigTypeAttr = OpenMaya.MFnEnumAttribute()
    self.rigType = rigTypeAttr.create("rigType", "rt", 0)
    rigTypeAttr.addField("Custom", 0)
    rigTypeAttr.addField("BipedAnatomical", 1)
    rigTypeAttr.addField("BipedCartoon", 2)
    rigTypeAttr.addField("Quadruped", 3)
    rigTypeAttr.setHidden(False)
    rigTypeAttr.setKeyable(False)
    rigTypeAttr.setDefault(1)
    self.addAttribute( self.rigType )

    # Picker Name
    pickerNameAttr = OpenMaya.MFnTypedAttribute()
    pickerNameAttrStr = OpenMaya.MFnStringData().create("Generic")
    self.pickerName = pickerNameAttr.create("pickerName", "pn", OpenMaya.MFnStringData.kString, pickerNameAttrStr)
    pickerNameAttr.setStorable(True)
    pickerNameAttr.setKeyable(False)
    self.addAttribute( self.pickerName )

    # Bacon Bits
    baconBitsAttr = OpenMaya.MFnMessageAttribute()
    self.baconBits = baconBitsAttr.create("baconBits", "bb")
    baconBitsAttr.setArray(True)
    baconBitsAttr.setStorable(True)
    baconBitsAttr.setKeyable(True)
    self.addAttribute( self.baconBits )

    # LOD Meshes
    typeAttr = OpenMaya.MFnTypedAttribute()
    lodMeshesAttr = OpenMaya.MFnCompoundAttribute()
    self.lodMeshes = lodMeshesAttr.create("LOD", "LOD")
    lodMeshesAttr.setArray(True)
    lodMeshesAttr.setKeyable(True)
    dagNodePlug = typeAttr.create('meshes', 'meshes', OpenMaya.MFnData.kString)
    typeAttr.setArray(True)
    lodMeshesAttr.addChild(dagNodePlug)
    lodMeshesAttr.setHidden(False)
    self.addAttribute( self.lodMeshes)


# initialize the script plug-in
def initializePlugin(mobject):
    mplugin = OpenMayaMPx.MFnPlugin(mobject)
    try:
        mplugin.registerNode( kPluginNodeTypeName, baconRigId, nodeCreator, nodeInitializer )
    except:
        sys.stderr.write( "Failed to register node: %s" % kPluginNodeTypeName )
        raise


# uninitialize the script plug-in
def uninitializePlugin(mobject):
    mplugin = OpenMayaMPx.MFnPlugin(mobject)
    try:
        mplugin.deregisterNode( baconRigId )
    except:
        sys.stderr.write( "Failed to deregister node: %s" % kPluginNodeTypeName )
        raise
