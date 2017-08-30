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
// 0x0012a943
//
////////////////////////////////////////////////////////////////////////
"""


import math, sys
import maya.OpenMaya as OpenMaya
import maya.OpenMayaMPx as OpenMayaMPx

kPluginNodeTypeName = "bacon_HumanHead"
bacon_HumanHeadId = OpenMaya.MTypeId(0x0012a943)

class bacon_HumanHead(OpenMayaMPx.MPxNode):
    # Default
    parentDef = OpenMaya.MObject()
    childrenDef = OpenMaya.MObject()
    prefix = OpenMaya.MObject()
    partName = OpenMaya.MObject()
    armature = OpenMaya.MObject()

    Head = OpenMaya.MObject()
    Neck = OpenMaya.MObject()
    HeadLookAt = OpenMaya.MObject()
    HeadLookAtZero = OpenMaya.MObject()
    HeadIKZero = OpenMaya.MObject()
    Neck2 = OpenMaya.MObject()
    HeadIK = OpenMaya.MObject()


    def __init__(self):
        OpenMayaMPx.MPxNode.__init__(self)

    def compute(self,plug,dataBlock):
        # INPUTS
        partNameHandle = dataBlock.inputValue(self.partName)


# creator
def nodeCreator():
    return OpenMayaMPx.asMPxPtr( bacon_HumanHead() )

# initializer
def nodeInitializer():
    self = bacon_HumanHead

    # DEFAULTS:
    # parentDef
    parentDefAttr = OpenMaya.MFnMessageAttribute()
    self.parentDef = parentDefAttr.create("parentDef", "p")
    parentDefAttr.setStorable(True)
    parentDefAttr.setKeyable(True)
    self.addAttribute( self.parentDef )

    # partName
    partNameAttr = OpenMaya.MFnTypedAttribute()
    partNameAttrStr = OpenMaya.MFnStringData().create("HeadDefinition")
    self.partName = partNameAttr.create("partName", "n", OpenMaya.MFnStringData.kString, partNameAttrStr)
    partNameAttr.setStorable(True)
    partNameAttr.setKeyable(False)
    self.addAttribute( self.partName )

    # prefix
    prefixAttr = OpenMaya.MFnTypedAttribute ()
    self.prefix = prefixAttr.create("prefix", "pre", OpenMaya.MFnStringData.kString)
    prefixAttr.setStorable(True)
    prefixAttr.setKeyable(False)
    self.addAttribute( self.prefix )

    # armature
    armatureAttr = OpenMaya.MFnMessageAttribute()
    self.armature = armatureAttr.create("armature", "arma")
    armatureAttr.setStorable(True)
    armatureAttr.setKeyable(True)
    self.addAttribute( self.armature )

    # childrenDef
    childrenDefAttr = OpenMaya.MFnMessageAttribute()
    self.childrenDef = childrenDefAttr.create("childrenDef", "c")
    childrenDefAttr.setArray(True)
    childrenDefAttr.setStorable(True)
    childrenDefAttr.setKeyable(True)
    self.addAttribute( self.childrenDef )


    # PARTS:
    # ############################################################################################

    # Head
    HeadAttr = OpenMaya.MFnMessageAttribute()
    self.Head = HeadAttr.create("Head", "Head")
    HeadAttr.setStorable(True)
    HeadAttr.setKeyable(True)
    self.addAttribute(self.Head)

    # Neck
    NeckAttr = OpenMaya.MFnMessageAttribute()
    self.Neck = NeckAttr.create("Neck", "Neck")
    NeckAttr.setStorable(True)
    NeckAttr.setKeyable(True)
    self.addAttribute(self.Neck)

    # HeadLookAt
    HeadLookAtAttr = OpenMaya.MFnMessageAttribute()
    self.HeadLookAt = HeadLookAtAttr.create("HeadLookAt", "HeadLookAt")
    HeadLookAtAttr.setStorable(True)
    HeadLookAtAttr.setKeyable(True)
    self.addAttribute(self.HeadLookAt)

    # HeadLookAtZero
    HeadLookAtZeroAttr = OpenMaya.MFnMessageAttribute()
    self.HeadLookAtZero = HeadLookAtZeroAttr.create("HeadLookAtZero", "HeadLookAtZero")
    HeadLookAtZeroAttr.setStorable(True)
    HeadLookAtZeroAttr.setKeyable(True)
    self.addAttribute(self.HeadLookAtZero)

    # HeadIKZero
    HeadIKZeroAttr = OpenMaya.MFnMessageAttribute()
    self.HeadIKZero = HeadIKZeroAttr.create("HeadIKZero", "HeadIKZero")
    HeadIKZeroAttr.setStorable(True)
    HeadIKZeroAttr.setKeyable(True)
    self.addAttribute(self.HeadIKZero)

    # Neck2
    Neck2Attr = OpenMaya.MFnMessageAttribute()
    self.Neck2 = Neck2Attr.create("Neck2", "Neck2")
    Neck2Attr.setStorable(True)
    Neck2Attr.setKeyable(True)
    self.addAttribute(self.Neck2)

    # HeadIK
    HeadIKAttr = OpenMaya.MFnMessageAttribute()
    self.HeadIK = HeadIKAttr.create("HeadIK", "HeadIK")
    HeadIKAttr.setStorable(True)
    HeadIKAttr.setKeyable(True)
    self.addAttribute(self.HeadIK)

# initialize the script plug-in
def initializePlugin(mobject):
    mplugin = OpenMayaMPx.MFnPlugin(mobject)
    try:
        mplugin.registerNode( kPluginNodeTypeName, bacon_HumanHeadId, nodeCreator, nodeInitializer )
    except:
        sys.stderr.write( "Failed to register node: %s" % kPluginNodeTypeName )
        raise

# uninitialize the script plug-in
def uninitializePlugin(mobject):
    mplugin = OpenMayaMPx.MFnPlugin(mobject)
    try:
        mplugin.deregisterNode( bacon_HumanHeadId )
    except:
        sys.stderr.write( "Failed to deregister node: %s" % kPluginNodeTypeName )
        raise
