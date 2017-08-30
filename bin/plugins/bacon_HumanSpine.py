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
// 0x0012a942
//
////////////////////////////////////////////////////////////////////////
"""

import math, sys
import maya.OpenMaya as OpenMaya
import maya.OpenMayaMPx as OpenMayaMPx

kPluginNodeTypeName = "bacon_HumanSpine"
bacon_HumanSpineId = OpenMaya.MTypeId(0x0012a942)

class bacon_HumanSpine(OpenMayaMPx.MPxNode):
    # Default
    parentDef = OpenMaya.MObject()
    childrenDef = OpenMaya.MObject()
    prefix = OpenMaya.MObject()
    partName = OpenMaya.MObject()
    armature = OpenMaya.MObject()

    Bip01 = OpenMaya.MObject()
    Spine = OpenMaya.MObject()
    Spine1 = OpenMaya.MObject()
    Spine3 = OpenMaya.MObject()
    Spine2 = OpenMaya.MObject()
    Pelvis = OpenMaya.MObject()
    Root = OpenMaya.MObject()


    def __init__(self):
        OpenMayaMPx.MPxNode.__init__(self)

    def compute(self,plug,dataBlock):
        # INPUTS
        partNameHandle = dataBlock.inputValue(self.partName)


# creator
def nodeCreator():
    return OpenMayaMPx.asMPxPtr( bacon_HumanSpine() )

# initializer
def nodeInitializer():
    self = bacon_HumanSpine

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

    # Bip01
    Bip01Attr = OpenMaya.MFnMessageAttribute()
    self.Bip01 = Bip01Attr.create("Bip01", "Bip01")
    Bip01Attr.setStorable(True)
    Bip01Attr.setKeyable(True)
    self.addAttribute(self.Bip01)

    # Spine
    SpineAttr = OpenMaya.MFnMessageAttribute()
    self.Spine = SpineAttr.create("Spine", "Spine")
    SpineAttr.setStorable(True)
    SpineAttr.setKeyable(True)
    self.addAttribute(self.Spine)

    # Spine1
    Spine1Attr = OpenMaya.MFnMessageAttribute()
    self.Spine1 = Spine1Attr.create("Spine1", "Spine1")
    Spine1Attr.setStorable(True)
    Spine1Attr.setKeyable(True)
    self.addAttribute(self.Spine1)

    # Spine3
    Spine3Attr = OpenMaya.MFnMessageAttribute()
    self.Spine3 = Spine3Attr.create("Spine3", "Spine3")
    Spine3Attr.setStorable(True)
    Spine3Attr.setKeyable(True)
    self.addAttribute(self.Spine3)

    # Spine2
    Spine2Attr = OpenMaya.MFnMessageAttribute()
    self.Spine2 = Spine2Attr.create("Spine2", "Spine2")
    Spine2Attr.setStorable(True)
    Spine2Attr.setKeyable(True)
    self.addAttribute(self.Spine2)

    # Pelvis
    PelvisAttr = OpenMaya.MFnMessageAttribute()
    self.Pelvis = PelvisAttr.create("Pelvis", "Pelvis")
    PelvisAttr.setStorable(True)
    PelvisAttr.setKeyable(True)
    self.addAttribute(self.Pelvis)

    # Root
    RootAttr = OpenMaya.MFnMessageAttribute()
    self.Root = RootAttr.create("Root", "Root")
    RootAttr.setStorable(True)
    RootAttr.setKeyable(True)
    self.addAttribute(self.Root)

# initialize the script plug-in
def initializePlugin(mobject):
    mplugin = OpenMayaMPx.MFnPlugin(mobject)
    try:
        mplugin.registerNode( kPluginNodeTypeName, bacon_HumanSpineId, nodeCreator, nodeInitializer )
    except:
        sys.stderr.write( "Failed to register node: %s" % kPluginNodeTypeName )
        raise

# uninitialize the script plug-in
def uninitializePlugin(mobject):
    mplugin = OpenMayaMPx.MFnPlugin(mobject)
    try:
        mplugin.deregisterNode( bacon_HumanSpineId )
    except:
        sys.stderr.write( "Failed to deregister node: %s" % kPluginNodeTypeName )
        raise
