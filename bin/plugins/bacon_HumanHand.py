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
// 0x0012a947
//
////////////////////////////////////////////////////////////////////////
"""


import math, sys
import maya.OpenMaya as OpenMaya
import maya.OpenMayaMPx as OpenMayaMPx

kPluginNodeTypeName = "bacon_HumanHand"
bacon_HumanHandId = OpenMaya.MTypeId(0x0012a947)

class bacon_HumanHand(OpenMayaMPx.MPxNode):
    # Default
    parentDef = OpenMaya.MObject()
    childrenDef = OpenMaya.MObject()
    prefix = OpenMaya.MObject()
    partName = OpenMaya.MObject()
    armature = OpenMaya.MObject()

    Finger11 = OpenMaya.MObject()
    Finger12 = OpenMaya.MObject()
    Finger13 = OpenMaya.MObject()
    Finger32 = OpenMaya.MObject()
    Finger33 = OpenMaya.MObject()
    Finger31 = OpenMaya.MObject()
    Finger4 = OpenMaya.MObject()
    Finger2 = OpenMaya.MObject()
    Finger3 = OpenMaya.MObject()
    Finger0 = OpenMaya.MObject()
    Finger1 = OpenMaya.MObject()
    Finger01 = OpenMaya.MObject()
    Finger02 = OpenMaya.MObject()
    Finger21 = OpenMaya.MObject()
    Finger23 = OpenMaya.MObject()
    Finger22 = OpenMaya.MObject()
    Finger43 = OpenMaya.MObject()
    Finger42 = OpenMaya.MObject()
    Finger41 = OpenMaya.MObject()


    def __init__(self):
        OpenMayaMPx.MPxNode.__init__(self)

    def compute(self,plug,dataBlock):
        # INPUTS
        partNameHandle = dataBlock.inputValue(self.partName)


# creator
def nodeCreator():
    return OpenMayaMPx.asMPxPtr( bacon_HumanHand() )

# initializer
def nodeInitializer():
    self = bacon_HumanHand

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

    # Finger11
    Finger11Attr = OpenMaya.MFnMessageAttribute()
    self.Finger11 = Finger11Attr.create("Finger11", "Finger11")
    Finger11Attr.setStorable(True)
    Finger11Attr.setKeyable(True)
    self.addAttribute(self.Finger11)

    # Finger12
    Finger12Attr = OpenMaya.MFnMessageAttribute()
    self.Finger12 = Finger12Attr.create("Finger12", "Finger12")
    Finger12Attr.setStorable(True)
    Finger12Attr.setKeyable(True)
    self.addAttribute(self.Finger12)

    # Finger13
    Finger13Attr = OpenMaya.MFnMessageAttribute()
    self.Finger13 = Finger13Attr.create("Finger13", "Finger13")
    Finger13Attr.setStorable(True)
    Finger13Attr.setKeyable(True)
    self.addAttribute(self.Finger13)

    # Finger32
    Finger32Attr = OpenMaya.MFnMessageAttribute()
    self.Finger32 = Finger32Attr.create("Finger32", "Finger32")
    Finger32Attr.setStorable(True)
    Finger32Attr.setKeyable(True)
    self.addAttribute(self.Finger32)

    # Finger33
    Finger33Attr = OpenMaya.MFnMessageAttribute()
    self.Finger33 = Finger33Attr.create("Finger33", "Finger33")
    Finger33Attr.setStorable(True)
    Finger33Attr.setKeyable(True)
    self.addAttribute(self.Finger33)

    # Finger31
    Finger31Attr = OpenMaya.MFnMessageAttribute()
    self.Finger31 = Finger31Attr.create("Finger31", "Finger31")
    Finger31Attr.setStorable(True)
    Finger31Attr.setKeyable(True)
    self.addAttribute(self.Finger31)

    # Finger4
    Finger4Attr = OpenMaya.MFnMessageAttribute()
    self.Finger4 = Finger4Attr.create("Finger4", "Finger4")
    Finger4Attr.setStorable(True)
    Finger4Attr.setKeyable(True)
    self.addAttribute(self.Finger4)

    # Finger2
    Finger2Attr = OpenMaya.MFnMessageAttribute()
    self.Finger2 = Finger2Attr.create("Finger2", "Finger2")
    Finger2Attr.setStorable(True)
    Finger2Attr.setKeyable(True)
    self.addAttribute(self.Finger2)

    # Finger3
    Finger3Attr = OpenMaya.MFnMessageAttribute()
    self.Finger3 = Finger3Attr.create("Finger3", "Finger3")
    Finger3Attr.setStorable(True)
    Finger3Attr.setKeyable(True)
    self.addAttribute(self.Finger3)

    # Finger0
    Finger0Attr = OpenMaya.MFnMessageAttribute()
    self.Finger0 = Finger0Attr.create("Finger0", "Finger0")
    Finger0Attr.setStorable(True)
    Finger0Attr.setKeyable(True)
    self.addAttribute(self.Finger0)

    # Finger1
    Finger1Attr = OpenMaya.MFnMessageAttribute()
    self.Finger1 = Finger1Attr.create("Finger1", "Finger1")
    Finger1Attr.setStorable(True)
    Finger1Attr.setKeyable(True)
    self.addAttribute(self.Finger1)

    # Finger01
    Finger01Attr = OpenMaya.MFnMessageAttribute()
    self.Finger01 = Finger01Attr.create("Finger01", "Finger01")
    Finger01Attr.setStorable(True)
    Finger01Attr.setKeyable(True)
    self.addAttribute(self.Finger01)

    # Finger02
    Finger02Attr = OpenMaya.MFnMessageAttribute()
    self.Finger02 = Finger02Attr.create("Finger02", "Finger02")
    Finger02Attr.setStorable(True)
    Finger02Attr.setKeyable(True)
    self.addAttribute(self.Finger02)

    # Finger21
    Finger21Attr = OpenMaya.MFnMessageAttribute()
    self.Finger21 = Finger21Attr.create("Finger21", "Finger21")
    Finger21Attr.setStorable(True)
    Finger21Attr.setKeyable(True)
    self.addAttribute(self.Finger21)

    # Finger23
    Finger23Attr = OpenMaya.MFnMessageAttribute()
    self.Finger23 = Finger23Attr.create("Finger23", "Finger23")
    Finger23Attr.setStorable(True)
    Finger23Attr.setKeyable(True)
    self.addAttribute(self.Finger23)

    # Finger22
    Finger22Attr = OpenMaya.MFnMessageAttribute()
    self.Finger22 = Finger22Attr.create("Finger22", "Finger22")
    Finger22Attr.setStorable(True)
    Finger22Attr.setKeyable(True)
    self.addAttribute(self.Finger22)

    # Finger43
    Finger43Attr = OpenMaya.MFnMessageAttribute()
    self.Finger43 = Finger43Attr.create("Finger43", "Finger43")
    Finger43Attr.setStorable(True)
    Finger43Attr.setKeyable(True)
    self.addAttribute(self.Finger43)

    # Finger42
    Finger42Attr = OpenMaya.MFnMessageAttribute()
    self.Finger42 = Finger42Attr.create("Finger42", "Finger42")
    Finger42Attr.setStorable(True)
    Finger42Attr.setKeyable(True)
    self.addAttribute(self.Finger42)

    # Finger41
    Finger41Attr = OpenMaya.MFnMessageAttribute()
    self.Finger41 = Finger41Attr.create("Finger41", "Finger41")
    Finger41Attr.setStorable(True)
    Finger41Attr.setKeyable(True)
    self.addAttribute(self.Finger41)

# initialize the script plug-in
def initializePlugin(mobject):
    mplugin = OpenMayaMPx.MFnPlugin(mobject)
    try:
        mplugin.registerNode( kPluginNodeTypeName, bacon_HumanHandId, nodeCreator, nodeInitializer )
    except:
        sys.stderr.write( "Failed to register node: %s" % kPluginNodeTypeName )
        raise

# uninitialize the script plug-in
def uninitializePlugin(mobject):
    mplugin = OpenMayaMPx.MFnPlugin(mobject)
    try:
        mplugin.deregisterNode( bacon_HumanHandId )
    except:
        sys.stderr.write( "Failed to deregister node: %s" % kPluginNodeTypeName )
        raise
