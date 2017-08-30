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
// 0x0012a944
//
////////////////////////////////////////////////////////////////////////
"""


import math, sys
import maya.OpenMaya as OpenMaya
import maya.OpenMayaMPx as OpenMayaMPx

kPluginNodeTypeName = "bacon_HumanLeg"
bacon_HumanLegId = OpenMaya.MTypeId(0x0012a944)

class bacon_HumanLeg(OpenMayaMPx.MPxNode):
    # Default
    parentDef = OpenMaya.MObject()
    childrenDef = OpenMaya.MObject()
    prefix = OpenMaya.MObject()
    partName = OpenMaya.MObject()
    armature = OpenMaya.MObject()

    LegUpVector = OpenMaya.MObject()
    ReverseAnkle = OpenMaya.MObject()
    HingeCalf = OpenMaya.MObject()
    ToeControl = OpenMaya.MObject()
    Calf = OpenMaya.MObject()
    ThighTwist = OpenMaya.MObject()
    SlidingContact = OpenMaya.MObject()
    SlidingRoot = OpenMaya.MObject()
    SlidingOrigin = OpenMaya.MObject()
    Foot = OpenMaya.MObject()
    Knee = OpenMaya.MObject()
    CalfTwist1 = OpenMaya.MObject()
    FootControlZero = OpenMaya.MObject()
    ThighTwist1 = OpenMaya.MObject()
    SlidingContactZero = OpenMaya.MObject()
    HingeThigh = OpenMaya.MObject()
    Gluteus = OpenMaya.MObject()
    Toe0 = OpenMaya.MObject()
    CalfTwist = OpenMaya.MObject()
    Thigh = OpenMaya.MObject()
    HingeKnee = OpenMaya.MObject()
    FootControl = OpenMaya.MObject()
    FootIK = OpenMaya.MObject()


    def __init__(self):
        OpenMayaMPx.MPxNode.__init__(self)

    def compute(self,plug,dataBlock):
        # INPUTS
        partNameHandle = dataBlock.inputValue(self.partName)


# creator
def nodeCreator():
    return OpenMayaMPx.asMPxPtr( bacon_HumanLeg() )

# initializer
def nodeInitializer():
    self = bacon_HumanLeg

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

    # LegUpVector
    LegUpVectorAttr = OpenMaya.MFnMessageAttribute()
    self.LegUpVector = LegUpVectorAttr.create("LegUpVector", "LegUpVector")
    LegUpVectorAttr.setStorable(True)
    LegUpVectorAttr.setKeyable(True)
    self.addAttribute(self.LegUpVector)

    # ReverseAnkle
    ReverseAnkleAttr = OpenMaya.MFnMessageAttribute()
    self.ReverseAnkle = ReverseAnkleAttr.create("ReverseAnkle", "ReverseAnkle")
    ReverseAnkleAttr.setStorable(True)
    ReverseAnkleAttr.setKeyable(True)
    self.addAttribute(self.ReverseAnkle)

    # HingeCalf
    HingeCalfAttr = OpenMaya.MFnMessageAttribute()
    self.HingeCalf = HingeCalfAttr.create("HingeCalf", "HingeCalf")
    HingeCalfAttr.setStorable(True)
    HingeCalfAttr.setKeyable(True)
    self.addAttribute(self.HingeCalf)

    # ToeControl
    ToeControlAttr = OpenMaya.MFnMessageAttribute()
    self.ToeControl = ToeControlAttr.create("ToeControl", "ToeControl")
    ToeControlAttr.setStorable(True)
    ToeControlAttr.setKeyable(True)
    self.addAttribute(self.ToeControl)

    # Calf
    CalfAttr = OpenMaya.MFnMessageAttribute()
    self.Calf = CalfAttr.create("Calf", "Calf")
    CalfAttr.setStorable(True)
    CalfAttr.setKeyable(True)
    self.addAttribute(self.Calf)

    # ThighTwist
    ThighTwistAttr = OpenMaya.MFnMessageAttribute()
    self.ThighTwist = ThighTwistAttr.create("ThighTwist", "ThighTwist")
    ThighTwistAttr.setStorable(True)
    ThighTwistAttr.setKeyable(True)
    self.addAttribute(self.ThighTwist)

    # SlidingContact
    SlidingContactAttr = OpenMaya.MFnMessageAttribute()
    self.SlidingContact = SlidingContactAttr.create("SlidingContact", "SlidingContact")
    SlidingContactAttr.setStorable(True)
    SlidingContactAttr.setKeyable(True)
    self.addAttribute(self.SlidingContact)

    # SlidingRoot
    SlidingRootAttr = OpenMaya.MFnMessageAttribute()
    self.SlidingRoot = SlidingRootAttr.create("SlidingRoot", "SlidingRoot")
    SlidingRootAttr.setStorable(True)
    SlidingRootAttr.setKeyable(True)
    self.addAttribute(self.SlidingRoot)

    # SlidingOrigin
    SlidingOriginAttr = OpenMaya.MFnMessageAttribute()
    self.SlidingOrigin = SlidingOriginAttr.create("SlidingOrigin", "SlidingOrigin")
    SlidingOriginAttr.setStorable(True)
    SlidingOriginAttr.setKeyable(True)
    self.addAttribute(self.SlidingOrigin)

    # Foot
    FootAttr = OpenMaya.MFnMessageAttribute()
    self.Foot = FootAttr.create("Foot", "Foot")
    FootAttr.setStorable(True)
    FootAttr.setKeyable(True)
    self.addAttribute(self.Foot)

    # Knee
    KneeAttr = OpenMaya.MFnMessageAttribute()
    self.Knee = KneeAttr.create("Knee", "Knee")
    KneeAttr.setStorable(True)
    KneeAttr.setKeyable(True)
    self.addAttribute(self.Knee)

    # CalfTwist1
    CalfTwist1Attr = OpenMaya.MFnMessageAttribute()
    self.CalfTwist1 = CalfTwist1Attr.create("CalfTwist1", "CalfTwist1")
    CalfTwist1Attr.setStorable(True)
    CalfTwist1Attr.setKeyable(True)
    self.addAttribute(self.CalfTwist1)

    # FootControlZero
    FootControlZeroAttr = OpenMaya.MFnMessageAttribute()
    self.FootControlZero = FootControlZeroAttr.create("FootControlZero", "FootControlZero")
    FootControlZeroAttr.setStorable(True)
    FootControlZeroAttr.setKeyable(True)
    self.addAttribute(self.FootControlZero)

    # ThighTwist1
    ThighTwist1Attr = OpenMaya.MFnMessageAttribute()
    self.ThighTwist1 = ThighTwist1Attr.create("ThighTwist1", "ThighTwist1")
    ThighTwist1Attr.setStorable(True)
    ThighTwist1Attr.setKeyable(True)
    self.addAttribute(self.ThighTwist1)

    # SlidingContactZero
    SlidingContactZeroAttr = OpenMaya.MFnMessageAttribute()
    self.SlidingContactZero = SlidingContactZeroAttr.create("SlidingContactZero", "SlidingContactZero")
    SlidingContactZeroAttr.setStorable(True)
    SlidingContactZeroAttr.setKeyable(True)
    self.addAttribute(self.SlidingContactZero)

    # HingeThigh
    HingeThighAttr = OpenMaya.MFnMessageAttribute()
    self.HingeThigh = HingeThighAttr.create("HingeThigh", "HingeThigh")
    HingeThighAttr.setStorable(True)
    HingeThighAttr.setKeyable(True)
    self.addAttribute(self.HingeThigh)

    # Gluteus
    GluteusAttr = OpenMaya.MFnMessageAttribute()
    self.Gluteus = GluteusAttr.create("Gluteus", "Gluteus")
    GluteusAttr.setStorable(True)
    GluteusAttr.setKeyable(True)
    self.addAttribute(self.Gluteus)

    # Toe0
    Toe0Attr = OpenMaya.MFnMessageAttribute()
    self.Toe0 = Toe0Attr.create("Toe0", "Toe0")
    Toe0Attr.setStorable(True)
    Toe0Attr.setKeyable(True)
    self.addAttribute(self.Toe0)

    # CalfTwist
    CalfTwistAttr = OpenMaya.MFnMessageAttribute()
    self.CalfTwist = CalfTwistAttr.create("CalfTwist", "CalfTwist")
    CalfTwistAttr.setStorable(True)
    CalfTwistAttr.setKeyable(True)
    self.addAttribute(self.CalfTwist)

    # Thigh
    ThighAttr = OpenMaya.MFnMessageAttribute()
    self.Thigh = ThighAttr.create("Thigh", "Thigh")
    ThighAttr.setStorable(True)
    ThighAttr.setKeyable(True)
    self.addAttribute(self.Thigh)

    # HingeKnee
    HingeKneeAttr = OpenMaya.MFnMessageAttribute()
    self.HingeKnee = HingeKneeAttr.create("HingeKnee", "HingeKnee")
    HingeKneeAttr.setStorable(True)
    HingeKneeAttr.setKeyable(True)
    self.addAttribute(self.HingeKnee)

    # FootControl
    FootControlAttr = OpenMaya.MFnMessageAttribute()
    self.FootControl = FootControlAttr.create("FootControl", "FootControl")
    FootControlAttr.setStorable(True)
    FootControlAttr.setKeyable(True)
    self.addAttribute(self.FootControl)

    # FootIK
    FootIKAttr = OpenMaya.MFnMessageAttribute()
    self.FootIK = FootIKAttr.create("FootIK", "FootIK")
    FootIKAttr.setStorable(True)
    FootIKAttr.setKeyable(True)
    self.addAttribute(self.FootIK)

# initialize the script plug-in
def initializePlugin(mobject):
    mplugin = OpenMayaMPx.MFnPlugin(mobject)
    try:
        mplugin.registerNode( kPluginNodeTypeName, bacon_HumanLegId, nodeCreator, nodeInitializer )
    except:
        sys.stderr.write( "Failed to register node: %s" % kPluginNodeTypeName )
        raise

# uninitialize the script plug-in
def uninitializePlugin(mobject):
    mplugin = OpenMayaMPx.MFnPlugin(mobject)
    try:
        mplugin.deregisterNode( bacon_HumanLegId )
    except:
        sys.stderr.write( "Failed to deregister node: %s" % kPluginNodeTypeName )
        raise
