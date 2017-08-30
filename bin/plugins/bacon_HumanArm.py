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
// 0x0012a946
//
////////////////////////////////////////////////////////////////////////
"""


import math, sys
import maya.OpenMaya as OpenMaya
import maya.OpenMayaMPx as OpenMayaMPx

kPluginNodeTypeName = "bacon_HumanArm"
bacon_HumanArmId = OpenMaya.MTypeId(0x0012a946)

class bacon_HumanArm(OpenMayaMPx.MPxNode):
    # Default
    parentDef = OpenMaya.MObject()
    childrenDef = OpenMaya.MObject()
    prefix = OpenMaya.MObject()
    partName = OpenMaya.MObject()
    armature = OpenMaya.MObject()

    Clavicle = OpenMaya.MObject()
    UpperArm = OpenMaya.MObject()
    HandIKZero = OpenMaya.MObject()
    ForeTwist1 = OpenMaya.MObject()
    Forearm = OpenMaya.MObject()
    UpArmTwist = OpenMaya.MObject()
    Hand = OpenMaya.MObject()
    UpArmTwist1 = OpenMaya.MObject()
    HandReach = OpenMaya.MObject()
    ForeTwist = OpenMaya.MObject()
    ArmUpVector = OpenMaya.MObject()
    Elbow = OpenMaya.MObject()
    HandAttach = OpenMaya.MObject()
    ArmUpVectorZero = OpenMaya.MObject()
    HandIK = OpenMaya.MObject()


    def __init__(self):
        OpenMayaMPx.MPxNode.__init__(self)

    def compute(self,plug,dataBlock):
        # INPUTS
        partNameHandle = dataBlock.inputValue(self.partName)


# creator
def nodeCreator():
    return OpenMayaMPx.asMPxPtr( bacon_HumanArm() )

# initializer
def nodeInitializer():
    self = bacon_HumanArm

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

    # Clavicle
    ClavicleAttr = OpenMaya.MFnMessageAttribute()
    self.Clavicle = ClavicleAttr.create("Clavicle", "Clavicle")
    ClavicleAttr.setStorable(True)
    ClavicleAttr.setKeyable(True)
    self.addAttribute(self.Clavicle)

    # UpperArm
    UpperArmAttr = OpenMaya.MFnMessageAttribute()
    self.UpperArm = UpperArmAttr.create("UpperArm", "UpperArm")
    UpperArmAttr.setStorable(True)
    UpperArmAttr.setKeyable(True)
    self.addAttribute(self.UpperArm)

    # HandIKZero
    HandIKZeroAttr = OpenMaya.MFnMessageAttribute()
    self.HandIKZero = HandIKZeroAttr.create("HandIKZero", "HandIKZero")
    HandIKZeroAttr.setStorable(True)
    HandIKZeroAttr.setKeyable(True)
    self.addAttribute(self.HandIKZero)

    # ForeTwist1
    ForeTwist1Attr = OpenMaya.MFnMessageAttribute()
    self.ForeTwist1 = ForeTwist1Attr.create("ForeTwist1", "ForeTwist1")
    ForeTwist1Attr.setStorable(True)
    ForeTwist1Attr.setKeyable(True)
    self.addAttribute(self.ForeTwist1)

    # Forearm
    ForearmAttr = OpenMaya.MFnMessageAttribute()
    self.Forearm = ForearmAttr.create("Forearm", "Forearm")
    ForearmAttr.setStorable(True)
    ForearmAttr.setKeyable(True)
    self.addAttribute(self.Forearm)

    # UpArmTwist
    UpArmTwistAttr = OpenMaya.MFnMessageAttribute()
    self.UpArmTwist = UpArmTwistAttr.create("UpArmTwist", "UpArmTwist")
    UpArmTwistAttr.setStorable(True)
    UpArmTwistAttr.setKeyable(True)
    self.addAttribute(self.UpArmTwist)

    # Hand
    HandAttr = OpenMaya.MFnMessageAttribute()
    self.Hand = HandAttr.create("Hand", "Hand")
    HandAttr.setStorable(True)
    HandAttr.setKeyable(True)
    self.addAttribute(self.Hand)

    # UpArmTwist1
    UpArmTwist1Attr = OpenMaya.MFnMessageAttribute()
    self.UpArmTwist1 = UpArmTwist1Attr.create("UpArmTwist1", "UpArmTwist1")
    UpArmTwist1Attr.setStorable(True)
    UpArmTwist1Attr.setKeyable(True)
    self.addAttribute(self.UpArmTwist1)

    # HandReach
    HandReachAttr = OpenMaya.MFnMessageAttribute()
    self.HandReach = HandReachAttr.create("HandReach", "HandReach")
    HandReachAttr.setStorable(True)
    HandReachAttr.setKeyable(True)
    self.addAttribute(self.HandReach)

    # ForeTwist
    ForeTwistAttr = OpenMaya.MFnMessageAttribute()
    self.ForeTwist = ForeTwistAttr.create("ForeTwist", "ForeTwist")
    ForeTwistAttr.setStorable(True)
    ForeTwistAttr.setKeyable(True)
    self.addAttribute(self.ForeTwist)

    # ArmUpVector
    ArmUpVectorAttr = OpenMaya.MFnMessageAttribute()
    self.ArmUpVector = ArmUpVectorAttr.create("ArmUpVector", "ArmUpVector")
    ArmUpVectorAttr.setStorable(True)
    ArmUpVectorAttr.setKeyable(True)
    self.addAttribute(self.ArmUpVector)

    # Elbow
    ElbowAttr = OpenMaya.MFnMessageAttribute()
    self.Elbow = ElbowAttr.create("Elbow", "Elbow")
    ElbowAttr.setStorable(True)
    ElbowAttr.setKeyable(True)
    self.addAttribute(self.Elbow)

    # HandAttach
    HandAttachAttr = OpenMaya.MFnMessageAttribute()
    self.HandAttach = HandAttachAttr.create("HandAttach", "HandAttach")
    HandAttachAttr.setStorable(True)
    HandAttachAttr.setKeyable(True)
    self.addAttribute(self.HandAttach)

    # ArmUpVectorZero
    ArmUpVectorZeroAttr = OpenMaya.MFnMessageAttribute()
    self.ArmUpVectorZero = ArmUpVectorZeroAttr.create("ArmUpVectorZero", "ArmUpVectorZero")
    ArmUpVectorZeroAttr.setStorable(True)
    ArmUpVectorZeroAttr.setKeyable(True)
    self.addAttribute(self.ArmUpVectorZero)

    # HandIK
    HandIKAttr = OpenMaya.MFnMessageAttribute()
    self.HandIK = HandIKAttr.create("HandIK", "HandIK")
    HandIKAttr.setStorable(True)
    HandIKAttr.setKeyable(True)
    self.addAttribute(self.HandIK)

# initialize the script plug-in
def initializePlugin(mobject):
    mplugin = OpenMayaMPx.MFnPlugin(mobject)
    try:
        mplugin.registerNode( kPluginNodeTypeName, bacon_HumanArmId, nodeCreator, nodeInitializer )
    except:
        sys.stderr.write( "Failed to register node: %s" % kPluginNodeTypeName )
        raise

# uninitialize the script plug-in
def uninitializePlugin(mobject):
    mplugin = OpenMayaMPx.MFnPlugin(mobject)
    try:
        mplugin.deregisterNode( bacon_HumanArmId )
    except:
        sys.stderr.write( "Failed to deregister node: %s" % kPluginNodeTypeName )
        raise
