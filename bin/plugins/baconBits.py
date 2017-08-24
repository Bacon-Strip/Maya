"""
// ==========================================================================
// www.Bacon-Strip.com
// Luis Alonso
// Creative Commons 2017
// bacon-Strip Node ID Block are: 0x0012a940 - 0x0012aa3f
// Registered for Bacon-Strip [Luis Alonso]
//
// ==========================================================================

////////////////////////////////////////////////////////////////////////
// DESCRIPTION:
// Produces the dependency graph node "baconBits".
// This plug-in is part of a rigging package known as The "Bacon-Strip"
//
// ID: 
// 0x0012a941
//
////////////////////////////////////////////////////////////////////////
"""



import math, sys
import maya.OpenMaya as OpenMaya
import maya.OpenMayaMPx as OpenMayaMPx

kPluginNodeTypeName = "baconBits"
baconBitsId = OpenMaya.MTypeId(0x0012a941)

class baconBits(OpenMayaMPx.MPxNode):

    BitName = OpenMaya.MObject()
    BitType = OpenMaya.MObject()
    BitComponents = OpenMaya.MObject()
    rigInterface = OpenMaya.MObject()
    guideInterface = OpenMaya.MObject()

    def __init__(self):
        OpenMayaMPx.MPxNode.__init__(self)

    def compute(self,plug,dataBlock):
        # INPUTS
        rootDagHandle = dataBlock.inputValue(self.rootDag)

def nodeCreator():
    return OpenMayaMPx.asMPxPtr( baconBits() )

def nodeInitializer():
    self = baconBits


    # BitName
    bitNameAttr = OpenMaya.MFnTypedAttribute ()
    bitNameAttrStr = OpenMaya.MFnStringData().create("bit")
    self.bitName = bitNameAttr.create("bitName", "bname", OpenMaya.MFnStringData.kString, bitNameAttrStr)
    bitNameAttr.setStorable(True)
    bitNameAttr.setKeyable(False)
    self.addAttribute( self.bitName )

    # bit Type
    bitTypeAttr = OpenMaya.MFnEnumAttribute()
    self.bitType = bitTypeAttr.create("bitType", "btype", 0)
    bitTypeAttr.addField("Custom", 0)
    bitTypeAttr.addField("HumanSpine", 1)
    bitTypeAttr.addField("HumanHead", 2)
    bitTypeAttr.addField("HumanArm", 3)
    bitTypeAttr.addField("HumanHand", 4)
    bitTypeAttr.addField("HumanLeg", 5)
    bitTypeAttr.addField("HumanFace", 6)
    bitTypeAttr.addField("HorseLeg", 7)
    bitTypeAttr.addField("ReverseFoot", 8)
    bitTypeAttr.addField("SlidingContactFoot", 9)
    bitTypeAttr.addField("DualHorseshoeFoot", 10)
    bitTypeAttr.addField("DualFreeContactFoot", 11)
    bitTypeAttr.setHidden(False)
    bitTypeAttr.setKeyable(False)
    bitTypeAttr.setDefault(1)
    self.addAttribute( self.bitType )

    # guide interface
    guideAttr = OpenMaya.MFnTypedAttribute ()
    self.guide = guideAttr.create("guide", "guide", OpenMaya.MFnStringData.kString)
    guideAttr.setStorable(True)
    guideAttr.setKeyable(True)
    guideAttr.setReadable(False)
    self.addAttribute( self.guide )


    # ------------------------------------------------------------
    """
    All Components of a bit inherit the following attributes:
        
        ** Please update this description upon adding an attribute **
    
        dagMessage:             a plug for connecting dagnodes 
        rigNodeName:            internal name used by the rig to determine its identity and role. 
        prefix:                 internal prefix used by the rig to determine its identity and role.
        pickerType:             picker display info.
        animatedChannelStr:     a string that determines animated channels for takes and layers.
        animatedDefaultStr      a string that determines the default values of the animatedChannelStr
        export                  a flag for the export process to filter nodes that are exportable or not.
        exportOffsetTM          a rig abstraction that allows for individual project skeleton transform needs.
        parentOverride          a rig abstraction that allows for individual project skeleton hierarchy needs.
        exportCollapseLOD0      a flag for bone skin transfer on skeleton simplification at LOD0 
        exportCollapseLOD1      a flag for bone skin transfer on skeleton simplification at LOD1 
        exportCollapseLOD2      a flag for bone skin transfer on skeleton simplification at LOD2 
        
    """
    # Bit Components
    bitComponentsAttr = OpenMaya.MFnCompoundAttribute()
    self.bitComponents = bitComponentsAttr.create("bitComponents", "bitComponents")
    bitComponentsAttr.setArray(True)
    bitComponentsAttr.setKeyable(True)

    # Component Attribute: Attrs
    typeAttr = OpenMaya.MFnTypedAttribute()
    numAttr = OpenMaya.MFnNumericAttribute()
    matrixAttr = OpenMaya.MFnMatrixAttribute()
    enumAttr = OpenMaya.MFnEnumAttribute()

    # Component Attribute: dagnode connection
    dagNodePlug = typeAttr.create('dagMessage', 'dagMessage', OpenMaya.MFnData.kString)
    bitComponentsAttr.addChild(dagNodePlug)

    # Component Attribute: name
    rigNodeNamePlug = typeAttr.create('name', 'name', OpenMaya.MFnData.kString)
    bitComponentsAttr.addChild(rigNodeNamePlug)

    # Component Attribute: prefix
    prefixPlug = typeAttr.create('prefix', 'prefix', OpenMaya.MFnData.kString)
    bitComponentsAttr.addChild(prefixPlug)

    # Component Attribute: picker type
    pickerTypePlug = enumAttr.create("pickerType", "pickerType", 0)
    enumAttr.addField("None", 0)
    enumAttr.addField("FK", 1)
    enumAttr.addField("IK", 2)
    enumAttr.addField("Root", 3)
    bitComponentsAttr.addChild(pickerTypePlug)

    # Component Attribute: animated channels
    animatedChannelStr = OpenMaya.MFnStringData().create("px py pz rx ry rz")
    animatedChannelsPlug = typeAttr.create('animatedChannels', 'animatedChannels', OpenMaya.MFnData.kString, animatedChannelStr)
    bitComponentsAttr.addChild(animatedChannelsPlug)

    # Component Attribute: channel defaults
    channelDefaultStr = OpenMaya.MFnStringData().create("0 0 0 0 0 0")
    channelDefaultsPlug = typeAttr.create('channelDefaults', 'channelDefaults', OpenMaya.MFnData.kString, channelDefaultStr)
    bitComponentsAttr.addChild(channelDefaultsPlug)

    # Component Attribute: export flag
    exportPlug = numAttr.create("export", "export", OpenMaya.MFnNumericData.kBoolean )
    bitComponentsAttr.addChild(exportPlug)

    # Component Attribute: export offset
    exportOffsetTMPlug = matrixAttr.create("exportOffsetTM", "exportOffsetTM", matrixAttr.kFloat)
    bitComponentsAttr.addChild(exportOffsetTMPlug)

    # Component Attribute: parent override
    parentOverrideNamePlug = typeAttr.create('parentOverrideName', 'parentOverrideName', OpenMaya.MFnData.kString)
    bitComponentsAttr.addChild(parentOverrideNamePlug)

    # Component Attribute: export collapse lod0
    exportCollapseLOD0Plug = typeAttr.create('exportCollapseLOD0', 'xLod0', OpenMaya.MFnData.kString)
    bitComponentsAttr.addChild(exportCollapseLOD0Plug)

    # Component Attribute: export collapse lod1
    exportCollapseLOD1Plug = typeAttr.create('exportCollapseLOD1', 'xLod1', OpenMaya.MFnData.kString)
    bitComponentsAttr.addChild(exportCollapseLOD1Plug)

    # Component Attribute: export collapse lod2
    exportCollapseLOD2Plug = typeAttr.create('exportCollapseLOD2', 'xLod2', OpenMaya.MFnData.kString)
    bitComponentsAttr.addChild(exportCollapseLOD2Plug)

    bitComponentsAttr.setHidden(False)
    self.addAttribute( self.bitComponents)
    # ------------------------------------------------------------

    # rig interface
    interfaceAttr = OpenMaya.MFnTypedAttribute ()
    self.interface = interfaceAttr.create("rig", "rig", OpenMaya.MFnStringData.kString)
    interfaceAttr.setStorable(True)
    interfaceAttr.setKeyable(True)
    interfaceAttr.setWritable(False)
    self.addAttribute( self.interface )



# initialize the script plug-in
def initializePlugin(mobject):
    mplugin = OpenMayaMPx.MFnPlugin(mobject)
    try:
        mplugin.registerNode( kPluginNodeTypeName, baconBitsId, nodeCreator, nodeInitializer )
    except:
        sys.stderr.write( "Failed to register node: %s" % kPluginNodeTypeName )
        raise


# uninitialize the script plug-in
def uninitializePlugin(mobject):
    mplugin = OpenMayaMPx.MFnPlugin(mobject)
    try:
        mplugin.deregisterNode( baconBitsId )
    except:
        sys.stderr.write( "Failed to deregister node: %s" % kPluginNodeTypeName )
        raise
