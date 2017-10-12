//-
// ==========================================================================
// BACON-STRIP.com RIG
//
// MIT License
// Copyright (c) 2017 Bacon-Strip
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// ==========================================================================
//+

////////////////////////////////////////////////////////////////////////
// DESCRIPTION:
// Produces the dependency graph node "baconFatBone".
// This plug-in is part of a rigging package known as The "Bacon-Strip Rig"
// Bacon-Strip ID Block is : 0x0012a940 - 0x0012aa3f
// Registered for Bacon-Strip[Luis Alonso]
//
// ID: 
// 0x0012a940
//
////////////////////////////////////////////////////////////////////////


#include <maya/MPxLocatorNode.h>
#include <maya/MString.h>
#include <maya/MTypeId.h>
#include <maya/MPlug.h>
#include <maya/MVector.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MColor.h>
#include <maya/M3dView.h>
#include <maya/MFnPlugin.h>
#include <maya/MDistance.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnAttribute.h>
#include <maya/MFnStringData.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MEulerRotation.h>
#include <maya/MPxCommand.h>

// Viewport 2.0 includes
#include <maya/MDrawRegistry.h>
#include <maya/MPxDrawOverride.h>
#include <maya/MUserData.h>
#include <maya/MDrawContext.h>
#include <maya/MHWGeometryUtilities.h>
#include <maya/MPointArray.h>
#include <maya/MGlobal.h>
#include <maya/MEventMessage.h>
#include <maya/MFnDependencyNode.h>

#include <assert.h>
#include "baconFatShapes.h" 




static bool sUseLegacyDraw = (getenv("MAYA_ENABLE_VP2_PLUGIN_LOCATOR_LEGACY_DRAW") != NULL);

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
// Node implementation with standard viewport draw
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

class baconFatBone : public MPxLocatorNode
{
public:
	baconFatBone();
	virtual ~baconFatBone();

	virtual MStatus   		compute(const MPlug& plug, MDataBlock& data);

	virtual void            draw(M3dView & view, const MDagPath & path,
		M3dView::DisplayStyle style,
		M3dView::DisplayStatus status);

	virtual MPxNode::SchedulingType schedulingType(const MPxNode* update);

	virtual bool            isBounded() const;
	virtual MBoundingBox    boundingBox() const;
	virtual bool			isTransparent() const;
	virtual bool			drawLast() const;

	static  void *          creator();
	static  MStatus         initialize();

	static  MObject			aEnableTransparencySort;
	static  MObject			aEnableDrawLast;
	static  MObject			aTheColor;
	static  MObject			aTransparency;

	static  MObject         size;         
	static  MObject         xRay;         
										  
	static  MString         labelName;    
	static  MObject         wireVisibility;
	static  MObject         solidVisibility;

public:
	static	MTypeId		id;
	static	MString		drawDbClassification;
	static	MString		drawRegistrantId;
	static  MObject		shapeType;
	static  MObject		pickerType;
	static  MObject		prestoMode;

	static  MObject		localRotationX;
	static  MObject		localRotationY;
	static  MObject		localRotationZ;



};

MObject baconFatBone::size;
MObject baconFatBone::xRay;
MObject baconFatBone::wireVisibility;
MObject baconFatBone::solidVisibility;
MTypeId baconFatBone::id(0x0012a940);
MObject baconFatBone::aEnableTransparencySort;
MObject baconFatBone::aEnableDrawLast;
MObject baconFatBone::aTheColor;
MObject baconFatBone::aTransparency;
MString	baconFatBone::drawDbClassification("drawdb/geometry/baconFatBone");
MString	baconFatBone::drawRegistrantId("baconFatBoneNodePlugin");
MObject	baconFatBone::shapeType;
MObject	baconFatBone::pickerType;
MObject baconFatBone::prestoMode;

MObject	baconFatBone::localRotationX;
MObject	baconFatBone::localRotationY;
MObject	baconFatBone::localRotationZ;


baconFatBone::baconFatBone() {}
baconFatBone::~baconFatBone() {}

MStatus baconFatBone::compute(const MPlug& /*plug*/, MDataBlock& /*data*/)
{
	return MS::kUnknownParameter;
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
// Legacy Viewport 
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

void baconFatBone::draw(M3dView & view, const MDagPath & /*path*/,
	M3dView::DisplayStyle style,
	M3dView::DisplayStatus status)
{
	// Get shapeType
	//
	MObject thisNode = thisMObject();
	MPlug shapeTypePlug(thisNode, shapeType);
	int shapeTypeVal;
	shapeTypePlug.getValue(shapeTypeVal);

	// Get the size
	//
	MPlug plug(thisNode, size);
	MDistance sizeVal;
	plug.getValue(sizeVal);
	float multiplier = (float)sizeVal.asCentimeters();

	// Get the local position
	//
	MPlug pluglocalPositionX(thisNode, localPositionX);
	MDistance localPositionValX;
	pluglocalPositionX.getValue(localPositionValX);

	MPlug pluglocalPositionY(thisNode, localPositionY);
	MDistance localPositionValY;
	pluglocalPositionY.getValue(localPositionValY);

	MPlug pluglocalPositionZ(thisNode, localPositionZ);
	MDistance localPositionValZ;
	pluglocalPositionZ.getValue(localPositionValZ);
	MVector localPosition
	(
		localPositionValX.asCentimeters(),
		localPositionValY.asCentimeters(),
		localPositionValZ.asCentimeters()
	);

	// Get the local scale
	//
	MPlug pluglocalScaleX(thisNode, localScaleX);
	MDistance localScaleValX;
	pluglocalScaleX.getValue(localScaleValX);

	MPlug pluglocalScaleY(thisNode, localScaleY);
	MDistance localScaleValY;
	pluglocalScaleY.getValue(localScaleValY);

	MPlug pluglocalScaleZ(thisNode, localScaleZ);
	MDistance localScaleValZ;
	pluglocalScaleZ.getValue(localScaleValZ);
	MVector localScale
	(
		localScaleValX.asCentimeters(),
		localScaleValY.asCentimeters(),
		localScaleValZ.asCentimeters()
	);


	// Adjust Vertex Values
	//
	unsigned int vertCount = sizeof(box_VertexList) / sizeof(box_VertexList[0]);
	double AdjustedVerts[256][3];
	unsigned int i;
	for (i = 0; i < vertCount; ++i)
	{
		AdjustedVerts[i][0] = (box_VertexList[i][0] * (localScale.x)) + (localPosition.x);
		AdjustedVerts[i][1] = (box_VertexList[i][1] * (localScale.y)) + (localPosition.y);
		AdjustedVerts[i][2] = (box_VertexList[i][2] * (localScale.z)) + (localPosition.z);
	}


	// Start Legacy Drawing
	//
	view.beginGL();


	if ((style == M3dView::kFlatShaded) ||
		(style == M3dView::kGouraudShaded))
	{
		// Push the color settings
		//
		glPushAttrib(GL_COLOR_BUFFER_BIT | GL_CURRENT_BIT | GL_ENABLE_BIT |
			GL_PIXEL_MODE_BIT);

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		glColor4f(0.5f, 0.5f, 0.0f, 0.2f);

		if (status == M3dView::kActive) {
			view.setDrawColor(2, M3dView::kActiveColors);
		}
		else {
			view.setDrawColor(2, M3dView::kDormantColors);
		}

		glPopAttrib();
	}

	// Draw the outline
	//
	unsigned int edgeCount = sizeof(box_EdgeList) / sizeof(box_EdgeList[0]);
	float miniOffset(1.001f);
	for (i = 0; i < edgeCount; i++)
	{
		glBegin(GL_LINES);
		glVertex3d(
			AdjustedVerts[box_EdgeList[i][0]][0] * miniOffset,
			AdjustedVerts[box_EdgeList[i][0]][1] * miniOffset,
			AdjustedVerts[box_EdgeList[i][0]][2] * miniOffset);
		glVertex3d(
			AdjustedVerts[box_EdgeList[i][1]][0] * miniOffset,
			AdjustedVerts[box_EdgeList[i][1]][1] * miniOffset,
			AdjustedVerts[box_EdgeList[i][1]][2] * miniOffset);
		glEnd();
	}

	view.endGL();

	// Draw the name of the baconFatBone
	//view.setDrawColor( MColor( 0.1f, 0.8f, 0.8f, 0.2f ) );
	//view.drawText( MString("draw name"), MPoint( 0.0, 0.0, 0.0 ), M3dView::kCenter );
}

bool baconFatBone::isTransparent() const
{
	MObject thisNode = thisMObject();
	MPlug plug(thisNode, aEnableTransparencySort);
	bool value;
	plug.getValue(value);
	return value;
}

bool baconFatBone::drawLast() const
{
	MObject thisNode = thisMObject();
	MPlug plug(thisNode, aEnableDrawLast);
	bool value;
	plug.getValue(value);
	return value;
}

bool baconFatBone::isBounded() const
{
	return true;
}

MBoundingBox baconFatBone::boundingBox() const
{
	// Get the size
	//
	MObject thisNode = thisMObject();

	// Get the local scale
	//
	MPlug pluglocalScaleX(thisNode, localScaleX);
	MDistance localScaleValX;
	pluglocalScaleX.getValue(localScaleValX);

	MPlug pluglocalScaleY(thisNode, localScaleY);
	MDistance localScaleValY;
	pluglocalScaleY.getValue(localScaleValY);

	MPlug pluglocalScaleZ(thisNode, localScaleZ);
	MDistance localScaleValZ;
	pluglocalScaleZ.getValue(localScaleValZ);
	MVector localScale
	(
		localScaleValX.asCentimeters(),
		localScaleValY.asCentimeters(),
		localScaleValZ.asCentimeters()
	);


	// Get the local Position
	//
	MPlug pluglocalPositionX(thisNode, localPositionX);
	MDistance localPositionValX;
	pluglocalPositionX.getValue(localPositionValX);

	MPlug pluglocalPositionY(thisNode, localPositionY);
	MDistance localPositionValY;
	pluglocalPositionY.getValue(localPositionValY);

	MPlug pluglocalPositionZ(thisNode, localPositionZ);
	MDistance localPositionValZ;
	pluglocalPositionZ.getValue(localPositionValZ);
	MVector localPosition
	(
		localPositionValX.asCentimeters(),
		localPositionValY.asCentimeters(),
		localPositionValZ.asCentimeters()
	);

	// Get the local Rotation
	//
	MPlug pluglocalRotationX(thisNode, localRotationX);
	double localRotationValX;
	pluglocalRotationX.getValue(localRotationValX);

	MPlug pluglocalRotationY(thisNode, localRotationY);
	double localRotationValY;
	pluglocalRotationY.getValue(localRotationValY);

	MPlug pluglocalRotationZ(thisNode, localRotationZ);
	double localRotationValZ;
	pluglocalRotationZ.getValue(localRotationValZ);

	MEulerRotation localRotation(localRotationValX, localRotationValY, localRotationValZ);
	MMatrix rotMatrix(localRotation.asMatrix());

	MPoint corner1(localScale.x / -1.0f, localScale.y / -1.0f, localScale.z / -1.0f);
	MPoint corner2(localScale.x / 1.0f, localScale.y / 1.0f, localScale.z / 1.0f);

	if (localRotationValX != 0.0 || localRotationValY != 0.0 || localRotationValZ != 0.0)
	{
		corner1 = corner1 * rotMatrix;
		corner2 = corner2 * rotMatrix;
	}
	return MBoundingBox(corner1 + localPosition, corner2 + localPosition);
}

// Is this evaluator capable of evaluating clusters in parallel?
MPxNode::SchedulingType baconFatBone::schedulingType(const MPxNode* update)
{
	return update->schedulingType();
}


void* baconFatBone::creator()
{
	return new baconFatBone();
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
// Viewport 2.0 override implementation
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

class baconFatBoneData : public MUserData
{
public:
	baconFatBoneData() : MUserData(false) {} // don't delete after draw
	virtual ~baconFatBoneData() {}

	MColor fColor;
	float fWireVisibility;
	float fSolidVisibility;
	bool fXRay;
	bool fpresto;
	int fShapeType;
	MPointArray fShapeLineList[256];
	MPointArray fShapeTriangleList;
};

class baconFatBoneDrawOverride : public MHWRender::MPxDrawOverride
{
public:
	static MHWRender::MPxDrawOverride* Creator(const MObject& obj)
	{
		return new baconFatBoneDrawOverride(obj);
	}

	virtual ~baconFatBoneDrawOverride();

	virtual MHWRender::DrawAPI supportedDrawAPIs() const;

	virtual bool isBounded(
		const MDagPath& objPath,
		const MDagPath& cameraPath) const;

	virtual MBoundingBox boundingBox(
		const MDagPath& objPath,
		const MDagPath& cameraPath) const;

	virtual MUserData* prepareForDraw(
		const MDagPath& objPath,
		const MDagPath& cameraPath,
		const MHWRender::MFrameContext& frameContext,
		MUserData* oldData);

	virtual bool hasUIDrawables() const { return true; }

	virtual void addUIDrawables(
		const MDagPath& objPath,
		MHWRender::MUIDrawManager& drawManager,
		const MHWRender::MFrameContext& frameContext,
		const MUserData* data);

	virtual bool traceCallSequence() const
	{
		// Return true if internal tracing is desired.
		return false;
	}
	virtual void handleTraceMessage(const MString &message) const
	{
		MGlobal::displayInfo("baconFatBoneDrawOverride: " + message);

		// Some simple custom message formatting.
		fprintf(stderr, "baconFatBoneDrawOverride: ");
		fprintf(stderr, message.asChar());
		fprintf(stderr, "\n");
	}


private:
	baconFatBoneDrawOverride(const MObject& obj);
	float getMultiplier(const MDagPath& objPath) const;
	float getWireVisibility(const MDagPath& objPath) const;
	float getSolidVisibility(const MDagPath& objPath) const;
	bool  getXRay(const MDagPath& objPath) const;
	bool  getPresto(const MDagPath& objPath) const;
	int	  getShapeType(const MDagPath& objPath) const;
	MVector getLocalPosition(const MDagPath& objPath) const;
	MVector getLocalScale(const MDagPath& objPath) const;
	MEulerRotation getLocalRotation(const MDagPath& objPath) const;

	static void OnModelEditorChanged(void *clientData);

	baconFatBone*  fbaconFatBone;
	MCallbackId fModelEditorChangedCbId;
};

// By setting isAlwaysDirty to false in MPxDrawOverride constructor, the
// draw override will be updated (via prepareForDraw()) only when the node
// is marked dirty via DG evaluation or dirty propagation. Additional
// callback is also added to explicitly mark the node as being dirty (via
// MRenderer::setGeometryDrawDirty()) for certain circumstances. Note that
// the draw callback in MPxDrawOverride constructor is set to NULL in order
// to achieve better performance.
baconFatBoneDrawOverride::baconFatBoneDrawOverride(const MObject& obj)
	: MHWRender::MPxDrawOverride(obj, NULL, false)
{
	fModelEditorChangedCbId = MEventMessage::addEventCallback(
		"modelEditorChanged", OnModelEditorChanged, this);

	MStatus status;
	MFnDependencyNode node(obj, &status);
	fbaconFatBone = status ? dynamic_cast<baconFatBone*>(node.userNode()) : NULL;
}

baconFatBoneDrawOverride::~baconFatBoneDrawOverride()
{
	fbaconFatBone = NULL;

	if (fModelEditorChangedCbId != 0)
	{
		MMessage::removeCallback(fModelEditorChangedCbId);
		fModelEditorChangedCbId = 0;
	}
}

void baconFatBoneDrawOverride::OnModelEditorChanged(void *clientData)
{
	// Mark the node as being dirty so that it can update on display appearance
	// switch among wireframe and shaded.
	baconFatBoneDrawOverride *ovr = static_cast<baconFatBoneDrawOverride*>(clientData);
	if (ovr && ovr->fbaconFatBone)
	{
		MHWRender::MRenderer::setGeometryDrawDirty(ovr->fbaconFatBone->thisMObject());
	}
}

MHWRender::DrawAPI baconFatBoneDrawOverride::supportedDrawAPIs() const
{
	// this plugin supports both GL and DX
	return (MHWRender::kOpenGL | MHWRender::kDirectX11 | MHWRender::kOpenGLCoreProfile);
}

MVector baconFatBoneDrawOverride::getLocalPosition(const MDagPath& objPath) const
{
	// Retrieve value of the local position attribute from the node
	MStatus status;
	MObject baconFatBoneNode = objPath.node(&status);
	MVector returnValue(0.0, 0.0, 0.0);
	if (status)
	{
		MPlug plugX(baconFatBoneNode, baconFatBone::localPositionX);
		if (!plugX.isNull())
		{
			MDistance localPositionXVal;
			if (plugX.getValue(localPositionXVal))
			{
				returnValue.x = localPositionXVal.asCentimeters();
			}
		}
		MPlug plugY(baconFatBoneNode, baconFatBone::localPositionY);
		if (!plugY.isNull())
		{
			MDistance localPositionYVal;
			if (plugY.getValue(localPositionYVal))
			{
				returnValue.y = localPositionYVal.asCentimeters();
			}
		}
		MPlug plugZ(baconFatBoneNode, baconFatBone::localPositionZ);
		if (!plugZ.isNull())
		{
			MDistance localPositionZVal;
			if (plugZ.getValue(localPositionZVal))
			{
				returnValue.z = localPositionZVal.asCentimeters();
			}
		}
	}

	return returnValue;
}

MVector baconFatBoneDrawOverride::getLocalScale(const MDagPath& objPath) const
{
	// Retrieve value of the local position attribute from the node
	MStatus status;
	MObject baconFatBoneNode = objPath.node(&status);
	MVector returnValue(0.0, 0.0, 0.0);
	if (status)
	{
		MPlug plugX(baconFatBoneNode, baconFatBone::localScaleX);
		if (!plugX.isNull())
		{
			MDistance localScaleXVal;
			if (plugX.getValue(localScaleXVal))
			{
				returnValue.x = localScaleXVal.asCentimeters();
			}
		}
		MPlug plugY(baconFatBoneNode, baconFatBone::localScaleY);
		if (!plugY.isNull())
		{
			MDistance localScaleYVal;
			if (plugY.getValue(localScaleYVal))
			{
				returnValue.y = localScaleYVal.asCentimeters();
			}
		}
		MPlug plugZ(baconFatBoneNode, baconFatBone::localScaleZ);
		if (!plugZ.isNull())
		{
			MDistance localScaleZVal;
			if (plugZ.getValue(localScaleZVal))
			{
				returnValue.z = localScaleZVal.asCentimeters();
			}
		}
	}

	return returnValue;
}


MEulerRotation baconFatBoneDrawOverride::getLocalRotation(const MDagPath& objPath) const
{
	// Retrieve value of the local position attribute from the node
	MStatus status;
	MObject baconFatBoneNode = objPath.node(&status);
	MEulerRotation returnValue(0, 0, 0);
	if (status)
	{
		MPlug plugX(baconFatBoneNode, baconFatBone::localRotationX);
		if (!plugX.isNull())
		{
			double localRotationXVal;
			if (plugX.getValue(localRotationXVal))
			{
				returnValue.x = localRotationXVal;
			}
		}
		MPlug plugY(baconFatBoneNode, baconFatBone::localRotationY);
		if (!plugY.isNull())
		{
			double localRotationYVal;
			if (plugY.getValue(localRotationYVal))
			{
				returnValue.y = localRotationYVal;
			}
		}
		MPlug plugZ(baconFatBoneNode, baconFatBone::localRotationZ);
		if (!plugZ.isNull())
		{
			double localRotationZVal;
			if (plugZ.getValue(localRotationZVal))
			{
				returnValue.z = localRotationZVal;
			}
		}
	}
	return returnValue;
}



float baconFatBoneDrawOverride::getMultiplier(const MDagPath& objPath) const
{
	// Retrieve value of the size attribute from the node
	MStatus status;
	MObject baconFatBoneNode = objPath.node(&status);
	if (status)
	{
		MPlug plug(baconFatBoneNode, baconFatBone::size);
		if (!plug.isNull())
		{
			MDistance sizeVal;
			if (plug.getValue(sizeVal))
			{
				return (float)sizeVal.asCentimeters();
			}
		}
	}

	return 1.0f;
}


float baconFatBoneDrawOverride::getWireVisibility(const MDagPath& objPath) const
{
	// Retrieve value of the WireColor attribute from the node
	MStatus status;
	MObject baconFatBoneNode = objPath.node(&status);
	if (status)
	{
		MPlug plug(baconFatBoneNode, baconFatBone::wireVisibility);
		if (!plug.isNull())
		{
			MDistance wireVisibilityVal;
			if (plug.getValue(wireVisibilityVal))
			{
				return (float)wireVisibilityVal.asCentimeters();
			}
		}
	}
	return 1.0f;
}

float baconFatBoneDrawOverride::getSolidVisibility(const MDagPath& objPath) const
{
	// Retrieve value of the WireColor attribute from the node
	MStatus status;
	MObject baconFatBoneNode = objPath.node(&status);
	if (status)
	{
		MPlug plug(baconFatBoneNode, baconFatBone::solidVisibility);
		if (!plug.isNull())
		{
			MDistance solidVisibilityVal;
			if (plug.getValue(solidVisibilityVal))
			{
				return (float)solidVisibilityVal.asCentimeters();
			}
		}
	}
	return 1.0f;
}

bool baconFatBoneDrawOverride::getXRay(const MDagPath& objPath) const
{
	// Retrieve value of the xRay attribute from the node
	MStatus status;
	MObject baconFatBoneNode = objPath.node(&status);
	if (status)
	{
		MPlug plug(baconFatBoneNode, baconFatBone::xRay);
		if (!plug.isNull())
		{
			bool xrayVal;
			if (plug.getValue(xrayVal))
			{
				return plug.asBool();
			}
		}
	}
	return false;
}

bool baconFatBoneDrawOverride::getPresto(const MDagPath& objPath) const
{
	// Retrieve value of the Presto Selection attribute from the node
	MStatus status;
	MObject baconFatBoneNode = objPath.node(&status);
	if (status)
	{
		MPlug plug(baconFatBoneNode, baconFatBone::prestoMode);
		if (!plug.isNull())
		{
			bool prestoVal;
			if (plug.getValue(prestoVal))
			{
				return plug.asBool();
			}
		}
	}
	return false;
}


int baconFatBoneDrawOverride::getShapeType(const MDagPath& objPath) const
{
	// Retrieve value of the shapeType attribute from the node
	MStatus status;
	MObject baconFatBoneNode = objPath.node(&status);
	if (status)
	{
		MPlug plug(baconFatBoneNode, baconFatBone::shapeType);
		if (!plug.isNull())
		{
			return plug.asInt();
		}
	}
	return 0;
}


bool baconFatBoneDrawOverride::isBounded(const MDagPath& /*objPath*/,
	const MDagPath& /*cameraPath*/) const
{
	return true;
}

MBoundingBox baconFatBoneDrawOverride::boundingBox(
	const MDagPath& objPath,
	const MDagPath& cameraPath) const
{
	MVector scaleAdjust = getLocalScale(objPath);
	MVector offsetAdjust = getLocalPosition(objPath);
	MEulerRotation rotAdjust = getLocalRotation(objPath);
	//int ShapeID = getShapeType(objPath);
	//MBoundingBox returnbox;

	MPoint corner1(scaleAdjust.x / -1.0f, scaleAdjust.y / -1.0f, scaleAdjust.z / -1.0f);
	MPoint corner2(scaleAdjust.x / 1.0f, scaleAdjust.y / 1.0f, scaleAdjust.z / 1.0f);

	if (rotAdjust.x != 0.0 || rotAdjust.y != 0.0 || rotAdjust.z != 0.0)
	{
		MMatrix rotMatrix(rotAdjust.asMatrix());
		corner1 = corner1 * rotMatrix;
		corner2 = corner2 * rotMatrix;
	}

	//MPoint corner1(0.0f, scaleAdjust.y / -2.0f, scaleAdjust.z / -2.0f);
	//MPoint corner2(scaleAdjust.x, scaleAdjust.y / 2.0f, scaleAdjust.z / 2.0f);
	MBoundingBox returnbox(corner1 + offsetAdjust, corner2 + offsetAdjust);

	//float multiplier = getMultiplier(objPath);


	return returnbox;
}


// Called by Maya each time the object needs to be drawn.
MUserData* baconFatBoneDrawOverride::prepareForDraw(
	const MDagPath& objPath,
	const MDagPath& cameraPath,
	const MHWRender::MFrameContext& frameContext,
	MUserData* oldData)
{
	// Any data needed from the Maya dependency graph must be retrieved and cached in this stage.
	// There is one cache data for each drawable instance, if it is not desirable to allow Maya to handle data
	// caching, simply return null in this method and ignore user data parameter in draw callback method.
	// e.g. in this sample, we compute and cache the data for usage later when we create the 
	// MUIDrawManager to draw baconFatBone in method addUIDrawables().
	baconFatBoneData* data = dynamic_cast<baconFatBoneData*>(oldData);
	if (!data)
	{
		data = new baconFatBoneData();
	}

	MStatus status;
	MObject baconFatBoneNode = objPath.node(&status);
	float fMultiplier = getMultiplier(objPath);

	// retrieve xRay flag
	{
		MPlug plug(baconFatBoneNode, baconFatBone::xRay);
		data->fXRay = plug.asBool();
	}

	// retrieve Presto flag
	{
		MPlug plug(baconFatBoneNode, baconFatBone::prestoMode);
		data->fpresto = plug.asBool();
	}

	// retrieve shapeType flag
	{
		MPlug plug(baconFatBoneNode, baconFatBone::shapeType);
		data->fShapeType = plug.asInt();
	}

	// retrieve wireVisibility flag
	{
		MPlug plug(baconFatBoneNode, baconFatBone::wireVisibility);
		data->fWireVisibility = plug.asFloat();
	}

	// retrieve solidVisibility flag
	{
		MPlug plug(baconFatBoneNode, baconFatBone::solidVisibility);
		data->fSolidVisibility = plug.asFloat();
	}


	MVector positionAdjust = getLocalPosition(objPath);
	MVector scaleAdjust = getLocalScale(objPath);
	MEulerRotation rotAdjust = getLocalRotation(objPath);
	bool isRotated = (rotAdjust.x != 0 || rotAdjust.y != 0 || rotAdjust.z != 0);
	MMatrix rotMatrix = rotAdjust.asMatrix();


	// Set Buffer with Choice of Shape
	//

	MVector	outputVerts[256];
	int3	outputFaces[256];
	int2	outputEdges[256];
	unsigned int vertCount = 0;
	unsigned int faceCount = 0;
	unsigned int edgeCount = 0;
	unsigned int i;

	if (data->fShapeType == 1) //ball
	{
		vertCount = sizeof(ball_VertexList) / sizeof(ball_VertexList[0]);
		for (i = 0; i < vertCount; ++i)
		{
			outputVerts[i] = MVector(ball_VertexList[i][0], ball_VertexList[i][1], ball_VertexList[i][2]);
		}

		faceCount = sizeof(ball_FaceList) / sizeof(ball_FaceList[0]);
		memcpy(outputFaces, ball_FaceList, sizeof(ball_FaceList));

		edgeCount = sizeof(ball_EdgeList) / sizeof(ball_EdgeList[0]);
		memcpy(outputEdges, ball_EdgeList, sizeof(ball_EdgeList));
	}
	else if (data->fShapeType == 2) //loop
	{
		vertCount = sizeof(loop_VertexList) / sizeof(loop_VertexList[0]);
		for (i = 0; i < vertCount; ++i)
		{
			outputVerts[i] = MVector(loop_VertexList[i][0], loop_VertexList[i][1], loop_VertexList[i][2]);
		}

		faceCount = sizeof(loop_FaceList) / sizeof(loop_FaceList[0]);
		memcpy(outputFaces, loop_FaceList, sizeof(loop_FaceList));

		edgeCount = sizeof(loop_EdgeList) / sizeof(loop_EdgeList[0]);
		memcpy(outputEdges, loop_EdgeList, sizeof(loop_EdgeList));
	}
	else if (data->fShapeType == 3) //base rig
	{
		vertCount = sizeof(rigBase_VertexList) / sizeof(rigBase_VertexList[0]);
		for (i = 0; i < vertCount; ++i)
		{
			outputVerts[i] = MVector(rigBase_VertexList[i][0], rigBase_VertexList[i][1], rigBase_VertexList[i][2]);
		}

		faceCount = sizeof(rigBase_FaceList) / sizeof(rigBase_FaceList[0]);
		memcpy(outputFaces, rigBase_FaceList, sizeof(rigBase_FaceList));

		edgeCount = sizeof(rigBase_EdgeList) / sizeof(rigBase_EdgeList[0]);
		memcpy(outputEdges, rigBase_EdgeList, sizeof(rigBase_EdgeList));
	}
	else
	{
		vertCount = sizeof(box_VertexList) / sizeof(box_VertexList[0]);
		for (i = 0; i < vertCount; ++i)
		{
			outputVerts[i] = MVector(box_VertexList[i][0], box_VertexList[i][1], box_VertexList[i][2]);
		}

		faceCount = sizeof(box_FaceList) / sizeof(box_FaceList[0]);
		memcpy(outputFaces, box_FaceList, sizeof(box_FaceList));

		edgeCount = sizeof(box_EdgeList) / sizeof(box_EdgeList[0]);
		memcpy(outputEdges, box_EdgeList, sizeof(box_EdgeList));

	}

	// Adjust Vertex Values
	//
	for (i = 0; i < vertCount; ++i)
	{
		MVector adjustedVert(
			outputVerts[i][0] * scaleAdjust[0],
			outputVerts[i][1] * scaleAdjust[1],
			outputVerts[i][2] * scaleAdjust[2]
		);

		if (isRotated)
		{
			adjustedVert = adjustedVert * rotMatrix;
		}

		outputVerts[i][0] = adjustedVert.x + positionAdjust[0];
		outputVerts[i][1] = adjustedVert.y + positionAdjust[1];
		outputVerts[i][2] = adjustedVert.z + positionAdjust[2];
		//outputVerts[i][0] = (outputVerts[i][0] * scaleAdjust[0]) + positionAdjust[0];
		//outputVerts[i][1] = (outputVerts[i][1] * scaleAdjust[1]) + positionAdjust[1];
		//outputVerts[i][2] = (outputVerts[i][2] * scaleAdjust[2]) + positionAdjust[2];
	}


	// Draw Lines
	if (data->fWireVisibility > 0.0f)
	{
		for (i = 0; i < 256; i++)
		{
			data->fShapeLineList[i].clear();
		}

		for (i = 0; i < edgeCount; i++)
		{
			//data->fShapeLineList[i].clear();
			data->fShapeLineList[i].append(
				outputVerts[outputEdges[i][0]][0],
				outputVerts[outputEdges[i][0]][1],
				outputVerts[outputEdges[i][0]][2]);
			data->fShapeLineList[i].append(
				outputVerts[outputEdges[i][1]][0],
				outputVerts[outputEdges[i][1]][1],
				outputVerts[outputEdges[i][1]][2]);

		}
	}

	// Draw triangles
	if (data->fSolidVisibility > 0.0f)
	{
		data->fShapeTriangleList.clear();
		for (i = 0; i < faceCount; i++)
		{
			data->fShapeTriangleList.append(
				outputVerts[outputFaces[i][0]][0],
				outputVerts[outputFaces[i][0]][1],
				outputVerts[outputFaces[i][0]][2]);
			data->fShapeTriangleList.append(
				outputVerts[outputFaces[i][1]][0],
				outputVerts[outputFaces[i][1]][1],
				outputVerts[outputFaces[i][1]][2]);
			data->fShapeTriangleList.append(
				outputVerts[outputFaces[i][2]][0],
				outputVerts[outputFaces[i][2]][1],
				outputVerts[outputFaces[i][2]][2]);
		}
	}

	// get correct color based on the state of object, e.g. active or dormant
	data->fColor = MHWRender::MGeometryUtilities::wireframeColor(objPath);

	return data;
}

// addUIDrawables() provides access to the MUIDrawManager, which can be used
// to queue up operations for drawing simple UI elements such as lines, circles and
// text. To enable addUIDrawables(), override hasUIDrawables() and make it return true.
void baconFatBoneDrawOverride::addUIDrawables(
	const MDagPath& objPath,
	MHWRender::MUIDrawManager& drawManager,
	const MHWRender::MFrameContext& frameContext,
	const MUserData* data)
{
	// Get data cached by prepareForDraw() for each drawable instance, then MUIDrawManager 
	// can draw simple UI by these data.
	baconFatBoneData* pLocatorData = (baconFatBoneData*)data;
	if (!pLocatorData)
	{
		return;
	}

	drawManager.beginDrawable();
	if (pLocatorData->fXRay)
		drawManager.beginDrawInXray();

	//drawManager.beginDrawInXray();

	//float fWireVisibility = getWireVisibility(objPath);
	//float fSolidVisibility = getSolidVisibility(objPath);

	// Draw the foot print solid/wireframe
	//drawManager.setColor(pLocatorData->fColor);
	//drawManager.setColor(MColor(0.1f, 0.8f, 0.8f, 0.2f));

	// displayStatus can tell us if it's selected
	MHWRender::DisplayStatus displayStatus = MHWRender::MGeometryUtilities::displayStatus(objPath);


	MColor locatorColor = pLocatorData->fColor;

	if (pLocatorData->fWireVisibility > 0.0)
	{
		if (pLocatorData->fpresto == true)
		{
			if (displayStatus == MHWRender::kLead || displayStatus == MHWRender::kActive)
			{
				locatorColor.a = 0.3f;
			}
			else
			{
				locatorColor.a = pLocatorData->fWireVisibility; // 1.0f;
			}
		}
		else
		{
			locatorColor.a = pLocatorData->fWireVisibility; // 1.0f;
		}

		drawManager.setColor(locatorColor);
		unsigned int i;
		unsigned int edgeCount = sizeof(pLocatorData->fShapeLineList) / sizeof(pLocatorData->fShapeLineList[0]);
		for (i = 0; i < edgeCount; i++)
		{
			drawManager.mesh(MHWRender::MUIDrawManager::kLines, pLocatorData->fShapeLineList[i]);
		}
	}


	if (pLocatorData->fSolidVisibility > 0.0)
	{
		if (pLocatorData->fpresto == true)
		{
			if (displayStatus == MHWRender::kLead || displayStatus == MHWRender::kActive)
			{
				locatorColor.a = 0.08f;
			}
			else
			{
				locatorColor.a = pLocatorData->fSolidVisibility; // 1.0f;
			}
		}
		else
		{
			locatorColor.a = pLocatorData->fSolidVisibility; // 0.1f;
		}
		drawManager.setColor(locatorColor);
		//drawManager.setDepthPriority(0);

		if (frameContext.getDisplayStyle() & MHWRender::MFrameContext::kGouraudShaded) {
			drawManager.mesh(MHWRender::MUIDrawManager::kTriangles, pLocatorData->fShapeTriangleList);
		}
	}

	// Draw Text
	/*
	MPoint pos( 0.0, 0.0, 0.0 ); // Position of the text
	MColor textColor( 0.1f, 0.8f, 0.8f, 1.0f ); // Text color
	drawManager.setColor( textColor );
	drawManager.setFontSize( MHWRender::MUIDrawManager::kSmallFontSize );
	drawManager.text( pos,  MString("Over name"), MHWRender::MUIDrawManager::kCenter );
	*/

	//drawManager.endDrawInXray();
	if (pLocatorData->fXRay)
		drawManager.endDrawInXray();

	drawManager.endDrawable();
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
// Plugin Registration
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

MStatus baconFatBone::initialize()
{

	MFnUnitAttribute	unitFn;
	MFnAttribute		attrFn;
	MFnStringData		stringFn;
	MFnEnumAttribute	enumAttr;
	MFnNumericAttribute nAttr;


	// DO NOT remove Inherited Attrs from UI for a bone
	MObject inheretedAttrs[] =
	{
		baconFatBone::localPositionX,
		baconFatBone::localPositionY,
		baconFatBone::localPositionZ,
		baconFatBone::localScaleX,
		baconFatBone::localScaleY,
		baconFatBone::localScaleZ
	};
	unsigned int i;
	for (i = 0; i < 6; ++i)
	{
		MFnAttribute inheritedFn(inheretedAttrs[i]);
		inheritedFn.setHidden(false);
	}





	MStatus				stat;

	// Local Rotation X
	localRotationX = unitFn.create("localRotationX", "lr", MFnUnitAttribute::kAngle);
	unitFn.setDefault(0.0);
	unitFn.setKeyable(true);
	unitFn.setHidden(false);
	unitFn.setStorable(true);
	stat = addAttribute(localRotationX);
	if (!stat) { stat.perror("addAttribute"); return stat; }

	// Local Rotation Y
	localRotationY = unitFn.create("localRotationY", "ly", MFnUnitAttribute::kAngle);
	unitFn.setDefault(0.0);
	unitFn.setKeyable(true);
	unitFn.setHidden(false);
	unitFn.setStorable(true);
	stat = addAttribute(localRotationY);
	if (!stat) { stat.perror("addAttribute"); return stat; }

	// Local Rotation Z
	localRotationZ = unitFn.create("localRotationZ", "lz", MFnUnitAttribute::kAngle);
	unitFn.setDefault(0.0);
	unitFn.setKeyable(true);
	unitFn.setHidden(false);
	unitFn.setStorable(true);
	stat = addAttribute(localRotationZ);
	if (!stat) { stat.perror("addAttribute"); return stat; }

	// Size
	size = unitFn.create("size", "sz", MFnUnitAttribute::kDistance);
	unitFn.setDefault(1.0);
	unitFn.setKeyable(true);
	unitFn.setStorable(true);
	stat = addAttribute(size);
	if (!stat) {
		stat.perror("addAttribute");
		return stat;
	}

	// Wire Visibility
	wireVisibility = unitFn.create("wireVisibility", "wv", MFnUnitAttribute::kDistance);
	unitFn.setDefault(0.3);
	unitFn.setKeyable(true);
	unitFn.setHidden(false);
	unitFn.setStorable(true);
	stat = addAttribute(wireVisibility);
	if (!stat) {
		stat.perror("addAttribute");
		return stat;
	}

	// Solid Visibility
	solidVisibility = unitFn.create("solidVisibility", "sv", MFnUnitAttribute::kDistance);
	unitFn.setDefault(0.04);
	unitFn.setKeyable(true);
	unitFn.setHidden(false);
	unitFn.setStorable(true);
	stat = addAttribute(solidVisibility);
	if (!stat) {
		stat.perror("addAttribute");
		return stat;
	}

	// shapeType
	shapeType = enumAttr.create("shapeType", "st", 0);
	enumAttr.addField("Box", 0);
	enumAttr.addField("Sphere", 1);
	enumAttr.addField("Loop", 2);
	enumAttr.addField("Rig Base", 3);
	enumAttr.setHidden(false);
	enumAttr.setKeyable(true);
	stat = addAttribute(shapeType);
	if (!stat) { stat.perror("addAttribute"); return stat; }

	// pickerType
	pickerType = enumAttr.create("pickerType", "pickType", 0);
	enumAttr.addField("None", 0);
	enumAttr.addField("FK", 1);
	enumAttr.addField("IK", 2);
	enumAttr.setHidden(false);
	enumAttr.setKeyable(true);
	stat = addAttribute(pickerType);
	if (!stat) { stat.perror("addAttribute"); return stat; }

	// xRay
	xRay = nAttr.create("xRay", "xr", MFnNumericData::kBoolean);
	nAttr.setDefault(false);
	nAttr.setKeyable(true);
	nAttr.setReadable(true);
	nAttr.setHidden(false);
	nAttr.setWritable(true);
	nAttr.setStorable(true);
	stat = addAttribute(xRay);
	if (!stat) { stat.perror("addAttribute"); return stat; }


	// Presto Select
	prestoMode = nAttr.create("prestoMode", "presto", MFnNumericData::kBoolean);
	nAttr.setDefault(false);
	nAttr.setKeyable(true);
	nAttr.setReadable(true);
	nAttr.setHidden(false);
	nAttr.setWritable(true);
	nAttr.setStorable(true);
	stat = addAttribute(prestoMode);
	if (!stat) { stat.perror("addAttribute"); return stat; }



	/*

	// draw last
	aEnableDrawLast = nAttr.create("drawLast", "dl", MFnNumericData::kBoolean);
	nAttr.setDefault(false);
	nAttr.setKeyable(true);
	nAttr.setReadable(true);
	nAttr.setHidden(false);
	nAttr.setWritable(true);
	nAttr.setStorable(true);
	stat = addAttribute(aEnableDrawLast);
	if (!stat) { stat.perror("addAttribute"); return stat; }


	aTransparency = nAttr.create("transparency", "t", MFnNumericData::kFloat);
	nAttr.setDefault(0.5);
	nAttr.setKeyable(true);
	nAttr.setReadable(true);
	nAttr.setWritable(true);
	nAttr.setStorable(true);
	stat = addAttribute(aTransparency);
	if (!stat) { stat.perror("addAttribute"); return stat; }

	aEnableTransparencySort = nAttr.create("transparencySort", "ts", MFnNumericData::kBoolean);
	nAttr.setDefault(true);
	nAttr.setKeyable(true);
	nAttr.setReadable(true);
	nAttr.setWritable(true);
	nAttr.setStorable(true);
	stat = addAttribute(aEnableTransparencySort);
	if (!stat) { stat.perror("addAttribute"); return stat; }
	*/


	return MS::kSuccess;
}

MStatus initializePlugin(MObject obj)
{
	MStatus   status;
	MFnPlugin plugin(obj, "Bacon-Strip.com", "1.0", "Any");

	status = plugin.registerNode(
		"baconFatBone",
		baconFatBone::id,
		&baconFatBone::creator,
		&baconFatBone::initialize,
		MPxNode::kLocatorNode,
		sUseLegacyDraw ? NULL : &baconFatBone::drawDbClassification);
	if (!status) {
		status.perror("registerNode");
		return status;
	}

	if (!sUseLegacyDraw)
	{
		status = MHWRender::MDrawRegistry::registerDrawOverrideCreator(
			baconFatBone::drawDbClassification,
			baconFatBone::drawRegistrantId,
			baconFatBoneDrawOverride::Creator);
		if (!status) {
			status.perror("registerDrawOverrideCreator");
			return status;
		}
	}

	return status;
}

MStatus uninitializePlugin(MObject obj)
{
	MStatus   status;
	MFnPlugin plugin(obj);

	if (!sUseLegacyDraw)
	{
		status = MHWRender::MDrawRegistry::deregisterDrawOverrideCreator(
			baconFatBone::drawDbClassification,
			baconFatBone::drawRegistrantId);
		if (!status) {
			status.perror("deregisterDrawOverrideCreator");
			return status;
		}
	}

	status = plugin.deregisterNode(baconFatBone::id);
	if (!status) {
		status.perror("deregisterNode");
		return status;
	}
	return status;
}
