//-
// ==========================================================================
// BACON-STRIP.com RIG
//
// licensed under the GNU General Public License v3.0
// Permissions of this strong copyleft license are conditioned on making 
// available complete source code of licensed works and modifications, 
// which include larger works using a licensed work, under the same license.
// Copyright and license notices must be preserved. Contributors provide an 
// express grant of patent rights.
//
// Luis Alonso - Creative Commons - 2017
//
// ==========================================================================
//+

////////////////////////////////////////////////////////////////////////
// DESCRIPTION:
// Produces the dependency graph node "baconFatArc".
// This plug-in is part of a rigging package known as The "Bacon-Strip Rig"
// Bacon-Strip ID Block is : 0x0012a940 - 0x0012aa3f
// Registered for Bacon-Strip[Luis Alonso]
//
// ID: 
// 0x0012a943
//
////////////////////////////////////////////////////////////////////////


#include <maya/MCallbackIdArray.h>
#include <maya/MDagMessage.h>
#include <maya/MDrawRegistry.h>
#include <maya/MFnDagNode.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnPlugin.h>
#include <maya/MFrameContext.h>
#include <maya/MItDag.h>
#include <maya/MPxDrawOverride.h>
#include <maya/MPxLocatorNode.h>
#include <maya/MVectorArray.h>
#include <maya/MPointArray.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFloatMatrix.h>
#include <maya/MTransformationMatrix.h>


#include <stdio.h>

using namespace MHWRender;


class baconFatArc : public MPxLocatorNode
{
public:


	enum ELineType
	{
		kSolid,
		kDotted,
		kDashed,
		kShortDotted,
		kShortDashed
	};

	static void* creator() { return new baconFatArc(); }

	static MStatus initialize();

	// Registration
	static char*   className;
	static MTypeId id;
	static MString drawDbClassification;
	static MString drawRegistrantId;

	// Attributes
	static MObject aWireColor;
	static MObject aLineType;
	static MObject parentTM;
	static MObject wireIntensity;
	static MObject xRay;
	static MObject splineThickness;
	static MObject startWorldMatrix;
	static MObject endWorldMatrix;
	
private:
	/*
	baconFatArc();
	*/
	virtual ~baconFatArc();

};

// Registration
char*   baconFatArc::className = "baconFatArc";
MTypeId baconFatArc::id(0x0012a943);
MString baconFatArc::drawDbClassification("drawdb/geometry/baconFatArc");
MString baconFatArc::drawRegistrantId(baconFatArc::className);

// Attributes
MObject baconFatArc::aWireColor;
MObject baconFatArc::aLineType;
MObject	baconFatArc::parentTM;
MObject	baconFatArc::wireIntensity;
MObject	baconFatArc::xRay;
MObject	baconFatArc::splineThickness;
MObject	baconFatArc::startWorldMatrix;
MObject	baconFatArc::endWorldMatrix;

MMatrix setRow(MMatrix matrix, MVector newVector, const int row)
{
	MMatrix returnTM = matrix;
	returnTM[row][0] = newVector[0];
	returnTM[row][1] = newVector[1];
	returnTM[row][2] = newVector[2];
	return returnTM;
}

MMatrix transMatrix(MVector pos)
{
	MMatrix returnTM = setRow(MMatrix(), pos, 3);
	return returnTM;
}

MVector Bezier4Interpolation(MVector P0, MVector P1, MVector P2, MVector P3, float u)
{
	float u2(u * u);
	float u3(u2 * u);
	return (P0 + (-P0 * 3.0f + u * (3.0f * P0 - P0 * u)) * u
		+ (3.0f * P1 + u * (-6.0f * P1 + P1 * 3.0f * u)) * u
		+ (P2 * 3.0f - P2 * 3.0f * u) * u2 + P3 * u3);
}

MMatrix FloatMatrixToMatrix(MFloatMatrix fTM)
{
	MMatrix returnTM = MMatrix();
	returnTM = setRow(returnTM, MVector(fTM[0][0], fTM[0][1], fTM[0][2]), 0);
	returnTM = setRow(returnTM, MVector(fTM[1][0], fTM[1][1], fTM[1][2]), 1);
	returnTM = setRow(returnTM, MVector(fTM[2][0], fTM[2][1], fTM[2][2]), 2);
	returnTM = setRow(returnTM, MVector(fTM[3][0], fTM[3][1], fTM[3][2]), 3);
	return returnTM;
}

MStatus baconFatArc::initialize()
{

	MStatus				stat;
	MFnNumericAttribute wAttr;
	MFnNumericAttribute nAttr;

	aWireColor = nAttr.create("wireColor", "wirec", MFnNumericData::k3Float);
	nAttr.setDefault(1.0f, 1.0f, 1.0f);
	nAttr.setStorable(true);
	nAttr.setKeyable(true);
	nAttr.setHidden(false);
	nAttr.setWritable(true);
	nAttr.setReadable(true);
	nAttr.setUsedAsColor(true);
	MPxNode::addAttribute(aWireColor);

	MFnEnumAttribute eAttr;
	aLineType = eAttr.create("lineType", "tt", kDotted);
	eAttr.addField("Solid Line", kSolid);
	eAttr.addField("Dots", kDotted);
	eAttr.addField("Dashes", kDashed);
	eAttr.addField("Spaced Dots", kShortDotted);
	eAttr.addField("Spaced Dashes", kShortDashed);
	eAttr.setStorable(true);
	eAttr.setKeyable(true);
	eAttr.setHidden(false);
	eAttr.setWritable(true);
	eAttr.setReadable(true);
	MPxNode::addAttribute(aLineType);

	// Line Thickness
	splineThickness = wAttr.create("lineThickness", "lt", MFnNumericData::kDouble);
	wAttr.setDefault(1.0);
	wAttr.setStorable(true);
	wAttr.setKeyable(true);
	wAttr.setHidden(false);
	wAttr.setWritable(true);
	wAttr.setReadable(true);
	MPxNode::addAttribute(splineThickness);
	MFnNumericAttribute tAttr;

	wireIntensity = wAttr.create("wireIntensity", "wi", MFnNumericData::kDouble);
	wAttr.setDefault(0.3);
	wAttr.setMin(0.0);
	wAttr.setMax(1.0);
	wAttr.setStorable(true);
	wAttr.setKeyable(true);
	wAttr.setHidden(false);
	wAttr.setWritable(true);
	wAttr.setReadable(true);
	MPxNode::addAttribute(wireIntensity);

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

	MFnMatrixAttribute	matrixAttr;

	// input parentTM
	parentTM = matrixAttr.create("parentTM", "pTM", matrixAttr.kFloat);
	matrixAttr.setStorable(true);
	matrixAttr.setKeyable(true);
	stat = addAttribute(parentTM);

	// input startWorldMatrix
	startWorldMatrix = matrixAttr.create("startWorldMatrix", "sTM", matrixAttr.kFloat);
	matrixAttr.setStorable(true);
	matrixAttr.setKeyable(true);
	stat = addAttribute(startWorldMatrix);

	// input endWorldMatrix
	endWorldMatrix = matrixAttr.create("endWorldMatrix", "eTM", matrixAttr.kFloat);
	matrixAttr.setStorable(true);
	matrixAttr.setKeyable(true);
	stat = addAttribute(endWorldMatrix);

	return MS::kSuccess;
}


baconFatArc::~baconFatArc()
{
	/*
	if (fWorldMatrixModifiedCbIds.length() > 0)
	{
		MMessage::removeCallbacks(fWorldMatrixModifiedCbIds);
	}

	if (fAllDagChangesCbId != 0)
	{
		MMessage::removeCallback(fAllDagChangesCbId);
	}
	*/
}

class transformDrawData : public MUserData
{
public:
	transformDrawData()
		: MUserData(false)
		, fWireColor(1.0f, 1.0f, 1.0f, 1.0f)
		, fTangentColor(1.0f, 1.0f, 1.0f, 1.0f)
		, fWireIntensity(0.4)
		, fContrappostoLength(0.0)
		, fContrappostoOffset(0.0)
		, fLineType(baconFatArc::kDotted)
	{
	}

	virtual ~transformDrawData() {}

	MColor fWireColor;
	MColor fTangentColor;
	double fWireIntensity;
	double fContrappostoLength;
	double fContrappostoOffset;
	baconFatArc::ELineType fLineType;
	bool fXRay;

	bool fShowContrapposto;
	bool fShowSpline;
	bool fShowTangents;
	double fSplineThickness;

	//MVectorArray fPositions;
	//MVectorArray fVectors;
	MVector fTargetPos;
	MVector fStartPos;
	MVector fEndPos;
	MVector fStartTangentPos;
	MVector fEndTangentPos;

	MVector fContrappossto1A;
	MVector fContrappossto1B;
	MVector fContrappossto2A;
	MVector fContrappossto2B;

};

class transformDrawOverride : public MPxDrawOverride
{
public:

	static MPxDrawOverride* creator(const MObject& obj)
	{
		return new transformDrawOverride(obj);
	}

	virtual DrawAPI supportedDrawAPIs() const { return kAllDevices; }

	virtual bool hasUIDrawables() const { return true; }

	virtual MUserData* prepareForDraw(
		const MDagPath& objPath,
		const MDagPath& cameraPath,
		const MFrameContext& frameContext,
		MUserData* oldData);

	virtual void addUIDrawables(
		const MDagPath& objPath,
		MUIDrawManager& drawManager,
		const MFrameContext& frameContext,
		const MUserData* data);

private:

	transformDrawOverride(const MObject& obj) : MPxDrawOverride(obj, NULL, true) {}
	virtual ~transformDrawOverride() {}
};


MUserData* transformDrawOverride::prepareForDraw(
	const MDagPath& objPath,
	const MDagPath& cameraPath,
	const MFrameContext& frameContext,
	MUserData* oldData)
{
	MStatus status;
	MObject obj = objPath.node(&status);
	if (!status) return NULL;

	transformDrawData* tdData = dynamic_cast<transformDrawData*>(oldData);
	if (!tdData)
	{
		tdData = new transformDrawData();
	}

	//MPlug plugTM(obj, baconFatArc::parentInverseMatrix);
	MPlug plugTM(obj, baconFatArc::parentTM);
	MDataHandle worldTMhandle = plugTM.asMDataHandle();
	MFloatMatrix worldTMValue = worldTMhandle.asFloatMatrix();
	MMatrix thisTM = FloatMatrixToMatrix(worldTMValue);

	MPlug xRayPlug(obj, baconFatArc::xRay);
	tdData->fXRay = xRayPlug.asBool();


	// Bezier 4 points
	MPlug startPlugtTM(obj, baconFatArc::startWorldMatrix);
	MDataHandle startTMhandle = startPlugtTM.asMDataHandle();
	MFloatMatrix startTMValue = startTMhandle.asFloatMatrix();
	MMatrix startTM = FloatMatrixToMatrix(startTMValue);
	MTransformationMatrix startLocalTM = startTM * thisTM.inverse();
	MVector startVec = startLocalTM.getTranslation(MSpace::kWorld);
	tdData->fStartPos = startVec;

	MPlug endPlugtTM(obj, baconFatArc::endWorldMatrix);
	MDataHandle endTMhandle = endPlugtTM.asMDataHandle();
	MFloatMatrix endTMValue = endTMhandle.asFloatMatrix();
	MMatrix endTM = FloatMatrixToMatrix(endTMValue);
	MTransformationMatrix endLocalTM = endTM * thisTM.inverse();
	MVector endVec = endLocalTM.getTranslation(MSpace::kWorld);
	tdData->fEndPos = endVec;

	MVector startTangentDir(startTM[2][0], startTM[2][1], startTM[2][2]);
	MVector startTangentVec = startVec + startTangentDir;
	tdData->fStartTangentPos = startTangentVec;

	MVector endTangentDir(endTM[2][0], endTM[2][1], endTM[2][2]);
	MVector endTangentVec = endVec + endTangentDir;
	tdData->fEndTangentPos = endTangentVec;



	// Wire color
	{
		MPlug plug(obj, baconFatArc::aWireColor);
		MObject o = plug.asMObject();
		MFnNumericData nData(o);
		nData.getData(tdData->fWireColor.r, tdData->fWireColor.g, tdData->fWireColor.b);

		MPlug plugIntensity(obj, baconFatArc::wireIntensity);
		MDataHandle wireIntensityHandle = plugIntensity.asMDataHandle();
		tdData->fWireIntensity = wireIntensityHandle.asDouble();

		MPlug plugThickness(obj, baconFatArc::splineThickness);
		MDataHandle ThicknessHandle = plugThickness.asMDataHandle();
		tdData->fSplineThickness = ThicknessHandle.asDouble();
	}

	// Type of Line
	MPlug plugLineType(obj, baconFatArc::aLineType);
	tdData->fLineType = (baconFatArc::ELineType)plugLineType.asInt();


	return tdData;
}

void transformDrawOverride::addUIDrawables(
	const MDagPath& objPath,
	MUIDrawManager& drawManager,
	const MFrameContext& frameContext,
	const MUserData* data)
{
	const transformDrawData* tdData = dynamic_cast<const transformDrawData*>(data);
	if (!tdData) return;

	MColor wColor(tdData->fWireColor);
	wColor.a = float(tdData->fWireIntensity); //0.4;

	MColor tColor(tdData->fTangentColor);
	tColor.a = float(tdData->fWireIntensity); //0.4;

	MPointArray bezierLine;
	bezierLine.append(tdData->fStartPos);
	float totalDivisions(15.0f);
	float curveSample(1.0f / totalDivisions);
	for (int i = 1; i < totalDivisions; i++)
	{
		MVector nextPos = Bezier4Interpolation(tdData->fStartPos, tdData->fStartTangentPos,
			tdData->fEndTangentPos, tdData->fEndPos, (i * curveSample));
		bezierLine.append(nextPos);
		bezierLine.append(nextPos); // using kLineStrip will fail on xRay, so using segmented kLine
	}
	bezierLine.append(tdData->fEndPos);

	

	drawManager.beginDrawable();

	switch (tdData->fLineType)
	{
	case 1:
		drawManager.setLineStyle(MHWRender::MUIDrawManager::kDotted);
		break;
	case 2:
		drawManager.setLineStyle(MHWRender::MUIDrawManager::kDashed);
		break;
	case 3:
		drawManager.setLineStyle(MHWRender::MUIDrawManager::kShortDotted);
		break;
	case 4:
		drawManager.setLineStyle(MHWRender::MUIDrawManager::kShortDashed);
		break;
	default:
		drawManager.setLineStyle(MHWRender::MUIDrawManager::kSolid);
		break;
	}

	if (tdData->fXRay)
		drawManager.beginDrawInXray();


	drawManager.setColor(tColor);
	drawManager.setLineWidth(3.0f);
	drawManager.setColor(wColor);
	drawManager.setLineWidth(tdData->fSplineThickness);
	drawManager.setLineStyle(MHWRender::MUIDrawManager::kSolid);

	if (tdData->fShowSpline)
	{
		drawManager.mesh(MHWRender::MUIDrawManager::kLines, bezierLine);
	}

	if (tdData->fXRay)
		drawManager.endDrawInXray();


	/*

	// text
	thisColor.a = 1.0;
	drawManager.setColor(thisColor);
	MPoint pos(tdData->fPositions[i]);
	MVector vec(tdData->fVectors[i]);
	char tmpStr[128] = {0};
	sprintf(tmpStr, "(%.3f, %.3f, %.3f)", vec.x, vec.y, vec.z);
	MString text(tmpStr);
	drawManager.text(pos, text, MUIDrawManager::kCenter);
	*/

	
	drawManager.endDrawable();
}


MStatus initializePlugin(MObject obj)
{
	MFnPlugin plugin(obj, "Bacon-Strip.com", "1.0", "Any");

	MStatus status = plugin.registerNode(
		baconFatArc::className,
		baconFatArc::id,
		baconFatArc::creator,
		baconFatArc::initialize,
		MPxNode::kLocatorNode,
		&baconFatArc::drawDbClassification);
	if (!status)
	{
		status.perror("registerNode");
		return status;
	}

	status = MDrawRegistry::registerDrawOverrideCreator(
		baconFatArc::drawDbClassification,
		baconFatArc::drawRegistrantId,
		transformDrawOverride::creator);
	if (!status)
	{
		status.perror("registerDrawOverrideCreator");
		return status;
	}

	return status;
}

MStatus uninitializePlugin(MObject obj)
{
	MFnPlugin plugin(obj);

	MStatus status = MDrawRegistry::deregisterDrawOverrideCreator(
		baconFatArc::drawDbClassification,
		baconFatArc::drawRegistrantId);
	if (!status)
	{
		status.perror("deregisterDrawOverrideCreator");
		return status;
	}

	status = plugin.deregisterNode(baconFatArc::id);
	if (!status)
	{
		status.perror("deregisterNode");
		return status;
	}

	return status;
}
