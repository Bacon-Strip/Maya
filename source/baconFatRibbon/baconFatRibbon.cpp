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
// Produces the dependency graph node "baconAlign".
// This plug-in is part of a rigging package known as The "Bacon-Strip Rig"
// Bacon-Strip ID Block is : 0x0012a940 - 0x0012aa3f
// Registered for Bacon-Strip[Luis Alonso]
//
// ID: 
// 0x0012a942
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


class baconFatRibbon : public MPxLocatorNode
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

	static void* creator() { return new baconFatRibbon(); }

	static MStatus initialize();

	// Registration
	static char*   className;
	static MTypeId id;
	static MString drawDbClassification;
	static MString drawRegistrantId;

	// Attributes
	static MObject aWireColor;
	static MObject aTangentColor;
	static MObject aLineType;
	static MObject targetMatrix;
	static MObject parentTM;
	static MObject wireIntensity;
	static MObject xRay;
	static MObject showTangents;
	static MObject showSpline;
	static MObject showContrapposto;
	static MObject splineThickness;
	static MObject ContrappostoLength;
	static MObject ContrappostoOffset;

	static MObject startWorldMatrix;
	static MObject endWorldMatrix;
	static MObject startTangentWorldMatrix;
	static MObject endTangentWorldMatrix;


private:

	/*
	baconFatRibbon();
	*/

	virtual ~baconFatRibbon();

	/* 
	// REMOVING CALLBACKS SINCE IT'S FASTER TO SIMPLY DRAW THE LINE
	// This plugin is based on "transformDrawNode" which is designed to 
	// draw lines on all meshes in the scene. 
	static void AllDagChangesCallback(
		MDagMessage::DagMessage msgType,
		MDagPath &child,
		MDagPath &parent,
		void *clientData);

	static void WorldMatrixModifiedCallback(
		MObject &transformNode,
		MDagMessage::MatrixModifiedFlags &modified,
		void *clientData);

	void processDagMessage(bool refreshWorldMatrixCbIds);

	MCallbackId fAllDagChangesCbId;
	MCallbackIdArray fWorldMatrixModifiedCbIds;
	*/

};

// Registration
char*   baconFatRibbon::className = "baconFatRibbon";
MTypeId baconFatRibbon::id(0x0012a942);
MString baconFatRibbon::drawDbClassification("drawdb/geometry/baconFatRibbon");
MString baconFatRibbon::drawRegistrantId(baconFatRibbon::className);

// Attributes
MObject baconFatRibbon::aWireColor;
MObject baconFatRibbon::aTangentColor;
MObject baconFatRibbon::aLineType;
MObject	baconFatRibbon::targetMatrix;
MObject	baconFatRibbon::parentTM;
MObject	baconFatRibbon::wireIntensity;
MObject	baconFatRibbon::xRay;
MObject	baconFatRibbon::ContrappostoLength;
MObject	baconFatRibbon::ContrappostoOffset;
MObject	baconFatRibbon::showTangents;
MObject	baconFatRibbon::showSpline;
MObject	baconFatRibbon::showContrapposto;
MObject	baconFatRibbon::splineThickness;

MObject	baconFatRibbon::startWorldMatrix;
MObject	baconFatRibbon::endWorldMatrix;
MObject	baconFatRibbon::startTangentWorldMatrix;
MObject	baconFatRibbon::endTangentWorldMatrix;

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
	float u2(u * u);	float u3(u2 * u);	return (P0 + (-P0 * 3.0f + u * (3.0f * P0 - P0 * u)) * u		+ (3.0f * P1 + u * (-6.0f * P1 + P1 * 3.0f * u)) * u		+ (P2 * 3.0f - P2 * 3.0f * u) * u2 + P3 * u3);}

MMatrix FloatMatrixToMatrix(MFloatMatrix fTM)
{
	MMatrix returnTM = MMatrix();
	returnTM = setRow(returnTM, MVector(fTM[0][0], fTM[0][1], fTM[0][2]), 0);
	returnTM = setRow(returnTM, MVector(fTM[1][0], fTM[1][1], fTM[1][2]), 1);
	returnTM = setRow(returnTM, MVector(fTM[2][0], fTM[2][1], fTM[2][2]), 2);
	returnTM = setRow(returnTM, MVector(fTM[3][0], fTM[3][1], fTM[3][2]), 3);
	return returnTM;
}




MStatus baconFatRibbon::initialize()
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

	MFnNumericAttribute tAttr;
	aTangentColor = tAttr.create("tangentColor", "tanc", MFnNumericData::k3Float);
	tAttr.setDefault(1.0f, 1.0f, 1.0f);
	tAttr.setStorable(true);
	tAttr.setKeyable(true);
	tAttr.setHidden(false);
	tAttr.setWritable(true);
	tAttr.setReadable(true);
	tAttr.setUsedAsColor(true);
	MPxNode::addAttribute(aTangentColor);

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

	// showSpline
	showSpline = nAttr.create("showLine", "showL", MFnNumericData::kBoolean);
	nAttr.setDefault(true);
	nAttr.setKeyable(true);
	nAttr.setReadable(true);
	nAttr.setHidden(false);
	nAttr.setWritable(true);
	nAttr.setStorable(true);
	stat = addAttribute(showSpline);
	if (!stat) { stat.perror("addAttribute"); return stat; }

	// showTangents
	showTangents = nAttr.create("showTangents", "showT", MFnNumericData::kBoolean);
	nAttr.setDefault(true);
	nAttr.setKeyable(true);
	nAttr.setReadable(true);
	nAttr.setHidden(false);
	nAttr.setWritable(true);
	nAttr.setStorable(true);
	stat = addAttribute(showTangents);
	if (!stat) { stat.perror("addAttribute"); return stat; }

	// showContrapposto
	showContrapposto = nAttr.create("showContrapposto", "showC", MFnNumericData::kBoolean);
	nAttr.setDefault(true);
	nAttr.setKeyable(true);
	nAttr.setReadable(true);
	nAttr.setHidden(false);
	nAttr.setWritable(true);
	nAttr.setStorable(true);
	stat = addAttribute(showContrapposto);
	if (!stat) { stat.perror("addAttribute"); return stat; }

	// Line Thickness
	splineThickness = wAttr.create("lineThickness", "lt", MFnNumericData::kDouble);
	wAttr.setDefault(1.0);
	wAttr.setStorable(true);
	wAttr.setKeyable(true);
	wAttr.setHidden(false);
	wAttr.setWritable(true);
	wAttr.setReadable(true);
	MPxNode::addAttribute(splineThickness);


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


	ContrappostoLength = wAttr.create("ContrappostoLength", "cl", MFnNumericData::kDouble);
	wAttr.setDefault(0.0);
	wAttr.setStorable(true);
	wAttr.setKeyable(true);
	wAttr.setHidden(false);
	wAttr.setWritable(true);
	wAttr.setReadable(true);
	MPxNode::addAttribute(ContrappostoLength);

	ContrappostoOffset = wAttr.create("ContrappostoOffset", "co", MFnNumericData::kDouble);
	wAttr.setDefault(0.0);
	wAttr.setStorable(true);
	wAttr.setKeyable(true);
	wAttr.setHidden(false);
	wAttr.setWritable(true);
	wAttr.setReadable(true);
	MPxNode::addAttribute(ContrappostoOffset);


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
	// input targetMatrix
	targetMatrix = matrixAttr.create("pelvicMatrix", "pelTM", matrixAttr.kFloat);
	matrixAttr.setStorable(true);
	matrixAttr.setKeyable(true);
	stat = addAttribute(targetMatrix);

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

	// input startTangentWorldMatrix
	startTangentWorldMatrix = matrixAttr.create("startTangentWorldMatrix", "stTM", matrixAttr.kFloat);
	matrixAttr.setStorable(true);
	matrixAttr.setKeyable(true);
	stat = addAttribute(startTangentWorldMatrix);

	// input endTangentWorldMatrix
	endTangentWorldMatrix = matrixAttr.create("endTangentWorldMatrix", "etTM", matrixAttr.kFloat);
	matrixAttr.setStorable(true);
	matrixAttr.setKeyable(true);
	stat = addAttribute(endTangentWorldMatrix);


	return MS::kSuccess;
}


/*
baconFatRibbon::baconFatRibbon() : MPxLocatorNode()
{
	fAllDagChangesCbId = MDagMessage::addAllDagChangesCallback(
		AllDagChangesCallback, this);
}
*/

baconFatRibbon::~baconFatRibbon()
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


/*
void baconFatRibbon::processDagMessage(bool refreshWorldMatrixModifiedCbIds)
{
	// Explicitly mark the node as being dirty on certain DAG message callbacks
	// so that the draw override can be updated.
	MRenderer::setGeometryDrawDirty(thisMObject());

	if (refreshWorldMatrixModifiedCbIds)
	{
		MStatus status;
		MItDag dagIt(MItDag::kDepthFirst, MFn::kMesh, &status);
		if (status)
		{
			// Remove existing callbacks
			if (fWorldMatrixModifiedCbIds.length() > 0)
			{
				MMessage::removeCallbacks(fWorldMatrixModifiedCbIds);
				fWorldMatrixModifiedCbIds.clear();
			}

			// Add new callbacks
			for (; !dagIt.isDone(); dagIt.next())
			{
				MDagPath dagPath;
				status = dagIt.getPath(dagPath);
				if (status)
				{
					MCallbackId id = MDagMessage::addWorldMatrixModifiedCallback(
						dagPath, WorldMatrixModifiedCallback, this);
					fWorldMatrixModifiedCbIds.append(id);
				}
			}
		}
	}
}

void baconFatRibbon::AllDagChangesCallback(
	MDagMessage::DagMessage msgType,
	MDagPath &child,
	MDagPath &parent,
	void *clientData)
{
	// We need to refresh the world matrix modified callbacks because the DAG
	// is just changed.
	baconFatRibbon *node = static_cast<baconFatRibbon*>(clientData);
	if (node) node->processDagMessage(true);
}

void baconFatRibbon::WorldMatrixModifiedCallback(
	MObject &transformNode,
	MDagMessage::MatrixModifiedFlags &modified,
	void *clientData)
{
	// We don't need to refresh the world matrix modified callbacks because the
	// DAG is not changed.

	baconFatRibbon *node = static_cast<baconFatRibbon*>(clientData);
	if (node) node->processDagMessage(false);

}

*/

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
		, fLineType(baconFatRibbon::kDotted)
	{
	}

	virtual ~transformDrawData() {}

	MColor fWireColor;
	MColor fTangentColor;
	double fWireIntensity;
	double fContrappostoLength;
	double fContrappostoOffset;
	baconFatRibbon::ELineType fLineType;
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

	// By setting isAlwaysDirty to false in MPxDrawOverride constructor, the
	// draw override will be updated (via prepareForDraw()) only when the node
	// is marked dirty via DG evaluation or dirty propagation. Additional
	// callbacks are also added to explicitly mark the node as being dirty (via
	// MRenderer::setGeometryDrawDirty()) for certain circumstances. Note that
	// the draw callback in MPxDrawOverride constructor is set to NULL in order
	// to achieve better performance.

	// "isAlwaysDirty" SET TO TRUE
	// This way it gets flagged as dirty when lookat objects transform. 
	// This is faster than callbacks since you have to navigate through dagpaths
	// if not.

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

	/*
	MStatus returnStatus;
	MDataHandle worldTMhandle = data.inputValue(baconFatRibbon::worldMatrix, &returnStatus);
	MFloatMatrix worldTMValue = worldTMhandle.asFloatMatrix();
	*/

	//MPlug plugTM(obj, baconFatRibbon::parentInverseMatrix);
	MPlug plugTM(obj, baconFatRibbon::parentTM);
	MDataHandle worldTMhandle = plugTM.asMDataHandle();
	MFloatMatrix worldTMValue = worldTMhandle.asFloatMatrix();
	MMatrix thisTM = FloatMatrixToMatrix(worldTMValue);

	MPlug plugtTM(obj, baconFatRibbon::targetMatrix);
	MDataHandle worldtTMhandle = plugtTM.asMDataHandle();
	MFloatMatrix worldtTMValue = worldtTMhandle.asFloatMatrix();
	MMatrix targetTM = FloatMatrixToMatrix(worldtTMValue);
	MTransformationMatrix localTM = targetTM * thisTM.inverse();
	MVector vec = localTM.getTranslation(MSpace::kWorld);
	tdData->fTargetPos = vec;

	MPlug xRayPlug(obj, baconFatRibbon::xRay);
	tdData->fXRay = xRayPlug.asBool();


	// show
	MPlug showSplinePlug(obj, baconFatRibbon::showSpline);
	tdData->fShowSpline = showSplinePlug.asBool();

	MPlug showTangentsPlug(obj, baconFatRibbon::showTangents);
	tdData->fShowTangents = showTangentsPlug.asBool();

	MPlug showContrappostoPlug(obj, baconFatRibbon::showContrapposto);
	tdData->fShowContrapposto = showContrappostoPlug.asBool();


	// Bezier 4 points
	MPlug plugContrappostoLength(obj, baconFatRibbon::ContrappostoLength);
	MDataHandle ContrappostoLengthHandle = plugContrappostoLength.asMDataHandle();
	double cpLength = ContrappostoLengthHandle.asDouble();
	tdData->fContrappostoLength = cpLength;

	MPlug plugContrappostoOffset(obj, baconFatRibbon::ContrappostoOffset);
	MDataHandle ContrappostoOffsetHandle = plugContrappostoOffset.asMDataHandle();
	double cpOffset = ContrappostoOffsetHandle.asDouble();
	tdData->fContrappostoOffset = cpOffset;

	MPlug startPlugtTM(obj, baconFatRibbon::startWorldMatrix);
	MDataHandle startTMhandle = startPlugtTM.asMDataHandle();
	MFloatMatrix startTMValue = startTMhandle.asFloatMatrix();
	MMatrix startTM = FloatMatrixToMatrix(startTMValue);
	MTransformationMatrix startLocalTM = startTM * thisTM.inverse();
	MVector startVec = startLocalTM.getTranslation(MSpace::kWorld);
	tdData->fStartPos = startVec;

	MMatrix cposto1ATM = (transMatrix(MVector(0, 0, cpLength)) * targetTM) * thisTM.inverse();
	tdData->fContrappossto1A = MVector(cposto1ATM[3][0], cposto1ATM[3][1], cposto1ATM[3][2]);
	MMatrix cposto1BTM = (transMatrix(MVector(0, 0, -1.0 * cpLength)) * targetTM) * thisTM.inverse();
	tdData->fContrappossto1B = MVector(cposto1BTM[3][0], cposto1BTM[3][1], cposto1BTM[3][2]);

	MPlug endPlugtTM(obj, baconFatRibbon::endWorldMatrix);
	MDataHandle endTMhandle = endPlugtTM.asMDataHandle();
	MFloatMatrix endTMValue = endTMhandle.asFloatMatrix();
	MMatrix endTM = FloatMatrixToMatrix(endTMValue);
	MTransformationMatrix endLocalTM = endTM * thisTM.inverse();
	MVector endVec = endLocalTM.getTranslation(MSpace::kWorld);
	tdData->fEndPos = endVec;

	MMatrix cposto2ATM = (transMatrix(MVector(cpOffset, 0, cpLength)) * endTM) * thisTM.inverse();
	tdData->fContrappossto2A = MVector(cposto2ATM[3][0], cposto2ATM[3][1], cposto2ATM[3][2]);
	MMatrix cposto2BTM = (transMatrix(MVector(cpOffset, 0, -1.0 * cpLength)) * endTM) * thisTM.inverse();
	tdData->fContrappossto2B = MVector(cposto2BTM[3][0], cposto2BTM[3][1], cposto2BTM[3][2]);



	MPlug startTangentPlugtTM(obj, baconFatRibbon::startTangentWorldMatrix);
	MDataHandle startTangentTMhandle = startTangentPlugtTM.asMDataHandle();
	MFloatMatrix startTangentTMValue = startTangentTMhandle.asFloatMatrix();
	MMatrix startTangentTM = FloatMatrixToMatrix(startTangentTMValue);
	MTransformationMatrix startTangentLocalTM = startTangentTM * thisTM.inverse();
	MVector startTangentVec = startTangentLocalTM.getTranslation(MSpace::kWorld);
	tdData->fStartTangentPos = startTangentVec;

	MPlug endTangentPlugtTM(obj, baconFatRibbon::endTangentWorldMatrix);
	MDataHandle endTangentTMhandle = endTangentPlugtTM.asMDataHandle();
	MFloatMatrix endTangentTMValue = endTangentTMhandle.asFloatMatrix();
	MMatrix endTangentTM = FloatMatrixToMatrix(endTangentTMValue);
	MTransformationMatrix endTangentLocalTM = endTangentTM * thisTM.inverse();
	MVector endTangentVec = endTangentLocalTM.getTranslation(MSpace::kWorld);
	tdData->fEndTangentPos = endTangentVec;


	//MObject oTM = plugTM.asMObject();
	//MFnMatrixAttribute nTM(oTM);

	//pluglocalScaleX.getValue(localScaleValX);


	// Wire color
	{
		MPlug plug(obj, baconFatRibbon::aWireColor);
		MObject o = plug.asMObject();
		MFnNumericData nData(o);
		nData.getData(tdData->fWireColor.r, tdData->fWireColor.g, tdData->fWireColor.b);

		MPlug plugIntensity(obj, baconFatRibbon::wireIntensity);
		MDataHandle wireIntensityHandle = plugIntensity.asMDataHandle();
		tdData->fWireIntensity = wireIntensityHandle.asDouble();

		MPlug plugThickness(obj, baconFatRibbon::splineThickness);
		MDataHandle ThicknessHandle = plugThickness.asMDataHandle();
		tdData->fSplineThickness = ThicknessHandle.asDouble();

	}

	// Tangent Color
	{
		MPlug tColorPlug(obj, baconFatRibbon::aTangentColor);
		MObject otColor = tColorPlug.asMObject();
		MFnNumericData nData(otColor);
		nData.getData(tdData->fTangentColor.r, tdData->fTangentColor.g, tdData->fTangentColor.b);
	}


	// Type of Line
	MPlug plugLineType(obj, baconFatRibbon::aLineType);
	tdData->fLineType = (baconFatRibbon::ELineType)plugLineType.asInt();



	// Transform type
	/*
	{
	MPlug plug(obj, baconFatRibbon::aLineType);
	tdData->fLineType = (baconFatRibbon::ELineType)plug.asInt();
	}

	tdData->fPositions.clear();
	tdData->fVectors.clear();

	MItDag dagIt(MItDag::kDepthFirst, MFn::kMesh, &status);
	if (status)
	{
	for ( ; !dagIt.isDone(); dagIt.next() )
	{
	MDagPath dagPath;
	status = dagIt.getPath(dagPath);
	if (!status)
	{
	status.perror("MItDag::getPath");
	continue;
	}

	MObject transformNode = dagPath.transform(&status);
	if (!status)
	{
	status.perror("MDagPath::transform");
	continue;
	}

	MFnDagNode transform(transformNode, &status);
	if (!status)
	{
	status.perror("MFnDagNode constructor");
	continue;
	}

	//MTransformationMatrix matrix(transform.transformationMatrix());
	MTransformationMatrix localTM = targetTM * thisTM.inverse();
	MVector vec = localTM.getTranslation(MSpace::kWorld);
	//MVector vec = matrix.getTranslation(MSpace::kWorld);

	//vec = vec * thisTM.inverse();
	tdData->fPositions.append(vec);

	double tmp[3];
	MTransformationMatrix::RotationOrder order;
	switch (tdData->fLineType)
	{
	case baconFatRibbon::kRotate:
	matrix.getRotation(tmp, order);
	vec = MVector(tmp);
	break;
	case baconFatRibbon::kScale:
	matrix.getScale(tmp, MSpace::kWorld);
	vec = MVector(tmp);
	break;
	case baconFatRibbon::kShear:
	matrix.getShear(tmp, MSpace::kWorld);
	vec = MVector(tmp);
	break;
	default:
	// Don't reset vec so that translation is drawn by default.
	break;
	}

	tdData->fVectors.append(vec);
	}
	}
	*/

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

	MPointArray startTangentLine;
	startTangentLine.append(tdData->fStartPos);
	startTangentLine.append(tdData->fStartTangentPos);

	MPointArray endTangentLine;
	endTangentLine.append(tdData->fEndPos);
	endTangentLine.append(tdData->fEndTangentPos);

	MPointArray bezierLine;
	bezierLine.append(tdData->fStartPos);
	float totalDivisions(10.0f);
	float curveSample(1.0f / totalDivisions);
	for (int i = 1; i < totalDivisions; i++)
	{
		MVector nextPos = Bezier4Interpolation(tdData->fStartPos, tdData->fStartTangentPos,
			tdData->fEndTangentPos, tdData->fEndPos, (i * curveSample));
		bezierLine.append(nextPos);
		bezierLine.append(nextPos); // using kLineStrip will fail on xRay, so using segmented kLine
	}
	bezierLine.append(tdData->fEndPos);

	MPointArray ContrappostoLines;
	endTangentLine.append(tdData->fContrappossto1A);
	endTangentLine.append(tdData->fContrappossto1B);
	endTangentLine.append(tdData->fContrappossto2A);
	endTangentLine.append(tdData->fContrappossto2B);
	

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
	if (tdData->fShowTangents)
	{
		drawManager.mesh(MHWRender::MUIDrawManager::kLines, startTangentLine);
		drawManager.mesh(MHWRender::MUIDrawManager::kLines, endTangentLine);
	}

	drawManager.setColor(wColor);
	drawManager.setLineWidth(tdData->fSplineThickness);
	drawManager.setLineStyle(MHWRender::MUIDrawManager::kSolid);
	if (tdData->fShowSpline)
	{
		drawManager.mesh(MHWRender::MUIDrawManager::kLines, bezierLine);
	}

	if (tdData->fShowContrapposto)
	{
		drawManager.mesh(MHWRender::MUIDrawManager::kLines, ContrappostoLines);
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
		baconFatRibbon::className,
		baconFatRibbon::id,
		baconFatRibbon::creator,
		baconFatRibbon::initialize,
		MPxNode::kLocatorNode,
		&baconFatRibbon::drawDbClassification);
	if (!status)
	{
		status.perror("registerNode");
		return status;
	}

	status = MDrawRegistry::registerDrawOverrideCreator(
		baconFatRibbon::drawDbClassification,
		baconFatRibbon::drawRegistrantId,
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
		baconFatRibbon::drawDbClassification,
		baconFatRibbon::drawRegistrantId);
	if (!status)
	{
		status.perror("deregisterDrawOverrideCreator");
		return status;
	}

	status = plugin.deregisterNode(baconFatRibbon::id);
	if (!status)
	{
		status.perror("deregisterNode");
		return status;
	}

	return status;
}
