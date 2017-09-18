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
// Produces the dependency graph node "baconArc".
// This plug-in is part of a rigging package known as The "Bacon-Strip Rig"
// Bacon-Strip ID Block is : 0x0012a940 - 0x0012aa3f
// Registered for Bacon-Strip[Luis Alonso]
//
// ID: 
// 0x0012a957
//
////////////////////////////////////////////////////////////////////////

#include <string.h>
#include <array>
#include <maya/MIOStream.h>
#include <math.h>

#include <maya/MPxNode.h> 

#include <maya/MFnNumericAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnMatrixData.h>
#include <maya/MFnPlugin.h>

#include <maya/MString.h> 
#include <maya/MTypeId.h> 
#include <maya/MPlug.h>
#include <maya/MVector.h>
#include <maya/MAngle.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MQuaternion.h>

#include <maya/MFloatPoint.h>
#include <maya/MFloatMatrix.h>
#include <maya/MEulerRotation.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MDoubleArray.h>
#include <maya/MScriptUtil.h>

 
class baconArc : public MPxNode
{
public:
						baconArc();
	virtual				~baconArc(); 

	virtual MStatus		compute( const MPlug& plug, MDataBlock& data );

	static  void*		creator();
	static  MStatus		initialize();

public:
	static	MTypeId		id;

	// inputs
	static  MObject		Curve1Percent;						// The Curve1Percent value for the start.
	static  MObject		Curve2Percent;							

	static  MObject		startCurve1WorldMatrix;
	static  MObject		endCurve1WorldMatrix;
	static  MObject		startCurve2WorldMatrix;
	static  MObject		endCurve2WorldMatrix;

	static  MObject		parentInverseMatrix;
	static  MObject		jointOrient;
	static  MObject		jointOrientX;
	static  MObject		jointOrientY;
	static  MObject		jointOrientZ;

	// ouputs
	static  MObject		alignedRotation;
	static  MObject		alignedRotationX;
	static  MObject		alignedRotationY;
	static  MObject		alignedRotationZ;
	static  MObject		alignedPosition;

};

MTypeId     baconArc::id( 0x0012a957 );
MObject     baconArc::Curve1Percent;
MObject     baconArc::Curve2Percent;
MObject		baconArc::startCurve1WorldMatrix;
MObject		baconArc::endCurve1WorldMatrix;
MObject		baconArc::startCurve2WorldMatrix;
MObject		baconArc::endCurve2WorldMatrix;
MObject		baconArc::parentInverseMatrix;
MObject		baconArc::jointOrient;
MObject		baconArc::jointOrientX;
MObject		baconArc::jointOrientY;
MObject		baconArc::jointOrientZ;
MObject		baconArc::alignedRotation;
MObject		baconArc::alignedRotationX;
MObject		baconArc::alignedRotationY;
MObject		baconArc::alignedRotationZ;
MObject		baconArc::alignedPosition;

baconArc::baconArc() {}
baconArc::~baconArc() {}

MMatrix setRow( MMatrix matrix, MVector newVector, const int row)
{
	MMatrix returnTM = matrix;
	returnTM[row][0] = newVector[0];
	returnTM[row][1] = newVector[1];
	returnTM[row][2] = newVector[2];
	//MScriptUtil ArrayUtil = MScriptUtil();
	//ArrayUtil.setDoubleArray(matrix[row], 0, newVector[0]);
	//ArrayUtil.setDoubleArray(matrix[row], 1, newVector[1]);
	//ArrayUtil.setDoubleArray(matrix[row], 2, newVector[2]);
	return returnTM;
}

MMatrix transMatrix(MVector pos)
{
	MMatrix returnTM = setRow(MMatrix(), pos, 3);
	return returnTM;
}

MMatrix rotateXMatrix(float angle)
{
	MMatrix returnTM = MMatrix();
	float CAngle = cos(angle);
	float SAngle = sin(angle);
	returnTM = setRow(returnTM, MVector(1.0f, 0.0f, 0.0f), 0);
	returnTM = setRow(returnTM, MVector(0.0f, CAngle, SAngle), 1);
	returnTM = setRow(returnTM, MVector(0.0f, -1.0f * SAngle, CAngle), 2);
	returnTM = setRow(returnTM, MVector(0.0f, 0.0f, 0.0f), 3);
	return returnTM;
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

MVector Bezier4Interpolation(MVector P0, MVector P1, MVector P2, MVector P3, float u)
{
	float u2(u * u);
	float u3(u2 * u);
	return (P0 + (-P0 * 3.0f + u * (3.0f * P0 - P0 * u)) * u 
		+ (3.0f * P1 + u * (-6.0f * P1 + P1 * 3.0f * u)) * u 
		+ (P2 * 3.0f - P2 * 3.0f * u) * u2 + P3 * u3);
}


MVector Bezier4Tangent(MVector P0, MVector P1, MVector P2, MVector P3, float u)
{
	MVector C1(P3 - (3.0f * P2) + (3.0f * P1) - P0);
	MVector C2((3.0f * P2) - (6.0f * P1) + (3.0f * P0));
	MVector C3((3.0f * P1) - (3.0f * P0));
	return ((3.0f * C1 * u * u) + (2.0f * C2 * u) + C3);

}

MMatrix matrix3(MVector row1, MVector row2, MVector row3, MVector row4)
{
	MMatrix returnTM = MMatrix();
	returnTM = setRow(returnTM, row1, 0);
	returnTM = setRow(returnTM, row2, 1);
	returnTM = setRow(returnTM, row3, 2);
	returnTM = setRow(returnTM, row4, 3);
	return returnTM;
}


MStatus baconArc::compute( const MPlug& plug, MDataBlock& data )
{
	
	MStatus returnStatus;
 
	if
	( 
		plug == alignedRotation		|| plug == alignedPosition		||
		plug == alignedRotationX	|| plug == alignedRotationY		|| plug == alignedRotationZ
	)
	{
		// Handles and Values
		MDataHandle Curve1PercentHandle = data.inputValue(Curve1Percent, &returnStatus);
		float Curve1PercentValue(Curve1PercentHandle.asFloat());

		MDataHandle Curve2PercentHandle = data.inputValue(Curve2Percent, &returnStatus);
		float Curve2PercentValue(Curve2PercentHandle.asFloat());

		MDataHandle startCurve1WorldMatrixHandle = data.inputValue(startCurve1WorldMatrix, &returnStatus);
		MFloatMatrix s1FM(startCurve1WorldMatrixHandle.asFloatMatrix());

		MDataHandle endCurve1WorldMatrixHandle = data.inputValue(endCurve1WorldMatrix, &returnStatus);
		MFloatMatrix e1FM(endCurve1WorldMatrixHandle.asFloatMatrix());

		MDataHandle startCurve2WorldMatrixHandle = data.inputValue(startCurve2WorldMatrix, &returnStatus);
		MFloatMatrix s2FM(startCurve2WorldMatrixHandle.asFloatMatrix());

		MDataHandle endCurve2WorldMatrixHandle = data.inputValue(endCurve2WorldMatrix, &returnStatus);
		MFloatMatrix e2FM(endCurve2WorldMatrixHandle.asFloatMatrix());

		MDataHandle parentInverseMatrixHandle = data.inputValue(parentInverseMatrix, &returnStatus);
		MFloatMatrix pFM = parentInverseMatrixHandle.asFloatMatrix();

		MDataHandle jointOrientXHandle = data.inputValue(jointOrientX, &returnStatus);
		MAngle jointOrientXValue = jointOrientXHandle.asAngle();
		MDataHandle jointOrientYHandle = data.inputValue(jointOrientY, &returnStatus);
		MAngle jointOrientYValue = jointOrientYHandle.asAngle();
		MDataHandle jointOrientZHandle = data.inputValue(jointOrientZ, &returnStatus);
		MAngle jointOrientZValue = jointOrientZHandle.asAngle();


		/////////////////////////////////////////////////////////
		// Calculation
		MMatrix s1TM = FloatMatrixToMatrix(s1FM);
		MMatrix e1TM = FloatMatrixToMatrix(e1FM);
		MMatrix s2TM = FloatMatrixToMatrix(s2FM);
		MMatrix e2TM = FloatMatrixToMatrix(e2FM);
		MMatrix ipTM = FloatMatrixToMatrix(pFM);


		// Curve 1
		MVector c1P0(s1TM[3][0], s1TM[3][1], s1TM[3][2]);
		MVector c1SDir(s1TM[2][0], s1TM[2][1], s1TM[2][2]);
		MVector c1P3(e1TM[3][0], e1TM[3][1], e1TM[3][2]);
		MVector c1EDir(e1TM[2][0], e1TM[2][1], e1TM[2][2]);
		MVector c1P1(c1P0 + c1SDir);
		MVector c1P2(c1P3 + c1EDir);
		MVector Bezier1Position = Bezier4Interpolation(c1P0, c1P1, c1P2, c1P3, Curve1PercentValue);
		MVector Tangent1Axis = Bezier4Tangent(c1P0, c1P1, c1P2, c1P3, Curve1PercentValue);
		Tangent1Axis.normalize();

		// Curve 2
		MVector c2P0(s2TM[3][0], s2TM[3][1], s2TM[3][2]);
		MVector c2SDir(s2TM[2][0], s2TM[2][1], s2TM[2][2]);
		MVector c2P3(e2TM[3][0], e2TM[3][1], e2TM[3][2]);
		MVector c2EDir(e2TM[2][0], e2TM[2][1], e2TM[2][2]);
		MVector c2P1(c2P0 + c2SDir);
		MVector c2P2(c2P3 + c2EDir);
		MVector Bezier2Position = Bezier4Interpolation(c2P0, c2P1, c2P2, c2P3, Curve2PercentValue);

		MVector XAxis(Bezier2Position - Bezier1Position);
		XAxis.normalize();

		MVector YAxis = Tangent1Axis ^ XAxis;
		YAxis.normalize();
		MVector ZAxis = XAxis ^ YAxis;
		ZAxis.normalize();
		MMatrix finalWorldTM = matrix3(XAxis, YAxis, ZAxis, Bezier1Position);

		MMatrix localTM = finalWorldTM * ipTM;
		MMatrix jointOrientTM = MEulerRotation(jointOrientXValue.value(), jointOrientYValue.value(),
			jointOrientZValue.value()).asMatrix();
		MMatrix outputTM = localTM * jointOrientTM.inverse();
		MTransformationMatrix ouputTransformationMatrix = MTransformationMatrix(outputTM);
		MEulerRotation outputAngles = ouputTransformationMatrix.eulerRotation();


		// Set OutPut Values
		if( returnStatus != MS::kSuccess )
			cerr << "ERROR getting data" << endl;
		else
		{

			// alignedRotationX
			MDataHandle alignedRotationXHandle = data.outputValue(baconArc::alignedRotationX);
			alignedRotationXHandle.setMAngle(MAngle(outputAngles.x));
			alignedRotationXHandle.setClean();
			// alignedRotationY
			MDataHandle alignedRotationYHandle = data.outputValue(baconArc::alignedRotationY);
			alignedRotationYHandle.setMAngle(MAngle(outputAngles.y));
			alignedRotationYHandle.setClean();
			// alignedRotationX
			MDataHandle alignedRotationZHandle = data.outputValue(baconArc::alignedRotationZ);
			alignedRotationZHandle.setMAngle(MAngle(outputAngles.z));
			alignedRotationZHandle.setClean();

			// aligned Position
			MDataHandle alignedPositionHandle = data.outputValue(baconArc::alignedPosition);
			alignedPositionHandle.set3Float(localTM[3][0], localTM[3][1], localTM[3][2]);
				//setMVector(MVector(outputTM[3][0], outputTM[3][1], outputTM[3][2]));
			alignedPositionHandle.setClean();

		}
	} else {
		return MS::kUnknownParameter;
	}

	return MS::kSuccess;
}

void* baconArc::creator()
{
	return new baconArc();
}


MStatus baconArc::initialize()
{
	MFnNumericAttribute numAttr;
	MFnUnitAttribute	uAttr;
	MFnMatrixAttribute	matrixAttr;
	MStatus				stat;

	// INPUTS ---------------------------------------------------------------------
	
	// Curve1Percent
	Curve1Percent = numAttr.create("Curve1Percent", "pp", MFnNumericData::kFloat, 0.0);
	numAttr.setMin(0.0f);
	numAttr.setMax(1.0f);
	numAttr.setStorable(true);
	numAttr.setKeyable(true);
	numAttr.setWritable(true);
	stat = addAttribute(Curve1Percent);

	// Curve2Percent
	Curve2Percent = numAttr.create("Curve2Percent", "pl", MFnNumericData::kFloat, 0.25);
	numAttr.setMin(0.0f);
	numAttr.setMax(1.0f);
	numAttr.setStorable(true);
	numAttr.setKeyable(true);
	numAttr.setWritable(true);
	stat = addAttribute(Curve2Percent);

	// input jointOrientX
	jointOrientX = uAttr.create("jointOrientX", "uox", uAttr.kAngle, 0.0);
	uAttr.setWritable(true);
	uAttr.setStorable(true);

	// input jointOrientY
	jointOrientY = uAttr.create("jointOrientY", "uoy", uAttr.kAngle, 0.0);
	uAttr.setWritable(true);
	uAttr.setStorable(true);

	// input jointOrientZ
	jointOrientZ = uAttr.create("jointOrientZ", "uoz", uAttr.kAngle, 0.0);
	uAttr.setWritable(true);
	uAttr.setStorable(true);

	// input jointOrient
	jointOrient = numAttr.create("jointOrient", "jo", jointOrientX, jointOrientY, jointOrientZ);
	numAttr.setWritable(true);
	numAttr.setStorable(true);
	numAttr.setKeyable(true);
	stat = addAttribute(jointOrient);


	// input startCurve1WorldMatrix
	startCurve1WorldMatrix = matrixAttr.create("startCurve1WorldMatrix", "s1TM", matrixAttr.kFloat);
	matrixAttr.setStorable(true);
	matrixAttr.setKeyable(true);
	stat = addAttribute(startCurve1WorldMatrix);

	// input endCurve1WorldMatrix
	endCurve1WorldMatrix = matrixAttr.create("endCurve1WorldMatrix", "e1TM", matrixAttr.kFloat);
	matrixAttr.setStorable(true);
	matrixAttr.setKeyable(true);
	stat = addAttribute(endCurve1WorldMatrix);

	// input startCurve2WorldMatrix
	startCurve2WorldMatrix = matrixAttr.create("startCurve2WorldMatrix", "s2TM", matrixAttr.kFloat);
	matrixAttr.setStorable(true);
	matrixAttr.setKeyable(true);
	stat = addAttribute(startCurve2WorldMatrix);

	// input endCurve2WorldMatrix
	endCurve2WorldMatrix = matrixAttr.create("endCurve2WorldMatrix", "e2TM", matrixAttr.kFloat);
	matrixAttr.setStorable(true);
	matrixAttr.setKeyable(true);
	stat = addAttribute(endCurve2WorldMatrix);

	// input parentInverseMatrix
	parentInverseMatrix = matrixAttr.create("parentInverseMatrix", "piTM", matrixAttr.kFloat);
	matrixAttr.setStorable(true);
	matrixAttr.setKeyable(true);
	stat = addAttribute(parentInverseMatrix);


	// OUTUTS ---------------------------------------------------------------------

	// Output aligned rotationX
	alignedRotationX = uAttr.create("alignedRotationX", "arotX", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output aligned rotationY
	alignedRotationY = uAttr.create("alignedRotationY", "arotY", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output aligned rotationZ
	alignedRotationZ = uAttr.create("alignedRotationZ", "arotZ", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output aligned rotation
	alignedRotation = numAttr.create("alignedRotation", "arot", alignedRotationX, alignedRotationY, alignedRotationZ);
	numAttr.setHidden(false);
	stat = addAttribute(alignedRotation);

	// Output Aligned Position
	alignedPosition = numAttr.createPoint("alignedPosition", "apos");
	numAttr.setStorable(false);
	numAttr.setHidden(false);
	stat = addAttribute(alignedPosition);

	
	//AFFECTS ---------------------------------------------------------------------
	MObject AffectedByMany[] =
	{	alignedRotation,	alignedPosition,
		alignedRotationX,	alignedRotationY,	alignedRotationZ
	};
	for (MObject& obj : AffectedByMany) 
	{
		attributeAffects(jointOrient,				obj);
		attributeAffects(jointOrientX,				obj);
		attributeAffects(jointOrientY,				obj);
		attributeAffects(jointOrientZ,				obj);
		attributeAffects(startCurve1WorldMatrix,	obj);
		attributeAffects(endCurve1WorldMatrix,		obj);
		attributeAffects(startCurve2WorldMatrix,	obj);
		attributeAffects(endCurve2WorldMatrix,		obj);
		attributeAffects(parentInverseMatrix,		obj);
		attributeAffects(Curve1Percent,				obj);
		attributeAffects(Curve2Percent,				obj);
	}

	return MS::kSuccess;
}

MStatus initializePlugin( MObject obj )
{ 
	MStatus   status;
	MFnPlugin plugin(obj, "Bacon-Strip.com", "1.0", "Any");

	status = plugin.registerNode( "baconArc", baconArc::id, baconArc::creator,
								  baconArc::initialize );
	if (!status) {
		status.perror("registerNode");
		return status;
	}

	return status;
}

MStatus uninitializePlugin( MObject obj)
{
	MStatus   status;
	MFnPlugin plugin( obj );

	status = plugin.deregisterNode( baconArc::id );
	if (!status) {
		status.perror("deregisterNode");
		return status;
	}

	return status;
}
