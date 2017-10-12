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
// Produces the dependency graph node "baconRibbon".
// This plug-in is part of a rigging package known as The "Bacon-Strip Rig"
// Bacon-Strip ID Block is : 0x0012a940 - 0x0012aa3f
// Registered for Bacon-Strip[Luis Alonso]
//
// ID: 
// 0x0012a955
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

 
class baconRibbon : public MPxNode
{
public:
						baconRibbon();
	virtual				~baconRibbon(); 

	virtual MStatus		compute( const MPlug& plug, MDataBlock& data );

	static  void*		creator();
	static  MStatus		initialize();

public:
	static	MTypeId		id;

	// inputs
	static  MObject		pathPercent;						// The pathPercent value for the start.
	static  MObject		pathLength;							
	static  MObject		twistStart;
	static  MObject		twistEnd;

	static  MObject		startWorldMatrix;
	static  MObject		endWorldMatrix;
	static  MObject		startTangentWorldMatrix;
	static  MObject		endTangentWorldMatrix;

	static  MObject		targetWorldMatrix;
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

MTypeId     baconRibbon::id( 0x0012a955 );
MObject     baconRibbon::pathPercent;
MObject     baconRibbon::pathLength;
MObject     baconRibbon::twistStart;
MObject     baconRibbon::twistEnd;
MObject		baconRibbon::startWorldMatrix;
MObject		baconRibbon::endWorldMatrix;
MObject		baconRibbon::startTangentWorldMatrix;
MObject		baconRibbon::endTangentWorldMatrix;
MObject		baconRibbon::targetWorldMatrix;
MObject		baconRibbon::parentInverseMatrix;
MObject		baconRibbon::jointOrient;
MObject		baconRibbon::jointOrientX;
MObject		baconRibbon::jointOrientY;
MObject		baconRibbon::jointOrientZ;
MObject		baconRibbon::alignedRotation;
MObject		baconRibbon::alignedRotationX;
MObject		baconRibbon::alignedRotationY;
MObject		baconRibbon::alignedRotationZ;
MObject		baconRibbon::alignedPosition;

baconRibbon::baconRibbon() {}
baconRibbon::~baconRibbon() {}

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


MStatus baconRibbon::compute( const MPlug& plug, MDataBlock& data )
{
	
	MStatus returnStatus;
 
	if
	( 
		plug == alignedRotation		|| plug == alignedPosition		||
		plug == alignedRotationX	|| plug == alignedRotationY		|| plug == alignedRotationZ
	)
	{
		// Handles and Values
		MDataHandle pathPercentHandle = data.inputValue(pathPercent, &returnStatus);
		float pathPercentValue(pathPercentHandle.asFloat());

		MDataHandle pathLengthHandle = data.inputValue(pathLength, &returnStatus);
		float pathLengthValue(pathLengthHandle.asFloat());


		MDataHandle twistStartHandle = data.inputValue(twistStart, &returnStatus);
		MAngle twistStartValue(twistStartHandle.asAngle());

		MDataHandle twistEndHandle = data.inputValue(twistEnd, &returnStatus);
		MAngle twistEndValue(twistEndHandle.asAngle());

		MDataHandle startWorldMatrixHandle = data.inputValue(startWorldMatrix, &returnStatus);
		MFloatMatrix sFM(startWorldMatrixHandle.asFloatMatrix());

		MDataHandle endWorldMatrixHandle = data.inputValue(endWorldMatrix, &returnStatus);
		MFloatMatrix eFM(endWorldMatrixHandle.asFloatMatrix());

		MDataHandle startTangentWorldMatrixHandle = data.inputValue(startTangentWorldMatrix, &returnStatus);
		MFloatMatrix stFM(startTangentWorldMatrixHandle.asFloatMatrix());

		MDataHandle endTangentWorldMatrixHandle = data.inputValue(endTangentWorldMatrix, &returnStatus);
		MFloatMatrix etFM(endTangentWorldMatrixHandle.asFloatMatrix());

		MDataHandle targetWorldMatrixHandle = data.inputValue(targetWorldMatrix, &returnStatus);
		MFloatMatrix tFM(targetWorldMatrixHandle.asFloatMatrix());

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
		MMatrix sTM = FloatMatrixToMatrix(sFM);
		MMatrix eTM = FloatMatrixToMatrix(eFM);
		MMatrix stTM = FloatMatrixToMatrix(stFM);
		MMatrix etTM = FloatMatrixToMatrix(etFM);
		MMatrix ipTM = FloatMatrixToMatrix(pFM);

		MVector P0(sTM[3][0], sTM[3][1], sTM[3][2]);
		MVector P1(stTM[3][0], stTM[3][1], stTM[3][2]);
		MVector P2(etTM[3][0], etTM[3][1], etTM[3][2]);
		MVector P3(eTM[3][0], eTM[3][1], eTM[3][2]);

		MVector BezierPosition = Bezier4Interpolation(P0, P1, P2, P3, pathPercentValue);
		//MVector TangentAxis = Bezier4Tangent(P0, P1, P2, P3, pathPercentValue);
		MVector TipPosition = Bezier4Interpolation(P0, P1, P2, P3, pathPercentValue + pathLengthValue);
		MVector TangentAxis = TipPosition - BezierPosition;
		TangentAxis.normalize();


		// get Axis diff.
		/*
		MVector StartAxis(sTM[0][0], sTM[0][1], sTM[0][2]);
		StartAxis.normalize();
		MVector EndAxis(eTM[0][0], eTM[0][1], eTM[0][2]);
		EndAxis.normalize();

		//MVector TangentAxis = Bezier4Tangent(P0, P1, P2, P3, pathPercentValue);
		MVector TangentAxis = (StartAxis * (1.0f - pathPercentValue)) + (EndAxis * pathPercentValue);
		TangentAxis.normalize();
		MQuaternion startRot(sTM);


		*/



		/*
		MVector OffAxis(StartAxis ^ TangentAxis);
		OffAxis.normalize();

		MAngle OffAngle(StartAxis ^ TangentAxis);
		*/

		/*
		MQuaternion OffsetRotation(TangentAxis, StartAxis);
		MMatrix OffsetTM(OffsetRotation);
		double startAngle = twistStartValue.value() * (1.0 - pathPercentValue);
		double endAngle = twistEndValue.value() * pathPercentValue;
		MMatrix twistMatrix = MEulerRotation(startAngle + endAngle, 0.0, 0.0).asMatrix();
		MMatrix finalWorldTM = OffsetTM * sTM;
		finalWorldTM[3][0] = BezierPosition.x;
		finalWorldTM[3][1] = BezierPosition.y;
		finalWorldTM[3][2] = BezierPosition.z;

		*/

		MVector UpVector(sTM[2][0], sTM[2][1], sTM[2][2]);
		MVector YAxis = UpVector ^ TangentAxis;
		YAxis.normalize();
		MVector ZAxis = TangentAxis ^ YAxis;
		ZAxis.normalize();
		MMatrix TangentMatrix = matrix3(TangentAxis, YAxis, ZAxis, BezierPosition);

		double Twist = (twistEndValue.value() - twistStartValue.value()) * pathPercentValue * 1.0;
		MMatrix twistMatrix = MEulerRotation(Twist, 0.0, 0.0).asMatrix();
		MMatrix finalWorldTM = twistMatrix * TangentMatrix;


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
			// float result = sinf(pathPercentData.asFloat()) * 10.0f;
			// MAngle uX = jointOrientXHandle.asAngle();
			//MAngle uX = jointOrientXHandle.asDouble();
			//double result = uX.;

			//humerusLengthHandle = inputValue.asFloat(bio2BoneIK.humerusLength)
			//SideA = humerusLengthHandle.asFloat()


			// pathPercent
			//MDataHandle pathPercentHandle = data.outputValue(baconRibbon::pathPercent);
			//pathPercentHandle.setDouble();


			// alignedRotationX
			MDataHandle alignedRotationXHandle = data.outputValue(baconRibbon::alignedRotationX);
			alignedRotationXHandle.setMAngle(MAngle(outputAngles.x));
			alignedRotationXHandle.setClean();
			// alignedRotationY
			MDataHandle alignedRotationYHandle = data.outputValue(baconRibbon::alignedRotationY);
			alignedRotationYHandle.setMAngle(MAngle(outputAngles.y));
			alignedRotationYHandle.setClean();
			// alignedRotationX
			MDataHandle alignedRotationZHandle = data.outputValue(baconRibbon::alignedRotationZ);
			alignedRotationZHandle.setMAngle(MAngle(outputAngles.z));
			alignedRotationZHandle.setClean();

			// aligned Position
			MDataHandle alignedPositionHandle = data.outputValue(baconRibbon::alignedPosition);
			alignedPositionHandle.set3Float(localTM[3][0], localTM[3][1], localTM[3][2]);
				//setMVector(MVector(outputTM[3][0], outputTM[3][1], outputTM[3][2]));
			alignedPositionHandle.setClean();

		}
	} else {
		return MS::kUnknownParameter;
	}

	return MS::kSuccess;
}

void* baconRibbon::creator()
{
	return new baconRibbon();
}


MStatus baconRibbon::initialize()
{
	MFnNumericAttribute numAttr;
	MFnUnitAttribute	uAttr;
	MFnMatrixAttribute	matrixAttr;
	MStatus				stat;

	// INPUTS ---------------------------------------------------------------------
	
	// pathPercent
	pathPercent = numAttr.create("pathPercent", "pp", MFnNumericData::kFloat, 0.0);
	numAttr.setMin(0.0f);
	numAttr.setMax(1.0f);
	numAttr.setStorable(true);
	numAttr.setKeyable(true);
	numAttr.setWritable(true);
	stat = addAttribute(pathPercent);

	// pathLength
	pathLength = numAttr.create("pathLength", "pl", MFnNumericData::kFloat, 0.25);
	numAttr.setMin(0.0f);
	numAttr.setMax(1.0f);
	numAttr.setStorable(true);
	numAttr.setKeyable(true);
	numAttr.setWritable(true);
	stat = addAttribute(pathLength);

	// twistStart
	twistStart = uAttr.create("twistStart", "ts", uAttr.kAngle, 0.0);
	uAttr.setWritable(true);
	uAttr.setKeyable(true);
	stat = addAttribute(twistStart);

	// twistEnd
	twistEnd = uAttr.create("twistEnd", "te", uAttr.kAngle, 0.0);
	uAttr.setWritable(true);
	uAttr.setKeyable(true);
	stat = addAttribute(twistEnd);

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



	// input targetWorldMatrix
	targetWorldMatrix = matrixAttr.create("targetWorldMatrix", "twTM", matrixAttr.kFloat);
	matrixAttr.setStorable(true);
	matrixAttr.setKeyable(true);
	stat = addAttribute(targetWorldMatrix);

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
		attributeAffects(startWorldMatrix,			obj);
		attributeAffects(endWorldMatrix,			obj);
		attributeAffects(startTangentWorldMatrix,	obj);
		attributeAffects(endTangentWorldMatrix,		obj);
		attributeAffects(targetWorldMatrix,			obj);
		attributeAffects(parentInverseMatrix, obj);
		attributeAffects(pathPercent, obj);
		attributeAffects(pathLength, obj);
		attributeAffects(twistStart, obj);
		attributeAffects(twistEnd, obj);
	}

	return MS::kSuccess;
}

MStatus initializePlugin( MObject obj )
{ 
	MStatus   status;
	MFnPlugin plugin(obj, "Bacon-Strip.com", "1.0", "Any");

	status = plugin.registerNode( "baconRibbon", baconRibbon::id, baconRibbon::creator,
								  baconRibbon::initialize );
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

	status = plugin.deregisterNode( baconRibbon::id );
	if (!status) {
		status.perror("deregisterNode");
		return status;
	}

	return status;
}
