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
// Produces the dependency graph node "baconRoll".
// This plug-in is part of a rigging package known as The "Bacon-Strip Rig"
// Bacon-Strip ID Block is : 0x0012a940 - 0x0012aa3f
// Registered for Bacon-Strip[Luis Alonso]
//
// ID: 
// 0x0012a952
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

//#include <maya/MFloatPoint.h>
#include <maya/MFloatMatrix.h>
#include <maya/MEulerRotation.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MQuaternion.h>
#include <maya/MDoubleArray.h>
#include <maya/MScriptUtil.h>


class baconRoll : public MPxNode
{
public:
	baconRoll();
	virtual				~baconRoll();

	virtual MStatus		compute(const MPlug& plug, MDataBlock& data);

	static  void*		creator();
	static  MStatus		initialize();

public:
	static	MTypeId		id;

	// inputs
	static  MObject		rollType;
	static  MObject		weight1;
	static  MObject		weight2;
	static  MObject		targetMatrix;
	static  MObject		parentMatrix;
	static  MObject		targetRotation;
	static  MObject		targetRotationX;
	static  MObject		targetRotationY;
	static  MObject		targetRotationZ;

	// ouputs
	static  MObject		rollRotation1;
	static  MObject		rollRotation1X;
	static  MObject		rollRotation1Y;
	static  MObject		rollRotation1Z;
	static  MObject		rollRotation2;
	static  MObject		rollRotation2X;
	static  MObject		rollRotation2Y;
	static  MObject		rollRotation2Z;

};

MTypeId     baconRoll::id(0x0012a952);
MObject     baconRoll::weight1;
MObject     baconRoll::weight2;
MObject		baconRoll::rollType;
MObject		baconRoll::targetMatrix;
MObject		baconRoll::parentMatrix;
MObject		baconRoll::targetRotation;
MObject		baconRoll::targetRotationX;
MObject		baconRoll::targetRotationY;
MObject		baconRoll::targetRotationZ;
MObject		baconRoll::rollRotation1;
MObject		baconRoll::rollRotation1X;
MObject		baconRoll::rollRotation1Y;
MObject		baconRoll::rollRotation1Z;
MObject		baconRoll::rollRotation2;
MObject		baconRoll::rollRotation2X;
MObject		baconRoll::rollRotation2Y;
MObject		baconRoll::rollRotation2Z;

baconRoll::baconRoll() {}
baconRoll::~baconRoll() {}

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


MVector getRow(MMatrix TM, int row)
{
	return (MVector(TM[row][0], TM[row][1], TM[row][2]));
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

MMatrix matrix3(MVector row1, MVector row2, MVector row3, MVector row4)
{
	MMatrix returnTM = MMatrix();
	returnTM = setRow(returnTM, row1, 0);
	returnTM = setRow(returnTM, row2, 1);
	returnTM = setRow(returnTM, row3, 2);
	returnTM = setRow(returnTM, row4, 3);
	return returnTM;
}



MStatus baconRoll::compute(const MPlug& plug, MDataBlock& data)
{

	MStatus returnStatus;

	if
		(
			plug == rollRotation1  || plug == rollRotation1X || plug == rollRotation1Y ||
			plug == rollRotation2  || plug == rollRotation2X || plug == rollRotation2Y ||
			plug == rollRotation1Z || plug == rollRotation1Z
		)
	{
		// Handles and Values
		MDataHandle weight1Handle = data.inputValue(weight1, &returnStatus);
		float weight1Value = weight1Handle.asFloat();
		MDataHandle weight2Handle = data.inputValue(weight2, &returnStatus);
		float weight2Value = weight2Handle.asFloat();

		MDataHandle rollTypeHandle = data.inputValue(rollType, &returnStatus);
		unsigned int rollTypeValue = rollTypeHandle.asInt();

		MDataHandle targetMatrixHandle = data.inputValue(targetMatrix, &returnStatus);
		MFloatMatrix tFM(targetMatrixHandle.asFloatMatrix());

		MDataHandle parentMatrixHandle = data.inputValue(parentMatrix, &returnStatus);
		MFloatMatrix pFM = parentMatrixHandle.asFloatMatrix();

		MDataHandle targetRotationXHandle = data.inputValue(targetRotationX, &returnStatus);
		MAngle targetRotationXValue = targetRotationXHandle.asAngle();
		MDataHandle targetRotationYHandle = data.inputValue(targetRotationY, &returnStatus);
		MAngle targetRotationYValue = targetRotationYHandle.asAngle();
		MDataHandle targetRotationZHandle = data.inputValue(targetRotationZ, &returnStatus);
		MAngle targetRotationZValue = targetRotationZHandle.asAngle();

		// Calculation
		MMatrix targetRotationTM = MEulerRotation(targetRotationXValue.value(), targetRotationYValue.value(),
			targetRotationZValue.value()).asMatrix();
		MMatrix pTM(FloatMatrixToMatrix(pFM));
		MMatrix tTM(targetRotationTM * FloatMatrixToMatrix(tFM));
		MMatrix localTM(tTM * pTM.inverse());
		MQuaternion localRot = MTransformationMatrix(localTM).rotation(); 

		MVector xAxis = getRow(localTM, 0);
		xAxis.normalize();
		MVector vector = xAxis ^ MVector(1, 0, 0);
		vector.normalize();
		double angle = acos( xAxis.x );
		float angleSign(1.0f);
		if (rollTypeValue == 0)
		{
			angleSign = -1.0f;
		}

		MQuaternion rollRot(angleSign * angle, vector);

		if (rollTypeValue == 0)
		{
			//parent (wrist)
			rollRot = rollRot * localRot.inverse();
		}
		else
		{
			// child (bicep)
			rollRot = localRot * rollRot;
		}

		// Weights
		MQuaternion rollRot1 = rollRot;
		MQuaternion rollRot2 = rollRot;
		rollRot1.w *= weight1Value;
		rollRot2.w *= weight2Value;

		// As Euler
		MEulerRotation outputAngles1 = rollRot1.asEulerRotation();
		MEulerRotation outputAngles2 = rollRot2.asEulerRotation();


		// rollRotation1X
		MDataHandle rollRotation1XHandle = data.outputValue(baconRoll::rollRotation1X);
		rollRotation1XHandle.setMAngle(MAngle(outputAngles1.x));
		rollRotation1XHandle.setClean();
		// rollRotation1Y
		MDataHandle rollRotation1YHandle = data.outputValue(baconRoll::rollRotation1Y);
		rollRotation1YHandle.setMAngle(MAngle(outputAngles1.y));
		rollRotation1YHandle.setClean();
		// rollRotation1X
		MDataHandle rollRotation1ZHandle = data.outputValue(baconRoll::rollRotation1Z);
		rollRotation1ZHandle.setMAngle(MAngle(outputAngles1.z));
		rollRotation1ZHandle.setClean();

		// rollRotation2X
		MDataHandle rollRotation2XHandle = data.outputValue(baconRoll::rollRotation2X);
		rollRotation2XHandle.setMAngle(MAngle(outputAngles2.x));
		rollRotation2XHandle.setClean();
		// rollRotation2Y
		MDataHandle rollRotation2YHandle = data.outputValue(baconRoll::rollRotation2Y);
		rollRotation2YHandle.setMAngle(MAngle(outputAngles2.y));
		rollRotation2YHandle.setClean();
		// rollRotation2X
		MDataHandle rollRotation2ZHandle = data.outputValue(baconRoll::rollRotation2Z);
		rollRotation2ZHandle.setMAngle(MAngle(outputAngles2.z));
		rollRotation2ZHandle.setClean();

	}
	else {
		return MS::kUnknownParameter;
	}

	return MS::kSuccess;
}

void* baconRoll::creator()
{
	return new baconRoll();
}


MStatus baconRoll::initialize()
{
	MFnNumericAttribute numAttr;
	MFnUnitAttribute	uAttr;
	MFnMatrixAttribute	matrixAttr;
	MFnEnumAttribute	enumAttr;
	MStatus				stat;

	// INPUTS ---------------------------------------------------------------------

	// weight1
	weight1 = numAttr.create("weight1", "w1", MFnNumericData::kFloat, 0.0);
	numAttr.setDefault(0.66f);
	numAttr.setStorable(true);
	numAttr.setKeyable(true);
	numAttr.setWritable(true);
	stat = addAttribute(weight1);

	// weight2
	weight2 = numAttr.create("weight2", "w2", MFnNumericData::kFloat, 0.0);
	numAttr.setDefault(0.33f);
	numAttr.setStorable(true);
	numAttr.setKeyable(true);
	numAttr.setWritable(true);
	stat = addAttribute(weight2);

	// Input Look Axis 
	rollType = enumAttr.create("rollType", "rt", 1);
	enumAttr.addField("Parent (Wrist)", 0);
	enumAttr.addField("Child (Bicep)", 1);
	enumAttr.setHidden(false);
	enumAttr.setKeyable(true);
	stat = addAttribute(rollType);
	
	// input targetRotationX
	targetRotationX = uAttr.create("targetRotationX", "tarx", uAttr.kAngle, 0.0);
	uAttr.setWritable(true);
	uAttr.setStorable(true);

	// input targetRotationY
	targetRotationY = uAttr.create("targetRotationY", "tary", uAttr.kAngle, 0.0);
	uAttr.setWritable(true);
	uAttr.setStorable(true);

	// input targetRotationZ
	targetRotationZ = uAttr.create("targetRotationZ", "tarz", uAttr.kAngle, 0.0);
	uAttr.setWritable(true);
	uAttr.setStorable(true);

	// input targetRotation
	targetRotation = numAttr.create("targetRotation", "tar", targetRotationX, targetRotationY, targetRotationZ);
	numAttr.setWritable(true);
	numAttr.setStorable(true);
	numAttr.setKeyable(true);
	stat = addAttribute(targetRotation);

	// input targetMatrix
	targetMatrix = matrixAttr.create("targetMatrix", "twTM", matrixAttr.kFloat);
	matrixAttr.setStorable(true);
	matrixAttr.setKeyable(true);
	stat = addAttribute(targetMatrix);

	// input parentMatrix
	parentMatrix = matrixAttr.create("parentMatrix", "pTM", matrixAttr.kFloat);
	matrixAttr.setStorable(true);
	matrixAttr.setKeyable(true);
	stat = addAttribute(parentMatrix);


	// OUTUTS ---------------------------------------------------------------------

	// Output lookAt rotationX
	rollRotation1X = uAttr.create("rollRotation1X", "tr1X", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output lookAt rotationY
	rollRotation1Y = uAttr.create("rollRotation1Y", "tr1Y", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output lookAt rotationZ
	rollRotation1Z = uAttr.create("rollRotation1Z", "tr1Z", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output lookAt rotation
	rollRotation1 = numAttr.create("rollRotation1", "t1", rollRotation1X, rollRotation1Y, rollRotation1Z);
	numAttr.setHidden(false);
	stat = addAttribute(rollRotation1);

	// Output lookAt rotationX
	rollRotation2X = uAttr.create("rollRotation2X", "tr2X", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output lookAt rotationY
	rollRotation2Y = uAttr.create("rollRotation2Y", "tr2Y", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output lookAt rotationZ
	rollRotation2Z = uAttr.create("rollRotation2Z", "tr2Z", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output lookAt rotation
	rollRotation2 = numAttr.create("rollRotation2", "t2", rollRotation2X, rollRotation2Y, rollRotation2Z);
	numAttr.setHidden(false);
	stat = addAttribute(rollRotation2);

	
	//AFFECTS ---------------------------------------------------------------------
	MObject AffectedByMany[] = { rollRotation1, rollRotation1X, rollRotation1Y, rollRotation1Z,
		rollRotation2, rollRotation2X, rollRotation2Y, rollRotation2Z };

	for (MObject& obj : AffectedByMany)
	{
		attributeAffects(targetRotation,	obj);
		attributeAffects(targetRotationX,	obj);
		attributeAffects(targetRotationY,	obj);
		attributeAffects(targetRotationZ,	obj);
		attributeAffects(targetMatrix,	obj);
		attributeAffects(parentMatrix,	obj);
		attributeAffects(weight1,		obj);
		attributeAffects(weight2,		obj);
		attributeAffects(rollType,		obj);
	}

	return MS::kSuccess;
}

MStatus initializePlugin(MObject obj)
{
	MStatus   status;
	MFnPlugin plugin(obj, "Bacon-Strip.com", "1.0", "Any");

	status = plugin.registerNode("baconRoll", baconRoll::id, baconRoll::creator,
		baconRoll::initialize);
	if (!status) {
		status.perror("registerNode");
		return status;
	}

	return status;
}

MStatus uninitializePlugin(MObject obj)
{
	MStatus   status;
	MFnPlugin plugin(obj);

	status = plugin.deregisterNode(baconRoll::id);
	if (!status) {
		status.perror("deregisterNode");
		return status;
	}

	return status;
}
