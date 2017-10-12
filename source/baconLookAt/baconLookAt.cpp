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
// Produces the dependency graph node "baconLookAt".
// This plug-in is part of a rigging package known as The "Bacon-Strip Rig"
// Bacon-Strip ID Block is : 0x0012a940 - 0x0012aa3f
// Registered for Bacon-Strip[Luis Alonso]
//
// ID: 
// 0x0012a951
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

#include <maya/MFloatPoint.h>
#include <maya/MFloatMatrix.h>
#include <maya/MEulerRotation.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MDoubleArray.h>
#include <maya/MScriptUtil.h>


class baconLookAt : public MPxNode
{
public:
	baconLookAt();
	virtual				~baconLookAt();

	virtual MStatus		compute(const MPlug& plug, MDataBlock& data);

	static  void*		creator();
	static  MStatus		initialize();

public:
	static	MTypeId		id;

	// inputs
	static  MObject		lookAxis;
	static  MObject		upAxis;
	static  MObject		weight;
	static  MObject		targetMatrix;
	static  MObject		parentMatrix;
	static  MObject		jointPosition;
	static  MObject		jointOrient;
	static  MObject		jointOrientX;
	static  MObject		jointOrientY;
	static  MObject		jointOrientZ;

	// ouputs
	static  MObject		lookAtRotation;
	static  MObject		lookAtRotationX;
	static  MObject		lookAtRotationY;
	static  MObject		lookAtRotationZ;
	static  MObject		debugOut;

};

MTypeId     baconLookAt::id(0x0012a951);
MObject     baconLookAt::weight;
MObject		baconLookAt::lookAxis;
MObject		baconLookAt::upAxis;
MObject		baconLookAt::targetMatrix;
MObject		baconLookAt::parentMatrix;
MObject		baconLookAt::jointPosition;
MObject		baconLookAt::jointOrient;
MObject		baconLookAt::jointOrientX;
MObject		baconLookAt::jointOrientY;
MObject		baconLookAt::jointOrientZ;
MObject		baconLookAt::lookAtRotation;
MObject		baconLookAt::lookAtRotationX;
MObject		baconLookAt::lookAtRotationY;
MObject		baconLookAt::lookAtRotationZ;
MObject		baconLookAt::debugOut;

baconLookAt::baconLookAt() {}
baconLookAt::~baconLookAt() {}

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



MStatus baconLookAt::compute(const MPlug& plug, MDataBlock& data)
{

	MStatus returnStatus;

	if
		(
			plug == lookAtRotation	||  plug == lookAtRotationX || plug == lookAtRotationY || 
			plug == lookAtRotationZ ||  plug == debugOut
		)
	{
		// Handles and Values
		MDataHandle lookAxisHandle = data.inputValue(lookAxis, &returnStatus);
		unsigned int lookAxisValue = lookAxisHandle.asInt();

		MDataHandle upAxisHandle = data.inputValue(upAxis, &returnStatus);
		unsigned int upAxisValue = upAxisHandle.asInt();

		MDataHandle targetMatrixHandle = data.inputValue(targetMatrix, &returnStatus);
		MFloatMatrix tFM(targetMatrixHandle.asFloatMatrix());

		MDataHandle parentMatrixHandle = data.inputValue(parentMatrix, &returnStatus);
		MFloatMatrix pFM = parentMatrixHandle.asFloatMatrix();

		MDataHandle jointPositionHandle = data.inputValue(jointPosition, &returnStatus);
		MVector jointPositionValue = jointPositionHandle.asFloatVector();

		MDataHandle jointOrientXHandle = data.inputValue(jointOrientX, &returnStatus);
		MAngle jointOrientXValue = jointOrientXHandle.asAngle();
		MDataHandle jointOrientYHandle = data.inputValue(jointOrientY, &returnStatus);
		MAngle jointOrientYValue = jointOrientYHandle.asAngle();
		MDataHandle jointOrientZHandle = data.inputValue(jointOrientZ, &returnStatus);
		MAngle jointOrientZValue = jointOrientZHandle.asAngle();

		
		// Calculation
		//MMatrix tTM = FloatMatrixToMatrix(tFM);

		double debugOutValue(2.0);
		if (lookAxisValue == 1)
		{
			debugOutValue = 3.0;
		}



		if (upAxisValue == lookAxisValue)
		{
			lookAxisValue = 2 - upAxisValue;
		}

		MMatrix pTM(FloatMatrixToMatrix(pFM));
		MMatrix tTM(FloatMatrixToMatrix(tFM));

		//MVector rootPos(pFM[3][0], pFM[3][1], pFM[3][2]);
		MVector upPos(tFM[3][0], tFM[3][1], tFM[3][2]);
		MMatrix jointTM(transMatrix(jointPositionValue) * pTM);
		MVector rootPos(jointTM[3][0], jointTM[3][1], jointTM[3][2]);

		MVector lookVector = upPos - rootPos;
		lookVector.normalize();


		/*
		MVector upVector(tFM[upAxisValue][0], tFM[upAxisValue][1], tFM[upAxisValue][2]);
		upVector.normalize();
		MVector crossVector(0,0,1);
		MVector orthoUpVector(0,1,0);

		if (lookAxisValue < upAxisValue || (lookAxisValue - upAxisValue) == 2)
		{
			// 0,1
			// 1,2
			// 2,0

			// look = 0, up = 1
			// look = 1, up = 2
			// look = 2, up = 0
			crossVector = lookVector ^ upVector;
			crossVector.normalize();
			orthoUpVector = crossVector ^ lookVector;
			orthoUpVector.normalize();
		}
		else
		{
			// 1,0
			// 2,1
			// 0,2

			crossVector = upVector ^ lookVector;
			crossVector.normalize();
			orthoUpVector = lookVector ^ crossVector;
			orthoUpVector.normalize();
		}

		int crossAxisValue = 3 - upAxisValue + lookAxisValue;
		MMatrix worldTM = MMatrix();
		worldTM = setRow(worldTM, lookVector,		lookAxisValue);
		worldTM = setRow(worldTM, orthoUpVector,	upAxisValue);
		worldTM = setRow(worldTM, crossVector,		crossAxisValue);
		*/

		// Temp Hardcode ------

		MVector upVector(tFM[0][0], tFM[0][1], tFM[0][2]);
		upVector.normalize();
		MVector crossVector(upVector ^ lookVector);
		crossVector.normalize();
		MVector orthoUpVector(lookVector ^ crossVector);
		orthoUpVector.normalize();
		MMatrix worldTM = matrix3(orthoUpVector, lookVector, crossVector, rootPos);

		//---------------------


		MMatrix localTM = worldTM * pTM.inverse();
		MMatrix jointOrientTM = MEulerRotation(jointOrientXValue.value(), jointOrientYValue.value(),
			jointOrientZValue.value()).asMatrix();
		MMatrix outputTM = localTM * jointOrientTM.inverse();
		MTransformationMatrix ouputTransformationMatrix = MTransformationMatrix(outputTM);
		MEulerRotation outputAngles = ouputTransformationMatrix.eulerRotation();

		//MStatus setUnit(MAngle::Unit kDegrees);
		//MEulerRotation outputAngles(20.0, 5.0, 10.0, MEulerRotation::kXYZ);

		// Set OutPut Values
		if (returnStatus != MS::kSuccess)
			cerr << "ERROR getting data" << endl;
		else
		{
			// lookAtRotationX
			MDataHandle lookAtRotationXHandle = data.outputValue(baconLookAt::lookAtRotationX);
			lookAtRotationXHandle.setMAngle(MAngle(outputAngles.x));
			lookAtRotationXHandle.setClean();
			// lookAtRotationY
			MDataHandle lookAtRotationYHandle = data.outputValue(baconLookAt::lookAtRotationY);
			lookAtRotationYHandle.setMAngle(MAngle(outputAngles.y));
			lookAtRotationYHandle.setClean();
			// lookAtRotationX
			MDataHandle lookAtRotationZHandle = data.outputValue(baconLookAt::lookAtRotationZ);
			lookAtRotationZHandle.setMAngle(MAngle(outputAngles.z));
			lookAtRotationZHandle.setClean();

			// Debug Output
			MDataHandle debugOutHandle = data.inputValue(debugOut, &returnStatus);
			debugOutHandle.setDouble(debugOutValue);
			debugOutHandle.setClean();



		}
	}
	else {
		return MS::kUnknownParameter;
	}

	return MS::kSuccess;
}

void* baconLookAt::creator()
{
	return new baconLookAt();
}


MStatus baconLookAt::initialize()
{
	MFnNumericAttribute numAttr;
	MFnUnitAttribute	uAttr;
	MFnMatrixAttribute	matrixAttr;
	MFnEnumAttribute	enumAttr;
	MStatus				stat;

	// INPUTS ---------------------------------------------------------------------

	// weight
	weight = numAttr.create("weight", "w", MFnNumericData::kFloat, 0.0);
	numAttr.setStorable(true);
	numAttr.setKeyable(true);
	numAttr.setWritable(true);

	// Input Look Axis 
	lookAxis = enumAttr.create("lookAxis", "la", 1);
	enumAttr.addField("X Axis", 0);
	enumAttr.addField("Y Axis", 1);
	enumAttr.addField("Z Axis", 2);
	enumAttr.setHidden(false);
	enumAttr.setKeyable(true);
	stat = addAttribute(lookAxis);
	
	// Input Up Axis 
	upAxis = enumAttr.create("rollAxis", "ra", 0);
	enumAttr.addField("X Axis", 0);
	enumAttr.addField("Y Axis", 1);
	enumAttr.addField("Z Axis", 2);
	enumAttr.setHidden(false);
	enumAttr.setKeyable(true);
	stat = addAttribute(upAxis);


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

	// input joint position
	jointPosition = numAttr.createPoint("jointPosition", "jp");
	numAttr.setWritable(true);
	numAttr.setStorable(true);
	numAttr.setKeyable(true);
	stat = addAttribute(jointPosition);
	


	// OUTUTS ---------------------------------------------------------------------

	// Output lookAt rotationX
	lookAtRotationX = uAttr.create("lookAtRotationX", "arotX", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output lookAt rotationY
	lookAtRotationY = uAttr.create("lookAtRotationY", "arotY", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output lookAt rotationZ
	lookAtRotationZ = uAttr.create("lookAtRotationZ", "arotZ", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output lookAt rotation
	lookAtRotation = numAttr.create("lookAtRotation", "arot", lookAtRotationX, lookAtRotationY, lookAtRotationZ);
	numAttr.setHidden(false);
	stat = addAttribute(lookAtRotation);

	// Output debugOut
	debugOut = uAttr.create("debugOut", "do", uAttr.kDistance, 0.0);
	uAttr.setHidden(false);
	uAttr.setWritable(false);
	uAttr.setStorable(true);
	uAttr.setKeyable(true);
	stat = addAttribute(debugOut);


	

	//AFFECTS ---------------------------------------------------------------------
	MObject AffectedByMany[] = { lookAtRotation, lookAtRotationX, lookAtRotationY, lookAtRotationZ, debugOut };

	for (MObject& obj : AffectedByMany)
	{
		attributeAffects(jointOrient,		obj);
		attributeAffects(jointOrientX,		obj);
		attributeAffects(jointOrientY,		obj);
		attributeAffects(jointOrientZ,		obj);
		attributeAffects(targetMatrix,		obj);
		attributeAffects(parentMatrix,		obj);
		attributeAffects(lookAxis,			obj);
		attributeAffects(upAxis,			obj);
	}

	return MS::kSuccess;
}

MStatus initializePlugin(MObject obj)
{
	MStatus   status;
	MFnPlugin plugin(obj, "Bacon-Strip.com", "1.0", "Any");

	status = plugin.registerNode("baconLookAt", baconLookAt::id, baconLookAt::creator,
		baconLookAt::initialize);
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

	status = plugin.deregisterNode(baconLookAt::id);
	if (!status) {
		status.perror("deregisterNode");
		return status;
	}

	return status;
}
