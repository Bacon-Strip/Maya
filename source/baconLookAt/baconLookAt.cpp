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
#include <maya/MQuaternion.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MDoubleArray.h>
#include <maya/MScriptUtil.h>

 
class baconLookAt : public MPxNode
{
public:
						baconLookAt();
	virtual				~baconLookAt(); 

	virtual MStatus		compute( const MPlug& plug, MDataBlock& data );

	static  void*		creator();
	static  MStatus		initialize();

public:
	static	MTypeId		id;

	// inputs
	static  MObject		targetWorldMatrix;
	static  MObject		parentWorldMatrix;
	static  MObject		referenceWorldMatrix;
	static  MObject		jointOrient;
	static  MObject		jointOrientX;
	static  MObject		jointOrientY;
	static  MObject		jointOrientZ;

	static  MObject		aimAxis;
	static  MObject		aimAxisFlip;
	static  MObject		upAxis;
	static  MObject		upAxisFlip;
	static  MObject		isMirror;
	static  MObject		alternateSolution;
	static  MObject		useNearestMethod;
	static  MObject		localPosition;
	static  MObject		weight;
	static  MObject		referencePosOffset;



	// ouputs
	static  MObject		lookAtRotation;
	static  MObject		lookAtRotationX;
	static  MObject		lookAtRotationY;
	static  MObject		lookAtRotationZ;

};

MTypeId     baconLookAt::id( 0x0012a951 );
MObject		baconLookAt::targetWorldMatrix;
MObject		baconLookAt::parentWorldMatrix;
MObject		baconLookAt::referenceWorldMatrix;
MObject		baconLookAt::jointOrient;
MObject		baconLookAt::jointOrientX;
MObject		baconLookAt::jointOrientY;
MObject		baconLookAt::jointOrientZ;
MObject		baconLookAt::lookAtRotation;
MObject		baconLookAt::lookAtRotationX;
MObject		baconLookAt::lookAtRotationY;
MObject		baconLookAt::lookAtRotationZ;

MObject		baconLookAt::aimAxis;
MObject		baconLookAt::aimAxisFlip;
MObject		baconLookAt::upAxis;
MObject		baconLookAt::upAxisFlip;
MObject		baconLookAt::isMirror;
MObject		baconLookAt::alternateSolution;
MObject		baconLookAt::useNearestMethod;
MObject		baconLookAt::localPosition;
MObject		baconLookAt::weight;
MObject		baconLookAt::referencePosOffset;

baconLookAt::baconLookAt() {}
baconLookAt::~baconLookAt() {}

MMatrix setRow( MMatrix matrix, MVector newVector, const int row)
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

MVector getRow(MMatrix TM, int row)
{
	return (MVector(TM[row][0], TM[row][1], TM[row][2]));
}

MMatrix mirrorMatrix(MMatrix inTM)
{
	MMatrix returnTM = MMatrix();
	returnTM = setRow(returnTM, (MVector(inTM[0][0] * -1.0, inTM[0][1] * -1.0, inTM[0][2] * -1.0)), 0);
	returnTM = setRow(returnTM, (MVector(inTM[1][0] * -1.0, inTM[1][1] * -1.0, inTM[1][2] * -1.0)), 1);
	returnTM = setRow(returnTM, (MVector(inTM[2][0] * -1.0, inTM[2][1] * -1.0, inTM[2][2] * -1.0)), 2);
	return returnTM;
}



MStatus baconLookAt::compute( const MPlug& plug, MDataBlock& data )
{
	
	MStatus returnStatus;
 
	if
	( 
		plug == lookAtRotation ||	plug == lookAtRotationX  ||  plug == lookAtRotationY  ||  plug == lookAtRotationZ
	)
	{
		// Handles and Values
		MDataHandle targetWorldMatrixHandle = data.inputValue(targetWorldMatrix, &returnStatus);
		MFloatMatrix tFM(targetWorldMatrixHandle.asFloatMatrix());
		MDataHandle parentWorldMatrixHandle = data.inputValue(parentWorldMatrix, &returnStatus);
		MFloatMatrix pFM = parentWorldMatrixHandle.asFloatMatrix();
		MDataHandle referenceWorldMatrixHandle = data.inputValue(referenceWorldMatrix, &returnStatus);
		MFloatMatrix unFM = referenceWorldMatrixHandle.asFloatMatrix();

		MDataHandle jointOrientXHandle = data.inputValue(jointOrientX, &returnStatus);
		MAngle jointOrientXValue = jointOrientXHandle.asAngle();
		MDataHandle jointOrientYHandle = data.inputValue(jointOrientY, &returnStatus);
		MAngle jointOrientYValue = jointOrientYHandle.asAngle();
		MDataHandle jointOrientZHandle = data.inputValue(jointOrientZ, &returnStatus);
		MAngle jointOrientZValue = jointOrientZHandle.asAngle();

		MDataHandle weightHandle = data.inputValue(weight, &returnStatus);
		double weightValue = weightHandle.asDouble();
		
		MDataHandle referencePosOffsetHandle = data.inputValue(referencePosOffset, &returnStatus);
		MVector referencePosOffsetValue = referencePosOffsetHandle.asFloatVector();



		MDataHandle aimAxisHandle = data.inputValue(aimAxis, &returnStatus);
		short aimAxisValue = aimAxisHandle.asShort();
		MDataHandle upAxisHandle = data.inputValue(upAxis, &returnStatus);
		short upAxisValue = upAxisHandle.asShort();
		MDataHandle aimAxisFlipHandle = data.inputValue(aimAxisFlip, &returnStatus);
		bool aimAxisFlipValue = aimAxisFlipHandle.asBool();
		MDataHandle upAxisFlipHandle = data.inputValue(upAxisFlip, &returnStatus);
		bool upAxisFlipValue = upAxisFlipHandle.asBool();
		MDataHandle alternateSolutionHandle = data.inputValue(alternateSolution, &returnStatus);
		bool alternateSolutionValue = alternateSolutionHandle.asBool();
		MDataHandle useNearestMethodHandle = data.inputValue(useNearestMethod, &returnStatus);
		bool useNearestMethodValue = useNearestMethodHandle.asBool();
		MDataHandle isMirrorHandle = data.inputValue(isMirror, &returnStatus);
		bool isMirrorValue = isMirrorHandle.asBool();

		MDataHandle localPositionHandle = data.inputValue(localPosition, &returnStatus);
		MVector localPositionValue = localPositionHandle.asFloatVector();


		// Calculation
		MMatrix targetTM = FloatMatrixToMatrix(tFM);
		MMatrix parentTM = FloatMatrixToMatrix(pFM);
		MMatrix referenceTM = FloatMatrixToMatrix(unFM);
		MMatrix localTM;


		// Aim Direction
		MVector targetPos(targetTM[3][0], targetTM[3][1], targetTM[3][2]);
		MMatrix localPositionTM = matrix3(MVector(), MVector(), MVector(), localPositionValue);
		MMatrix worldPositionTM = localPositionTM * parentTM;
		MVector originPos(worldPositionTM[3][0], worldPositionTM[3][1], worldPositionTM[3][2]);
		MVector aimDir = targetPos - originPos;
		aimDir.normalize();
		if (aimAxisFlipValue) aimDir.operator*=(-1.0);

		if (useNearestMethodValue)
		{
			MVector localAimDir = aimDir * parentTM.inverse();
			localAimDir.normalize();
			MVector testAxisDir;
			if (aimAxisValue == 0) testAxisDir = MVector(1, 0, 0);
			if (aimAxisValue == 1) testAxisDir = MVector(0, 1, 0);
			if (aimAxisValue == 2) testAxisDir = MVector(0, 0, 1);
			if (upAxisFlipValue) testAxisDir.operator*=(-1.0);

			MVector quatAxis = testAxisDir ^ localAimDir;
			quatAxis.normalize();
			double quatAngle = acos(testAxisDir * localAimDir);
			MQuaternion quatResult(quatAngle, quatAxis);
			localTM = quatResult.asMatrix();

		}
		else
		{
			// UP Direction
			MVector upDir;
			if (upAxisValue == 0) upDir = MVector(targetTM[0][0], targetTM[0][1], targetTM[0][2]);
			if (upAxisValue == 1) upDir = MVector(targetTM[1][0], targetTM[1][1], targetTM[1][2]);
			if (upAxisValue == 2) upDir = MVector(targetTM[2][0], targetTM[2][1], targetTM[2][2]);
			if (upAxisValue == 3) upDir = MVector(parentTM[0][0], parentTM[0][1], parentTM[0][2]);
			if (upAxisValue == 4) upDir = MVector(parentTM[1][0], parentTM[1][1], parentTM[1][2]);
			if (upAxisValue == 5) upDir = MVector(parentTM[2][0], parentTM[2][1], parentTM[2][2]);
			if (upAxisValue == 6) upDir = MVector(referenceTM[0][0], referenceTM[0][1], referenceTM[0][2]);
			if (upAxisValue == 7) upDir = MVector(referenceTM[1][0], referenceTM[1][1], referenceTM[1][2]);
			if (upAxisValue == 8) upDir = MVector(referenceTM[2][0], referenceTM[2][1], referenceTM[2][2]);
			if (upAxisValue == 9)
			{
				MMatrix refOffset = transMatrix(referencePosOffsetValue) * referenceTM;
				upDir = MVector(refOffset[3][0], refOffset[3][1], refOffset[3][2]) - originPos;
			}

			if (upAxisFlipValue)  upDir.operator*=(-1.0);
			upDir.normalize();

			// Solving Matrix
			MVector crossDir;
			MVector orthoUpDir;
			if (alternateSolutionValue == true)
			{
				orthoUpDir = aimDir.operator^(upDir);
				orthoUpDir.normalize();
				crossDir = orthoUpDir.operator^(aimDir);
				crossDir.normalize();
			}
			else
			{
				crossDir = aimDir.operator^(upDir);
				crossDir.normalize();
				orthoUpDir = crossDir.operator^(aimDir);
				orthoUpDir.normalize();
			}


			MMatrix worldTM;
			if (aimAxisValue == 0) worldTM = matrix3(aimDir, orthoUpDir, crossDir, MVector());
			if (aimAxisValue == 1) worldTM = matrix3(crossDir, aimDir, orthoUpDir, MVector());
			if (aimAxisValue == 2) worldTM = matrix3(orthoUpDir, crossDir, aimDir, MVector());
			localTM = worldTM * parentTM.inverse();

		}

		if (isMirrorValue) 
		{ 
			localTM = mirrorMatrix(localTM); 
		}

		MMatrix jointOrientTM = MEulerRotation(jointOrientXValue.value(), jointOrientYValue.value(),
			jointOrientZValue.value()).asMatrix();
		MMatrix outputTM = localTM * jointOrientTM.inverse();
		MTransformationMatrix ouputTransformationMatrix = MTransformationMatrix(outputTM);


		//MEulerRotation outputAngles = ouputTransformationMatrix.eulerRotation();
		MQuaternion outQuat = slerp(MQuaternion(), ouputTransformationMatrix.rotation(), weightValue);
		MEulerRotation outputAngles = outQuat.asEulerRotation();


		// Set OutPut Values
		if( returnStatus != MS::kSuccess )
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
		}
	
	} else {
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
	
	// input weight
	weight = numAttr.create("weight", "w", MFnNumericData::kDouble, 1.0);
	numAttr.setWritable(true);
	numAttr.setStorable(true);
	numAttr.setKeyable(true);
	numAttr.setMin(0.0);
	numAttr.setMax(1.0);
	stat = addAttribute(weight);


	// aimAxis
	aimAxis = enumAttr.create("aimAxis", "aima", 0);
	enumAttr.addField("X", 0);
	enumAttr.addField("Y", 1);
	enumAttr.addField("Z", 2);
	enumAttr.setHidden(false);
	enumAttr.setKeyable(true);
	stat = addAttribute(aimAxis);
	if (!stat) { stat.perror("addAttribute"); return stat; }

	// upAxis
	upAxis = enumAttr.create("upAxis", "upa", 0);
	enumAttr.addField("Target X", 0);
	enumAttr.addField("Target Y", 1);
	enumAttr.addField("Target Z", 2);
	enumAttr.addField("Parent X", 3);
	enumAttr.addField("Parent Y", 4);
	enumAttr.addField("Parent Z", 5);
	enumAttr.addField("Reference X", 6);
	enumAttr.addField("Reference Y", 7);
	enumAttr.addField("Reference Z", 8);
	enumAttr.addField("Reference Pos", 9);
	enumAttr.setHidden(false);
	enumAttr.setKeyable(true);
	stat = addAttribute(upAxis);
	if (!stat) { stat.perror("addAttribute"); return stat; }

	// ReferencePosOffset
	referencePosOffset = numAttr.createPoint("ReferencePosOffset", "RefPosOff");
	stat = addAttribute(referencePosOffset);


	// aimAxisFlip
	aimAxisFlip = numAttr.create("aimAxisFlip", "aaf", MFnNumericData::kBoolean);
	numAttr.setDefault(false);
	numAttr.setKeyable(true);
	numAttr.setReadable(true);
	numAttr.setWritable(true);
	numAttr.setStorable(true);
	stat = addAttribute(aimAxisFlip);

	// upAxisFlip
	upAxisFlip = numAttr.create("upAxisFlip", "upf", MFnNumericData::kBoolean);
	numAttr.setDefault(false);
	numAttr.setKeyable(true);
	numAttr.setReadable(true);
	numAttr.setWritable(true);
	numAttr.setStorable(true);
	stat = addAttribute(upAxisFlip);

	// alternateSolution
	alternateSolution = numAttr.create("alternateSolution", "uas", MFnNumericData::kBoolean);
	numAttr.setDefault(false);
	numAttr.setKeyable(true);
	numAttr.setReadable(true);
	numAttr.setWritable(true);
	numAttr.setStorable(true);
	stat = addAttribute(alternateSolution);

	// useNearestMethod
	useNearestMethod = numAttr.create("useNearestMethod", "useSM", MFnNumericData::kBoolean);
	numAttr.setDefault(false);
	numAttr.setKeyable(true);
	numAttr.setReadable(true);
	numAttr.setWritable(true);
	numAttr.setStorable(true);
	stat = addAttribute(useNearestMethod);

	// isMirror
	isMirror = numAttr.create("isMirror", "imirr", MFnNumericData::kBoolean);
	numAttr.setDefault(false);
	numAttr.setKeyable(true);
	numAttr.setReadable(true);
	numAttr.setWritable(true);
	numAttr.setStorable(true);
	stat = addAttribute(isMirror);


	// localPosition
	localPosition = numAttr.createPoint("localPosition", "lpos");
	numAttr.setStorable(true);
	numAttr.setWritable(true);
	numAttr.setKeyable(true);
	stat = addAttribute(localPosition);


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

	// input targetWorldMatrix
	targetWorldMatrix = matrixAttr.create("targetWorldMatrix", "twTM", matrixAttr.kFloat);
	matrixAttr.setStorable(true);
	matrixAttr.setKeyable(true);
	stat = addAttribute(targetWorldMatrix);

	// input parentWorldMatrix
	parentWorldMatrix = matrixAttr.create("parentWorldMatrix", "piTM", matrixAttr.kFloat);
	matrixAttr.setStorable(true);
	matrixAttr.setKeyable(true);
	stat = addAttribute(parentWorldMatrix);

	// input referenceWorldMatrix
	referenceWorldMatrix = matrixAttr.create("referenceWorldMatrix", "refTM", matrixAttr.kFloat);
	matrixAttr.setStorable(true);
	matrixAttr.setKeyable(true);
	stat = addAttribute(referenceWorldMatrix);




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

	//AFFECTS ---------------------------------------------------------------------
	MObject AffectedByMany[] = { lookAtRotation, lookAtRotationX, lookAtRotationY, lookAtRotationZ };

	for (MObject& obj : AffectedByMany)
	{
		attributeAffects(jointOrient,		obj);
		attributeAffects(jointOrientX,		obj);
		attributeAffects(jointOrientY,		obj);
		attributeAffects(jointOrientZ,		obj);
		attributeAffects(targetWorldMatrix,	obj);
		attributeAffects(parentWorldMatrix, obj);
		attributeAffects(referenceWorldMatrix, obj);
		attributeAffects(aimAxis,			obj);
		attributeAffects(aimAxisFlip,		obj);
		attributeAffects(upAxis,			obj);
		attributeAffects(upAxisFlip,		obj);
		attributeAffects(localPosition,		obj);
		attributeAffects(alternateSolution, obj);
		attributeAffects(useNearestMethod,  obj);
		attributeAffects(weight, 			obj);
		attributeAffects(isMirror, 			obj);
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
