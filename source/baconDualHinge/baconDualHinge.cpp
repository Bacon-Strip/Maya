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
// Produces the dependency graph node "baconDualHinge".
// This plug-in is part of a rigging package known as The "Bacon-Strip Rig"
// Bacon-Strip ID Block is : 0x0012a940 - 0x0012aa3f
// Registered for Bacon-Strip[Luis Alonso]
//
// ID: 
// 0x0012a960
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


class baconDualHinge : public MPxNode
{
public:
	baconDualHinge();
	virtual				~baconDualHinge();

	virtual MStatus		compute(const MPlug& plug, MDataBlock& data);

	static  void*		creator();
	static  MStatus		initialize();

public:
	static	MTypeId		id;

	// inputs
	static  MObject		bone1WorldTM;
	static  MObject		bone2WorldTM;
	static  MObject		inverseParentTM;
	static  MObject		bone1Length;
	static  MObject		bone2Length;
	static  MObject		hingeLength;

	// ouputs
	static  MObject		output1Rotation;
	static  MObject		output1RotationX;
	static  MObject		output1RotationY;
	static  MObject		output1RotationZ;
	static  MObject		output2Rotation;
	static  MObject		output2RotationX;
	static  MObject		output2RotationY;
	static  MObject		output2RotationZ;
	static  MObject		output3Rotation;
	static  MObject		output3RotationX;
	static  MObject		output3RotationY;
	static  MObject		output3RotationZ;

};

MTypeId     baconDualHinge::id(0x0012a960);
MObject		baconDualHinge::bone1WorldTM;
MObject		baconDualHinge::bone2WorldTM;
MObject		baconDualHinge::inverseParentTM;
MObject		baconDualHinge::bone1Length;
MObject		baconDualHinge::bone2Length;
MObject		baconDualHinge::hingeLength;


MObject		baconDualHinge::output1Rotation;
MObject		baconDualHinge::output1RotationX;
MObject		baconDualHinge::output1RotationY;
MObject		baconDualHinge::output1RotationZ;
MObject		baconDualHinge::output2Rotation;
MObject		baconDualHinge::output2RotationX;
MObject		baconDualHinge::output2RotationY;
MObject		baconDualHinge::output2RotationZ;
MObject		baconDualHinge::output3Rotation;
MObject		baconDualHinge::output3RotationX;
MObject		baconDualHinge::output3RotationY;
MObject		baconDualHinge::output3RotationZ;

baconDualHinge::baconDualHinge() {}
baconDualHinge::~baconDualHinge() {}

MMatrix setRow(MMatrix matrix, MVector newVector, const int row)
{
	MMatrix returnTM = matrix;
	returnTM[row][0] = newVector[0];
	returnTM[row][1] = newVector[1];
	returnTM[row][2] = newVector[2];
	return returnTM;
}

MVector getRow(MMatrix matrix, int row)
{
	return ( MVector(matrix[row][0], matrix[row][1], matrix[row][2]) );
}

bool eulerValueCheck(MEulerRotation &inRot )
{
	if (isnan(inRot.x)) { inRot.x = 0.0; }
	if (isnan(inRot.y)) { inRot.y = 0.0; }
	if (isnan(inRot.z)) { inRot.z = 0.0; }
	return true;
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

MAngle cosineLaw(float A, float B, float C)
{
	float A2(pow(A, 2));
	float B2(pow(B, 2));
	float C2(pow(C, 2));
	float AB(2.0f * A * B);
	MAngle angle(0);
	try
	{
		angle = acos((A2 + B2 - C2) / AB);
	}
	catch (...)
	{
		angle = MAngle(0);
	}
	return angle;
}



MStatus baconDualHinge::compute(const MPlug& plug, MDataBlock& data)
{
	MStatus returnStatus;
	if (
			plug == output1Rotation			|| plug == output2Rotation	|| plug == output3Rotation ||
			plug == output1RotationX		|| plug == output1RotationY	|| plug == output1RotationZ ||
			plug == output2RotationX		|| plug == output2RotationY	|| plug == output2RotationZ ||
			plug == output3RotationX		|| plug == output3RotationY	|| plug == output3RotationZ
		)
	{
		// Handles and Values
		MDataHandle inverseParentTMHandle = data.inputValue(inverseParentTM, &returnStatus);
		MFloatMatrix inverseParentFM(inverseParentTMHandle.asFloatMatrix());

		MDataHandle bone1WorldTMHandle = data.inputValue(bone1WorldTM, &returnStatus);
		MFloatMatrix bone1FM(bone1WorldTMHandle.asFloatMatrix());

		MDataHandle bone2WorldTMHandle = data.inputValue(bone2WorldTM, &returnStatus);
		MFloatMatrix bone2FM = bone2WorldTMHandle.asFloatMatrix();

		MDataHandle bone1LengthHandle = data.inputValue(bone1Length, &returnStatus);
		float bone1LengthValue = bone1LengthHandle.asFloat();

		MDataHandle bone2LengthHandle = data.inputValue(bone2Length, &returnStatus);
		float bone2LengthValue = bone2LengthHandle.asFloat();

		MDataHandle hingeLengthHandle = data.inputValue(hingeLength, &returnStatus);
		float hingeLengthValue = hingeLengthHandle.asFloat();

		// Calculation /////////////////////////////////////////////

		// Inputs Positions
		MMatrix invParentTM(FloatMatrixToMatrix(inverseParentFM));
		MMatrix bone1TM(FloatMatrixToMatrix(bone1FM));
		MMatrix bone2TM(FloatMatrixToMatrix(bone2FM));
		MVector bone1Dir(bone1TM[0][0], bone1TM[0][1], bone1TM[0][2]);
		MVector bone2Dir(bone2TM[0][0], bone2TM[0][1], bone2TM[0][2]);
		MVector bone1Pos(bone1TM[3][0], bone1TM[3][1], bone1TM[3][2]);
		MVector bone2Pos(bone2TM[3][0], bone2TM[3][1], bone2TM[3][2]);
		MVector goalPos(bone2Pos + (bone2LengthValue * bone2Dir));
		float halfHinge(hingeLengthValue / 2.0f);
		float fracBone1Length(bone1LengthValue - halfHinge);
		float fracBone2Length(bone2LengthValue - halfHinge);
		MVector startToGoalVector = bone1Pos - goalPos;
		float startToGoalLength = float(startToGoalVector.length());
		float maxLength(bone1LengthValue + bone2LengthValue);
		MEulerRotation output1Angles(0, 0, 0);
		MEulerRotation output2Angles(0, 0, 0);
		MEulerRotation output3Angles(0, 0, 0);

		if (startToGoalLength < maxLength)
		{
			// output 1 Rotation
			MVector relationalPointA(bone2Pos + (-1.0f * halfHinge * bone1Dir));
			MVector relationalPointB(goalPos + (-1.0f * fracBone2Length * bone2Dir));
			MVector relationalVector(relationalPointA - relationalPointB);
			float relationalDifference(hingeLengthValue - float(relationalVector.length()));
			MAngle relationalAngle = cosineLaw(fracBone1Length, fracBone1Length, relationalDifference);
			MMatrix relationalZTM = MEulerRotation(0.0, 0.0, relationalAngle.value()).asMatrix();
			MMatrix output1WorldTM = (relationalZTM * bone1TM);
			MMatrix output1LocalTM = output1WorldTM * invParentTM;
			MTransformationMatrix output1TransformationMatrix = MTransformationMatrix(output1LocalTM);
			output1Angles = output1TransformationMatrix.eulerRotation();

			// output 2 Rotation
			MVector output1Dir(output1WorldTM[0][0], output1WorldTM[0][1], output1WorldTM[0][2]);
			MVector hingePos(bone1Pos + (output1Dir * fracBone1Length));
			MVector hingeToGoalVector = hingePos - goalPos;
			float hingeToGoalLength = float(hingeToGoalVector.length());
			MAngle alphaRotation(cosineLaw(hingeToGoalLength, hingeLengthValue, fracBone2Length));
			MAngle betaRotation(cosineLaw(hingeToGoalLength, fracBone1Length, startToGoalLength));
			MAngle hingeAngle(M_PI - alphaRotation.value() - betaRotation.value());
			output2Angles = MEulerRotation(0.0, 0.0, -1.0f * hingeAngle.value());

			// output 3 Rotation
			MMatrix output2WorldTM = output2Angles.asMatrix() * output1WorldTM;
			MVector output2Dir(output2WorldTM[0][0], output2WorldTM[0][1], output2WorldTM[0][2]);
			MVector bone2UpDir(bone2TM[2][0], bone2TM[2][1], bone2TM[2][2]);
			MVector output3Pos = hingePos + (output2Dir * hingeLengthValue);
			MVector output3AxisX = (goalPos - output3Pos).normal();
			MVector output3AxisY = (bone2UpDir ^ output3AxisX).normal();
			MVector output3AxisZ = (output3AxisX ^ output3AxisY).normal();
			MMatrix output3WorldTM = matrix3(output3AxisX, output3AxisY, output3AxisZ, output3Pos);
			MMatrix output3localTM = output3WorldTM * output2WorldTM.inverse(); //row3 will not need to be accurate.
			MTransformationMatrix output3TransformationMatrix = MTransformationMatrix(output3localTM);
			output3Angles = output3TransformationMatrix.eulerRotation();
		}
		else
		{
			MMatrix output1LocalTM = bone1TM * invParentTM;
			MTransformationMatrix output1TransformationMatrix = MTransformationMatrix(output1LocalTM);
			output1Angles = output1TransformationMatrix.eulerRotation();
		}

		eulerValueCheck(output1Angles);
		eulerValueCheck(output2Angles);
		eulerValueCheck(output3Angles);

		// OUTPUT
		if (returnStatus != MS::kSuccess)
			cerr << "ERROR getting data" << endl;
		else
		{
			// output1RotationX
			MDataHandle output1RotationXHandle = data.outputValue(baconDualHinge::output1RotationX);
			output1RotationXHandle.setMAngle(MAngle(output1Angles.x));
			output1RotationXHandle.setClean();
			// output1RotationY
			MDataHandle output1RotationYHandle = data.outputValue(baconDualHinge::output1RotationY);
			output1RotationYHandle.setMAngle(MAngle(output1Angles.y));
			output1RotationYHandle.setClean();
			// output1RotationX
			MDataHandle output1RotationZHandle = data.outputValue(baconDualHinge::output1RotationZ);
			output1RotationZHandle.setMAngle(MAngle(output1Angles.z));
			output1RotationZHandle.setClean();

			// output2RotationX
			MDataHandle output2RotationXHandle = data.outputValue(baconDualHinge::output2RotationX);
			output2RotationXHandle.setMAngle(MAngle(output2Angles.x));
			output2RotationXHandle.setClean();
			// output2RotationY
			MDataHandle output2RotationYHandle = data.outputValue(baconDualHinge::output2RotationY);
			output2RotationYHandle.setMAngle(MAngle(output2Angles.y));
			output2RotationYHandle.setClean();
			// output2RotationX
			MDataHandle output2RotationZHandle = data.outputValue(baconDualHinge::output2RotationZ);
			output2RotationZHandle.setMAngle(MAngle(output2Angles.z));
			output2RotationZHandle.setClean();

			// output3RotationX
			MDataHandle output3RotationXHandle = data.outputValue(baconDualHinge::output3RotationX);
			output3RotationXHandle.setMAngle(MAngle(output3Angles.x));
			output3RotationXHandle.setClean();
			// output3RotationY
			MDataHandle output3RotationYHandle = data.outputValue(baconDualHinge::output3RotationY);
			output3RotationYHandle.setMAngle(MAngle(output3Angles.y));
			output3RotationYHandle.setClean();
			// output3RotationX
			MDataHandle output3RotationZHandle = data.outputValue(baconDualHinge::output3RotationZ);
			output3RotationZHandle.setMAngle(MAngle(output3Angles.z));
			output3RotationZHandle.setClean();

		}
	}
	else {
		return MS::kUnknownParameter;
	}

	return MS::kSuccess;
}

void* baconDualHinge::creator()
{
	return new baconDualHinge();
}


MStatus baconDualHinge::initialize()
{
	MFnNumericAttribute numAttr;
	MFnNumericData		numData;
	MFnUnitAttribute	uAttr;
	MFnMatrixAttribute	matrixAttr;
	MFnMatrixData		matrixData;
	MFnEnumAttribute	enumAttr;
	MStatus				stat;


	/*
	// weight
	weight = numAttr.create("weight", "w", MFnNumericData::kFloat, 0.0);
	numAttr.setStorable(true);
	numAttr.setKeyable(true);
	stat = addAttribute(weight);
	*/


	// INPUTS ---------------------------------------------------------------------

	// Input bone1Length
	bone1Length = numAttr.create("bone1Length", "femlen", numData.kFloat, 0.0f);
	numAttr.setStorable(true);
	numAttr.setKeyable(true);
	stat = addAttribute(bone1Length);

	// Input bone2Length
	bone2Length = numAttr.create("bone2Length", "tiblen", numData.kFloat, 0.0f);
	numAttr.setStorable(true);
	numAttr.setKeyable(true);
	stat = addAttribute(bone2Length);

	// Input hingeLength
	hingeLength = numAttr.create("hingeLength", "tarlen", numData.kFloat, 0.0f);
	numAttr.setStorable(true);
	numAttr.setKeyable(true);
	stat = addAttribute(hingeLength);

	// Input Root Matrix
	bone1WorldTM = matrixAttr.create("bone1WorldTM", "b1tm", matrixAttr.kFloat);
	matrixAttr.setStorable(true);
	matrixAttr.setKeyable(true);
	stat = addAttribute(bone1WorldTM);

	// Input Goal Matrix
	bone2WorldTM = matrixAttr.create("bone2WorldTM", "b2tm", matrixAttr.kFloat);
	matrixAttr.setStorable(true);
	matrixAttr.setKeyable(true);
	stat = addAttribute(bone2WorldTM);

	// Input Up Matrix
	inverseParentTM = matrixAttr.create("inverseParentTM", "invptm", matrixAttr.kFloat);
	matrixAttr.setStorable(true);
	matrixAttr.setKeyable(true);
	stat = addAttribute(inverseParentTM);


	// OUTUTS ---------------------------------------------------------------------
	// Output femur RotationX
	output1RotationX = uAttr.create("output1RotationX", "out1X", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output femur RotationY
	output1RotationY = uAttr.create("output1RotationY", "out1Y", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output femur RotationZ
	output1RotationZ = uAttr.create("output1RotationZ", "out1Z", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output femur Rotation
	output1Rotation = numAttr.create("output1Rotation", "out1", output1RotationX, output1RotationY, output1RotationZ);
	numAttr.setHidden(false);
	stat = addAttribute(output1Rotation);


	// Output tibia RotationX
	output2RotationX = uAttr.create("output2RotationX", "out2X", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output tibia RotationY
	output2RotationY = uAttr.create("output2RotationY", "out2Y", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output tibia RotationZ
	output2RotationZ = uAttr.create("output2RotationZ", "out2Z", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output tibia Rotation
	output2Rotation = numAttr.create("output2Rotation", "out2", output2RotationX, output2RotationY, output2RotationZ);
	numAttr.setHidden(false);
	stat = addAttribute(output2Rotation);

	// Output tarsal RotationX
	output3RotationX = uAttr.create("output3RotationX", "out3X", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output tarsal RotationY
	output3RotationY = uAttr.create("output3RotationY", "out3Y", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output tarsal RotationZ
	output3RotationZ = uAttr.create("output3RotationZ", "out3Z", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output tarsal Rotation
	output3Rotation = numAttr.create("output3Rotation", "out3", output3RotationX, output3RotationY, output3RotationZ);
	numAttr.setHidden(false);
	stat = addAttribute(output3Rotation);

	// DIRTY EVENTS ---------------------------------------------------------------------
	MObject AffectedByMany[] =
	{	output1Rotation,		output2Rotation,		output3Rotation,
		output1RotationX,		output1RotationY,		output1RotationZ,
		output2RotationX,		output2RotationY,		output2RotationZ,
		output3RotationX,		output3RotationY,		output3RotationZ
	};
	for (MObject& obj : AffectedByMany)
	{
		attributeAffects(bone1Length,		obj);
		attributeAffects(bone2Length,		obj);
		attributeAffects(hingeLength,		obj);
		attributeAffects(bone1WorldTM,		obj);
		attributeAffects(bone2WorldTM,		obj);
		attributeAffects(inverseParentTM,	obj);
	}

	return MS::kSuccess;
}

MStatus initializePlugin(MObject obj)
{
	MStatus   status;
	MFnPlugin plugin(obj, "Bacon-Strip.com", "1.0", "Any");

	status = plugin.registerNode("baconDualHinge", baconDualHinge::id, baconDualHinge::creator,
		baconDualHinge::initialize);
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

	status = plugin.deregisterNode(baconDualHinge::id);
	if (!status) {
		status.perror("deregisterNode");
		return status;
	}

	return status;
}
