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
// Produces the dependency graph node "bacon3BoneIK".
// This plug-in is part of a rigging package known as The "Bacon-Strip Rig"
// Bacon-Strip ID Block is : 0x0012a940 - 0x0012aa3f
// Registered for Bacon-Strip[Luis Alonso]
//
// ID: 
// 0x0012a959
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
#include <stdio.h>




class bacon3BoneIK : public MPxNode
{
public:
	bacon3BoneIK();
	virtual				~bacon3BoneIK();

	virtual MStatus		compute(const MPlug& plug, MDataBlock& data);

	static  void*		creator();
	static  MStatus		initialize();

public:
	static	MTypeId		id;

	// inputs
	static  MObject		weight;
	static  MObject		bias;
	static  MObject		PrimaryRotations;
	static  MObject		rootMatrix;
	static  MObject		goalMatrix;
	static  MObject		upMatrix;
	static  MObject		bone1Length;
	static  MObject		bone2Length;
	static  MObject		bone3Length;
	static  MObject		bone1Position;
	static  MObject		bone1Orient;
	static  MObject		bone1OrientX;
	static  MObject		bone1OrientY;
	static  MObject		bone1OrientZ;
	static  MObject		bone2Orient;
	static  MObject		bone2OrientX;
	static  MObject		bone2OrientY;
	static  MObject		bone2OrientZ;
	static  MObject		bone3Orient;
	static  MObject		bone3OrientX;
	static  MObject		bone3OrientY;
	static  MObject		bone3OrientZ;
	static  MObject		BoneDirection;

	// ouputs
	static  MObject		bone1Rotation;
	static  MObject		bone1RotationX;
	static  MObject		bone1RotationY;
	static  MObject		bone1RotationZ;
	static  MObject		bone2Rotation;
	static  MObject		bone2RotationX;
	static  MObject		bone2RotationY;
	static  MObject		bone2RotationZ;
	static  MObject		bone3Rotation;
	static  MObject		bone3RotationX;
	static  MObject		bone3RotationY;
	static  MObject		bone3RotationZ;

};

MTypeId     bacon3BoneIK::id(0x0012a959);
MObject     bacon3BoneIK::weight;
MObject     bacon3BoneIK::bias;
MObject		bacon3BoneIK::PrimaryRotations;
MObject		bacon3BoneIK::rootMatrix;
MObject		bacon3BoneIK::goalMatrix;
MObject		bacon3BoneIK::upMatrix;
MObject		bacon3BoneIK::bone1Length;
MObject		bacon3BoneIK::bone2Length;
MObject		bacon3BoneIK::bone3Length;
MObject		bacon3BoneIK::bone1Position;
MObject		bacon3BoneIK::bone1Orient;
MObject		bacon3BoneIK::bone1OrientX;
MObject		bacon3BoneIK::bone1OrientY;
MObject		bacon3BoneIK::bone1OrientZ;
MObject		bacon3BoneIK::bone2Orient;
MObject		bacon3BoneIK::bone2OrientX;
MObject		bacon3BoneIK::bone2OrientY;
MObject		bacon3BoneIK::bone2OrientZ;
MObject		bacon3BoneIK::bone3Orient;
MObject		bacon3BoneIK::bone3OrientX;
MObject		bacon3BoneIK::bone3OrientY;
MObject		bacon3BoneIK::bone3OrientZ;
MObject		bacon3BoneIK::BoneDirection;
//MObject		bacon3BoneIK::swingBias;


MObject		bacon3BoneIK::bone1Rotation;
MObject		bacon3BoneIK::bone1RotationX;
MObject		bacon3BoneIK::bone1RotationY;
MObject		bacon3BoneIK::bone1RotationZ;
MObject		bacon3BoneIK::bone2Rotation;
MObject		bacon3BoneIK::bone2RotationX;
MObject		bacon3BoneIK::bone2RotationY;
MObject		bacon3BoneIK::bone2RotationZ;
MObject		bacon3BoneIK::bone3Rotation;
MObject		bacon3BoneIK::bone3RotationX;
MObject		bacon3BoneIK::bone3RotationY;
MObject		bacon3BoneIK::bone3RotationZ;

bacon3BoneIK::bacon3BoneIK() {}
bacon3BoneIK::~bacon3BoneIK() {}

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

MMatrix rotateZMatrix(double angle)
{
	MMatrix returnTM = MMatrix();
	double CAngle = cos(angle);
	double SAngle = sin(angle);
	returnTM = setRow(returnTM, MVector(CAngle, SAngle, 0.0f), 0);
	returnTM = setRow(returnTM, MVector(-1.0f * SAngle, CAngle, 0.0f), 1);
	returnTM = setRow(returnTM, MVector(0.0f, 0.0f, 1.0f), 2);
	returnTM = setRow(returnTM, MVector(0.0f, 0.0f, 0.0f), 3);
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

MMatrix MirrorMatrix(MMatrix inTM, bool mirror = true)
{
	MMatrix returnTM = inTM;
	if (mirror)
	{
		float neg(-1.0f);
		returnTM = setRow(returnTM, MVector(inTM[0][0] * neg, inTM[0][1] * neg, inTM[0][2] * neg), 0);
		returnTM = setRow(returnTM, MVector(inTM[1][0] * neg, inTM[1][1] * neg, inTM[1][2] * neg), 1);
		returnTM = setRow(returnTM, MVector(inTM[2][0] * neg, inTM[2][1] * neg, inTM[2][2] * neg), 2);
	}
	return returnTM;
}


MStatus bacon3BoneIK::compute(const MPlug& plug, MDataBlock& data)
{
	MStatus returnStatus;
	if (
			plug == bone1Rotation		|| plug == bone2Rotation	|| plug == bone3Rotation ||
			plug == bone1RotationX		|| plug == bone1RotationY	|| plug == bone1RotationZ ||
			plug == bone2RotationX		|| plug == bone2RotationY	|| plug == bone2RotationZ ||
			plug == bone3RotationX		|| plug == bone3RotationY	|| plug == bone3RotationZ
		)
	{
		// Handles and Values
		MDataHandle BoneDirectionHandle = data.inputValue(BoneDirection, &returnStatus);
		short BoneDirectionValue = BoneDirectionHandle.asShort();
		bool isMirrored(BoneDirectionValue == 1);

		MDataHandle biasHandle = data.inputValue(bias, &returnStatus);
		float biasValue = biasHandle.asFloat();

		MDataHandle PrimaryRotationsHandle = data.inputValue(PrimaryRotations, &returnStatus);
		short PrimaryRotationsValue = PrimaryRotationsHandle.asShort();

		MDataHandle bone1PositionHandle = data.inputValue(bone1Position, &returnStatus);
		MVector bone1PositionValue = bone1PositionHandle.asFloatVector();
		
		MDataHandle rootMatrixHandle = data.inputValue(rootMatrix, &returnStatus);
		MFloatMatrix rootFM(rootMatrixHandle.asFloatMatrix());

		MDataHandle goalMatrixHandle = data.inputValue(goalMatrix, &returnStatus);
		MFloatMatrix goalFM = goalMatrixHandle.asFloatMatrix();

		MDataHandle upMatrixHandle = data.inputValue(upMatrix, &returnStatus);
		MFloatMatrix upFM = upMatrixHandle.asFloatMatrix();

		MDataHandle bone1LengthHandle = data.inputValue(bone1Length, &returnStatus);
		float bone1LengthValue = bone1LengthHandle.asFloat();

		MDataHandle bone2LengthHandle = data.inputValue(bone2Length, &returnStatus);
		float bone2LengthValue = bone2LengthHandle.asFloat();

		MDataHandle bone3LengthHandle = data.inputValue(bone3Length, &returnStatus);
		float bone3LengthValue = bone3LengthHandle.asFloat();

		MDataHandle bone1OrientXHandle = data.inputValue(bone1OrientX, &returnStatus);
		MAngle bone1OrientXValue = bone1OrientXHandle.asAngle();
		MDataHandle bone1OrientYHandle = data.inputValue(bone1OrientY, &returnStatus);
		MAngle bone1OrientYValue = bone1OrientYHandle.asAngle();
		MDataHandle bone1OrientZHandle = data.inputValue(bone1OrientZ, &returnStatus);
		MAngle bone1OrientZValue = bone1OrientZHandle.asAngle();

		MDataHandle bone2OrientXHandle = data.inputValue(bone2OrientX, &returnStatus);
		MAngle bone2OrientXValue = bone2OrientXHandle.asAngle();
		MDataHandle bone2OrientYHandle = data.inputValue(bone2OrientY, &returnStatus);
		MAngle bone2OrientYValue = bone2OrientYHandle.asAngle();
		MDataHandle bone2OrientZHandle = data.inputValue(bone2OrientZ, &returnStatus);
		MAngle bone2OrientZValue = bone2OrientZHandle.asAngle();

		MDataHandle bone3OrientXHandle = data.inputValue(bone3OrientX, &returnStatus);
		MAngle bone3OrientXValue = bone3OrientXHandle.asAngle();
		MDataHandle bone3OrientYHandle = data.inputValue(bone3OrientY, &returnStatus);
		MAngle bone3OrientYValue = bone3OrientYHandle.asAngle();
		MDataHandle bone3OrientZHandle = data.inputValue(bone3OrientZ, &returnStatus);
		MAngle bone3OrientZValue = bone3OrientZHandle.asAngle();


		// Calculation /////////////////////////////////////////////

		// Input Positions
			MMatrix rootTM = FloatMatrixToMatrix(rootFM);
			MMatrix rootOffsetTM(transMatrix(bone1PositionValue) * rootTM);
			MVector rootPos(rootOffsetTM[3][0], rootOffsetTM[3][1], rootOffsetTM[3][2]);

			MMatrix invRootTM(FloatMatrixToMatrix(rootFM).inverse());
			MVector upPos(upFM[3][0], upFM[3][1], upFM[3][2]);
			MVector goalPos(goalFM[3][0], goalFM[3][1], goalFM[3][2]);
			MVector goalVector = goalPos - rootPos;
			float goalLength = float(goalVector.length());
			float totalLength = bone1LengthValue + bone2LengthValue + bone3LengthValue;
			float minLength = bone1LengthValue - bone2LengthValue + bone3LengthValue;
			float nSwing = (biasValue + 1.0f) / 2.0f;
			MMatrix bone1WorldTM = MMatrix().setToIdentity();
			MMatrix HalfXTM = MEulerRotation(M_PI, 0.0, 0.0).asMatrix();

		// SOLUTION PLANE
			MVector up = (upPos - rootPos).normal();
			MVector rowX = goalVector.normal();
			MVector rowZ = (up ^ rowX).normal();
			MVector rowY = (rowZ ^ rowX).normal();
			MMatrix solutionPlaneTM = matrix3(rowX, rowY, rowZ, rootPos);

		// bone1
			if (goalLength < totalLength && goalLength > minLength) // over-extended test
			{
				float sideA = (goalLength - bone3LengthValue);
				float sideB = (bone1LengthValue);
				float sideC = (bone2LengthValue);
				MAngle maxAngle(cosineLaw(sideA, sideB, sideC));
				MAngle bestAngle = std::max(maxAngle.value(), 0.0);
				MMatrix bestAngleZTM = MEulerRotation(0.0, 0.0, nSwing * bestAngle.value()).asMatrix();
				bone1WorldTM = bestAngleZTM * HalfXTM * solutionPlaneTM;
			}
			else
			{
				bone1WorldTM = HalfXTM * solutionPlaneTM;
			}

			MMatrix bone1localTM = (MirrorMatrix(bone1WorldTM, isMirrored)) * invRootTM;
			MMatrix bone1JointOrientTM = MEulerRotation(bone1OrientXValue.value(), bone1OrientYValue.value(),
				bone1OrientZValue.value()).asMatrix();
			MMatrix bone1MatrixPLocal = bone1localTM * bone1JointOrientTM.inverse();
			MTransformationMatrix bone1TransformationMatrix = MTransformationMatrix(bone1MatrixPLocal);
			MEulerRotation bone1Angles = bone1TransformationMatrix.eulerRotation();


		// bone2
			MVector bone1XAxis = getRow(bone1WorldTM, 0);
			MVector bone1TipPos = getRow(bone1WorldTM, 3) + (bone1XAxis * bone1LengthValue);
			MVector toGoalVector = goalPos - bone1TipPos;
			float dotVector = float (bone1XAxis * toGoalVector.normal());
			MAngle projectionAngle = cosineLaw(float(toGoalVector.length()), bone2LengthValue, bone3LengthValue);
			MAngle bone2FullAngle(0.0);
			if (goalLength < totalLength) // over-extended test
			{
				if (goalLength > minLength) // cramped leg test
				{
					bone2FullAngle = MAngle(acos(dotVector));
				}
				else
				{
					bone2FullAngle = MAngle(M_PI);
					projectionAngle = MAngle(0.0);
				}
			}
			else
			{
				bone2FullAngle = MAngle(0.0);
				projectionAngle = MAngle(0.0);
			}
			MAngle bone2AngleZ(-1.0f * (bone2FullAngle.value() + projectionAngle.value()));
			MMatrix bone2LocalTM = MEulerRotation(0.0, 0.0, bone2AngleZ.value()).asMatrix();
			bone2LocalTM = setRow(bone2LocalTM, MVector(bone1LengthValue, 0.0, 0.0), 3);
			MMatrix bone2JointOrientTM = MEulerRotation(bone2OrientXValue.value(), bone2OrientYValue.value(),
				bone2OrientZValue.value()).asMatrix();
			MMatrix bone2MatrixPLocal = bone2LocalTM * bone2JointOrientTM.inverse();
			MTransformationMatrix bone2TransformationMatrix = MTransformationMatrix(bone2MatrixPLocal);
			MEulerRotation bone2Angles = bone2TransformationMatrix.eulerRotation();

		// bone3
			MMatrix bone2WorldTM(bone2LocalTM * bone1WorldTM);
			MVector bone2RowX = getRow(bone2WorldTM, 0);
			MVector bone2TipPos = getRow(bone2WorldTM, 3) + (bone2RowX * bone2LengthValue);
			MVector bone3Up = getRow(bone2WorldTM, 2);
			MVector bone3RowX = (goalPos - bone2TipPos).normal();
			MVector bone3RowY = (bone3Up ^ bone3RowX).normal();
			MVector bone3RowZ = (bone3RowX ^ bone3RowY).normal();
			MMatrix bone3WorldTM = matrix3(bone3RowX, bone3RowY, bone3RowZ, bone2TipPos);
			MMatrix bone3LocalTM((MirrorMatrix(bone3WorldTM, isMirrored)) * bone2WorldTM.inverse());
			MMatrix bone3JointOrientTM = MEulerRotation(bone3OrientXValue.value(), bone3OrientYValue.value(),
				bone3OrientZValue.value()).asMatrix();
			MMatrix bone3MatrixPLocal = bone3LocalTM * bone3JointOrientTM.inverse();
			MTransformationMatrix bone3TransformationMatrix = MTransformationMatrix(bone3MatrixPLocal);
			MEulerRotation bone3Angles = bone3TransformationMatrix.eulerRotation();


		// OUTPUT
		if (returnStatus != MS::kSuccess)
			cerr << "ERROR getting data" << endl;
		else
		{
			// bone1RotationX
			MDataHandle bone1RotationXHandle = data.outputValue(bacon3BoneIK::bone1RotationX);
			bone1RotationXHandle.setMAngle(MAngle(bone1Angles.x));
			bone1RotationXHandle.setClean();
			// bone1RotationY
			MDataHandle bone1RotationYHandle = data.outputValue(bacon3BoneIK::bone1RotationY);
			bone1RotationYHandle.setMAngle(MAngle(bone1Angles.y));
			bone1RotationYHandle.setClean();
			// bone1RotationX
			MDataHandle bone1RotationZHandle = data.outputValue(bacon3BoneIK::bone1RotationZ);
			bone1RotationZHandle.setMAngle(MAngle(bone1Angles.z));
			bone1RotationZHandle.setClean();

			// bone2RotationX
			MDataHandle bone2RotationXHandle = data.outputValue(bacon3BoneIK::bone2RotationX);
			bone2RotationXHandle.setMAngle(MAngle(bone2Angles.x));
			bone2RotationXHandle.setClean();
			// bone2RotationY
			MDataHandle bone2RotationYHandle = data.outputValue(bacon3BoneIK::bone2RotationY);
			bone2RotationYHandle.setMAngle(MAngle(bone2Angles.y));
			bone2RotationYHandle.setClean();
			// bone2RotationX
			MDataHandle bone2RotationZHandle = data.outputValue(bacon3BoneIK::bone2RotationZ);
			bone2RotationZHandle.setMAngle(MAngle(bone2Angles.z));
			bone2RotationZHandle.setClean();

			// bone3RotationX
			MDataHandle bone3RotationXHandle = data.outputValue(bacon3BoneIK::bone3RotationX);
			bone3RotationXHandle.setMAngle(MAngle(bone3Angles.x));
			bone3RotationXHandle.setClean();
			// bone3RotationY
			MDataHandle bone3RotationYHandle = data.outputValue(bacon3BoneIK::bone3RotationY);
			bone3RotationYHandle.setMAngle(MAngle(bone3Angles.y));
			bone3RotationYHandle.setClean();
			// bone3RotationX
			MDataHandle bone3RotationZHandle = data.outputValue(bacon3BoneIK::bone3RotationZ);
			bone3RotationZHandle.setMAngle(MAngle(bone3Angles.z));
			bone3RotationZHandle.setClean();

		}
	}
	else {
		return MS::kUnknownParameter;
	}

	return MS::kSuccess;
}

void* bacon3BoneIK::creator()
{
	return new bacon3BoneIK();
}


MStatus bacon3BoneIK::initialize()
{
	MFnNumericAttribute numAttr;
	MFnNumericData		numData;
	MFnUnitAttribute	uAttr;
	MFnMatrixAttribute	matrixAttr;
	MFnMatrixData		matrixData;
	MFnEnumAttribute	enumAttr;
	MStatus				stat;



	// INPUTS ---------------------------------------------------------------------
	
	// Bone Direction
	BoneDirection = enumAttr.create("BoneDirection", "boneDir", 0);
	enumAttr.addField("X", 0);
	enumAttr.addField("-X", 1);
	enumAttr.setHidden(false);
	enumAttr.setKeyable(false);
	stat = addAttribute(BoneDirection);
	if (!stat) { stat.perror("addAttribute"); return stat; }

	// bias
	bias = numAttr.create("bias", "bias", MFnNumericData::kFloat, 0.0);
	numAttr.setStorable(true);
	numAttr.setKeyable(true);
	numAttr.setMin(-1.0f);
	numAttr.setMax(1.0f);
	stat = addAttribute(bias);

	// Input primary rotation
	PrimaryRotations = enumAttr.create("PrimaryRotations", "pr", 0);
	enumAttr.addField("ZZ", 0);
	enumAttr.addField("YZ", 1);
	enumAttr.addField("YY", 2);
	enumAttr.setHidden(false);
	enumAttr.setKeyable(false);
	stat = addAttribute(PrimaryRotations);
	if (!stat) { stat.perror("addAttribute"); return stat; }

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

	// Input bone3Length
	bone3Length = numAttr.create("bone3Length", "tarlen", numData.kFloat, 0.0f);
	numAttr.setStorable(true);
	numAttr.setKeyable(true);
	stat = addAttribute(bone3Length);

	// Input bone1 Position
	bone1Position = numAttr.createPoint("bone1Position", "fpos");
	numAttr.setStorable(true);
	numAttr.setWritable(true);
	numAttr.setKeyable(true);
	stat = addAttribute(bone1Position);

	// Input bone1OrientX
	bone1OrientX = uAttr.create("bone1OrientX", "femox", uAttr.kAngle, 0.0);
	uAttr.setWritable(true);
	uAttr.setStorable(true);

	// Input bone1OrientY
	bone1OrientY = uAttr.create("bone1OrientY", "femoy", uAttr.kAngle, 0.0);
	uAttr.setWritable(true);
	uAttr.setStorable(true);

	// Input bone1OrientZ
	bone1OrientZ = uAttr.create("bone1OrientZ", "femoz", uAttr.kAngle, 0.0);
	uAttr.setWritable(true);
	uAttr.setStorable(true);

	// Input bone1Orient
	bone1Orient = numAttr.create("bone1Orient", "femo", bone1OrientX, bone1OrientY, bone1OrientZ);
	numAttr.setWritable(true);
	numAttr.setStorable(true);
	numAttr.setKeyable(true);
	stat = addAttribute(bone1Orient);

	// Input bone2OrientX
	bone2OrientX = uAttr.create("bone2OrientX", "tibox", uAttr.kAngle, 0.0);
	uAttr.setWritable(true);
	uAttr.setStorable(true);

	// Input bone2OrientY
	bone2OrientY = uAttr.create("bone2OrientY", "tiboy", uAttr.kAngle, 0.0);
	uAttr.setWritable(true);
	uAttr.setStorable(true);

	// Input bone2OrientZ
	bone2OrientZ = uAttr.create("bone2OrientZ", "tiboz", uAttr.kAngle, 0.0);
	uAttr.setWritable(true);
	uAttr.setStorable(true);

	// Input bone2Orient
	bone2Orient = numAttr.create("bone2Orient", "tibo", bone2OrientX, bone2OrientY, bone2OrientZ);
	numAttr.setWritable(true);
	numAttr.setStorable(true);
	numAttr.setKeyable(true);
	stat = addAttribute(bone2Orient);

	// Input bone3OrientX
	bone3OrientX = uAttr.create("bone3OrientX", "taox", uAttr.kAngle, 0.0);
	uAttr.setWritable(true);
	uAttr.setStorable(true);

	// Input bone3OrientY
	bone3OrientY = uAttr.create("bone3OrientY", "taoy", uAttr.kAngle, 0.0);
	uAttr.setWritable(true);
	uAttr.setStorable(true);

	// Input bone3OrientZ
	bone3OrientZ = uAttr.create("bone3OrientZ", "taoz", uAttr.kAngle, 0.0);
	uAttr.setWritable(true);
	uAttr.setStorable(true);

	// Input bone3Orient
	bone3Orient = numAttr.create("bone3Orient", "tao", bone3OrientX, bone3OrientY, bone3OrientZ);
	numAttr.setWritable(true);
	numAttr.setStorable(true);
	numAttr.setKeyable(true);
	stat = addAttribute(bone3Orient);

	// Input Root Matrix
	rootMatrix = matrixAttr.create("rootMatrix", "rootTM", matrixAttr.kFloat);
	matrixAttr.setStorable(true);
	matrixAttr.setKeyable(true);
	stat = addAttribute(rootMatrix);

	// Input Goal Matrix
	goalMatrix = matrixAttr.create("goalMatrix", "goalTM", matrixAttr.kFloat);
	matrixAttr.setStorable(true);
	matrixAttr.setKeyable(true);
	stat = addAttribute(goalMatrix);

	// Input Up Matrix
	upMatrix = matrixAttr.create("upMatrix", "upTM", matrixAttr.kFloat);
	matrixAttr.setStorable(true);
	matrixAttr.setKeyable(true);
	stat = addAttribute(upMatrix);


	// OUTUTS ---------------------------------------------------------------------
	// Output bone1 RotationX
	bone1RotationX = uAttr.create("bone1RotationX", "femX", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output bone1 RotationY
	bone1RotationY = uAttr.create("bone1RotationY", "femY", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output bone1 RotationZ
	bone1RotationZ = uAttr.create("bone1RotationZ", "femZ", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output bone1 Rotation
	bone1Rotation = numAttr.create("bone1Rotation", "fem", bone1RotationX, bone1RotationY, bone1RotationZ);
	numAttr.setHidden(false);
	stat = addAttribute(bone1Rotation);


	// Output bone2 RotationX
	bone2RotationX = uAttr.create("bone2RotationX", "tibX", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output bone2 RotationY
	bone2RotationY = uAttr.create("bone2RotationY", "tibY", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output bone2 RotationZ
	bone2RotationZ = uAttr.create("bone2RotationZ", "tibZ", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output bone2 Rotation
	bone2Rotation = numAttr.create("bone2Rotation", "tib", bone2RotationX, bone2RotationY, bone2RotationZ);
	numAttr.setHidden(false);
	stat = addAttribute(bone2Rotation);

	// Output bone3 RotationX
	bone3RotationX = uAttr.create("bone3RotationX", "tarX", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output bone3 RotationY
	bone3RotationY = uAttr.create("bone3RotationY", "tarY", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output bone3 RotationZ
	bone3RotationZ = uAttr.create("bone3RotationZ", "tarZ", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output bone3 Rotation
	bone3Rotation = numAttr.create("bone3Rotation", "tar", bone3RotationX, bone3RotationY, bone3RotationZ);
	numAttr.setHidden(false);
	stat = addAttribute(bone3Rotation);


	// DIRTY EVENTS ---------------------------------------------------------------------
	MObject AffectedByMany[] =
	{	bone1Rotation,		bone2Rotation,		bone3Rotation,
		bone1RotationX,		bone1RotationY,		bone1RotationZ,
		bone2RotationX,		bone2RotationY,		bone2RotationZ,
		bone3RotationX,	bone3RotationY,	bone3RotationZ
	};
	for (MObject& obj : AffectedByMany)
	{
		attributeAffects(bias,			obj);
		attributeAffects(bone1Position, obj);
		attributeAffects(bone1Orient,	obj);
		attributeAffects(bone1OrientX,	obj);
		attributeAffects(bone1OrientY,	obj);
		attributeAffects(bone1OrientZ,	obj);
		attributeAffects(bone2Orient,	obj);
		attributeAffects(bone2OrientX,	obj);
		attributeAffects(bone2OrientY,	obj);
		attributeAffects(bone2OrientZ,	obj);
		attributeAffects(bone3Orient,	obj);
		attributeAffects(bone3OrientX, obj);
		attributeAffects(bone3OrientY, obj);
		attributeAffects(bone3OrientZ, obj);
		attributeAffects(bone1Length,	obj);
		attributeAffects(bone2Length,	obj);
		attributeAffects(bone3Length,	obj);
		attributeAffects(rootMatrix,	obj);
		attributeAffects(goalMatrix,	obj);
		attributeAffects(upMatrix,		obj);
	}

	return MS::kSuccess;
}

MStatus initializePlugin(MObject obj)
{
	MStatus   status;
	MFnPlugin plugin(obj, "Bacon-Strip.com", "1.0", "Any");

	status = plugin.registerNode("bacon3BoneIK", bacon3BoneIK::id, bacon3BoneIK::creator,
		bacon3BoneIK::initialize);
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

	status = plugin.deregisterNode(bacon3BoneIK::id);
	if (!status) {
		status.perror("deregisterNode");
		return status;
	}

	return status;
}
