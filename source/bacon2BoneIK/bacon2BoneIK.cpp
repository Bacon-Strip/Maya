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
// Produces the dependency graph node "bacon2BoneIK".
// This plug-in is part of a rigging package known as The "Bacon-Strip Rig"
// Bacon-Strip ID Block is : 0x0012a940 - 0x0012aa3f
// Registered for Bacon-Strip[Luis Alonso]
//
// ID: 
// 0x0012a954
//
////////////////////////////////////////////////////////////////////////

#include <string.h>
#include <array>
#include <maya/MIOStream.h>
#include <math.h>

#include <maya/MPxNode.h> 

#include <maya/MFnNumericAttribute.h>
#include <maya/MFnStringData.h>
#include <maya/MFnCompoundAttribute.h>
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


class bacon2BoneIK : public MPxNode
{
public:
	bacon2BoneIK();
	virtual				~bacon2BoneIK();

	virtual MStatus		compute(const MPlug& plug, MDataBlock& data);

	static  void*		creator();
	static  MStatus		initialize();

public:
	static	MTypeId		id;

	static	MObject		stretchComp;
	static	MObject		Bone1Comp;
	static	MObject		Bone2Comp;

	// inputs
	static  MObject		weight;
	static  MObject		BoneDirection;
	static  MObject		rootMatrix;
	static  MObject		goalMatrix;
	static  MObject		upMatrix;
	static  MObject		Bone1Length;
	static  MObject		Bone2Length;
	static  MObject		Bone1InitialPosition;
	static  MObject		Bone1Orient;
	static  MObject		Bone1OrientX;
	static  MObject		Bone1OrientY;
	static  MObject		Bone1OrientZ;
	static  MObject		Bone2Orient;
	static  MObject		Bone2OrientX;
	static  MObject		Bone2OrientY;
	static  MObject		Bone2OrientZ;
	static  MObject		stretchIK;
	static  MObject		stretchBlend;
	static  MObject		stretchScale;
	static  MObject		handStretch;
	static  MObject		hingeLength;


	// ouputs
	static  MObject		Bone1StretchPosition;
	static  MObject		Bone1Rotation;
	static  MObject		Bone1RotationX;
	static  MObject		Bone1RotationY;
	static  MObject		Bone1RotationZ;
	static  MObject		Bone2StretchPosition;
	static  MObject		Bone2Rotation;
	static  MObject		Bone2RotationX;
	static  MObject		Bone2RotationY;
	static  MObject		Bone2RotationZ;
	static  MObject		handStretchPosition;

};

MTypeId     bacon2BoneIK::id(0x0012a954);
MObject     bacon2BoneIK::weight;
MObject     bacon2BoneIK::stretchComp;
MObject     bacon2BoneIK::Bone1Comp;
MObject     bacon2BoneIK::Bone2Comp;

MObject		bacon2BoneIK::BoneDirection;
MObject		bacon2BoneIK::rootMatrix;
MObject		bacon2BoneIK::goalMatrix;
MObject		bacon2BoneIK::upMatrix;
MObject		bacon2BoneIK::Bone1Length;
MObject		bacon2BoneIK::Bone2Length;
MObject		bacon2BoneIK::Bone1InitialPosition;
MObject		bacon2BoneIK::Bone1Orient;
MObject		bacon2BoneIK::Bone1OrientX;
MObject		bacon2BoneIK::Bone1OrientY;
MObject		bacon2BoneIK::Bone1OrientZ;
MObject		bacon2BoneIK::Bone2Orient;
MObject		bacon2BoneIK::Bone2OrientX;
MObject		bacon2BoneIK::Bone2OrientY;
MObject		bacon2BoneIK::Bone2OrientZ;
MObject		bacon2BoneIK::stretchIK;
MObject		bacon2BoneIK::stretchBlend;
MObject		bacon2BoneIK::stretchScale;
MObject		bacon2BoneIK::handStretch;
MObject		bacon2BoneIK::hingeLength;


MObject		bacon2BoneIK::Bone1StretchPosition;
MObject		bacon2BoneIK::Bone1Rotation;
MObject		bacon2BoneIK::Bone1RotationX;
MObject		bacon2BoneIK::Bone1RotationY;
MObject		bacon2BoneIK::Bone1RotationZ;
MObject		bacon2BoneIK::Bone2StretchPosition;
MObject		bacon2BoneIK::Bone2Rotation;
MObject		bacon2BoneIK::Bone2RotationX;
MObject		bacon2BoneIK::Bone2RotationY;
MObject		bacon2BoneIK::Bone2RotationZ;
MObject		bacon2BoneIK::handStretchPosition;

bacon2BoneIK::bacon2BoneIK() {}
bacon2BoneIK::~bacon2BoneIK() {}

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


MStatus bacon2BoneIK::compute(const MPlug& plug, MDataBlock& data)
{
	MStatus returnStatus;
	if (
			plug == Bone1Rotation			|| plug == Bone2Rotation				||
			plug == Bone1StretchPosition	|| plug == Bone2StretchPosition		|| plug == handStretchPosition	||
			plug == Bone1RotationX		|| plug == Bone1RotationY			|| plug == Bone1RotationZ		||
			plug == Bone2RotationX			|| plug == Bone2RotationY			|| plug == Bone2RotationZ
		)
	{
		// Handles and Values
		MDataHandle stretchIKHandle = data.inputValue(stretchIK, &returnStatus);
		bool stetchIKValue = stretchIKHandle.asBool();

		MDataHandle handStretchHandle = data.inputValue(handStretch, &returnStatus);
		double handStretchValue = handStretchHandle.asDouble();

		MDataHandle stretchBlendHandle = data.inputValue(stretchBlend, &returnStatus);
		double stretchBlendValue = stretchBlendHandle.asDouble();

		MDataHandle stretchScaleHandle = data.inputValue(stretchScale, &returnStatus);
		double stretchScaleValue = stretchScaleHandle.asDouble();

		MDataHandle BoneDirectionHandle = data.inputValue(BoneDirection, &returnStatus);
		short BoneDirectionValue = BoneDirectionHandle.asShort();
		bool isMirrored(BoneDirectionValue==1);

		MDataHandle Bone1InitialPositionHandle = data.inputValue(Bone1InitialPosition, &returnStatus);
		MVector Bone1InitialPositionValue = Bone1InitialPositionHandle.asFloatVector();
		
		MDataHandle rootMatrixHandle = data.inputValue(rootMatrix, &returnStatus);
		MFloatMatrix rootTM(rootMatrixHandle.asFloatMatrix());

		MDataHandle goalMatrixHandle = data.inputValue(goalMatrix, &returnStatus);
		MFloatMatrix goalTM = goalMatrixHandle.asFloatMatrix();

		MDataHandle upMatrixHandle = data.inputValue(upMatrix, &returnStatus);
		MFloatMatrix upTM = upMatrixHandle.asFloatMatrix();

		MDataHandle Bone1LengthHandle = data.inputValue(Bone1Length, &returnStatus);
		float Bone1LengthValue = Bone1LengthHandle.asFloat();

		MDataHandle Bone2LengthHandle = data.inputValue(Bone2Length, &returnStatus);
		float Bone2LengthValue = Bone2LengthHandle.asFloat();

		MDataHandle Bone1OrientXHandle = data.inputValue(Bone1OrientX, &returnStatus);
		MAngle Bone1OrientXValue = Bone1OrientXHandle.asAngle();
		MDataHandle Bone1OrientYHandle = data.inputValue(Bone1OrientY, &returnStatus);
		MAngle Bone1OrientYValue = Bone1OrientYHandle.asAngle();
		MDataHandle Bone1OrientZHandle = data.inputValue(Bone1OrientZ, &returnStatus);
		MAngle Bone1OrientZValue = Bone1OrientZHandle.asAngle();

		MDataHandle Bone2OrientXHandle = data.inputValue(Bone2OrientX, &returnStatus);
		MAngle Bone2OrientXValue = Bone2OrientXHandle.asAngle();
		MDataHandle Bone2OrientYHandle = data.inputValue(Bone2OrientY, &returnStatus);
		MAngle Bone2OrientYValue = Bone2OrientYHandle.asAngle();
		MDataHandle Bone2OrientZHandle = data.inputValue(Bone2OrientZ, &returnStatus);
		MAngle Bone2OrientZValue = Bone2OrientZHandle.asAngle();




		// Calculation
		MMatrix MrootTM = FloatMatrixToMatrix(rootTM);
		MMatrix rootPosTM(transMatrix(Bone1InitialPositionValue) * MrootTM);
		MVector rootPos(rootPosTM[3][0], rootPosTM[3][1], rootPosTM[3][2]);
		MVector goalPos(goalTM[3][0], goalTM[3][1], goalTM[3][2]);
		MVector upPos(upTM[3][0], upTM[3][1], upTM[3][2]);
		MVector goalLine = goalPos - rootPos;

		float SideA = Bone1LengthValue * float(stretchScaleValue);
		float SideB = Bone2LengthValue * float(stretchScaleValue);
		double maxReach = SideA + SideB;
		double SideC = goalLine.length();
		double goalLength(SideC);
		double angleB = 0.0;

		if (SideC <= (SideB - SideA)) //ie: leg is cramped
		{
			SideC = SideB - SideA;
			angleB = M_PI;
		}
		else
		{
			double SideA2 = pow(SideA, 2.0f);
			double SideB2 = pow(SideB, 2.0f);
			double SideC2 = pow(SideC, 2.0f);
			double SideAC = 2 * SideA * SideC;
			double cosineAngleB = 0.0f;

			if (SideC <= maxReach)
			{
				try
				{
					cosineAngleB = (SideA2 + SideC2 - SideB2) / SideAC;
				}
				catch (...)
				{
					cosineAngleB = 1.0f;
				}

				try
				{
					angleB = acos(cosineAngleB);
				}
				catch (...)
				{
					angleB = 0.0f;
				}
			}
		}

		// Solution Plane
		MVector upDirection = upPos - rootPos;
		upDirection.normalize();

		MVector boneAxis = goalLine;
		boneAxis.normalize();
		MVector bendAxis = boneAxis ^ upDirection;
		bendAxis.normalize();




		MVector solutionPlaneX = goalLine;
		solutionPlaneX.normalize();
		MVector solutionPlaneZ = solutionPlaneX ^ upDirection;
		solutionPlaneZ.normalize();
		MVector solutionPlaneY = solutionPlaneZ ^ solutionPlaneX;
		solutionPlaneY.normalize();
		MMatrix SolutionPlaneMatrix = matrix3(solutionPlaneX, solutionPlaneY, solutionPlaneZ, rootPos);





		// Bone1
		MMatrix Bone1AngleMatrix = rotateZMatrix(angleB);
		MMatrix Bone1MatrixWorld = Bone1AngleMatrix * SolutionPlaneMatrix;
		MMatrix Bone1MatrixLocal = (MirrorMatrix(Bone1MatrixWorld, isMirrored)) * MrootTM.inverse();
		MMatrix Bone1JointOrientTM = MEulerRotation(Bone1OrientXValue.value(), Bone1OrientYValue.value(),
			Bone1OrientZValue.value()).asMatrix();
		MMatrix Bone1MatrixPLocal = Bone1MatrixLocal * Bone1JointOrientTM.inverse();
		MTransformationMatrix Bone1TransformationMatrix = MTransformationMatrix(Bone1MatrixPLocal);
		MEulerRotation Bone1Angles = Bone1TransformationMatrix.eulerRotation();

		// Bone2
		MMatrix Bone2RootWorld = transMatrix(MVector(SideA, 0.0f, 0.0f)) * Bone1MatrixWorld;
		MVector Bone2RootPos(Bone2RootWorld[3][0], Bone2RootWorld[3][1], Bone2RootWorld[3][2]);
		MVector Bone2DirectionX(goalPos - Bone2RootPos);
		Bone2DirectionX.normalize();
		MVector Bone2Up(Bone1MatrixWorld[2][0], Bone1MatrixWorld[2][1], Bone1MatrixWorld[2][2]);
		MVector Bone2DirectionY(Bone2Up ^ Bone2DirectionX);
		Bone2DirectionY.normalize();
		MVector Bone2DirectionZ(Bone2DirectionX ^ Bone2DirectionY);
		Bone2DirectionZ.normalize();
		MMatrix Bone2MatrixWorld = matrix3(Bone2DirectionX, Bone2DirectionY, Bone2DirectionZ, Bone2RootPos);
		MMatrix Bone2JointOrientTM = MEulerRotation(Bone2OrientXValue.value(), Bone2OrientYValue.value(),
			Bone2OrientZValue.value()).asMatrix();
		MMatrix Bone2MatrixLocal((MirrorMatrix(Bone2MatrixWorld, isMirrored)) * Bone1MatrixWorld.inverse());
		MMatrix Bone2MatrixPLocal(Bone2MatrixLocal * Bone2JointOrientTM.inverse());
		MTransformationMatrix Bone2TransformationMatrix(Bone2MatrixPLocal);
		MEulerRotation Bone2Angles(Bone2TransformationMatrix.eulerRotation());

		// Ouput Calculation





		//Stretch Values
		MVector Bone1Direction(Bone1MatrixLocal[0][0], Bone1MatrixLocal[0][1], Bone1MatrixLocal[0][2]);
		double Bone1ScaleOffset = SideA - Bone1LengthValue;
		double handScaleOffset = SideB - Bone2LengthValue;
		MVector Bone1StretchPositionValue = Bone1InitialPositionValue + (Bone1Direction * Bone1ScaleOffset);
		MVector Bone2StretchPositionValue(Bone1LengthValue, 0.0, 0.0);
		MVector handStretchPositionValue(Bone2LengthValue + handScaleOffset, 0.0, 0.0);
		if (stetchIKValue && goalLength >= maxReach)
		{
			double stretchValue = (goalLength - maxReach) * stretchBlendValue;
			Bone1StretchPositionValue += (stretchValue * (1.0 - handStretchValue)) * Bone1Direction;
			handStretchPositionValue.x += stretchValue * handStretchValue;
		}



		// Set Clean OutPut Values
		if (returnStatus != MS::kSuccess)
			cerr << "ERROR getting data" << endl;
		else
		{
			// Bone1RotationX
			MDataHandle Bone1RotationXHandle = data.outputValue(bacon2BoneIK::Bone1RotationX);
			Bone1RotationXHandle.setMAngle(MAngle(Bone1Angles.x));
			Bone1RotationXHandle.setClean();
			// Bone1RotationY
			MDataHandle Bone1RotationYHandle = data.outputValue(bacon2BoneIK::Bone1RotationY);
			Bone1RotationYHandle.setMAngle(MAngle(Bone1Angles.y));
			Bone1RotationYHandle.setClean();
			// Bone1RotationX
			MDataHandle Bone1RotationZHandle = data.outputValue(bacon2BoneIK::Bone1RotationZ);
			Bone1RotationZHandle.setMAngle(MAngle(Bone1Angles.z));
			Bone1RotationZHandle.setClean();

			// Bone2RotationX
			MDataHandle Bone2RotationXHandle = data.outputValue(bacon2BoneIK::Bone2RotationX);
			Bone2RotationXHandle.setMAngle(MAngle(Bone2Angles.x));
			Bone2RotationXHandle.setClean();
			// Bone2RotationY
			MDataHandle Bone2RotationYHandle = data.outputValue(bacon2BoneIK::Bone2RotationY);
			Bone2RotationYHandle.setMAngle(MAngle(Bone2Angles.y));
			Bone2RotationYHandle.setClean();
			// Bone2RotationX
			MDataHandle Bone2RotationZHandle = data.outputValue(bacon2BoneIK::Bone2RotationZ);
			Bone2RotationZHandle.setMAngle(MAngle(Bone2Angles.z));
			Bone2RotationZHandle.setClean();

			// Bone1StretchPosition
			MDataHandle Bone1StretchPositionHandle = data.outputValue(bacon2BoneIK::Bone1StretchPosition);
			Bone1StretchPositionHandle.set3Float(float(Bone1StretchPositionValue.x), float(Bone1StretchPositionValue.y), float(Bone1StretchPositionValue.z) );
			Bone1StretchPositionHandle.setClean();

			// Bone2StretchPosition
			MDataHandle Bone2StretchPositionHandle = data.outputValue(bacon2BoneIK::Bone2StretchPosition);
			Bone2StretchPositionHandle.set3Float(float(Bone2StretchPositionValue.x), float(Bone2StretchPositionValue.y), float(Bone2StretchPositionValue.z));
			Bone2StretchPositionHandle.setClean();

			// handStretchPosition
			MDataHandle handStretchPositionHandle = data.outputValue(bacon2BoneIK::handStretchPosition);
			handStretchPositionHandle.set3Float(float(handStretchPositionValue.x), float(handStretchPositionValue.y), float(handStretchPositionValue.z));
			handStretchPositionHandle.setClean();



		}
	}
	else {
		return MS::kUnknownParameter;
	}


	return MS::kSuccess;
}

void* bacon2BoneIK::creator()
{
	return new bacon2BoneIK();
}


MStatus bacon2BoneIK::initialize()
{
	MFnCompoundAttribute	compAttr;
	MFnNumericAttribute		numAttr;
	MFnNumericData			numData;
	MFnUnitAttribute		uAttr;
	MFnMatrixAttribute		matrixAttr;
	MFnMatrixData			matrixData;
	MFnEnumAttribute		enumAttr;
	MStatus					stat;
	MFnNumericAttribute		nAttr;

	// default example
	weight = numAttr.create("weight", "w", MFnNumericData::kFloat, 0.0);
	numAttr.setStorable(true);
	numAttr.setKeyable(true);
	stat = addAttribute(weight);



/*

bodyPartMembersAttr = OpenMaya.MFnCompoundAttribute()
self.bodyPartMembers = bodyPartMembersAttr.create("bodyPartMembers", "bodyPartMembers")
bodyPartMembersAttr.setArray(True)
bodyPartMembersAttr.setKeyable(True)

*/

// Input Bend Direction

	// Input Bone Direction
	BoneDirection = enumAttr.create("BoneDirection", "boneDir", 0);
	enumAttr.addField("X", 0);
	enumAttr.addField("-X", 1);
	enumAttr.setHidden(false);
	enumAttr.setKeyable(false);
	stat = addAttribute(BoneDirection);
	if (!stat) { stat.perror("addAttribute"); return stat; }


	// Output Bone1 RotationX
	Bone1RotationX = uAttr.create("Bone1RotationX", "humX", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output Bone1 RotationY
	Bone1RotationY = uAttr.create("Bone1RotationY", "humY", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output Bone1 RotationZ
	Bone1RotationZ = uAttr.create("Bone1RotationZ", "humZ", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output Bone1 Rotation
	Bone1Rotation = numAttr.create("Bone1Rotation", "hum", Bone1RotationX, Bone1RotationY, Bone1RotationZ);
	numAttr.setHidden(false);
	stat = addAttribute(Bone1Rotation);
	//compAttr.addChild(Bone1Rotation);



	// Output Bone2 RotationX
	Bone2RotationX = uAttr.create("Bone2RotationX", "ulnX", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output Bone2 RotationY
	Bone2RotationY = uAttr.create("Bone2RotationY", "ulnY", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output Bone2 RotationZ
	Bone2RotationZ = uAttr.create("Bone2RotationZ", "ulnZ", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output Bone2 Rotation
	Bone2Rotation = numAttr.create("Bone2Rotation", "uln", Bone2RotationX, Bone2RotationY, Bone2RotationZ);
	numAttr.setHidden(false);
	stat = addAttribute(Bone2Rotation);
	//compAttr.addChild(Bone2Rotation);

	///////////////////////////////////
	// Compound Stretch
	stretchComp = compAttr.create("Stretch", "Stretch");
	compAttr.setKeyable(true);
	compAttr.setWritable(true);

	// stretch blend
	stretchBlend = numAttr.create("stretchBlend", "stretchBlend", MFnNumericData::kDouble, 1.0);
	numAttr.setMin(0.0);
	numAttr.setStorable(true);
	numAttr.setDefault(1.0);
	numAttr.setKeyable(true);
	compAttr.addChild(stretchBlend);


	// stretch
	stretchIK = nAttr.create("stretchIK", "stretchIK", MFnNumericData::kBoolean);
	nAttr.setDefault(false);
	nAttr.setKeyable(true);
	nAttr.setReadable(true);
	nAttr.setWritable(true);
	nAttr.setStorable(true);
	compAttr.addChild(stretchIK);

	// hand stretch
	stretchScale = numAttr.create("IKScale", "IKScale", MFnNumericData::kDouble, 1.0);
	numAttr.setMin(0.0);
	numAttr.setStorable(true);
	numAttr.setDefault(1.0);
	numAttr.setKeyable(true);
	compAttr.addChild(stretchScale);


	// hand stretch
	handStretch = numAttr.create("handStretch", "hs", MFnNumericData::kDouble, 0.0);
	numAttr.setStorable(true);
	numAttr.setKeyable(true);
	compAttr.addChild(handStretch);

	// Output Bone1 Stretch Position
	Bone1StretchPosition = numAttr.createPoint("Bone1StretchPosition", "hsp");
	//stat = addAttribute(Bone1StretchPosition);
	compAttr.addChild(Bone1StretchPosition);

	// Output Bone2 Stretch Position
	Bone2StretchPosition = numAttr.createPoint("Bone2StretchPosition", "usp");
	//stat = addAttribute(Bone2StretchPosition);
	compAttr.addChild(Bone2StretchPosition);

	// Output Hand Stretch Position
	handStretchPosition = numAttr.createPoint("handStretchPosition", "nsp");
	//stat = addAttribute(Bone1StretchPosition);
	compAttr.addChild(handStretchPosition);

	stat = addAttribute(stretchComp);


	///////////////////////////////////
	// Compound Bone1
	Bone1Comp = compAttr.create("Bone1Inputs", "Bone1Inputs");
	compAttr.setKeyable(true);
	compAttr.setWritable(true);

	// Input Bone1Length
	Bone1Length = numAttr.create("Bone1Length", "hl", numData.kFloat, 0.0f);
	numAttr.setStorable(true);
	numAttr.setKeyable(true);
	//stat = addAttribute(Bone1Length);
	compAttr.addChild(Bone1Length);

	// Input Bone1 Initial Position
	Bone1InitialPosition = numAttr.createPoint("Bone1InitialPosition", "hpos");
	numAttr.setStorable(true);
	numAttr.setWritable(true);
	numAttr.setKeyable(true);
	//stat = addAttribute(Bone1InitialPosition);
	compAttr.addChild(Bone1InitialPosition);

	// Input Bone1OrientX
	Bone1OrientX = uAttr.create("Bone1OrientX", "hox", uAttr.kAngle, 0.0);
	uAttr.setWritable(true);
	uAttr.setStorable(true);
		// Input Bone1OrientY
	Bone1OrientY = uAttr.create("Bone1OrientY", "hoy", uAttr.kAngle, 0.0);
	uAttr.setWritable(true);
	uAttr.setStorable(true);
		// Input Bone1OrientZ
	Bone1OrientZ = uAttr.create("Bone1OrientZ", "hoz", uAttr.kAngle, 0.0);
	uAttr.setWritable(true);
	uAttr.setStorable(true);
		// Input Bone1Orient
	Bone1Orient = numAttr.create("Bone1Orient", "ho", Bone1OrientX, Bone1OrientY, Bone1OrientZ);
	numAttr.setWritable(true);
	numAttr.setStorable(true);
	numAttr.setKeyable(true);
	//stat = addAttribute(Bone1Orient);
	compAttr.addChild(Bone1Orient);


	stat = addAttribute(Bone1Comp);

	///////////////////////////////////
	// Compound Bone2
	Bone2Comp = compAttr.create("Bone2Inputs", "Bone2Inputs");
	compAttr.setKeyable(true);
	compAttr.setWritable(true);


	// Input Bone2Length
	Bone2Length = numAttr.create("Bone2Length", "ul", numData.kFloat, 0.0f);
	numAttr.setStorable(true);
	numAttr.setKeyable(true);
	//stat = addAttribute(Bone2Length);
	compAttr.addChild(Bone2Length);


	// Input Bone2OrientX
	Bone2OrientX = uAttr.create("Bone2OrientX", "uox", uAttr.kAngle, 0.0);
	uAttr.setWritable(true);
	uAttr.setStorable(true);

	// Input Bone2OrientY
	Bone2OrientY = uAttr.create("Bone2OrientY", "uoy", uAttr.kAngle, 0.0);
	uAttr.setWritable(true);
	uAttr.setStorable(true);

	// Input Bone2OrientZ
	Bone2OrientZ = uAttr.create("Bone2OrientZ", "uoz", uAttr.kAngle, 0.0);
	uAttr.setWritable(true);
	uAttr.setStorable(true);

	// Input Bone2Orient
	Bone2Orient = numAttr.create("Bone2Orient", "uo", Bone2OrientX, Bone2OrientY, Bone2OrientZ);
	numAttr.setWritable(true);
	numAttr.setStorable(true);
	numAttr.setKeyable(true);
	//stat = addAttribute(Bone2Orient);
	compAttr.addChild(Bone2Orient);

	stat = addAttribute(Bone2Comp);


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

	

	// DIRTY EVENTS ---------------------------------------------------------------------
	MObject AffectedByMany[] =
	{	
		Bone1Rotation,				Bone1RotationX,				Bone1RotationY,			Bone1RotationZ,
		Bone2Rotation,				Bone2RotationX,				Bone2RotationY,			Bone2RotationZ,
		Bone1StretchPosition,		Bone2StretchPosition,		handStretchPosition

	};
	for (MObject& obj : AffectedByMany)
	{
		attributeAffects(stretchScale,			obj);
		attributeAffects(stretchBlend,			obj);
		attributeAffects(stretchIK,				obj);
		attributeAffects(handStretch,			obj);
		attributeAffects(Bone1InitialPosition,	obj);
		attributeAffects(Bone1Orient,			obj);
		attributeAffects(Bone1OrientX,			obj);
		attributeAffects(Bone1OrientY,			obj);
		attributeAffects(Bone1OrientZ,			obj);
		attributeAffects(Bone2Orient,			obj);
		attributeAffects(Bone2OrientX,			obj);
		attributeAffects(Bone2OrientY,			obj);
		attributeAffects(Bone2OrientZ,			obj);
		attributeAffects(Bone1Length,			obj);
		attributeAffects(Bone2Length,			obj);
		attributeAffects(rootMatrix,			obj);
		attributeAffects(goalMatrix,			obj);
		attributeAffects(upMatrix,				obj);
		attributeAffects(BoneDirection,			obj);

	}

	/*
	MObject AffectedStretch[] =
	{
		Bone1StretchPosition,		Bone2StretchPosition,	handStretchPosition
	};
	for (MObject& obj : AffectedStretch)
	{
		attributeAffects(stretchIK,		obj);
		attributeAffects(handStretch,	obj);
		attributeAffects(stretchScale,	obj);
		attributeAffects(Bone1Length, obj);
		attributeAffects(Bone2Length,	obj);
		attributeAffects(rootMatrix,	obj);
		attributeAffects(goalMatrix,	obj);
		attributeAffects(upMatrix,		obj);
	}
	*/


	return MS::kSuccess;
}

MStatus initializePlugin(MObject obj)
{
	MStatus   status;
	MFnPlugin plugin(obj, "Bacon-Strip.com", "1.0", "Any");

	status = plugin.registerNode("bacon2BoneIK", bacon2BoneIK::id, bacon2BoneIK::creator,
		bacon2BoneIK::initialize);
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

	status = plugin.deregisterNode(bacon2BoneIK::id);
	if (!status) {
		status.perror("deregisterNode");
		return status;
	}

	return status;
}
