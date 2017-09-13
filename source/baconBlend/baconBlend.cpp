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
// Produces the dependency graph node "baconBlend".
// This plug-in is part of a rigging package known as The "Bacon-Strip Rig"
// Bacon-Strip ID Block is : 0x0012a940 - 0x0012aa3f
// Registered for Bacon-Strip[Luis Alonso]
//
// ID: 
// 0x0012a961
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
#include <maya/MFnTransform.h>
#include <maya/MFnCompoundAttribute.h>

#include <maya/MString.h> 
#include <maya/MTypeId.h> 
#include <maya/MPlug.h>
#include <maya/MVector.h>
#include <maya/MAngle.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>

#include <maya/MFloatPoint.h>
#include <maya/MQuaternion.h>
#include <maya/MFloatMatrix.h>
#include <maya/MEulerRotation.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MDoubleArray.h>
#include <maya/MScriptUtil.h>

 
class baconBlend : public MPxNode
{
public:
						baconBlend();
	virtual				~baconBlend(); 

	virtual MStatus		compute( const MPlug& plug, MDataBlock& data );

	static  void*		creator();
	static  MStatus		initialize();

public:
	static	MTypeId		id;

	// inputs
	static  MObject		blendWeight;
	static  MObject		target1;
	static  MObject		target2;
	static  MObject		targetTM1;
	static  MObject		targetTM2;
	static	MObject		targetPosition1;
	static	MObject		targetPosition2;
	static	MObject		targetRotation1;
	static	MObject		targetRotation1X;
	static	MObject		targetRotation1Y;
	static	MObject		targetRotation1Z;
	static	MObject		targetRotation2;
	static	MObject		targetRotation2X;
	static	MObject		targetRotation2Y;
	static	MObject		targetRotation2Z;
	static	MObject		targetJointOrient1;
	static	MObject		targetJointOrient1X;
	static	MObject		targetJointOrient1Y;
	static	MObject		targetJointOrient1Z;
	static	MObject		targetJointOrient2;
	static	MObject		targetJointOrient2X;
	static	MObject		targetJointOrient2Y;
	static	MObject		targetJointOrient2Z;

	static  MObject		parentInverseMatrix;
	static  MObject		jointOrient;
	static  MObject		jointOrientX;
	static  MObject		jointOrientY;
	static  MObject		jointOrientZ;

	// ouputs
	static  MObject		blendRotation;
	static  MObject		blendRotationX;
	static  MObject		blendRotationY;
	static  MObject		blendRotationZ;
	static  MObject		blendPosition;

};

MTypeId     baconBlend::id( 0x0012a961 );
MObject     baconBlend::blendWeight;        
MObject		baconBlend::target1;
MObject		baconBlend::target2;
MObject		baconBlend::targetTM1;
MObject		baconBlend::targetTM2;
MObject		baconBlend::targetPosition1;
MObject		baconBlend::targetPosition2;
MObject		baconBlend::targetRotation1;
MObject		baconBlend::targetRotation1X;
MObject		baconBlend::targetRotation1Y;
MObject		baconBlend::targetRotation1Z;
MObject		baconBlend::targetRotation2;
MObject		baconBlend::targetRotation2X;
MObject		baconBlend::targetRotation2Y;
MObject		baconBlend::targetRotation2Z;
MObject		baconBlend::targetJointOrient1;
MObject		baconBlend::targetJointOrient1X;
MObject		baconBlend::targetJointOrient1Y;
MObject		baconBlend::targetJointOrient1Z;
MObject		baconBlend::targetJointOrient2;
MObject		baconBlend::targetJointOrient2X;
MObject		baconBlend::targetJointOrient2Y;
MObject		baconBlend::targetJointOrient2Z;
MObject		baconBlend::parentInverseMatrix;
MObject		baconBlend::jointOrient;
MObject		baconBlend::jointOrientX;
MObject		baconBlend::jointOrientY;
MObject		baconBlend::jointOrientZ;
MObject		baconBlend::blendRotation;
MObject		baconBlend::blendRotationX;
MObject		baconBlend::blendRotationY;
MObject		baconBlend::blendRotationZ;
MObject		baconBlend::blendPosition;

baconBlend::baconBlend() {}
baconBlend::~baconBlend() {}

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

double quatDot(MQuaternion q1, MQuaternion q2)
{
	return (q1.x * q2.x) + (q1.y * q2.y) + (q1.z * q2.z) + (q1.w * q2.w);
}

MQuaternion baconSlerp(MQuaternion a, MQuaternion b, double t)
{
	double omega = quatDot(a, b);
	if (omega < 0.0)
	{
		omega = quatDot(a, b.negateIt());
	}
	double theta = acos(omega);
	double epsilon = sin(theta);

	double w1(0.0);
	double w2(0.0);

	if (epsilon > 0.0001)
	{
		w1 = sin(theta * (1.0 - t)) / epsilon;
		w2 = sin(theta * t) / epsilon;
	}
	else
	{
		w1 = 1.0 - t;
		w2 = t;
	}

	MQuaternion aa(a.scaleIt(w1));
	MQuaternion bb(b.scaleIt(w2));
	return aa + bb;
}


MQuaternion MSlerp(MQuaternion qa, MQuaternion qb, double t) 
{
	// quaternion to return
	MQuaternion qm(MQuaternion::identity);

	double cosHalfTheta = qa.w * qb.w + qa.x * qb.x + qa.y * qb.y + qa.z * qb.z;
	if (abs(cosHalfTheta) >= 1.0) {
		qm.w = qa.w; qm.x = qa.x; qm.y = qa.y; qm.z = qa.z;
		return qm;
	}
	// Calculate temporary values.
	double halfTheta = acos(cosHalfTheta);
	double sinHalfTheta = sqrt(1.0 - cosHalfTheta*cosHalfTheta);
	if (abs(sinHalfTheta) < 0.001) { 
		qm.w = (qa.w * 0.5 + qb.w * 0.5);
		qm.x = (qa.x * 0.5 + qb.x * 0.5);
		qm.y = (qa.y * 0.5 + qb.y * 0.5);
		qm.z = (qa.z * 0.5 + qb.z * 0.5);
		return qm;
	}
	double ratioA = sin((1.0 - t) * halfTheta) / sinHalfTheta;
	double ratioB = sin(t * halfTheta) / sinHalfTheta;

	//calculate Quaternion.
	qm.w = (qa.w * ratioA + qb.w * ratioB);
	qm.x = (qa.x * ratioA + qb.x * ratioB);
	qm.y = (qa.y * ratioA + qb.y * ratioB);
	qm.z = (qa.z * ratioA + qb.z * ratioB);
	return qm;
}



MStatus baconBlend::compute( const MPlug& plug, MDataBlock& data )
{
	
	MStatus returnStatus;
 
	if
	( 
		plug == blendRotation	|| plug == blendPosition		||
		plug == blendRotationX	|| plug == blendRotationY		|| plug == blendRotationZ
	)
	{
		// Handles and Values
		MDataHandle blendWeightHandle = data.inputValue(blendWeight, &returnStatus);
		float BW(blendWeightHandle.asFloat());

		MDataHandle targetTM1Handle = data.inputValue(targetTM1, &returnStatus);
		MFloatMatrix FM1(targetTM1Handle.asFloatMatrix());

		MDataHandle targetTM2Handle = data.inputValue(targetTM2, &returnStatus);
		MFloatMatrix FM2(targetTM2Handle.asFloatMatrix());

		MDataHandle parentInverseMatrixHandle = data.inputValue(parentInverseMatrix, &returnStatus);
		MFloatMatrix pFM = parentInverseMatrixHandle.asFloatMatrix();

		MDataHandle jointOrientXHandle = data.inputValue(jointOrientX, &returnStatus);
		MAngle jointOrientXValue = jointOrientXHandle.asAngle();
		MDataHandle jointOrientYHandle = data.inputValue(jointOrientY, &returnStatus);
		MAngle jointOrientYValue = jointOrientYHandle.asAngle();
		MDataHandle jointOrientZHandle = data.inputValue(jointOrientZ, &returnStatus);
		MAngle jointOrientZValue = jointOrientZHandle.asAngle();

		MDataHandle targetRotation1XHandle = data.inputValue(targetRotation1X, &returnStatus);
		MAngle targetRotation1XValue = targetRotation1XHandle.asAngle();
		MDataHandle targetRotation1YHandle = data.inputValue(targetRotation1Y, &returnStatus);
		MAngle targetRotation1YValue = targetRotation1YHandle.asAngle();
		MDataHandle targetRotation1ZHandle = data.inputValue(targetRotation1Z, &returnStatus);
		MAngle targetRotation1ZValue = targetRotation1ZHandle.asAngle();

		MDataHandle targetRotation2XHandle = data.inputValue(targetRotation2X, &returnStatus);
		MAngle targetRotation2XValue = targetRotation2XHandle.asAngle();
		MDataHandle targetRotation2YHandle = data.inputValue(targetRotation2Y, &returnStatus);
		MAngle targetRotation2YValue = targetRotation2YHandle.asAngle();
		MDataHandle targetRotation2ZHandle = data.inputValue(targetRotation2Z, &returnStatus);
		MAngle targetRotation2ZValue = targetRotation2ZHandle.asAngle();

		MDataHandle targetJointOrient1XHandle = data.inputValue(targetJointOrient1X, &returnStatus);
		MAngle targetJointOrient1XValue = targetJointOrient1XHandle.asAngle();
		MDataHandle targetJointOrient1YHandle = data.inputValue(targetJointOrient1Y, &returnStatus);
		MAngle targetJointOrient1YValue = targetJointOrient1YHandle.asAngle();
		MDataHandle targetJointOrient1ZHandle = data.inputValue(targetJointOrient1Z, &returnStatus);
		MAngle targetJointOrient1ZValue = targetJointOrient1ZHandle.asAngle();

		MDataHandle targetJointOrient2XHandle = data.inputValue(targetJointOrient2X, &returnStatus);
		MAngle targetJointOrient2XValue = targetJointOrient2XHandle.asAngle();
		MDataHandle targetJointOrient2YHandle = data.inputValue(targetJointOrient2Y, &returnStatus);
		MAngle targetJointOrient2YValue = targetJointOrient2YHandle.asAngle();
		MDataHandle targetJointOrient2ZHandle = data.inputValue(targetJointOrient2Z, &returnStatus);
		MAngle targetJointOrient2ZValue = targetJointOrient2ZHandle.asAngle();

		MDataHandle targetPosition1XHandle = data.inputValue(targetPosition1, &returnStatus);
		MVector targetPosition1Value = targetPosition1XHandle.asVector();
		MDataHandle targetPosition2XHandle = data.inputValue(targetPosition2, &returnStatus);
		MVector targetPosition2Value = targetPosition2XHandle.asVector();


		// Calculation
		// World TMs
		MMatrix M1 = FloatMatrixToMatrix(FM1);
		MMatrix M2 = FloatMatrixToMatrix(FM2);
		MMatrix targetRotation1TM = MEulerRotation(targetRotation1XValue.value(), targetRotation1YValue.value(),
			targetRotation1ZValue.value()).asMatrix();
		MMatrix targetRotation2TM = MEulerRotation(targetRotation2XValue.value(), targetRotation2YValue.value(),
			targetRotation2ZValue.value()).asMatrix();
		MMatrix targetJointOrient1TM = MEulerRotation(targetJointOrient1XValue.value(), targetJointOrient1YValue.value(),
			targetJointOrient1ZValue.value()).asMatrix();
		MMatrix targetJointOrient2TM = MEulerRotation(targetJointOrient2XValue.value(), targetJointOrient2YValue.value(),
			targetJointOrient2ZValue.value()).asMatrix();
		MMatrix PM1(transMatrix(targetPosition1Value));
		MMatrix PM2(transMatrix(targetPosition2Value));
		MMatrix WM1 = targetRotation1TM * targetJointOrient1TM * PM1 * M1;
		MMatrix WM2 = targetRotation2TM * targetJointOrient2TM * PM2 * M2;

		// blend

		MTransformationMatrix TM1(WM1);
		MTransformationMatrix TM2(WM2);
		MQuaternion BlendQuat = slerp(TM1.rotation(), TM2.rotation(), BW);
		MTransformationMatrix worldTM(BlendQuat);
		MVector worldPos = (TM1.translation(MSpace::kWorld) * (1.0f - BW)) + (TM2.translation(MSpace::kWorld) * BW);
		worldTM.setTranslation(worldPos, MSpace::kWorld);

		// remove jointOrient and parentTM and set.
		MMatrix ipTM = FloatMatrixToMatrix(pFM);
		MMatrix localTM = worldTM.asMatrix() * ipTM;
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
			// blendRotationX
			MDataHandle blendRotationXHandle = data.outputValue(baconBlend::blendRotationX);
			blendRotationXHandle.setMAngle(MAngle(outputAngles.x));
			blendRotationXHandle.setClean();
			// blendRotationY
			MDataHandle blendRotationYHandle = data.outputValue(baconBlend::blendRotationY);
			blendRotationYHandle.setMAngle(MAngle(outputAngles.y));
			blendRotationYHandle.setClean();
			// blendRotationX
			MDataHandle blendRotationZHandle = data.outputValue(baconBlend::blendRotationZ);
			blendRotationZHandle.setMAngle(MAngle(outputAngles.z));
			blendRotationZHandle.setClean();

			// blend Position
			MDataHandle blendPositionHandle = data.outputValue(baconBlend::blendPosition);
			blendPositionHandle.set3Float(float(localTM[3][0]), float(localTM[3][1]), float(localTM[3][2]));
			blendPositionHandle.setClean();

		}
	} else {
		return MS::kUnknownParameter;
	}

	return MS::kSuccess;
}

void* baconBlend::creator()
{
	return new baconBlend();
}


MStatus baconBlend::initialize()
{
	MFnNumericAttribute		numAttr;
	MFnUnitAttribute		uAttr;
	MFnMatrixAttribute		matrixAttr;
	MStatus					stat;
	MFnCompoundAttribute	CompAttr;

	// INPUTS ---------------------------------------------------------------------
	
	// blendWeight
	blendWeight = numAttr.create("blendAmount", "ba", MFnNumericData::kFloat, 0.0);
	numAttr.setStorable(true);
	numAttr.setKeyable(true);
	numAttr.setWritable(true);
	numAttr.setMin(0.0);
	numAttr.setMax(1.0);
	stat = addAttribute(blendWeight);

	// input jointOrient
	jointOrientX = uAttr.create("jointOrientX", "uox", uAttr.kAngle, 0.0);
	uAttr.setWritable(true);
	uAttr.setStorable(true);
	jointOrientY = uAttr.create("jointOrientY", "uoy", uAttr.kAngle, 0.0);
	uAttr.setWritable(true);
	uAttr.setStorable(true);
	jointOrientZ = uAttr.create("jointOrientZ", "uoz", uAttr.kAngle, 0.0);
	uAttr.setWritable(true);
	uAttr.setStorable(true);
	jointOrient = numAttr.create("jointOrient", "jo", jointOrientX, jointOrientY, jointOrientZ);
	numAttr.setWritable(true);
	numAttr.setStorable(true);
	numAttr.setKeyable(true);
	stat = addAttribute(jointOrient);

	// input parentInverseMatrix
	parentInverseMatrix = matrixAttr.create("parentInverseMatrix", "piTM", matrixAttr.kFloat);
	matrixAttr.setStorable(true);
	matrixAttr.setKeyable(true);
	stat = addAttribute(parentInverseMatrix);



	/*
    self.bodyPartMembers = bodyPartMembersAttr.create("bodyPartMembers", "bodyPartMembers")
    bodyPartMembersAttr.setArray(True)
    bodyPartMembersAttr.setKeyable(True)

    dagNodePlug = typeAttr.create('dagMessage', 'dagMessage', OpenMaya.MFnData.kString)
    bodyPartMembersAttr.addChild(dagNodePlug)

	*/



	// Target1
	target1 = CompAttr.create("target1", "target1");
	CompAttr.setKeyable(true);

	// input targetPosition1
	targetPosition1 = numAttr.createPoint("targetPosition1", "tarpos1");
	CompAttr.addChild(targetPosition1);
	targetTM1 = matrixAttr.create("targetParentTM1", "tarPTM1", matrixAttr.kFloat);
	CompAttr.addChild(targetTM1);

	// input targetRotation1
	targetRotation1X = uAttr.create("targetRotation1X", "tarr1x", uAttr.kAngle, 0.0);
	targetRotation1Y = uAttr.create("targetRotation1Y", "tarr1y", uAttr.kAngle, 0.0);
	targetRotation1Z = uAttr.create("targetRotation1Z", "tarr1z", uAttr.kAngle, 0.0);
	targetRotation1 = numAttr.create("targetRotation1", "tarr1", targetRotation1X, targetRotation1Y, targetRotation1Z);
	CompAttr.addChild(targetRotation1);

	// input targetJointOrient1
	targetJointOrient1X = uAttr.create("targetJointOrient1X", "tarjo1x", uAttr.kAngle, 0.0);
	targetJointOrient1Y = uAttr.create("targetJointOrient1Y", "tarjo1y", uAttr.kAngle, 0.0);
	targetJointOrient1Z = uAttr.create("targetJointOrient1Z", "tarjo1z", uAttr.kAngle, 0.0);
	targetJointOrient1 = numAttr.create("targetJointOrient1", "tarjo1", targetJointOrient1X,
		targetJointOrient1Y, targetJointOrient1Z);
	CompAttr.addChild(targetJointOrient1);

	CompAttr.setHidden(false);
	stat = addAttribute(target1);

	// Target2
	target2 = CompAttr.create("target2", "target2");
	CompAttr.setKeyable(true);

	// input targetPosition2
	targetPosition2 = numAttr.createPoint("targetPosition2", "tarpos2");
	CompAttr.addChild(targetPosition2);

	// input targetTM2
	targetTM2 = matrixAttr.create("targetParentTM2", "tarPTM2", matrixAttr.kFloat);
	CompAttr.addChild(targetTM2);

	// input targetRotation2
	targetRotation2X = uAttr.create("targetRotation2X", "tarr2x", uAttr.kAngle, 0.0);
	targetRotation2Y = uAttr.create("targetRotation2Y", "tarr2y", uAttr.kAngle, 0.0);
	targetRotation2Z = uAttr.create("targetRotation2Z", "tarr2z", uAttr.kAngle, 0.0);
	targetRotation2 = numAttr.create("targetRotation2", "tarr2", targetRotation2X, targetRotation2Y, targetRotation2Z);
	CompAttr.addChild(targetRotation2);

	// input targetJointOrient2
	targetJointOrient2X = uAttr.create("targetJointOrient2X", "tarjo2x", uAttr.kAngle, 0.0);
	targetJointOrient2Y = uAttr.create("targetJointOrient2Y", "tarjo2y", uAttr.kAngle, 0.0);
	targetJointOrient2Z = uAttr.create("targetJointOrient2Z", "tarjo2z", uAttr.kAngle, 0.0);
	targetJointOrient2 = numAttr.create("targetJointOrient2", "tarjo2", targetJointOrient2X, 
		targetJointOrient2Y, targetJointOrient2Z);
	CompAttr.addChild(targetJointOrient2);

	CompAttr.setHidden(false);
	stat = addAttribute(target2);

	

	// OUTUTS ---------------------------------------------------------------------

	// Output blend rotationX
	blendRotationX = uAttr.create("blendRotationX", "brotX", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output blend rotationY
	blendRotationY = uAttr.create("blendRotationY", "brotY", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output blend rotationZ
	blendRotationZ = uAttr.create("blendRotationZ", "brotZ", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output blend rotation
	blendRotation = numAttr.create("blendRotation", "brot", blendRotationX, blendRotationY, blendRotationZ);
	numAttr.setHidden(false);
	stat = addAttribute(blendRotation);

	// Output blend Position
	blendPosition = numAttr.createPoint("blendPosition", "bpos");
	numAttr.setStorable(false);
	numAttr.setHidden(false);
	stat = addAttribute(blendPosition);

	
	//AFFECTS ---------------------------------------------------------------------
	MObject AffectedByMany[] =
	{	blendRotation,	blendPosition,
		blendRotationX,	blendRotationY,	blendRotationZ
	};
	for (MObject& obj : AffectedByMany) 
	{

		attributeAffects(targetRotation1,		obj);
		attributeAffects(targetRotation1X,		obj);
		attributeAffects(targetRotation1Y,		obj);
		attributeAffects(targetRotation1Z,		obj);
		attributeAffects(targetRotation2,		obj);
		attributeAffects(targetRotation2X,		obj);
		attributeAffects(targetRotation2Y,		obj);
		attributeAffects(targetRotation2Z,		obj);
		attributeAffects(targetJointOrient1,	obj);
		attributeAffects(targetJointOrient1X,	obj);
		attributeAffects(targetJointOrient1Y,	obj);
		attributeAffects(targetJointOrient1Z,	obj);
		attributeAffects(targetJointOrient2,	obj);
		attributeAffects(targetJointOrient2X,	obj);
		attributeAffects(targetJointOrient2Y,	obj);
		attributeAffects(targetJointOrient2Z,	obj);

		attributeAffects(jointOrient,			obj);
		attributeAffects(jointOrientX,			obj);
		attributeAffects(jointOrientY,			obj);
		attributeAffects(jointOrientZ,			obj);
		attributeAffects(targetTM1,				obj);
		attributeAffects(targetTM2,				obj);
		attributeAffects(parentInverseMatrix,	obj);
		attributeAffects(blendWeight,			obj);
	}

	return MS::kSuccess;
}

MStatus initializePlugin( MObject obj )
{ 
	MStatus   status;
	MFnPlugin plugin(obj, "Bacon-Strip.com", "1.0", "Any");

	status = plugin.registerNode( "baconBlend", baconBlend::id, baconBlend::creator,
								  baconBlend::initialize );
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

	status = plugin.deregisterNode( baconBlend::id );
	if (!status) {
		status.perror("deregisterNode");
		return status;
	}

	return status;
}
