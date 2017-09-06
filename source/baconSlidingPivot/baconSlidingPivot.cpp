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
// Produces the dependency graph node "baconSlidingPivot".
// This plug-in is part of a rigging package known as The "Bacon-Strip Rig"
// Bacon-Strip ID Block is : 0x0012a940 - 0x0012aa3f
// Registered for Bacon-Strip[Luis Alonso]
//
// ID: 
// 0x0012a956
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

 
class baconSlidingPivot : public MPxNode
{
public:
						baconSlidingPivot();
	virtual				~baconSlidingPivot(); 

	virtual MStatus		compute( const MPlug& plug, MDataBlock& data );

	static  void*		creator();
	static  MStatus		initialize();

public:
	static	MTypeId		id;

	// inputs
	static  MObject		originMatrix;
	static  MObject		contactOriginMatrix;
	static  MObject		floorWorldMatrix;
	static  MObject		contactWorldMatrix;
	static  MObject		jointOrient;
	static  MObject		jointOrientX;
	static  MObject		jointOrientY;
	static  MObject		jointOrientZ;

	// ouputs
	static  MObject		slidingRotation;
	static  MObject		slidingRotationX;
	static  MObject		slidingRotationY;
	static  MObject		slidingRotationZ;
	static  MObject		slidingPosition;

};

MTypeId     baconSlidingPivot::id( 0x0012a956 );
MObject		baconSlidingPivot::floorWorldMatrix;
MObject		baconSlidingPivot::contactWorldMatrix;
MObject		baconSlidingPivot::originMatrix;
MObject		baconSlidingPivot::contactOriginMatrix;
MObject		baconSlidingPivot::jointOrient;
MObject		baconSlidingPivot::jointOrientX;
MObject		baconSlidingPivot::jointOrientY;
MObject		baconSlidingPivot::jointOrientZ;
MObject		baconSlidingPivot::slidingRotation;
MObject		baconSlidingPivot::slidingRotationX;
MObject		baconSlidingPivot::slidingRotationY;
MObject		baconSlidingPivot::slidingRotationZ;
MObject		baconSlidingPivot::slidingPosition;

baconSlidingPivot::baconSlidingPivot() {}
baconSlidingPivot::~baconSlidingPivot() {}

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

MMatrix FloatMatrixToMatrix(MFloatMatrix fTM)
{
	MMatrix returnTM = MMatrix();
	returnTM = setRow(returnTM, MVector(fTM[0][0], fTM[0][1], fTM[0][2]), 0);
	returnTM = setRow(returnTM, MVector(fTM[1][0], fTM[1][1], fTM[1][2]), 1);
	returnTM = setRow(returnTM, MVector(fTM[2][0], fTM[2][1], fTM[2][2]), 2);
	returnTM = setRow(returnTM, MVector(fTM[3][0], fTM[3][1], fTM[3][2]), 3);
	return returnTM;
}

MStatus baconSlidingPivot::compute( const MPlug& plug, MDataBlock& data )
{
	
	MStatus returnStatus;
 
	if
	( 
		plug == slidingRotation		|| plug == slidingPosition		||
		plug == slidingRotationX	|| plug == slidingRotationY		|| plug == slidingRotationZ
	)
	{
		// Handles and Values
		MDataHandle originMatrixHandle = data.inputValue(originMatrix, &returnStatus);
		MFloatMatrix originFM(originMatrixHandle.asFloatMatrix());

		MDataHandle contactOriginMatrixHandle = data.inputValue(contactOriginMatrix, &returnStatus);
		MFloatMatrix contactOriginFM(contactOriginMatrixHandle.asFloatMatrix());

		MDataHandle contactWorldMatrixHandle = data.inputValue(contactWorldMatrix, &returnStatus);
		MFloatMatrix contactFM(contactWorldMatrixHandle.asFloatMatrix());

		MDataHandle floorWorldMatrixHandle = data.inputValue(floorWorldMatrix, &returnStatus);
		MFloatMatrix floorFM(floorWorldMatrixHandle.asFloatMatrix());


		MDataHandle jointOrientXHandle = data.inputValue(jointOrientX, &returnStatus);
		MAngle jointOrientXValue = jointOrientXHandle.asAngle();
		MDataHandle jointOrientYHandle = data.inputValue(jointOrientY, &returnStatus);
		MAngle jointOrientYValue = jointOrientYHandle.asAngle();
		MDataHandle jointOrientZHandle = data.inputValue(jointOrientZ, &returnStatus);
		MAngle jointOrientZValue = jointOrientZHandle.asAngle();


		// Calculation

		// inputs 
		MMatrix originTM = FloatMatrixToMatrix(originFM);
		MMatrix contactOriginTM = FloatMatrixToMatrix(contactOriginFM);
		MMatrix contactTM = FloatMatrixToMatrix(contactFM);
		MMatrix floorTM = FloatMatrixToMatrix(floorFM);


		// Roll From Origin
		MMatrix rollOriginWorld = contactOriginTM * floorTM;
		MMatrix rollTM = contactTM * rollOriginWorld.inverse();
		rollTM[3][0] = 0;
		rollTM[3][1] = 0;
		rollTM[3][2] = 0;
		rollTM[3][3] = 1;

		// Sliding matrix
		MMatrix sideLocalTM = (originTM * floorTM) * contactTM.inverse();
		MMatrix sideWorld = (sideLocalTM * rollTM) * contactTM;
		MMatrix localTM = sideWorld * floorTM.inverse();

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
			// slidingRotationX
			MDataHandle slidingRotationXHandle = data.outputValue(baconSlidingPivot::slidingRotationX);
			slidingRotationXHandle.setMAngle(MAngle(outputAngles.x));
			slidingRotationXHandle.setClean();
			// slidingRotationY
			MDataHandle slidingRotationYHandle = data.outputValue(baconSlidingPivot::slidingRotationY);
			slidingRotationYHandle.setMAngle(MAngle(outputAngles.y));
			slidingRotationYHandle.setClean();
			// slidingRotationX
			MDataHandle slidingRotationZHandle = data.outputValue(baconSlidingPivot::slidingRotationZ);
			slidingRotationZHandle.setMAngle(MAngle(outputAngles.z));
			slidingRotationZHandle.setClean();

			// aligned Position
			MDataHandle slidingPositionHandle = data.outputValue(baconSlidingPivot::slidingPosition);
			slidingPositionHandle.set3Float(localTM[3][0], localTM[3][1], localTM[3][2]);
				//setMVector(MVector(outputTM[3][0], outputTM[3][1], outputTM[3][2]));
			slidingPositionHandle.setClean();

		}
	} else {
		return MS::kUnknownParameter;
	}

	return MS::kSuccess;
}

void* baconSlidingPivot::creator()
{
	return new baconSlidingPivot();
}



MStatus baconSlidingPivot::initialize()
{
	MFnNumericAttribute numAttr;
	MFnUnitAttribute	uAttr;
	MFnMatrixAttribute	matrixAttr;
	MStatus				stat;

	// INPUTS ---------------------------------------------------------------------
	
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
		
	// input contactWorldMatrix
	contactWorldMatrix = matrixAttr.create("worldContactMatrix", "wcTM", matrixAttr.kFloat);
	matrixAttr.setStorable(true);
	matrixAttr.setKeyable(true);
	stat = addAttribute(contactWorldMatrix);

	// input floorWorldMatrix
	floorWorldMatrix = matrixAttr.create("worldFloorMatrix", "wrTM", matrixAttr.kFloat);
	matrixAttr.setStorable(true);
	matrixAttr.setKeyable(true);
	stat = addAttribute(floorWorldMatrix);

	// input originMatrix
	originMatrix = matrixAttr.create("originRootMatrix", "orTM", matrixAttr.kFloat);
	matrixAttr.setStorable(true);
	matrixAttr.setKeyable(true);
	stat = addAttribute(originMatrix);

	// input originMatrix
	contactOriginMatrix = matrixAttr.create("originContactMatrix", "ocTM", matrixAttr.kFloat);
	matrixAttr.setStorable(true);
	matrixAttr.setKeyable(true);
	stat = addAttribute(contactOriginMatrix);


	// OUTUTS ---------------------------------------------------------------------

	// Output aligned rotationX
	slidingRotationX = uAttr.create("slidingRotationX", "arotX", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output aligned rotationY
	slidingRotationY = uAttr.create("slidingRotationY", "arotY", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output aligned rotationZ
	slidingRotationZ = uAttr.create("slidingRotationZ", "arotZ", uAttr.kAngle, 0.0);
	uAttr.setWritable(false);
	// Output aligned rotation
	slidingRotation = numAttr.create("slidingRotation", "arot", slidingRotationX, slidingRotationY, slidingRotationZ);
	numAttr.setHidden(false);
	stat = addAttribute(slidingRotation);

	// Output Aligned Position
	slidingPosition = numAttr.createPoint("slidingPosition", "apos");
	numAttr.setStorable(false);
	numAttr.setHidden(false);
	stat = addAttribute(slidingPosition);

	
	//AFFECTS ---------------------------------------------------------------------
	MObject AffectedByMany[] =
	{	slidingRotation,	slidingPosition,
		slidingRotationX,	slidingRotationY,	slidingRotationZ
	};
	for (MObject& obj : AffectedByMany) 
	{
		attributeAffects(jointOrient,			obj);
		attributeAffects(jointOrientX,			obj);
		attributeAffects(jointOrientY,			obj);
		attributeAffects(jointOrientZ,			obj);
		attributeAffects(originMatrix,			obj);
		attributeAffects(contactOriginMatrix,	obj);
		attributeAffects(floorWorldMatrix,		obj);
		attributeAffects(contactWorldMatrix,	obj);
	}
	return MS::kSuccess;
}

MStatus initializePlugin( MObject obj )
{ 
	MStatus   status;
	MFnPlugin plugin(obj, "Bacon-Strip.com", "1.0", "Any");

	status = plugin.registerNode( "baconSlidingPivot", baconSlidingPivot::id, baconSlidingPivot::creator,
								  baconSlidingPivot::initialize );
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

	status = plugin.deregisterNode( baconSlidingPivot::id );
	if (!status) {
		status.perror("deregisterNode");
		return status;
	}

	return status;
}
