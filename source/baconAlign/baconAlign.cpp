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
// Produces the dependency graph node "baconAlign".
// This plug-in is part of a rigging package known as The "Bacon-Strip Rig"
// Bacon-Strip ID Block is : 0x0012a940 - 0x0012aa3f
// Registered for Bacon-Strip[Luis Alonso]
//
// ID: 
// 0x0012a950
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

 
class baconAlign : public MPxNode
{
public:
						baconAlign();
	virtual				~baconAlign(); 

	virtual MStatus		compute( const MPlug& plug, MDataBlock& data );

	static  void*		creator();
	static  MStatus		initialize();

public:
	static	MTypeId		id;

	// inputs
	static  MObject		weight;						// The weight value.
	static  MObject		targetWorldMatrix;
	static  MObject		parentInverseMatrix;
	static  MObject		offsetMatrix;
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

MTypeId     baconAlign::id( 0x0012a950 );
MObject     baconAlign::weight;        
MObject		baconAlign::targetWorldMatrix;
MObject		baconAlign::parentInverseMatrix;
MObject		baconAlign::offsetMatrix;
MObject		baconAlign::jointOrient;
MObject		baconAlign::jointOrientX;
MObject		baconAlign::jointOrientY;
MObject		baconAlign::jointOrientZ;
MObject		baconAlign::alignedRotation;
MObject		baconAlign::alignedRotationX;
MObject		baconAlign::alignedRotationY;
MObject		baconAlign::alignedRotationZ;
MObject		baconAlign::alignedPosition;

baconAlign::baconAlign() {}
baconAlign::~baconAlign() {}

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

MStatus baconAlign::compute( const MPlug& plug, MDataBlock& data )
{
	
	MStatus returnStatus;
 
	if
	( 
		plug == alignedRotation		|| plug == alignedPosition		||
		plug == alignedRotationX	|| plug == alignedRotationY		|| plug == alignedRotationZ
	)
	{
		// Handles and Values
		MDataHandle targetWorldMatrixHandle = data.inputValue(targetWorldMatrix, &returnStatus);
		MFloatMatrix tFM(targetWorldMatrixHandle.asFloatMatrix());

		MDataHandle parentInverseMatrixHandle = data.inputValue(parentInverseMatrix, &returnStatus);
		MFloatMatrix pFM = parentInverseMatrixHandle.asFloatMatrix();

		MDataHandle offsetMatrixHandle = data.inputValue(offsetMatrix, &returnStatus);
		MFloatMatrix offsetFM = offsetMatrixHandle.asFloatMatrix();

		MDataHandle jointOrientXHandle = data.inputValue(jointOrientX, &returnStatus);
		MAngle jointOrientXValue = jointOrientXHandle.asAngle();
		MDataHandle jointOrientYHandle = data.inputValue(jointOrientY, &returnStatus);
		MAngle jointOrientYValue = jointOrientYHandle.asAngle();
		MDataHandle jointOrientZHandle = data.inputValue(jointOrientZ, &returnStatus);
		MAngle jointOrientZValue = jointOrientZHandle.asAngle();


		// Calculation
		MMatrix targetTM = FloatMatrixToMatrix(tFM);
		MMatrix iparentTM = FloatMatrixToMatrix(pFM);
		MMatrix offsetTM = FloatMatrixToMatrix(offsetFM);
		MMatrix localTM = (offsetTM * targetTM) * iparentTM;
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
			MDataHandle alignedRotationXHandle = data.outputValue(baconAlign::alignedRotationX);
			alignedRotationXHandle.setMAngle(MAngle(outputAngles.x));
			alignedRotationXHandle.setClean();
			// alignedRotationY
			MDataHandle alignedRotationYHandle = data.outputValue(baconAlign::alignedRotationY);
			alignedRotationYHandle.setMAngle(MAngle(outputAngles.y));
			alignedRotationYHandle.setClean();
			// alignedRotationX
			MDataHandle alignedRotationZHandle = data.outputValue(baconAlign::alignedRotationZ);
			alignedRotationZHandle.setMAngle(MAngle(outputAngles.z));
			alignedRotationZHandle.setClean();

			// aligned Position
			MDataHandle alignedPositionHandle = data.outputValue(baconAlign::alignedPosition);
			alignedPositionHandle.set3Double(localTM[3][0], localTM[3][1], localTM[3][2]);
			alignedPositionHandle.setClean();

		}
	} else {
		return MS::kUnknownParameter;
	}

	return MS::kSuccess;
}

void* baconAlign::creator()
{
	return new baconAlign();
}


MStatus baconAlign::initialize()
{
	MFnNumericAttribute numAttr;
	MFnUnitAttribute	uAttr;
	MFnMatrixAttribute	matrixAttr;
	MStatus				stat;

	// INPUTS ---------------------------------------------------------------------
	
	// weight
	weight = numAttr.create("weight", "w", MFnNumericData::kFloat, 0.0);
	numAttr.setStorable(true);
	numAttr.setKeyable(true);
	numAttr.setWritable(true);
	
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

	// input offsetMatrix
	offsetMatrix = matrixAttr.create("offsetMatrix", "offfsetTM", matrixAttr.kFloat);
	matrixAttr.setStorable(true);
	matrixAttr.setKeyable(true);
	stat = addAttribute(offsetMatrix);

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
		attributeAffects(jointOrient,			obj);
		attributeAffects(jointOrientX,			obj);
		attributeAffects(jointOrientY,			obj);
		attributeAffects(jointOrientZ,			obj);
		attributeAffects(targetWorldMatrix,		obj);
		attributeAffects(parentInverseMatrix,	obj);
		attributeAffects(offsetMatrix, obj);
	}

	return MS::kSuccess;
}

MStatus initializePlugin( MObject obj )
{ 
	MStatus   status;
	MFnPlugin plugin( obj, "Bacon-Strip.com", "1.0", "Any");

	status = plugin.registerNode( "baconAlign", baconAlign::id, baconAlign::creator,
								  baconAlign::initialize );
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

	status = plugin.deregisterNode( baconAlign::id );
	if (!status) {
		status.perror("deregisterNode");
		return status;
	}

	return status;
}
