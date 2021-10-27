// *************************************************************************************************
//
// Horde3D
//   Next-Generation Graphics Engine
// --------------------------------------
// Copyright (C) 2006-2021 Nicolas Schulz and Horde3D team
//
// This software is distributed under the terms of the Eclipse Public License v1.0.
// A copy of the license may be obtained at: http://www.eclipse.org/legal/epl-v10.html
//
// *************************************************************************************************

#include "utEndian.h"
#include "egGeometry.h"
#include "egResource.h"
#include "egAnimation.h"
#include "egModules.h"
#include "egCom.h"
#include "egRenderer.h"
#include <cstring>

#include "utDebug.h"


namespace Horde3D {

using namespace std;


uint32 GeometryResource::defVertBuffer = 0;
uint32 GeometryResource::defIndexBuffer = 0;
int GeometryResource::mappedWriteStream = -1;


void GeometryResource::initializationFunc()
{
	defVertBuffer = Modules::renderer().getRenderDevice()->createVertexBuffer( 0, 0x0 );
	defIndexBuffer = Modules::renderer().getRenderDevice()->createIndexBuffer( 0, 0x0 );
}


void GeometryResource::releaseFunc()
{
	Modules::renderer().getRenderDevice()->destroyBuffer( defVertBuffer );
	Modules::renderer().getRenderDevice()->destroyBuffer( defIndexBuffer );
}


GeometryResource::GeometryResource( const string &name, int flags ) :
	Resource( ResourceTypes::Geometry, name, flags )
{
	initDefault();
}


GeometryResource::~GeometryResource()
{
	release();
}


Resource *GeometryResource::clone()
{
	GeometryResource *res = new GeometryResource( "", _flags );

	*res = *this;

	RenderDeviceInterface *rdi = Modules::renderer().getRenderDevice();

	// TODO: Check if elemcpy_le should be used
	// Make a deep copy of the data
	res->_indexData = new char[_indexCount * (_16BitIndices ? 2 : 4)];
	res->_vertPosData = new Vec3f[_vertCount];
	res->_vertTanData = new VertexDataTan[_vertCount];
	res->_vertStaticData = new VertexDataStatic[_vertCount];
	memcpy( res->_indexData, _indexData, _indexCount * (_16BitIndices ? 2 : 4) );
	memcpy( res->_vertPosData, _vertPosData, _vertCount * sizeof( Vec3f ) );
	memcpy( res->_vertTanData, _vertTanData, _vertCount * sizeof( VertexDataTan ) );
	memcpy( res->_vertStaticData, _vertStaticData, _vertCount * sizeof( VertexDataStatic ) );

	res->_16BitIndices = _16BitIndices;
	res->_geoObj = rdi->beginCreatingGeometry( Modules::renderer().getDefaultVertexLayout( DefaultVertexLayouts::Model ) );

	res->_indexBuf = rdi->createIndexBuffer( _indexCount * (_16BitIndices ? 2 : 4), _indexData );
	res->_posVBuf = rdi->createVertexBuffer( _vertCount * sizeof( Vec3f ), _vertPosData );
	res->_tanVBuf = rdi->createVertexBuffer( _vertCount * sizeof( VertexDataTan ), _vertTanData );
	res->_staticVBuf = rdi->createVertexBuffer( _vertCount * sizeof( VertexDataStatic ), _vertStaticData );

	rdi->setGeomVertexParams( res->_geoObj, res->_posVBuf, 0, 0, sizeof( Vec3f ) );
	rdi->setGeomVertexParams( res->_geoObj, res->_tanVBuf, 1, 0, sizeof( VertexDataTan ) );
	rdi->setGeomVertexParams( res->_geoObj, res->_tanVBuf, 2, sizeof( Vec3f ), sizeof( VertexDataTan ) );
	rdi->setGeomVertexParams( res->_geoObj, res->_staticVBuf, 3, 0, sizeof( VertexDataStatic ) );
	rdi->setGeomIndexParams( res->_geoObj, res->_indexBuf, _16BitIndices ? IDXFMT_16 : IDXFMT_32 );

	rdi->finishCreatingGeometry( res->_geoObj );

	return res;
}

GeometryResource *GeometryResource::initModularGeo()
{
	GeometryResource *res = new GeometryResource( "|tmpZob|", _flags );

//	*res = *this;
	// RES members
//	res->_name = "";
	res->_type = _type;
	res->_userRefCount = 1;
	res->_refCount = 0;

	res->_loaded = true;
	res->_noQuery = _noQuery;

	// GEO RES members
	res->_minMorphIndex = _minMorphIndex;
	res->_maxMorphIndex = _maxMorphIndex;
	res->_skelAABB = _skelAABB;
	res->_16BitIndices = _16BitIndices;

	copy(_joints.begin(), _joints.end(), back_inserter(res->_joints));
	copy(_morphTargets.begin(), _morphTargets.end(), back_inserter(res->_morphTargets));

	res->_indexCount = _indexCount;
	res->_vertCount = _vertCount;

	// TODO: Check if elemcpy_le should be used
	// Make a deep copy of the data
	res->_indexData = new char[_indexCount * (_16BitIndices ? 2 : 4)];
	res->_vertPosData = new Vec3f[_vertCount];
	res->_vertTanData = new VertexDataTan[_vertCount];
	res->_vertStaticData = new VertexDataStatic[_vertCount];
	memcpy( res->_indexData, _indexData, _indexCount * (_16BitIndices ? 2 : 4) );
	memcpy( res->_vertPosData, _vertPosData, _vertCount * sizeof( Vec3f ) );
	memcpy( res->_vertTanData, _vertTanData, _vertCount * sizeof( VertexDataTan ) );
	memcpy( res->_vertStaticData, _vertStaticData, _vertCount * sizeof( VertexDataStatic ) );

	return res;
}

void GeometryResource::appendModularGeo(GeometryResource* other_res, float* seamx, float* seamy, float* seamz, uint seamCount, bool x_sym)
{

	// this geo data before append
	char* tmpIndex = _indexData;
	Vec3f* tmpVertPos = _vertPosData;
	VertexDataTan* tmpTan = _vertTanData;
	VertexDataStatic* tmpStatic = _vertStaticData;
	unsigned int tmpIndCount = _indexCount;
	unsigned int tmpVertCount = _vertCount;

	// updated counts + prepare arrays
	_indexCount = tmpIndCount + other_res->_indexCount;
	_vertCount = tmpVertCount + other_res->_vertCount;

	_indexData = new char[_indexCount * (_16BitIndices ? 2 : 4)];
	_vertPosData = new Vec3f[_vertCount];
	_vertTanData = new VertexDataTan[_vertCount];
	_vertStaticData = new VertexDataStatic[_vertCount];


	ulong tmpIndSize = tmpIndCount * (_16BitIndices ? 2 : 4);
	memcpy(_indexData, tmpIndex, tmpIndSize);

	ulong tmpVertPosSize = tmpVertCount * sizeof( Vec3f );
	memcpy(_vertPosData, tmpVertPos, tmpVertPosSize);

	ulong tmpTanSize = tmpVertCount * sizeof( VertexDataTan );
	memcpy(_vertTanData, tmpTan, tmpTanSize);

	ulong tmpStaticSize = tmpVertCount * sizeof( VertexDataStatic );
	memcpy(_vertStaticData, tmpStatic, tmpStaticSize);


	for (uint i = 0; i < other_res->_indexCount; i++){
		((uint16*)_indexData)[i+tmpIndCount] = ((uint16*)other_res->_indexData)[i];
	}

	// offset indices
	if( _16BitIndices )
	{
		for (uint i = tmpIndCount; i < _indexCount; i++){
			uint16 t = ((uint16*)_indexData)[i] + tmpVertCount;
			memcpy( _indexData + i*2 , &t, sizeof (uint16));
		}
	}
//	else
//	{

//	}

	const float thresh = 0.0001f;
	const uint cap = 2147483647;

	int offset  = 0;

	for (uint i = 0; i < other_res->_vertCount; i++){

		bool in_sym = false;// symetric of same vertex was found

		// search for overlapping vert between 2 meshes
		// we have a list of overlapping positions : the seam
		uint seamInd = cap;
		for (uint j = 0; j < seamCount; j++){
			if (fabsf(other_res->_vertPosData[i].x - seamx[j]) < thresh  &&
					fabsf(other_res->_vertPosData[i].y - seamy[j]) < thresh &&
					fabsf(other_res->_vertPosData[i].z - seamz[j]) < thresh){
				seamInd = j;
				break;
			}

			if (x_sym){
				if (fabsf(other_res->_vertPosData[i].x + seamx[j]) < thresh  &&
						fabsf(other_res->_vertPosData[i].y - seamy[j]) < thresh &&
						fabsf(other_res->_vertPosData[i].z - seamz[j]) < thresh){
					seamInd = j;
					in_sym = true;
					break;
				}
			}
		}

		if (seamInd == cap){
			// this is a new unique vert
			_vertPosData[i + tmpVertCount - offset] = other_res->_vertPosData[i];
			_vertTanData[i + tmpVertCount - offset] = other_res->_vertTanData[i];
			_vertStaticData[i + tmpVertCount - offset] = other_res->_vertStaticData[i];
			for (uint k = tmpIndCount; k < _indexCount; k++){
				// some index in the list disappered
				if (((uint16*)_indexData)[k] == i + tmpVertCount){
					((uint16*)_indexData)[k] -= offset;
				}
			}
		}else{
			// this vert already exists
			// find index in base part
			offset++;
			uint baseInd = cap;

			if (in_sym){
				for (uint j = 0; j < tmpVertCount; j++){
					if (fabsf(_vertPosData[j].x + seamx[seamInd]) < thresh  &&
							fabsf(_vertPosData[j].y - seamy[seamInd]) < thresh &&
							fabsf(_vertPosData[j].z - seamz[seamInd]) < thresh){
						baseInd = j;
						break;
					}
				}
			}else{
				for (uint j = 0; j < tmpVertCount; j++){
					if (fabsf(_vertPosData[j].x - seamx[seamInd]) < thresh  &&
							fabsf(_vertPosData[j].y - seamy[seamInd]) < thresh &&
							fabsf(_vertPosData[j].z - seamz[seamInd]) < thresh){
						baseInd = j;
						break;
					}
				}
			}

			if (baseInd == cap){
				Modules::log().writeDebugInfo( "didn't find vertex in base geo");
			}

			// update index list
			for (uint k = tmpIndCount; k < _indexCount; k++){
				// any indice pointing to this vertex should point to its clone from the base geo
				if (((uint16*)_indexData)[k] == i + tmpVertCount){
					((uint16*)_indexData)[k] = baseInd;
				}
			}
		}
	}

//	Modules::log().writeDebugInfo( "ind count %d", _indexCount);
//	Modules::log().writeDebugInfo( "vert count %d", _vertCount);
//	Modules::log().writeDebugInfo( "joint count %d", _joints.size());
//	Modules::log().writeDebugInfo( "joint count %d", other_res->_joints.size());

	/*
	Modules::log().writeDebugInfo( "Searching seam verts ------------------------");

	for (uint i = 0; i < seamCount; i++){
//		Modules::log().writeDebugInfo( "seam x= %f", seamx[i]);

		// find one vertex in old geo and one vertex in new geo with those coords
		uint prev_ind = cap;
		uint appe_ind = cap;

		for (uint j = 0; j < tmpVertCount; j++){
			if (fabsf(_vertPosData[j].x - seamx[i]) < thresh  &&
					fabsf(_vertPosData[j].y - seamy[i]) < thresh &&
					fabsf(_vertPosData[j].z - seamz[i]) < thresh){
				prev_ind = j;
				break;
			}
		}

		for (uint j = tmpVertCount; j < _vertCount; j++){
			if (fabsf(_vertPosData[j].x - seamx[i]) < thresh  &&
					fabsf(_vertPosData[j].y - seamy[i]) < thresh &&
					fabsf(_vertPosData[j].z - seamz[i]) < thresh){
				appe_ind = j;
				break;
			}
		}

//		if (prev_ind != cap)
//			Modules::log().writeDebugInfo( "prev found %d   %f = %f", i , _vertPosData[prev_ind].x, seamx[i]);
//		if(appe_ind != cap)
//			Modules::log().writeDebugInfo( "append found %d", i);

		if (prev_ind != cap && appe_ind != cap){
			// fix normal
//			Modules::log().writeDebugInfo( "prev normal  %f  %f  %f",
//				_vertTanData[prev_ind].normal.x , _vertTanData[prev_ind].normal.y, _vertTanData[prev_ind].normal.z);
//			Modules::log().writeDebugInfo( "appe normal  %f  %f  %f",
//				_vertTanData[appe_ind].normal.x , _vertTanData[appe_ind].normal.y, _vertTanData[appe_ind].normal.z);

			_vertTanData[appe_ind] =_vertTanData[prev_ind];
		}
	}
	*/

	delete tmpIndex;
	delete tmpVertPos;
	delete tmpTan;
	delete tmpStatic;
}

void GeometryResource::completeModularGeo()
{
	RenderDeviceInterface *rdi = Modules::renderer().getRenderDevice();

	_geoObj = rdi->beginCreatingGeometry( Modules::renderer().getDefaultVertexLayout( DefaultVertexLayouts::Model ) );

	_indexBuf = rdi->createIndexBuffer( _indexCount * (_16BitIndices ? 2 : 4), _indexData );
	_posVBuf = rdi->createVertexBuffer( _vertCount * sizeof( Vec3f ), _vertPosData );
	_tanVBuf = rdi->createVertexBuffer( _vertCount * sizeof( VertexDataTan ), _vertTanData );
	_staticVBuf = rdi->createVertexBuffer( _vertCount * sizeof( VertexDataStatic ), _vertStaticData );

	rdi->setGeomVertexParams( _geoObj, _posVBuf, 0, 0, sizeof( Vec3f ) );
	rdi->setGeomVertexParams( _geoObj, _tanVBuf, 1, 0, sizeof( VertexDataTan ) );
	rdi->setGeomVertexParams( _geoObj, _tanVBuf, 2, sizeof( Vec3f ), sizeof( VertexDataTan ) );
	rdi->setGeomVertexParams( _geoObj, _staticVBuf, 3, 0, sizeof( VertexDataStatic ) );
	rdi->setGeomIndexParams( _geoObj, _indexBuf, _16BitIndices ? IDXFMT_16 : IDXFMT_32 );

	rdi->finishCreatingGeometry( _geoObj );

//	Modules::log().writeDebugInfo( "completeModularGeo");
}


void GeometryResource::initDefault()
{
	_indexCount = 0;
	_vertCount = 0;
	_indexData = 0x0;
	_vertPosData = 0x0;
	_vertTanData = 0x0;
	_vertStaticData = 0x0;
	_16BitIndices = false;
	_indexBuf = defIndexBuffer;
	_posVBuf = defVertBuffer;
	_tanVBuf = defVertBuffer;
	_staticVBuf = defVertBuffer;
	_geoObj = 0;
	_minMorphIndex = 0; _maxMorphIndex = 0;
	_skelAABB.min = Vec3f( 0, 0, 0 );
	_skelAABB.max = Vec3f( 0, 0, 0 );
}


void GeometryResource::release()
{
	RenderDeviceInterface *rdi = Modules::renderer().getRenderDevice();

	if ( _geoObj != 0 )
		rdi->destroyGeometry( _geoObj, false );

	if( _posVBuf != 0 && _posVBuf != defVertBuffer )
		rdi->destroyBuffer( _posVBuf );
	if( _tanVBuf != 0 && _tanVBuf != defVertBuffer )
		rdi->destroyBuffer( _tanVBuf );
	if( _staticVBuf != 0 && _staticVBuf != defVertBuffer )
		rdi->destroyBuffer( _staticVBuf );
	
	if( _indexBuf != 0 && _indexBuf != defIndexBuffer )
		rdi->destroyBuffer( _indexBuf );

	delete[] _indexData; _indexData = 0x0;
	delete[] _vertPosData; _vertPosData = 0x0;
	delete[] _vertTanData; _vertTanData = 0x0;
	delete[] _vertStaticData; _vertStaticData = 0x0;
	
	_joints.clear();
	_morphTargets.clear();
}


bool GeometryResource::raiseError( const string &msg )
{
	// Reset
	release();
	initDefault();

	Modules::log().writeError( "Geometry resource '%s': %s", _name.c_str(), msg.c_str() );
	
	return false;
}


bool GeometryResource::load( const char *data, int size )
{
	if( !Resource::load( data, size ) ) return false;

	// Make sure header is available
	if( size < 8 )
		return raiseError( "Invalid geometry resource" );
	
	char *pData = (char *)data;
	
	// Check header and version
	char id[4];
	pData = elemcpy_le(id, (char*)(pData), 4);
	if( id[0] != 'H' || id[1] != '3' || id[2] != 'D' || id[3] != 'G' )
		return raiseError( "Invalid geometry resource" );

	uint32 version;
	pData = elemcpy_le(&version, (uint32*)(pData), 1);
	if( version != 5 ) return raiseError( "Unsupported version of geometry file" );

	// Load joints
	uint32 count;
	pData = elemcpy_le(&count, (uint32*)(pData), 1);

	if ( count > Modules::renderer().getRenderDevice()->getCaps().maxJointCount )
	{
		Modules::log().writeWarning( "Geometry resource '%s': Model has more than %d joints; this may cause defective behavior", _name.c_str(),
									  Modules::renderer().getRenderDevice()->getCaps().maxJointCount );
	}

	_joints.resize( count );
	for( uint32 i = 0; i < count; ++i )
	{
		Joint &joint = _joints[i];
		
		// Inverse bind matrix
		for( uint32 j = 0; j < 16; ++j )
		{
			pData = elemcpy_le(&joint.invBindMat.x[j], (float*)(pData), 1);
		}
	}
	
	// Load vertex stream data
	uint32 streamSize;
	pData = elemcpy_le(&count, (uint32*)(pData), 1);		// Number of streams
	pData = elemcpy_le(&streamSize, (uint32*)(pData), 1);	// Number of vertices

	_vertCount = streamSize;
	_vertPosData = new Vec3f[_vertCount];
	_vertTanData = new VertexDataTan[_vertCount];
	_vertStaticData = new VertexDataStatic[_vertCount];
	Vec3f *bitangents = new Vec3f[_vertCount];

	// Init with default data
	memset( _vertPosData, 0, _vertCount * sizeof( Vec3f ) );
	memset( _vertTanData, 0, _vertCount * sizeof( VertexDataTan ) );
	memset( _vertStaticData, 0, _vertCount * sizeof( VertexDataStatic ) );
	for( uint32 i = 0; i < _vertCount; ++i ) _vertStaticData[i].weightVec[0] = 1;

	for( uint32 i = 0; i < count; ++i )
	{
		unsigned char uc;
		short sh;
		uint32 streamID, streamElemSize;
		pData = elemcpy_le(&streamID, (uint32*)(pData), 1);
		pData = elemcpy_le(&streamElemSize, (uint32*)(pData), 1);
		std::string errormsg;

		switch( streamID )
		{
		case 0:		// Position
			if( streamElemSize != 12 )
			{
				errormsg = "Invalid position base stream";
				break;
			}
			for( uint32 j = 0; j < streamSize; ++j )
			{
				pData = elemcpy_le(&_vertPosData[j].x, (float*)(pData), 1);
				pData = elemcpy_le(&_vertPosData[j].y, (float*)(pData), 1);
				pData = elemcpy_le(&_vertPosData[j].z, (float*)(pData), 1);
			}
			break;
		case 1:		// Normal
			if( streamElemSize != 6 )
			{
				errormsg = "Invalid normal base stream";
				break;
			}
			for( uint32 j = 0; j < streamSize; ++j )
			{
				pData = elemcpy_le(&sh, (short*)(pData), 1); _vertTanData[j].normal.x = sh / 32767.0f;
				pData = elemcpy_le(&sh, (short*)(pData), 1); _vertTanData[j].normal.y = sh / 32767.0f;
				pData = elemcpy_le(&sh, (short*)(pData), 1); _vertTanData[j].normal.z = sh / 32767.0f;
			}
			break;
		case 2:		// Tangent
			if( streamElemSize != 6 )
			{
				errormsg = "Invalid tangent base stream";
				break;
			}
			for( uint32 j = 0; j < streamSize; ++j )
			{
				pData = elemcpy_le(&sh, (short*)(pData), 1); _vertTanData[j].tangent.x = sh / 32767.0f;
				pData = elemcpy_le(&sh, (short*)(pData), 1); _vertTanData[j].tangent.y = sh / 32767.0f;
				pData = elemcpy_le(&sh, (short*)(pData), 1); _vertTanData[j].tangent.z = sh / 32767.0f;
			}
			break;
		case 3:		// Bitangent
			if( streamElemSize != 6 )
			{
				errormsg = "Invalid bitangent base stream";
				break;
			}
			for( uint32 j = 0; j < streamSize; ++j )
			{
				pData = elemcpy_le(&sh, (short*)(pData), 1); bitangents[j].x = sh / 32767.0f;
				pData = elemcpy_le(&sh, (short*)(pData), 1); bitangents[j].y = sh / 32767.0f;
				pData = elemcpy_le(&sh, (short*)(pData), 1); bitangents[j].z = sh / 32767.0f;
			}
			break;
		case 4:		// Joint indices
			if( streamElemSize != 4 )
			{
				errormsg = "Invalid joint stream";
				break;
			}
			for( uint32 j = 0; j < streamSize; ++j )
			{
				pData = elemcpy_le(&uc, (unsigned char*)(pData), 1); _vertStaticData[j].jointVec[0] = (float)uc;
				pData = elemcpy_le(&uc, (unsigned char*)(pData), 1); _vertStaticData[j].jointVec[1] = (float)uc;
				pData = elemcpy_le(&uc, (unsigned char*)(pData), 1); _vertStaticData[j].jointVec[2] = (float)uc;
				pData = elemcpy_le(&uc, (unsigned char*)(pData), 1); _vertStaticData[j].jointVec[3] = (float)uc;
			}
			break;
		case 5:		// Weights
			if( streamElemSize != 4 )
			{
				errormsg = "Invalid weight stream";
				break;
			}
			for( uint32 j = 0; j < streamSize; ++j )
			{
				pData = elemcpy_le(&uc, (unsigned char*)(pData), 1); _vertStaticData[j].weightVec[0] = uc / 255.0f;
				pData = elemcpy_le(&uc, (unsigned char*)(pData), 1); _vertStaticData[j].weightVec[1] = uc / 255.0f;
				pData = elemcpy_le(&uc, (unsigned char*)(pData), 1); _vertStaticData[j].weightVec[2] = uc / 255.0f;
				pData = elemcpy_le(&uc, (unsigned char*)(pData), 1); _vertStaticData[j].weightVec[3] = uc / 255.0f;
			}
			break;
		case 6:		// Texture Coord Set 1
			if( streamElemSize != 8 )
			{
				errormsg = "Invalid texCoord1 stream";
				break;
			}
			for( uint32 j = 0; j < streamSize; ++j )
			{
				pData = elemcpy_le(&_vertStaticData[j].u0, (float*)(pData), 1);
				pData = elemcpy_le(&_vertStaticData[j].v0, (float*)(pData), 1);
			}
			break;
		case 7:		// Texture Coord Set 2
			if( streamElemSize != 8 )
			{
				errormsg = "Invalid texCoord2 stream";
				break;
			}
			for( uint32 j = 0; j < streamSize; ++j )
			{
				pData = elemcpy_le(&_vertStaticData[j].u1, (float*)(pData), 1);
				pData = elemcpy_le(&_vertStaticData[j].v1, (float*)(pData), 1);
			}
			break;
		default:
			pData += streamElemSize * streamSize;
			Modules::log().writeWarning( "Geometry resource '%s': Ignoring unsupported vertex base stream", _name.c_str() );
			continue;
		}
		if (!errormsg.empty())
		{
			delete[] bitangents;
			return raiseError(errormsg);
		}
	}

	// Prepare bitangent data (TODO: Should be done in ColladaConv)
	for( uint32 i = 0; i < _vertCount; ++i )
	{
		_vertTanData[i].handedness = _vertTanData[i].normal.cross( _vertTanData[i].tangent ).dot( bitangents[i] ) < 0 ? -1.0f : 1.0f;
	}
	delete[] bitangents;
		
	// Load triangle indices
	pData = elemcpy_le(&count, (uint32*)(pData), 1);

	_indexCount = count;
	_16BitIndices = _vertCount <= 65536;
	_indexData = new char[count * (_16BitIndices ? 2 : 4)];
	if( _16BitIndices )
	{
		uint32 index;
		uint16 *pIndexData = (uint16 *)_indexData;
		for( uint32 i = 0; i < count; ++i )
		{
			pData = elemcpy_le(&index, (uint32*)(pData), 1);
			pIndexData[i] = (uint16)index;
		}
	}
	else
	{
		uint32 *pIndexData = (uint32 *)_indexData;
		for( uint32 i = 0; i < count; ++i )
		{
			pData = elemcpy_le(&pIndexData[i], (uint32*)(pData), 1);
		}
	}

	// Load morph targets
	uint32 numTargets;
	pData = elemcpy_le(&numTargets, (uint32*)(pData), 1);

	_morphTargets.resize( numTargets );
	for( uint32 i = 0; i < numTargets; ++i )
	{
		MorphTarget &mt = _morphTargets[i];
		char name[256];
		
		memcpy( name, pData, 256 ); pData += 256;
		mt.name = name;
		
		// Read vertex indices
		uint32 morphStreamSize;
		pData = elemcpy_le(&morphStreamSize, (uint32*)(pData), 1);
		mt.diffs.resize( morphStreamSize );
		for( uint32 j = 0; j < morphStreamSize; ++j )
		{
			pData = elemcpy_le(&mt.diffs[j].vertIndex, (uint32*)(pData), 1);
		}
		
		// Loop over streams
		pData = elemcpy_le(&count, (uint32*)(pData), 1);
		for( uint32 j = 0; j < count; ++j )
		{
			uint32 streamID, streamElemSize;
			pData = elemcpy_le(&streamID, (uint32*)(pData), 1);
			pData = elemcpy_le(&streamElemSize, (uint32*)(pData), 1);

			switch( streamID )
			{
			case 0:		// Position
				if( streamElemSize != 12 ) return raiseError( "Invalid position morph stream" );
				for( uint32 k = 0; k < morphStreamSize; ++k )
				{
					pData = elemcpy_le(&mt.diffs[k].posDiff.x, (float*)(pData), 1);
					pData = elemcpy_le(&mt.diffs[k].posDiff.y, (float*)(pData), 1);
					pData = elemcpy_le(&mt.diffs[k].posDiff.z, (float*)(pData), 1);
				}
				break;
			case 1:		// Normal
				if( streamElemSize != 12 ) return raiseError( "Invalid normal morph stream" );
				for( uint32 k = 0; k < morphStreamSize; ++k )
				{
					pData = elemcpy_le(&mt.diffs[k].normDiff.x, (float*)(pData), 1);
					pData = elemcpy_le(&mt.diffs[k].normDiff.y, (float*)(pData), 1);
					pData = elemcpy_le(&mt.diffs[k].normDiff.z, (float*)(pData), 1);
				}
				break;
			case 2:		// Tangent
				if( streamElemSize != 12 ) return raiseError( "Invalid tangent morph stream" );
				for( uint32 k = 0; k < morphStreamSize; ++k )
				{
					pData = elemcpy_le(&mt.diffs[k].tanDiff.x, (float*)(pData), 1);
					pData = elemcpy_le(&mt.diffs[k].tanDiff.y, (float*)(pData), 1);
					pData = elemcpy_le(&mt.diffs[k].tanDiff.z, (float*)(pData), 1);
				}
				break;
			case 3:		// Bitangent
				if( streamElemSize != 12 ) return raiseError( "Invalid bitangent morph stream" );
				
				// Skip data (TODO: remove from format)
				pData += morphStreamSize * sizeof( float ) * 3;
				break;
			default:
				pData += streamElemSize * morphStreamSize;
				Modules::log().writeWarning( "Geometry resource '%s': Ignoring unsupported vertex morph stream", _name.c_str() );
				continue;
			}
		}
	}

	// Find min/max morph target vertex indices
	_minMorphIndex = (unsigned)_vertCount;
	_maxMorphIndex = 0;
	for( uint32 i = 0; i < _morphTargets.size(); ++i )
	{
		for( uint32 j = 0; j < _morphTargets[i].diffs.size(); ++j )
		{
			_minMorphIndex = std::min( _minMorphIndex, _morphTargets[i].diffs[j].vertIndex );
			_maxMorphIndex = std::max( _maxMorphIndex, _morphTargets[i].diffs[j].vertIndex );
		}
	}
	if( _minMorphIndex > _maxMorphIndex )
	{
		_minMorphIndex = 0; _maxMorphIndex = 0;
	}

	// Find AABB of skeleton in bind pose
	for( uint32 i = 0; i < (uint32)_joints.size(); ++i )
	{
		Vec3f pos = _joints[i].invBindMat.inverted() * Vec3f( 0, 0, 0 );
		if( pos.x < _skelAABB.min.x ) _skelAABB.min.x = pos.x;
		if( pos.y < _skelAABB.min.y ) _skelAABB.min.y = pos.y;
		if( pos.z < _skelAABB.min.z ) _skelAABB.min.z = pos.z;
		if( pos.x > _skelAABB.max.x ) _skelAABB.max.x = pos.x;
		if( pos.y > _skelAABB.max.y ) _skelAABB.max.y = pos.y;
		if( pos.z > _skelAABB.max.z ) _skelAABB.max.z = pos.z;
	}

	// Add default joint if necessary
	if( _joints.empty() )
	{
		_joints.push_back( Joint() );
	}

	// Upload data
	if( _vertCount > 0 && _indexCount > 0 )
	{
		RenderDeviceInterface *rdi = Modules::renderer().getRenderDevice();

		_geoObj = rdi->beginCreatingGeometry( Modules::renderer().getDefaultVertexLayout( DefaultVertexLayouts::Model ) );

		// Upload indices
		_indexBuf = rdi->createIndexBuffer( _indexCount * (_16BitIndices ? 2 : 4), _indexData );
		
		// Upload vertices
		_posVBuf = rdi->createVertexBuffer(_vertCount * sizeof( Vec3f ), _vertPosData );
		_tanVBuf = rdi->createVertexBuffer( _vertCount * sizeof( VertexDataTan ), _vertTanData );
		_staticVBuf = rdi->createVertexBuffer( _vertCount * sizeof( VertexDataStatic ), _vertStaticData );

		rdi->setGeomVertexParams( _geoObj, _posVBuf, 0, 0, sizeof( Vec3f ) );
		rdi->setGeomVertexParams( _geoObj, _tanVBuf, 1, 0, sizeof( VertexDataTan ) );
		rdi->setGeomVertexParams( _geoObj, _tanVBuf, 2, sizeof( Vec3f ), sizeof( VertexDataTan ) );
		rdi->setGeomVertexParams( _geoObj, _staticVBuf, 3, 0, sizeof( VertexDataStatic ) );

		rdi->setGeomIndexParams( _geoObj, _indexBuf, _16BitIndices ? IDXFMT_16 : IDXFMT_32 );

		rdi->finishCreatingGeometry( _geoObj );
	}
	
	return true;
}

int GeometryResource::getElemCount( int elem ) const
{
	switch( elem )
	{
	case GeometryResData::GeometryElem:
		return 1;
	default:
		return Resource::getElemCount( elem );
	}
}


int GeometryResource::getElemParamI( int elem, int elemIdx, int param ) const
{
	switch( elem )
	{
	case GeometryResData::GeometryElem:
		switch( param )
		{
		case GeometryResData::GeoIndexCountI:
			return (int)_indexCount;
		case GeometryResData::GeoIndices16I:
			return _16BitIndices ? 1 : 0;
		case GeometryResData::GeoVertexCountI:
			return (int)_vertCount;
		}
		break;
	}
	
	return Resource::getElemParamI( elem, elemIdx, param );
}


void *GeometryResource::mapStream( int elem, int elemIdx, int stream, bool read, bool write )
{
	if( (read || write) && mappedWriteStream == -1 )
	{
		switch( elem )
		{
		case GeometryResData::GeometryElem:
			switch( stream )
			{
			case GeometryResData::GeoIndexStream:
				if( write ) mappedWriteStream = GeometryResData::GeoIndexStream;
				return _indexData;
			case GeometryResData::GeoVertPosStream:
				if( write ) mappedWriteStream = GeometryResData::GeoVertPosStream;
				return _vertPosData != 0x0 ? _vertPosData : 0x0;
			case GeometryResData::GeoVertTanStream:
				if( write ) mappedWriteStream = GeometryResData::GeoVertTanStream;
				return _vertTanData != 0x0 ? _vertTanData : 0x0;
			case GeometryResData::GeoVertStaticStream:
				if( write ) mappedWriteStream = GeometryResData::GeoVertStaticStream;
				return _vertStaticData != 0x0 ? _vertStaticData : 0x0;
			}
		}
	}

	return Resource::mapStream( elem, elemIdx, stream, read, write );
}


void GeometryResource::unmapStream()
{
	if( mappedWriteStream >= 0 )
	{
		RenderDeviceInterface *rdi = Modules::renderer().getRenderDevice();

		switch( mappedWriteStream )
		{
		case GeometryResData::GeoIndexStream:
			if( _indexData != 0x0 )
				rdi->updateBufferData( _geoObj, _indexBuf, 0, _indexCount * (_16BitIndices ? 2 : 4), _indexData );
			break;
		case GeometryResData::GeoVertPosStream:
			if( _vertPosData != 0x0 )
				rdi->updateBufferData( _geoObj, _posVBuf, 0, _vertCount * sizeof( Vec3f ), _vertPosData );
			break;
		case GeometryResData::GeoVertTanStream:
			if( _vertTanData != 0x0 )
				rdi->updateBufferData( _geoObj, _tanVBuf, 0, _vertCount * sizeof( VertexDataTan ), _vertTanData );
			break;
		case GeometryResData::GeoVertStaticStream:
			if( _vertStaticData != 0x0 )
				rdi->updateBufferData( _geoObj, _staticVBuf, 0, _vertCount * sizeof( VertexDataStatic ), _vertStaticData );
			break;
		}

		mappedWriteStream = -1;
	}
}


void GeometryResource::updateDynamicVertData()
{
	// Upload dynamic stream data
	if( _vertPosData != 0x0 )
	{
		Modules::renderer().getRenderDevice()->updateBufferData( _geoObj, _posVBuf, 0, _vertCount * sizeof( Vec3f ), _vertPosData );
	}
	if( _vertTanData != 0x0 )
	{
		Modules::renderer().getRenderDevice()->updateBufferData( _geoObj, _tanVBuf, 0, _vertCount * sizeof( VertexDataTan ), _vertTanData );
	}
}

}  // namespace
