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

#ifndef _egModel_H_
#define _egModel_H_

#include "egPrerequisites.h"
#include "egScene.h"
#include "egAnimatables.h"
#include "egGeometry.h"
#include "egAnimation.h"
#include "egMaterial.h"
#include "utMath.h"


namespace Horde3D {

const uint32 ModelCustomVecCount = 4;

// =================================================================================================
// Model Node
// =================================================================================================

struct ModelNodeParams
{
	enum List
	{
		GeoResI = 200,
		SWSkinningI,
		AnimCountI
	};
};

struct ModelUpdateFlags
{
	enum Flags
	{
		Animation = 1,
		Geometry = 2,
		ChildNodes = 3
	};
};

// =================================================================================================

struct ModelNodeTpl : public SceneNodeTpl
{
	PGeometryResource  geoRes;
	bool               softwareSkinning;

	ModelNodeTpl( const std::string &name, GeometryResource *geoRes ) :
		SceneNodeTpl( SceneNodeTypes::Model, name ), geoRes( geoRes ),
			softwareSkinning( false )
	{
	}
};

// =================================================================================================

struct Morpher	// Morph modifier
{
	std::string  name;
	uint32       index;  // Index of morph target in Geometry resource
	float        weight;
};

// =================================================================================================

class ModelNode : public SceneNode
{
public:
	static SceneNodeTpl *parsingFunc( std::map< std::string, std::string > &attribs );
	static SceneNode *factoryFunc( const SceneNodeTpl &nodeTpl );

	~ModelNode();

	void recreateNodeList();
	void setupAnimStage( int stage, AnimationResource *anim, int layer,
	                     const std::string &startNode, bool additive );
	void getAnimParams( int stage, float *time, float *weight ) const;
	void setAnimParams( int stage, float time, float weight );
	bool setMorphParam( const std::string &targetName, float weight );

	int getParamI( int param ) const;
	void setParamI( int param, int value );
	float getParamF( int param, int compIdx ) const;
	void setParamF( int param, int compIdx, float value );

	void update( int flags );

	void setCustomInstData( const float *data, uint32 count );

	GeometryResource *getGeometryResource() const { return _geometryRes; }
	bool jointExists( uint32 jointIndex ) const { return jointIndex < _skinMatRows.size() / 3; }
	void setSkinningMat( uint32 index, const Matrix4f &mat )
		{ _skinMatRows[index * 3 + 0] = mat.getRow( 0 );
		  _skinMatRows[index * 3 + 1] = mat.getRow( 1 );
		  _skinMatRows[index * 3 + 2] = mat.getRow( 2 ); }
	void markNodeListDirty() { _nodeListDirty = true; }

	int get_meshes(int **handles);

protected:
	ModelNode( const ModelNodeTpl &modelTpl );

	void recreateNodeListRec( SceneNode *node, bool firstCall );
	void updateLocalMeshAABBs();
	void setGeometryRes( GeometryResource &geoRes );

	bool updateGeometry();

	void onPostUpdate();
	void onFinishedUpdate();

protected:
	PGeometryResource             _geometryRes;
	PGeometryResource             _baseGeoRes;	// NULL if model does not have a private geometry copy
	
	std::vector< MeshNode * >     _meshList;  // List of the model's meshes
	std::vector< JointNode * >    _jointList;
	std::vector< Vec4f >          _skinMatRows;
	AnimationController           _animCtrl;

	Vec4f                         _customInstData[ModelCustomVecCount];

	std::vector< Morpher >        _morphers;
	bool                          _softwareSkinning, _skinningDirty;
	bool                          _nodeListDirty;  // An animatable node has been attached to model
	bool                          _morpherUsed, _morpherDirty;

	friend class SceneManager;
	friend class SceneNode;
	friend class Renderer;
};

}
#endif // _egModel_H_
