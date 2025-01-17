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

#ifndef _egScene_H_
#define _egScene_H_

#include "egPrerequisites.h"
#include "utMath.h"
#include "egPrimitives.h"
#include "egPipeline.h"
#include <map>


namespace Horde3D {

struct SceneNodeTpl;
class CameraNode;
class SceneGraphResource;


const int RootNode = 1;


// =================================================================================================
// Scene Node
// =================================================================================================

struct SceneNodeTypes
{
	enum List
	{
		Undefined = 0,
		Group,
		Model,
		Mesh,
		Joint,
		Light,
		Camera,
		Emitter,
		Compute
	};
};

struct SceneNodeParams
{
	enum List
	{
		NameStr = 1,
		AttachmentStr
	};
};

struct SceneNodeFlags
{
	enum List
	{
		NoDraw = 0x1,
		NoCastShadow = 0x2,
		NoRayQuery = 0x4,
		Inactive = 0x7  // NoDraw | NoCastShadow | NoRayQuery
	};
};

// =================================================================================================

struct SceneNodeTpl
{
	int                            type;
	std::string                    name;
	Vec3f                          trans, rot, scale;
	std::string                    attachmentString;
	std::vector< SceneNodeTpl * >  children;

	SceneNodeTpl( int type, const std::string &name ) :
		type( type ), name( name ), scale( Vec3f ( 1, 1, 1 ) )
	{
	}
	
	virtual ~SceneNodeTpl()
	{
		for( uint32 i = 0; i < children.size(); ++i ) delete children[i];
	}
};

// =================================================================================================

class SceneNode
{
public:
	SceneNode( const SceneNodeTpl &tpl );
	virtual ~SceneNode();

	void getTransform( Vec3f &trans, Vec3f &rot, Vec3f &scale ) const;	// Not virtual for performance
	void setTransform( Vec3f trans, Vec3f rot, Vec3f scale );	// Not virtual for performance
	void setTransform( const Matrix4f &mat );
	void getTransMatrices( const float **relMat, const float **absMat ) const;

	int getFlags() const { return _flags; }
	void setFlags( int flags, bool recursive );

	virtual int getParamI( int param ) const;
	virtual void setParamI( int param, int value );
	virtual float getParamF( int param, int compIdx ) const;
	virtual void setParamF( int param, int compIdx, float value );
	virtual const char *getParamStr( int param ) const;
	virtual void setParamStr( int param, const char* value );

	virtual uint32 calcLodLevel( const Vec3f &viewPoint ) const;
	virtual bool checkLodCorrectness( uint32 lodLevel ) const;

	bool checkOcclusionSupported() { return _occlusionCullingSupported; }
	uint32 getOcclusionResult( uint32 occlusionSet );

	bool checkLodSupport() { return _lodSupported; }

	virtual bool canAttach( SceneNode &parent ) const;
	void markDirty();
	void updateTree();
	virtual bool checkIntersection( const Vec3f &rayOrig, const Vec3f &rayDir, Vec3f &intsPos ) const;

	virtual void setCustomInstData( const float *data, uint32 count ) {}

	int getType() const { return _type; };
	NodeHandle getHandle() const { return _handle; }
	SceneNode *getParent() const { return _parent; }
	const std::string getName() const { return _name; }
	std::vector< SceneNode * > &getChildren() { return _children; }
	Matrix4f &getRelTrans() { return _relTrans; }
	Matrix4f &getAbsTrans() { return _absTrans; }
	BoundingBox &getBBox() { return _bBox; }
	const std::string &getAttachmentString() const { return _attachment; }
	void setAttachmentString( const char* attachmentData ) { _attachment = attachmentData; }
	bool checkTransformFlag( bool reset )
		{ bool b = _transformed; if( reset ) _transformed = false; return b; }

protected:
	void markChildrenDirty();

	virtual void onPostUpdate() {}  // Called after absolute transformation has been updated
	virtual void onFinishedUpdate() {}  // Called after children have been updated
	virtual void onAttach( SceneNode &parentNode ) {}  // Called when node is attached to parent
	virtual void onDetach( SceneNode &parentNode ) {}  // Called when node is detached from parent

protected:
	Matrix4f                    _relTrans, _absTrans;  // Transformation matrices
	std::vector< SceneNode * >  _children;  // Child nodes

	std::vector< uint32 >		_occQueries;
	std::vector< uint32 >		_occQueriesLastVisited;

	std::string                 _name;
	std::string                 _attachment;  // User defined data
	BoundingBox                 _bBox;  // AABB in world space
	SceneNode                   *_parent;  // Parent node

	int                         _type;
	NodeHandle                  _handle;
	uint32                      _sgHandle;  // Spatial graph handle
	uint32                      _flags;
	float                       _sortKey;
	bool                        _dirty;  // Does the node need to be updated?
	bool                        _transformed;
	bool                        _renderable;
	bool						_lodSupported;
	bool						_occlusionCullingSupported;

	friend class SceneManager;
	friend class SpatialGraph;
	friend class Renderer;
};


// =================================================================================================
// Group Node
// =================================================================================================

struct GroupNodeTpl : public SceneNodeTpl
{
	GroupNodeTpl( const std::string &name ) :
		SceneNodeTpl( SceneNodeTypes::Group, name )
	{
	}
};

// =================================================================================================

class GroupNode : public SceneNode
{
public:
	static SceneNodeTpl *parsingFunc( std::map< std::string, std::string > &attribs );
	static SceneNode *factoryFunc( const SceneNodeTpl &nodeTpl );

	friend class Renderer;
	friend class SceneManager;

protected:
	GroupNode( const GroupNodeTpl &groupTpl );
};


// =================================================================================================
// Spatial Graph
// =================================================================================================

enum class RenderViewType
{
	Unknown,
	Camera,
	Light,
	Shadow
};

struct RenderQueueItem
{
	SceneNode  *node;
	int        type;  // Type is stored explicitly for better cache efficiency when iterating over list
	float      sortKey;

	RenderQueueItem() {}
	RenderQueueItem( int type, float sortKey, SceneNode *node )
		: node( node ), type( type ), sortKey( sortKey )
	{
	}
};

typedef std::vector< RenderQueueItem > RenderQueue;

struct RenderView
{
	Frustum			frustum;
	SceneNode		*node;

	BoundingBox		objectsAABB;
	BoundingBox		auxObjectsAABB; // auxiliary aabb, for objects that passed additional filtering.
									// Currently used for light types, ignores objects with NoCastShadow flag
	RenderQueue		objects;

	RenderViewType	type;
	int				linkedView;

	uint32			auxFilter; // additional filter information

	bool			updated;

	RenderView();

	RenderView( RenderViewType viewType, SceneNode *viewNode, const Frustum &f, int link, uint32 additionalFilter );
};


class SpatialGraph
{
public:
	SpatialGraph();
	
	virtual ~SpatialGraph();

	virtual void addNode( SceneNode &sceneNode );
	virtual void removeNode( uint32 sgHandle );
	virtual void updateNode( uint32 sgHandle );

	virtual void updateQueues( const Frustum &frustum1, const Frustum *frustum2,
	                   RenderingOrder::List order, uint32 filterIgnore, bool lightQueue, bool renderQueue );

	virtual void updateQueues( uint32 filterIgnore, bool forceUpdateAllViews = false, bool preparingViews = false );

	// Render view handling
	void clearViews();
	int addView( RenderViewType type, SceneNode *node, const Frustum &f, int link, uint32 additionalFilter );
	void setCurrentView( int viewID );
	int getRenderViewCount() { return _totalViews; }

	void sortViewObjects( RenderingOrder::List order );
	void sortViewObjects( int viewID, RenderingOrder::List order );

	std::vector< RenderView > &getRenderViews() { return _views; }

	std::vector< SceneNode * > &getLightQueue() { return _lightQueue; }
	RenderQueue &getRenderQueue();
protected:
	std::vector< SceneNode * >     _nodes;		// Renderable nodes and lights
	std::vector< uint32 >          _freeList;

	std::vector< RenderView >	   _views;

	std::vector< SceneNode * >     _lightQueue;
	RenderQueue                    _renderQueue;

	int							   _currentView;
	int							   _totalViews;
};


// =================================================================================================
// Scene Manager
// =================================================================================================

typedef SceneNodeTpl *(*NodeTypeParsingFunc)( std::map< std::string, std::string > &attribs );
typedef SceneNode *(*NodeTypeFactoryFunc)( const SceneNodeTpl &tpl );

struct NodeRegEntry
{
	std::string          typeString;
	NodeTypeParsingFunc  parsingFunc;
	NodeTypeFactoryFunc  factoryFunc;
};

struct CastRayResult
{
	SceneNode  *node;
	float      distance;
	Vec3f      intersection;
};

// =================================================================================================

class SceneManager
{
public:
	SceneManager();
	~SceneManager();

	void registerSpatialGraph( SpatialGraph *graph );

	void registerNodeType( int nodeType, const std::string &typeString, NodeTypeParsingFunc pf,
	                       NodeTypeFactoryFunc ff );
	NodeRegEntry *findType( int type );
	NodeRegEntry *findType( const std::string &typeString );
	
	void updateNodes();
	
	NodeHandle addNode( SceneNode *node, SceneNode &parent );
	NodeHandle addNodes( SceneNode &parent, SceneGraphResource &sgRes );
	void removeNode( SceneNode &node );
	bool relocateNode( SceneNode &node, SceneNode &parent );
	
	int findNodes( SceneNode &startNode, const std::string &name, int type );
	void clearFindResults() { _findResults.resize( 0 ); }
	SceneNode *getFindResult( int index ) const { return (unsigned)index < _findResults.size() ? _findResults[index] : 0x0; }
	
	int castRay( SceneNode &node, const Vec3f &rayOrig, const Vec3f &rayDir, int numNearest );
	bool getCastRayResult( int index, CastRayResult &crr );

	int checkNodeVisibility( SceneNode &node, CameraNode &cam, bool checkOcclusion, bool calcLod );

	SceneNode &getRootNode() const { return *_nodes[0]; }
	SceneNode &getDefCamNode() const { return *_nodes[1]; }
	
	SceneNode *resolveNodeHandle( NodeHandle handle ) const
		{ return (handle != 0 && (unsigned)(handle - 1) < _nodes.size()) ? _nodes[handle - 1] : 0x0; }

	//
	// Spatial graph related functions
	//
	void updateSpatialNode( uint32 sgHandle ) { _spatialGraph->updateNode( sgHandle ); }

	/* Function: updateQueues
			Updates internal queues of the spatial graph.
		
		Details:
			This function updates spatial nodes registered in the graph. It performs scene graph update,
			if required, but its main purpose is to perform culling for registered render views and
			generate render queues.
		
		Parameters:
			filterIgnore - allows filtering nodes that have the provided flags (see SceneNodeFlags).
			forceUpdateAllViews - allows forcing full update for all render views, will perform culling
								  for all objects and render views again (default:false).
			preparingViews - is updateQueues called from prepareRenderViews function. If yes, additional
							 root node update is not called (default:false)
			
		Returns:
			nothing
	*/
	void updateQueues( uint32 filterIgnore, bool forceUpdateAllViews = false, bool preparingViews = false );
	void updateQueues( const Frustum &frustum1, const Frustum *frustum2,
		RenderingOrder::List order, uint32 filterIgnore, bool lightQueue, bool renderableQueue );

	void sortViewObjects( RenderingOrder::List order );
	void sortViewObjects( int viewID, RenderingOrder::List order );

	int addRenderView( RenderViewType type, SceneNode *node, const Frustum &f, int link = -1, uint32 additionalFilter = 0 );
	std::vector< RenderView > &getRenderViews() const { return _spatialGraph->getRenderViews(); }
	void clearRenderViews();
	int getActiveRenderViewCount() { return _spatialGraph->getRenderViewCount(); }

	void setCurrentView( int viewID );
	std::vector< SceneNode * > &getLightQueue() const { return _spatialGraph->getLightQueue(); }
	RenderQueue &getRenderQueue() const { return _spatialGraph->getRenderQueue(); }

protected:
	NodeHandle parseNode( SceneNodeTpl &tpl, SceneNode *parent );
	void removeNodeRec( SceneNode &node );

	void castRayInternal( SceneNode &node );

protected:
	std::vector< SceneNode *>      _nodes;  // _nodes[0] is root node
	std::vector< uint32 >          _freeList;  // List of free slots
	std::vector< SceneNode * >     _findResults;
	std::vector< CastRayResult >   _castRayResults;
	SpatialGraph                   *_spatialGraph;

	std::map< int, NodeRegEntry >  _registry;  // Registry of node types

	Vec3f                          _rayOrigin;  // Don't put these values on the stack during recursive search
	Vec3f                          _rayDirection;  // Ditto
	int                            _rayNum;  // Ditto

	friend class Renderer;
};

}
#endif // _egScene_H_
