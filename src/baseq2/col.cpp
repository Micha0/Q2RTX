#include <stdlib.h>
#include <cstdint>

#include "common/bsp_util.h"
// #include "common/bsp.h"

extern "C" {
	//Yeah, mixing shared, common, baseq2 and server :rolleyes:
	//not sure how to solve this, or what is "the way (tm)"
#include "shared/shared.h"
#include "common/common.h"
#include "server/server.h"
#include "g_local.h"
}

#undef world	//geez

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btGeometryUtil.h"

#include <vector>
#include <map>

class QuakeDebugDrawer;

struct collision_body_t {
    btRigidBody* body;
    btCollisionShape* shape;
	mmodel_t* model;
	edict_t* ent;
};

struct bsp_collision_t {    
    btDiscreteDynamicsWorld *physicsWorld;
    btDefaultCollisionConfiguration *physicsConfig;
    btCollisionDispatcher *physicsDispatcher;
    btAxisSweep3 *physicsBroadphase;
    btSequentialImpulseConstraintSolver *physicsSolver;

	QuakeDebugDrawer* debugDrawer;

    std::vector<collision_body_t> bodies;
};

namespace {
	bsp_t* bsp = nullptr;
	static bsp_collision_t bsp_col;
}


#define BT_LINE_BATCH_SIZE 512

class QuakeDebugDrawer : public btIDebugDraw
{
	struct qddVec {
		float x = 0.0f, y = 0.0f, z = 0.0f;
		qddVec(const btVector3& v)
		: x(v[0]), y(v[1]), z(v[2])
		{}
	};

	btAlignedObjectArray<qddVec> mLinePoints;
	btAlignedObjectArray<unsigned int> mLineIndices;

	btVector3 mCurrentLineColor;
	// DefaultColors mDefaultColors;
	int mDrawMode;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	QuakeDebugDrawer()
		: mDrawMode(btIDebugDraw::DBG_DrawWireframe | btIDebugDraw::DBG_DrawAabb)
		, mCurrentLineColor(-1, -1, -1)
	{
	}

	virtual ~QuakeDebugDrawer()
	{
	}

/*
	virtual DefaultColors getDefaultColors() const
	{
		return mDefaultColors;
	}

	///the default implementation for setDefaultColors has no effect. A derived class can implement it and store the colors.
	virtual void setDefaultColors(const DefaultColors& colors)
	{
		mDefaultColors = colors;
	}
*/
	virtual void drawLine(const btVector3& from1, const btVector3& to1, const btVector3& color1) override
	{
		if (mCurrentLineColor != color1 || mLinePoints.size() >= BT_LINE_BATCH_SIZE)
		{
			flushLines();
			mCurrentLineColor = color1;
		}
		qddVec from(from1);
		qddVec to(to1);

		mLineIndices.push_back(mLinePoints.size());
		mLinePoints.push_back(from);
		mLineIndices.push_back(mLinePoints.size());
		mLinePoints.push_back(to);
	}

	virtual void drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color) override
	{
		drawLine(PointOnB, PointOnB + normalOnB * distance, color);
		btVector3 ncolor(0, 0, 0);
		drawLine(PointOnB, PointOnB + normalOnB * 0.01, ncolor);
	}

	virtual void reportErrorWarning(const char* warningString) override
	{
		Com_LPrintf(PRINT_DEVELOPER, "Bullet Physics: Warning: %s\n", warningString);
	}

	virtual void draw3dText(const btVector3& location, const char* textString) override
	{
	}

	virtual void setDebugMode(int debugMode) override
	{
		mDrawMode = debugMode;
	}

	virtual int getDebugMode() const override
	{
		return mDrawMode;
	}

	virtual void flushLines() override
	{
		int sz = mLinePoints.size();
		if (sz)
		{
			float debugColor[4];
			debugColor[0] = mCurrentLineColor.x();
			debugColor[1] = mCurrentLineColor.y();
			debugColor[2] = mCurrentLineColor.z();
			debugColor[3] = 1.f;

			gi.R_DrawLines(&mLinePoints[0].x, mLinePoints.size(), &mLineIndices[0], mLineIndices.size(), debugColor);
			
			mLinePoints.clear();
			mLineIndices.clear();
		}
	}

	virtual void clearLines() override
	{
		mLinePoints.clear();
		mLineIndices.clear();
	}
};


btRigidBody* _BSP_CreateRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape, const btVector4& color = btVector4(1, 0, 0, 1))
{
    btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

    //rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0, 0, 0);
    if (isDynamic)
        shape->calculateLocalInertia(mass, localInertia);

        //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

    btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);

    btRigidBody* body = new btRigidBody(cInfo);
    //body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

    return body;
}

btVector3 LocalizeVerticesNewOrigin(btAlignedObjectArray<btVector3>& vertices)
{
	btVector3 computeMin = vertices.at(0);
	btVector3 computeMax = vertices.at(0);
	for(size_t i = 1; i < vertices.size(); ++i)
	{
		computeMin[0] = computeMin[0] < vertices[i][0] ? computeMin[0] : vertices[i][0];
		computeMin[1] = computeMin[1] < vertices[i][1] ? computeMin[1] : vertices[i][1];
		computeMin[2] = computeMin[2] < vertices[i][2] ? computeMin[2] : vertices[i][2];
		computeMax[0] = computeMax[0] > vertices[i][0] ? computeMax[0] : vertices[i][0];
		computeMax[1] = computeMax[1] > vertices[i][1] ? computeMax[1] : vertices[i][1];
		computeMax[2] = computeMax[2] > vertices[i][2] ? computeMax[2] : vertices[i][2];
	}
	btVector3 computeCenter = (computeMax - computeMin) * 0.5f + computeMin;
	for(size_t i = 0; i < vertices.size(); ++i)
	{
		vertices[i] = vertices[i] - computeCenter;
	}
	return computeCenter;
}

EXPORT void COL_Init(bsp_t* _bsp)
{
	if(bsp)
	{
	    Com_LPrintf(PRINT_ERROR, "Double initialization of Collision!!\n");
		return;
	}

	bsp = _bsp;

    //assert(!bsp->collision);
    Com_LPrintf(PRINT_NOTICE, "Initializing Collision\n");

    bsp_col.physicsConfig = new btDefaultCollisionConfiguration();
///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	bsp_col.physicsDispatcher = new	btCollisionDispatcher(bsp_col.physicsConfig);
    //TODO: get these values from bsp_t
    bsp_col.physicsBroadphase = new btAxisSweep3(btVector3(-40960, -40960, -40960), btVector3(40960, 40960, 40960));
	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	bsp_col.physicsSolver = new btSequentialImpulseConstraintSolver;
	bsp_col.physicsWorld = new btDiscreteDynamicsWorld(
        bsp_col.physicsDispatcher,
        bsp_col.physicsBroadphase,
        bsp_col.physicsSolver,
        bsp_col.physicsConfig);
	bsp_col.physicsWorld->getSolverInfo().m_numIterations = 10;
	bsp_col.physicsWorld->setGravity(btVector3(0, 0, -10));

	bsp_col.debugDrawer = new QuakeDebugDrawer();
	bsp_col.physicsWorld->setDebugDrawer(bsp_col.debugDrawer);

	bsp_col.debugDrawer->setDebugMode(
		btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawAabb + btIDebugDraw::DBG_DrawContactPoints + btIDebugDraw::DBG_DrawNormals
		//btIDebugDraw::DBG_DrawContactPoints
	);

    //Iterate Map brush planes and collect their vertices
	struct BrushPlaneContext_T {
		std::map<mmodel_t*, std::map<mbrush_t*, btAlignedObjectArray<btVector3> > > planeEquations;
	};
	BrushPlaneContext_T context;

	gi.BSP_RecurseBrushPlanes(
		bsp, &context, (CONTENTS_SOLID|CONTENTS_WINDOW),
		+[](bsp_t* bsp, mmodel_t* model, mbrush_t* brush, float plane_nx, float plane_ny, float plane_nz, float plane_d, void* context)
		{
			BrushPlaneContext_T& brushPlaneContext = *(BrushPlaneContext_T*)context;

			btVector3 planeEquation;
			planeEquation.setValue(plane_nx, plane_ny, plane_nz);
			planeEquation[3] = plane_d;

			brushPlaneContext.planeEquations[model][brush].push_back(planeEquation);
		}
	);

	struct HullTransformPair {
		btConvexHullShape* shape;
		btTransform transform;
		float mass;
	};
	using ConvexHullShapeVector = std::vector<HullTransformPair>;

    for(const std::pair<mmodel_t*, std::map<mbrush_t*, btAlignedObjectArray<btVector3> > >& modelBrushPlanesMap : context.planeEquations)
    {
        Com_LPrintf(PRINT_DEVELOPER, "Collecting Collision MapModel\n");

		ConvexHullShapeVector shapeVector;

		for(const std::pair<mbrush_t*, btAlignedObjectArray<btVector3> >& brushPlanesMap : modelBrushPlanesMap.second)
		{
			btAlignedObjectArray<btVector3>	vertices;
			btGeometryUtil::getVerticesFromPlaneEquations(brushPlanesMap.second, vertices);

			btTransform transform;
			transform.setIdentity();

			btVector3 newOrigin = LocalizeVerticesNewOrigin(vertices);
			Com_LPrintf(PRINT_DEVELOPER, "Collecting %d Vertices: Origin: %.2f, %.2f, %.2f\n", vertices.size(), newOrigin[0], newOrigin[1], newOrigin[2]);
			for(int i = 0; i < vertices.size(); ++i)
			{
				Com_LPrintf(PRINT_DEVELOPER, "vertex #%d: %.2f, %.2f, %.2f\n", i, vertices[i][0], vertices[i][1], vertices[i][2]);
			}
			transform.setOrigin(newOrigin);
			
			btConvexHullShape* shape = new btConvexHullShape(&(vertices[0].getX()), vertices.size());
			shape->setUserPointer(brushPlanesMap.first);

			shapeVector.push_back(HullTransformPair { shape, transform, 0.0f });
		}

		if(shapeVector.size() == 0)
		{
			continue;
		}
		
		btRigidBody* body = nullptr;

		Com_LPrintf(PRINT_DEVELOPER, "Combining %d ConvexHulls\n", shapeVector.size());

		const bool isCompoundShape = shapeVector.size() != 1;

		btCompoundShape* compoundShape = new btCompoundShape;
		btVector3 localInertiaTmp(0, 0, 0);
		btVector3 totalInertia(0, 0, 0);
		float totalMass = 0.0f;
		btTransform motionTransform;
		motionTransform.setIdentity();
		btTransform compoundTransform;
		compoundTransform.setIdentity();

		//Calculate mass among all shapes
		for(HullTransformPair& htPair : shapeVector)
		{
			const bool isDynamic = (htPair.mass != 0.f);
			if (isDynamic)
			{
				totalMass += htPair.mass;
			}
		}

		//Calculate new center 
		if(isCompoundShape)
		{
			btVector3 compoundCenter(0, 0, 0);
			for(HullTransformPair& htPair : shapeVector)
			{
				compoundCenter += htPair.transform.getOrigin();
			}

			compoundCenter /= shapeVector.size();
			compoundTransform.setOrigin(compoundCenter);
			Com_LPrintf(PRINT_DEVELOPER, "Compound center: %.2f, %.2f, %.2f\n", compoundTransform.getOrigin()[0], compoundTransform.getOrigin()[1], compoundTransform.getOrigin()[2]);
		}

		//Combine shapes into compound
		for(HullTransformPair& htPair : shapeVector)
		{
			const bool isDynamic = (htPair.mass != 0.f);
			if (isDynamic)
			{
				htPair.shape->calculateLocalInertia(htPair.mass, localInertiaTmp);
				totalInertia += localInertiaTmp * (htPair.mass / totalMass);
			}

			// htPair.shape->optimizeConvexHull();
			btTransform localTransform;
			localTransform.setIdentity();

			if(isCompoundShape)
			{
				localTransform.setOrigin(htPair.transform.getOrigin() - compoundTransform.getOrigin());
				compoundShape->addChildShape(localTransform, htPair.shape);
			}
			else
			{
				//>>> This doesn't change the center of mass
				// btTransform lt;
				// lt.setIdentity();

				// compoundShape->addChildShape(lt, htPair.shape);
				// motionTransform.setOrigin(htPair.transform.getOrigin());
				// <<<<

				// compoundShape->addChildShape(htPair.transform, htPair.shape);
				// motionTransform.setOrigin(htPair.transform.getOrigin());
			}
		}

		btCollisionShape* colShape = static_cast<btCollisionShape*>(compoundShape);
		if(isCompoundShape)
		{
			motionTransform.setOrigin(compoundTransform.getOrigin());
		}
		else
		{
			delete compoundShape;
			compoundShape = nullptr;

			colShape = static_cast<btCollisionShape*>(shapeVector.front().shape);
			motionTransform.setOrigin(shapeVector.front().transform.getOrigin());
		}
		// btTransform transform;
		// transform.setIdentity();
		// transform.setOrigin(center / shapeVector.size());

		btDefaultMotionState* motionState = new btDefaultMotionState(motionTransform);
		btRigidBody::btRigidBodyConstructionInfo cInfo(totalMass, motionState, colShape, totalInertia);
		body = new btRigidBody(cInfo);

		body->setActivationState(DISABLE_DEACTIVATION);
		Com_LPrintf(PRINT_DEVELOPER, "RigidBody Pos: %.2f, %.2f, %.2f\n", motionTransform.getOrigin()[0], motionTransform.getOrigin()[1], motionTransform.getOrigin()[2]);
		// body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

		body->setUserPointer(modelBrushPlanesMap.first);
		body->activate(true);

		bsp_col.bodies.push_back(collision_body_t{body, colShape, modelBrushPlanesMap.first, nullptr});
		bsp_col.physicsWorld->addRigidBody(body);
    }
}

EXPORT void COL_Test()
{
	for (size_t i = 0; i < bsp_col.bodies.size(); ++i)
	{
		auto ent = bsp_col.bodies[i].ent;
		if(!ent)
		{
			continue;
		}

		if(ent->mass == 0)
		{
			continue;
		}
		
		// if(ent->s.origin[0] 
	}
}

vec3_t bulletQuakeMat[3] = {
	{-1, 0, 0},
	{0, -1, 0},
	{0, 0, 1},
};

EXPORT void COL_Think()
{	
    static btClock realClock;

	//Apply entities to simulation
	// for (size_t i = 0; i < bsp_col.bodies.size(); ++i)
	// {
	// 	auto ent = bsp_col.bodies[i].ent;
	// 	if(!ent)
	// 	{
	// 		continue;
	// 	}

	// 	if(ent->mass > 0)
	// 	{
	// 		continue;
	// 	}

	// 	btRigidBody* body = bsp_col.bodies[i].body;

	// 	if(body->getActivationState() == DISABLE_SIMULATION)
	// 	{
	// 		continue;
	// 	}

	// 	btTransform fx = btTransform::getIdentity();
	// 	btVector3 origin;

	// 	VectorCopy(ent->s.origin, origin);

	// 	btScalar x = RAD2DEG(ent->s.angles[0]), y = RAD2DEG(ent->s.angles[1]), z = RAD2DEG(ent->s.angles[2]);
	// 	fx.getBasis().setEulerZYX(x, y, z);

	// 	body->getMotionState()->setWorldTransform(fx);
	// }

	if (bsp_col.physicsWorld != NULL)
	{
		auto x = realClock.getTimeSeconds();
		realClock.reset();

		bsp_col.physicsWorld->stepSimulation(x, 10);
	}

	//Apply simulation to entities
	for (size_t i = 0; i < bsp_col.bodies.size(); ++i)
	{
		auto ent = bsp_col.bodies[i].ent;
		if(!ent)
		{
			continue;
		}

		if(ent->mass == 0)
		{
			continue;
		}


		btRigidBody* body = bsp_col.bodies[i].body;

		btTransform fx = btTransform::getIdentity();
		body->getMotionState()->getWorldTransform(fx);

		btVector3 vec = fx.getOrigin();
		// btVector3 oldPos = vec;
		// MatrixMult(bulletQuakeMat, oldPos, vec);
		VectorCopy(vec, ent->s.origin);

		// >>> Not working
		// vec3_t displacement;
		// VectorSubtract(ent->absmin, vec, displacement);
		// VectorCopy(vec, ent->absmin);
		// VectorAdd(displacement, ent->absmax, ent->absmax);
		// <<

		// auto model = bsp_col.bodies[i].model;
		// VectorCopy(vec, model->origin);

		btScalar yaw, pitch, roll;
		body->getCenterOfMassTransform().getBasis().getEulerZYX(yaw, pitch, roll, 1);

		ent->s.angles[YAW] = anglemod(RAD2DEG(yaw));//fmod(RAD2DEG(yaw) + 540.0, 360.0) - 180.0;
		ent->s.angles[PITCH] = anglemod(RAD2DEG(pitch));//fmod(RAD2DEG(pitch) + 540.0, 360.0) - 180.0;
		ent->s.angles[ROLL] = anglemod(RAD2DEG(roll));//fmod(RAD2DEG(roll) + 540.0, 360.0) - 180.0;
		
		// btVector3 vel = body->getLinearVelocity();
		// btVector3 avel = body->getAngularVelocity();
		
		//ent->movetype = MOVETYPE_FLY;//(MOVETYPE_FLY | MOVETYPE_TOSS | MOVETYPE_PUSH);

		// ent->inuse = qtrue;
        gi.linkentity(ent);
		// ent->inuse = qfalse;
        //G_TouchTriggers(ent);

		// if(i == 1)
		// Com_LPrintf(PRINT_DEVELOPER, "Shape #%d (%d) Pos: %.2f, %.2f, %.2f -> %.2f, %.2f, %.2f\n", i, body->getActivationState(), ent->s.origin[0], ent->s.origin[1], ent->s.origin[2], oldPos[0], oldPos[1], oldPos[2]);



		// if (entity->touch)
		// {
		// 	auto vel = bsp_col.bodies[i].body->getLinearVelocity() / (WORLDSCALE * 15);

		// 	vec3_t start, end;

		// 	VectorCopy(vec, start);
		// 	auto endV = vec + vel;
		// 	VectorCopy(endV, end);

		// 	trace_t trace = gi.trace(start, NULL, NULL, end, entity, CONTENTS_SOLID|CONTENTS_WINDOW|CONTENTS_MONSTER|CONTENTS_DEADMONSTER);

		// 	if (trace.fraction < 1.0)
		// 	{
		// 		if ((trace.ent == Q_World && !bsp_col.bodies[i].body->getLinearVelocity().isZero()) || (trace.ent != Q_World))
		// 			entity->touch(entity, trace.ent, &trace.plane, trace.surface);
		// 	}
		// }


		// if (entity->physicBody == NULL)
		// 	continue;

		//TODO: find a way to signal .quat should be used with bullet
		//entity->s.type |= ET_QUATERNION;

		// auto pc = gi.pointcontents(entity->s.origin);

		// vec3_t tempOrg;
		// VectorCopy(entity->s.origin, tempOrg);
		// tempOrg[2] -= 10;

		// if (pc & MASK_WATER)
		// {
		// 	vec3_t temp;
		// 	VectorCopy(entity->s.origin, temp);

		// 	temp[2] -= 2048;
		// 	trace_t downTrace = gi.trace(entity->s.origin, vec3_origin, vec3_origin, temp, NULL, CONTENTS_SOLID);

		// 	temp[2] += 4096;
		// 	trace_t upTrace = gi.trace(entity->s.origin, vec3_origin, vec3_origin, temp, NULL, CONTENTS_SOLID);

		// 	vec3_t temp2;
		// 	VectorCopy(temp, temp2);
		// 	temp2[2] -= 8096;
		// 	trace_t downFromUpTrace = gi.trace(temp, vec3_origin, vec3_origin, temp2, NULL, MASK_WATER);

		// 	float entDistanceToLip = downFromUpTrace.endpos[2] - entity->s.origin[2];

		// 	bsp_col.bodies[i].body->setGravity(btVector3(0, 0, 60 * (entDistanceToLip / 100)));
		// 	bsp_col.bodies[i].body->setDamping(bsp_col.bodies[i].body->getNormalLinearDamping() + 0.35f, bsp_col.bodies[i].body->getNormalAngularDamping() + 0.35f);
		// }
		// else if (gi.pointcontents(tempOrg) & MASK_WATER)
		// {
		// 	bsp_col.bodies[i].body->setGravity(btVector3(0, 0, -(34)));
		// 	bsp_col.bodies[i].body->setDamping(bsp_col.bodies[i].body->getNormalLinearDamping() + 0.35f, bsp_col.bodies[i].body->getNormalAngularDamping() + 0.35f);
		// }
		// else
		// {
		// 	bsp_col.bodies[i].body->setDamping(bsp_col.bodies[i].body->getNormalLinearDamping(), bsp_col.bodies[i].body->getNormalAngularDamping());
		// 	bsp_col.bodies[i].body->setGravity(btVector3(0, 0, -(170)));

		// }

		// gi.linkentity(entity);
	}
}

EXPORT void COL_Destroy(bsp_t* bsp)
{
	if(!bsp)
	{
	    Com_LPrintf(PRINT_ERROR, "Double free of Collision!!\n");
		return;
	}

    Com_LPrintf(PRINT_NOTICE, "Destroying Collision\n");

	//delete collision shapes
	for (size_t j = bsp_col.bodies.size() - 1; j < bsp_col.bodies.size(); --j)
	{
        btRigidBody* body = bsp_col.bodies[j].body;
		if (body->getMotionState())
		{
			delete body->getMotionState();
		}
		bsp_col.physicsWorld->removeCollisionObject(body);

		btCollisionShape* shape = bsp_col.bodies[j].shape;
		bsp_col.bodies[j].shape = nullptr;
		bsp_col.bodies[j].body = nullptr;
		delete shape;
        delete body;
	}
    bsp_col.bodies.clear();

    if(bsp_col.physicsWorld->getCollisionObjectArray().size() > 0)
    {
        Com_LPrintf(PRINT_ERROR, "Not all Collision objects have been deleted!\n");

        for (size_t i = bsp_col.physicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
        {
            btCollisionObject* obj = bsp_col.physicsWorld->getCollisionObjectArray()[i];
            if(!obj) continue;
            btRigidBody* body = btRigidBody::upcast(obj);
            if (body && body->getMotionState())
            {
                delete body->getMotionState();
            }
            bsp_col.physicsWorld->removeCollisionObject(obj);
            delete body;
        }
    }

	delete bsp_col.physicsWorld;
	delete bsp_col.physicsSolver;
	delete bsp_col.physicsBroadphase;
	delete bsp_col.physicsDispatcher;
	delete bsp_col.physicsConfig;

	bsp_col.physicsWorld = nullptr;
	bsp_col.physicsSolver = nullptr;
	bsp_col.physicsBroadphase = nullptr;
	bsp_col.physicsDispatcher = nullptr;
	bsp_col.physicsConfig = nullptr;
}

EXPORT void COL_SpawnEntity(edict_t* ent)
{		
	if(ent->s.modelindex > bsp_col.bodies.size())
	{
        Com_LPrintf(PRINT_ERROR, "Unabtransformle to link entity to collision body: modelindex out of bounds!\n");
		return;
	}

	if(ent->mass > 0)
	{
		collision_body_t& col_body = bsp_col.bodies[ent->s.modelindex - 1];

		col_body.ent = ent;
		btRigidBody* body = col_body.body;

		bsp_col.physicsWorld->removeRigidBody(body);

		btVector3 inertia(0, 0, 0);
		body->getCollisionShape()->calculateLocalInertia(ent->mass, inertia);
		body->setMassProps(static_cast<float>(ent->mass), inertia);
		body->forceActivationState(ACTIVE_TAG);

		btTransform transform = body->getWorldTransform();
		btVector3 origin(0, 0, 0);
		// VectorCopy(ent->absmin, origin);
		VectorCopy(ent->s.origin, origin);
		transform.setOrigin(origin);

		auto q = btQuaternion::getIdentity();
		q.setEulerZYX(DEG2RAD(ent->s.angles[1]), DEG2RAD(ent->s.angles[0]), DEG2RAD(ent->s.angles[2]));
		transform.setRotation(q);

		body->getMotionState()->setWorldTransform(transform);
		// body->setGravity(btVector3(0, 0, -10));
		// body->setLinearVelocity(btVector3(0, 0.3, 0));
		body->applyCentralForce(btVector3(0, -0.5, 0.25));
		body->setAngularVelocity(btVector3(0, 0.9, 0));

		bsp_col.physicsWorld->addRigidBody(body);

		ent->movetype = MOVETYPE_BULLETPHYSICS;
	}
}

EXPORT void COL_DebugDraw()
{
	bsp_col.physicsWorld->debugDrawWorld();
}
