#include <stdlib.h>
#include <cstdint>
#include <algorithm>

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
#include "BulletDynamics/ConstraintSolver/btNNCGConstraintSolver.h"

#include <vector>
#include <map>

class QuakeDebugDrawer;

struct collision_body_t {
    btRigidBody* body;
    btCollisionShape* shape;
	mmodel_t* model;
	edict_t* ent;
	btVector3 spawnOffset;
};

struct collision_constraint_t {
	edict_t* ent;
	btGeneric6DofSpring2Constraint* constraint;
};

struct collision_waiting_constraint_t {
	edict_t* ent;
    vec3_t minaxis;
    vec3_t maxaxis;
    vec3_t minangle;
    vec3_t maxangle;
};

struct bsp_collision_t {
    btDiscreteDynamicsWorld *physicsWorld;
    btDefaultCollisionConfiguration *physicsConfig;
    btCollisionDispatcher *physicsDispatcher;
    btAxisSweep3 *physicsBroadphase;
    btSequentialImpulseConstraintSolver *physicsSolver;

	QuakeDebugDrawer* debugDrawer;

	btRigidBody* fixedBody; //Because btTypedConstraint::getFixedBody() is an abomination

    std::vector<collision_body_t> bodies;
    std::vector<collision_constraint_t> constraints;
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
	//bsp_col.physicsSolver = new btSequentialImpulseConstraintSolver;
	bsp_col.physicsSolver = new btNNCGConstraintSolver;
	bsp_col.physicsWorld = new btDiscreteDynamicsWorld(
        bsp_col.physicsDispatcher,
        bsp_col.physicsBroadphase,
        bsp_col.physicsSolver,
        bsp_col.physicsConfig);
	bsp_col.physicsWorld->getSolverInfo().m_numIterations = 30;
	bsp_col.physicsWorld->setGravity(btVector3(0, 0, -640));

	bsp_col.debugDrawer = new QuakeDebugDrawer();
	bsp_col.physicsWorld->setDebugDrawer(bsp_col.debugDrawer);

	bsp_col.debugDrawer->setDebugMode(
		btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawAabb + btIDebugDraw::DBG_DrawContactPoints + btIDebugDraw::DBG_DrawNormals
		 + btIDebugDraw::DBG_DrawConstraints + btIDebugDraw::DBG_DrawConstraintLimits
		//btIDebugDraw::DBG_DrawContactPoints
	);

	bsp_col.fixedBody = new btRigidBody(0, 0, 0);
	bsp_col.fixedBody->setMassProps(btScalar(0.), btVector3(btScalar(0.), btScalar(0.), btScalar(0.)));

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
				btTransform lt;
				lt.setIdentity();

				compoundShape->addChildShape(lt, htPair.shape);
				motionTransform.setOrigin(htPair.transform.getOrigin());
			}
		}

		btCollisionShape* colShape = static_cast<btCollisionShape*>(compoundShape);
		if(isCompoundShape)
		{
			motionTransform.setOrigin(compoundTransform.getOrigin());
		}
		else
		{
			// delete compoundShape;
			// compoundShape = nullptr;

			// colShape = static_cast<btCollisionShape*>(shapeVector.front().shape);
			// motionTransform.setOrigin(shapeVector.front().transform.getOrigin());
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

		bsp_col.bodies.push_back(collision_body_t{body, colShape, modelBrushPlanesMap.first, nullptr, {motionTransform.getOrigin()[0], motionTransform.getOrigin()[1], motionTransform.getOrigin()[2]}});
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

namespace {
	static btQuaternion ConvertQuat (btQuaternion &quat)
	{
		btMatrix3x3 mat (quat);
		btMatrix3x3 tempMat;

		for (int x = 0; x < 3; ++x)
			for (int y = 0; y < 3; ++y)
			{
				tempMat[x][y] = mat[y][x];
			}

			btQuaternion q;
			tempMat.getRotation(q);
			return q;
	}
}

EXPORT void COL_Think()
{	
    static btClock realClock;

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

		btQuaternion oriQuat = fx.getRotation();
		btVector3 transFormedSpawnOffset = quatRotate(oriQuat, bsp_col.bodies[i].spawnOffset);
		btVector3 vec = fx.getOrigin() - transFormedSpawnOffset;
		VectorCopy(vec, ent->s.origin);

		btScalar yaw, pitch, roll;
		body->getCenterOfMassTransform().getBasis().getEulerYPR(yaw, pitch, roll);

		ent->s.angles[YAW] = anglemod(RAD2DEG(yaw));
		ent->s.angles[PITCH] = anglemod(RAD2DEG(pitch));
		ent->s.angles[ROLL] = anglemod(RAD2DEG(roll));
		
        gi.linkentity(ent);
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

	//delete constraints
	for (size_t c = bsp_col.constraints.size() - 1; c < bsp_col.constraints.size(); --c)
	{
		btGeneric6DofSpring2Constraint* constraint = bsp_col.constraints[c].constraint;
		bsp_col.physicsWorld->removeConstraint(constraint);
		delete constraint;
	}
	bsp_col.constraints.clear();

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
	delete bsp_col.fixedBody; //This is why we needed this: 
							  // we cannot delete btTypedConstraint::getFixedBody(), 
							  // which causes an assert upon destruction.
							  // Bullet, sad.

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

void Col_DoSpawnConstraint(collision_waiting_constraint_t& constraintEnt, collision_body_t& bodyEnt1)
{
	if(!bodyEnt1.ent)
	{
		Com_LPrintf(PRINT_DEVELOPER, "Not spawning constraint: body missing entity\n");
		return;
	}
	if(!(bodyEnt1.ent->mass > 0))
	{
		Com_LPrintf(PRINT_DEVELOPER, "Not spawning constraint: body is static\n");
		return;
	}

	Com_LPrintf(PRINT_DEVELOPER, "Adding constraint for %s\n", constraintEnt.ent->target);

	btTransform frame1 = btTransform::getIdentity();
	btTransform world1 = btTransform::getIdentity();

	world1.setOrigin(bodyEnt1.spawnOffset);

	if(!(constraintEnt.ent->spawnflags & 4))
	{
		btVector3 origin;
		VectorCopy(constraintEnt.ent->s.origin, origin);
		frame1.setOrigin(origin - world1.getOrigin());
	}

	btTransform frame2 = bodyEnt1.body->getCenterOfMassTransform() * frame1;

	btGeneric6DofSpring2Constraint* constraint = new btGeneric6DofSpring2Constraint(*bsp_col.fixedBody, *bodyEnt1.body, frame2, frame1);

	//Limits
	constraint->setLimit(0, constraintEnt.minaxis[0], constraintEnt.maxaxis[0]);
	constraint->setLimit(1, constraintEnt.minaxis[1], constraintEnt.maxaxis[1]);
	constraint->setLimit(2, constraintEnt.minaxis[2], constraintEnt.maxaxis[2]);
	constraint->setLimit(3, DEG2RAD(constraintEnt.minangle[0]), DEG2RAD(constraintEnt.maxangle[0]));
	constraint->setLimit(4, DEG2RAD(constraintEnt.minangle[1]), DEG2RAD(constraintEnt.maxangle[1]));
	constraint->setLimit(5, DEG2RAD(constraintEnt.minangle[2]), DEG2RAD(constraintEnt.maxangle[2]));

	//Motors
	// spawnflags & 1 enable motor
	// spawnflags & 2 start activated
	// style -> axis
	// target velocity -> wait
	// max force -> delay
	if(constraintEnt.ent->spawnflags & 1)
	{
		constraint->enableMotor(constraintEnt.ent->style, constraintEnt.ent->spawnflags & 2);
		constraint->setTargetVelocity(constraintEnt.ent->style, DEG2RAD(constraintEnt.ent->wait));
		constraint->setMaxMotorForce(constraintEnt.ent->style, DEG2RAD(constraintEnt.ent->delay));

	/*
	TODO: connect (*use) to activate/deactivate motor
	*/
	}

	//Secret sauce to increase stability, make it more gamey
	for(int i = 0; i < 6; ++i)
	{
		//CFM == 0 -> hard constraint
		//CFM > 0  -> softer constraint
		//CFM < 0  -> unstable shit
		constraint->setParam(BT_CONSTRAINT_STOP_CFM, 0.1f, i);

		//ERP == 0 -> no correcting force
		//ERP == 1 -> full correcting force, but unstable
		constraint->setParam(BT_CONSTRAINT_STOP_ERP, 0.8f, i);
	}

	constraint->setDbgDrawSize(btScalar(16.f));

	bodyEnt1.body->setActivationState(DISABLE_DEACTIVATION);

	bsp_col.physicsWorld->addConstraint(constraint, true);
	bsp_col.constraints.push_back({constraintEnt.ent, constraint});

	if(constraintEnt.ent->spawnflags & (1 | 2)) // Motor enabled and on
	{
		//Give initial impulse
		if(constraintEnt.ent->style < 3)
		{
			btVector3 impulse(0,0,0);
			impulse[constraintEnt.ent->style] = 1.0f;
			bodyEnt1.body->setLinearVelocity(impulse * constraintEnt.ent->wait);
		}
		else
		{
			btVector3 torque(0,0,0);
			torque[constraintEnt.ent->style - 3] = DEG2RAD(1.0f);
			// bodyEnt1.body->setTurnVelocity(torque * constraintEnt.ent->wait);
			bodyEnt1.body->setAngularVelocity(torque * constraintEnt.ent->wait);
		}
	}
}

void Col_DoSpawnConstraint2(collision_waiting_constraint_t constraintEnt, collision_body_t& bodyEnt1, collision_body_t& bodyEnt2)
{
	if(!bodyEnt1.ent && !bodyEnt2.ent)
	{
		Com_LPrintf(PRINT_DEVELOPER, "Not spawning constraint: both bodies missing entity\n");
		return;
	}
	if(!(bodyEnt1.ent->mass > 0) && !(bodyEnt2.ent->mass > 0))
	{
		Com_LPrintf(PRINT_DEVELOPER, "Not spawning constraint: both bodies are static\n");
		return;
	}

	Com_LPrintf(PRINT_DEVELOPER, "Adding constraint %s -> %s\n", constraintEnt.ent->target, bodyEnt1.ent->target);

	btTransform frame1;
	btTransform world1;
	frame1.setIdentity();
	bodyEnt1.body->getMotionState()->getWorldTransform(world1);

	btTransform frame2;
	btTransform world2;
	frame2.setIdentity();
	bodyEnt2.body->getMotionState()->getWorldTransform(world2);

	btVector3 origin;
	VectorCopy(constraintEnt.ent->s.origin, origin);
	frame1.setOrigin(origin - world1.getOrigin());
	frame2.setOrigin(origin - world2.getOrigin());

	btGeneric6DofSpring2Constraint* constraint = new btGeneric6DofSpring2Constraint(*bodyEnt1.body, *bodyEnt2.body, frame1, frame2);

	//Limits
	constraint->setLimit(0, constraintEnt.minaxis[0], constraintEnt.maxaxis[0]);
	constraint->setLimit(1, constraintEnt.minaxis[1], constraintEnt.maxaxis[1]);
	constraint->setLimit(2, constraintEnt.minaxis[2], constraintEnt.maxaxis[2]);
	constraint->setLimit(3, DEG2RAD(constraintEnt.minangle[0]), DEG2RAD(constraintEnt.maxangle[0]));
	constraint->setLimit(4, DEG2RAD(constraintEnt.minangle[1]), DEG2RAD(constraintEnt.maxangle[1]));
	constraint->setLimit(5, DEG2RAD(constraintEnt.minangle[2]), DEG2RAD(constraintEnt.maxangle[2]));

	//Motors
	// spawnflags & 1 enable motor
	// spawnflags & 2 start activated
	// style -> axis
	// target velocity -> wait
	// max force -> delay
	if(constraintEnt.ent->spawnflags & 1)
	{
		constraint->enableMotor(constraintEnt.ent->style, constraintEnt.ent->spawnflags & 2);
		constraint->setTargetVelocity(constraintEnt.ent->style, constraintEnt.ent->wait);
		constraint->setMaxMotorForce(constraintEnt.ent->style, constraintEnt.ent->delay);

	/*
	TODO: connect (*use) to activate/deactivate motor
	*/
	}

	constraint->setDbgDrawSize(btScalar(16.f));

	bodyEnt1.body->setActivationState(DISABLE_DEACTIVATION);

	bsp_col.physicsWorld->addConstraint(constraint, true);
	bsp_col.constraints.push_back({constraintEnt.ent, constraint});
}


std::vector<collision_waiting_constraint_t> WaitingConstraints;

EXPORT void COL_SpawnEntity(edict_t* ent)
{		
	if(ent->s.modelindex > bsp_col.bodies.size())
	{
        Com_LPrintf(PRINT_ERROR, "Unable to link entity to collision body: modelindex out of bounds!\n");
		return;
	}

	if(ent->mass > 0)
	{
		collision_body_t& col_body = bsp_col.bodies[ent->s.modelindex - 1];

		col_body.ent = ent;
		btRigidBody* body = col_body.body;

		bsp_col.physicsWorld->removeRigidBody(body);

		btVector3 inertia(0, 0, 0);
		body->getCollisionShape()->calculateLocalInertia(ent->mass * 64.0f, inertia);
		body->setMassProps(static_cast<float>(ent->mass * 64.0f), inertia);
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
		body->setAngularVelocity(btVector3(0.9, 0, 0));

		bsp_col.physicsWorld->addRigidBody(body);

		ent->movetype = MOVETYPE_BULLETPHYSICS;

		std::remove_if(WaitingConstraints.begin(), WaitingConstraints.end(), [&col_body](collision_waiting_constraint_t& waitingEnt){
			if(!col_body.ent)
			{
				return false;
			}
			if(!col_body.ent->targetname)
			{
				return false;
			}
			if (Q_stricmp(col_body.ent->targetname, waitingEnt.ent->target) == 0)
			{
				Col_DoSpawnConstraint(waitingEnt, col_body);
				return true;
			}
			return false;
		});
	}
}

EXPORT void COL_DebugDraw()
{
	bsp_col.physicsWorld->debugDrawWorld();
}

EXPORT void Col_SpawnConstraint(edict_t* ent)
{
	if(!ent->target)
	{
		return;
	}

	collision_waiting_constraint_t waitConstraint {
		ent, 
		{st.minaxis[0], st.minaxis[1], st.minaxis[2]}, 
		{st.maxaxis[0], st.maxaxis[1], st.maxaxis[2]}, 
		{st.minangle[0], st.minangle[1], st.minangle[2]}, 
		{st.maxangle[0], st.maxangle[1], st.maxangle[2]}
	};
	for(auto& col : bsp_col.bodies)
	{
		if(!col.ent)
		{
			continue;
		}
		if(!col.ent->targetname)
		{
			continue;
		}
		if (Q_stricmp(col.ent->targetname, ent->target) == 0)
		{
			Col_DoSpawnConstraint(waitConstraint, col);
			return;
		}
	}

	//Not found, store for later
	WaitingConstraints.push_back(waitConstraint);
}