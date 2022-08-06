#include "col.h"

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

#undef clamp
#undef min
#undef max

#include <vector>
#include <list>
#include <map>
#include <thread>
#include <cstdint>
using uint8 = uint8_t;

#include <Jolt/Jolt.h>

// Jolt includes
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>

#undef DEG2RAD
#undef RAD2DEG
#define DEG2RAD(a)      ((a) * (JPH::JPH_PI / 180.0f))
#define RAD2DEG(a)      ((a) * (180.0f / JPH::JPH_PI))


using namespace JPH; //Ugh, I do not approve

namespace {
	Mat44 worldToPhysicsTransform;
	Mat44 physicsToWorldTransform;
	Vec3 worldScale = Vec3::sReplicate(1.0f/32.0f);
}

Float3 operator* (const Mat44& xf, Float3& v)
{
	Vec3 vv = xf * Vec3(v.x, v.y, v.z);
	return Float3(vv.GetX(), vv.GetY(), vv.GetZ());
}

class QuakeDebugRenderer : public DebugRenderer
{
	struct Vertex {
		Float3 xyz;
		Float4 rgba;
		Vertex() = default;
		Vertex(const Float3& p, const Float4& c) : xyz(p), rgba(c) { }
		Vertex(const Vec3& p, const Float4& c) : xyz(p.GetX(), p.GetY(), p.GetZ()), rgba(c) {}
		Vertex(const Float3& p, const Color& c) : xyz(p), rgba(float(c.r / 255), float(c.g / 255), float(c.b / 255), float(c.a / 255)) {}
		Vertex(const Vec3& p, const Color& c) : xyz(p.GetX(), p.GetY(), p.GetZ()), rgba(float(c.r / 255), float(c.g / 255), float(c.b / 255), float(c.a / 255)) {}
	};

	std::vector<Vertex> mLineVertices;
	std::vector<unsigned int> mLineIndices;
	std::vector<Vertex> mTriangleVertices;
	std::vector<unsigned int> mTriangleIndices;

	struct TriBatch : public RefTargetVirtual
	{
	public:
		JPH_OVERRIDE_NEW_DELETE

										TriBatch(uint32 inID)		: mID(inID) {  }

		virtual void					AddRef() override			{ ++mRefCount; }
		virtual void					Release() override			{ if (--mRefCount == 0) delete this; }

		atomic<uint32>					mRefCount = 0;
		uint32							mID;
		std::vector<Vertex> mTriangleVertices;
		std::vector<unsigned int> mTriangleIndices;		
	};

	std::list<TriBatch> mBatches;

	friend Vertex operator*(const Mat44& mat, const Vertex& v);
	public:

	virtual ~QuakeDebugRenderer() override
	{
		mLineVertices.clear();
		mLineIndices.clear();
		mTriangleVertices.clear();
		mTriangleIndices.clear();
	}

	void DrawLine(const Float3 &inFrom, const Float3 &inTo, ColorArg inColor) override
	{
		mLineIndices.push_back(mLineVertices.size());
		mLineVertices.push_back(
			Vertex {
				inFrom,
				inColor
			}
		);
		mLineIndices.push_back(mLineVertices.size());
		mLineVertices.push_back(
			Vertex {
				inTo,
				inColor
			}
		);
	}

	void DrawTriangle(Vec3Arg inV1, Vec3Arg inV2, Vec3Arg inV3, ColorArg inColor) override
	{
		mTriangleIndices.push_back(mTriangleVertices.size());
		mTriangleVertices.push_back(
			Vertex {
				inV1,
				inColor
			}
		);
		mTriangleIndices.push_back(mTriangleVertices.size());
		mTriangleVertices.push_back(
			Vertex {
				inV2,
				inColor
			}
		);
		mTriangleIndices.push_back(mTriangleVertices.size());
		mTriangleVertices.push_back(
			Vertex {
				inV3,
				inColor
			}
		);

	}

	Batch CreateTriangleBatch(const Triangle *inTriangles, int inTriangleCount) override
	{
		TriBatch& batch = mBatches.emplace_back(static_cast<uint32>(mBatches.size()));
		batch.mTriangleIndices.reserve(inTriangleCount);
		batch.mTriangleVertices.reserve(inTriangleCount);
		for(int i = 0; i < inTriangleCount; ++i)
		{
			const Triangle& tri = inTriangles[i];
			for(int t = 0; t < 3; ++t)
			{
				batch.mTriangleIndices.push_back(batch.mTriangleVertices.size());
				batch.mTriangleVertices.push_back(Vertex {
					tri.mV[t].mPosition,
					tri.mV[t].mColor,
				});
			}
		}

		return &batch;
	}

	Batch CreateTriangleBatch(const DebugRenderer::Vertex *inVertices, int inVertexCount, const uint32 *inIndices, int inIndexCount) override
	{
		TriBatch& batch = mBatches.emplace_back(static_cast<uint32>(mBatches.size()));
		batch.mTriangleIndices.reserve(inVertexCount);
		batch.mTriangleVertices.reserve(inIndexCount);
		for(int i = 0; i < inVertexCount; ++i)
		{
			batch.mTriangleVertices.push_back(Vertex {
				inVertices[i].mPosition,
				inVertices[i].mColor,
			});
		}
		for(int i = 0; i < inIndexCount; ++i)
		{
			batch.mTriangleIndices.push_back(inIndices[i]);
		}

		return &batch;
	}

	void DrawGeometry(Mat44Arg inModelMatrix, const AABox &inWorldSpaceBounds, float inLODScaleSq, ColorArg inModelColor, const GeometryRef &inGeometry, ECullMode inCullMode = ECullMode::CullBackFace, ECastShadow inCastShadow = ECastShadow::On, EDrawMode inDrawMode = EDrawMode::Solid) override;

	void DrawText3D(Vec3Arg inPosition, const string_view &inString, ColorArg inColor = Color::sWhite, float inHeight = 0.5f)
	{

	}

	void FlushDraw()
	{
		if (mTriangleVertices.size())
		{

			gi.R_DrawTriangles_Color((float*)mTriangleVertices.data(), mTriangleVertices.size(), &mTriangleIndices[0], mTriangleIndices.size(), sizeof(Vertex) / sizeof(float));
			
			mTriangleVertices.clear();
			mTriangleIndices.clear();
		}

		if (mLineVertices.size())
		{

			gi.R_DrawLines_Color((float*)mLineVertices.data(), mLineVertices.size(), &mLineIndices[0], mLineIndices.size(), sizeof(Vertex) / sizeof(float));
			
			mLineVertices.clear();
			mLineIndices.clear();
		}
	}
};

QuakeDebugRenderer::Vertex operator*(const Mat44& mat, const QuakeDebugRenderer::Vertex& v)
{
	return QuakeDebugRenderer::Vertex {
		mat * Vec3(v.xyz[0], v.xyz[1], v.xyz[2]),
		v.rgba
	};
}

void QuakeDebugRenderer::DrawGeometry(Mat44Arg inModelMatrix, const AABox &inWorldSpaceBounds, float inLODScaleSq, ColorArg inModelColor, const GeometryRef &inGeometry, ECullMode inCullMode, ECastShadow inCastShadow, EDrawMode inDrawMode)
{
	if(!inGeometry->mLODs.size())
	{
		return;
	}

	const TriBatch* batch = static_cast<const TriBatch*>(inGeometry->mLODs.front().mTriangleBatch.GetPtr());
	if (inDrawMode == EDrawMode::Wireframe)
	{
		for(const unsigned int i : batch->mTriangleIndices)
		{
			for(int i = 0; i < batch->mTriangleIndices.size(); i += 3)
			{
				for(int j = 0; j < 3; ++j)
				{
					int i0 = batch->mTriangleIndices[i + j];
					int i1 = batch->mTriangleIndices[i + ((j + 1) % 3)];

					const Vertex& v0 = batch->mTriangleVertices[i0];
					const Vertex& v1 = batch->mTriangleVertices[i1];
					mLineIndices.push_back(mLineVertices.size());
					mLineVertices.push_back(inModelMatrix * v0);
					mLineIndices.push_back(mLineVertices.size());
					mLineVertices.push_back(inModelMatrix * v1);
				}
			}
		}
	}
	else
	{
		size_t indexStart = mTriangleVertices.size();
		for(const Vertex& v : batch->mTriangleVertices)
		{
			mTriangleVertices.push_back(inModelMatrix * v);
		}
		for(const unsigned int i : batch->mTriangleIndices)
		{
			mTriangleIndices.push_back(i + indexStart);
		}
	}
}

static void TraceImpl(const char *inFMT, ...)
{ 
	// Format the message
	va_list list;
	va_start(list, inFMT);
	char buffer[1024];
	vsnprintf(buffer, sizeof(buffer), inFMT, list);

	// Print to the TTY
    Com_LPrintf(PRINT_DEVELOPER, "Jolt Physics: %s\n", buffer);
}

#ifdef JPH_ENABLE_ASSERTS
// Callback for asserts, connect this to your own assert handler if you have one
static bool AssertFailedImpl(const char *inExpression, const char *inMessage, const char *inFile, uint inLine)
{ 
	// Print to the TTY
    Com_LPrintf(PRINT_DEVELOPER, "Jolt Physics: ASSERT: %s:%d: (%s) %s\n", inFile, inLine, inExpression, inMessage);

	// Breakpoint
	return true;
};
#endif // JPH_ENABLE_ASSERTS

namespace Layers
{
	static constexpr uint8 NON_MOVING = 0;
	static constexpr uint8 MOVING = 1;
	static constexpr uint8 NUM_LAYERS = 2;
};

namespace BroadPhaseLayers
{
	static constexpr BroadPhaseLayer NON_MOVING(0);
	static constexpr BroadPhaseLayer MOVING(1);
	static constexpr uint NUM_LAYERS(2);
};

static bool MyBroadPhaseCanCollide(ObjectLayer inLayer1, BroadPhaseLayer inLayer2)
{
	switch (inLayer1)
	{
	case Layers::NON_MOVING:
		return inLayer2 == BroadPhaseLayers::MOVING;
	case Layers::MOVING:
		return true;	
	default:
		JPH_ASSERT(false);
		return false;
	}
}

// Function that determines if two object layers can collide
static bool MyObjectCanCollide(ObjectLayer inObject1, ObjectLayer inObject2)
{
	switch (inObject1)
	{
	case Layers::NON_MOVING:
		return inObject2 == Layers::MOVING; // Non moving only collides with moving
	case Layers::MOVING:
		return true; // Moving collides with everything
	default:
		JPH_ASSERT(false);
		return false;
	}
};

class MyContactListener : public ContactListener
{
public:
	// See: ContactListener
	ValidateResult OnContactValidate(const Body &inBody1, const Body &inBody2, const CollideShapeResult &inCollisionResult) override
	{
        // Com_LPrintf(PRINT_DEVELOPER, "Jolt Physics: ContactListener: OnContactValidate\n");

		// Allows you to ignore a contact before it is created (using layers to not make objects collide is cheaper!)
		return ValidateResult::AcceptAllContactsForThisBodyPair;
	}

	void OnContactAdded(const Body &inBody1, const Body &inBody2, const ContactManifold &inManifold, ContactSettings &ioSettings) override
	{
        // Com_LPrintf(PRINT_DEVELOPER, "Jolt Physics: ContactListener: OnContactAdded\n");
	}

	void OnContactPersisted(const Body &inBody1, const Body &inBody2, const ContactManifold &inManifold, ContactSettings &ioSettings) override
	{
        // Com_LPrintf(PRINT_DEVELOPER, "Jolt Physics: ContactListener: OnContactPersisted\n");
	}

	void OnContactRemoved(const SubShapeIDPair &inSubShapePair) override
	{ 
        // Com_LPrintf(PRINT_DEVELOPER, "Jolt Physics: ContactListener: OnContactRemoved\n");
	}
};

class MyBodyActivationListener : public BodyActivationListener
{
public:
	void OnBodyActivated(const BodyID &inBodyID, uint64 inBodyUserData) override
	{
        // Com_LPrintf(PRINT_DEVELOPER, "Jolt Physics: BodyActivationListener: OnBodyActivated\n");
	}

	void OnBodyDeactivated(const BodyID &inBodyID, uint64 inBodyUserData) override
	{
        // Com_LPrintf(PRINT_DEVELOPER, "Jolt Physics: BodyActivationListener: OnBodyDeactivated\n");
	}
};

// BroadPhaseLayerInterface implementation
// This defines a mapping between object and broadphase layers.
class BPLayerInterfaceImpl final : public BroadPhaseLayerInterface
{
public:
    BPLayerInterfaceImpl()
	{
		// Create a mapping table from object to broad phase layer
		mObjectToBroadPhase[Layers::NON_MOVING] = BroadPhaseLayers::NON_MOVING;
		mObjectToBroadPhase[Layers::MOVING] = BroadPhaseLayers::MOVING;
	}

	uint GetNumBroadPhaseLayers() const override
	{
		return BroadPhaseLayers::NUM_LAYERS;
	}

	BroadPhaseLayer GetBroadPhaseLayer(ObjectLayer inLayer) const override
	{
		JPH_ASSERT(inLayer < Layers::NUM_LAYERS);
		return mObjectToBroadPhase[inLayer];
	}

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
	const char* GetBroadPhaseLayerName(BroadPhaseLayer inLayer) const override
	{
		switch ((BroadPhaseLayer::Type)inLayer)
		{
		case (BroadPhaseLayer::Type)BroadPhaseLayers::NON_MOVING:	return "NON_MOVING";
		case (BroadPhaseLayer::Type)BroadPhaseLayers::MOVING:		return "MOVING";
		default:													JPH_ASSERT(false); return "INVALID";
		}
	}
#endif // JPH_EXTERNAL_PROFILE || JPH_PROFILE_ENABLED

private:
	BroadPhaseLayer mObjectToBroadPhase[Layers::NUM_LAYERS];
};

struct collision_body_t {    
    Body* body;
	Vec3 spawnOffset;
	edict_t* ent;
};

struct collision_constraint_t {
	edict_t* ent;
	Constraint* constraint;
};

struct collision_prepared_constraint_t {
	edict_t* ent;
    vec3_t minaxis;
    vec3_t maxaxis;
    vec3_t minangle;
    vec3_t maxangle;
};

struct pre_hull_t {
	Array<Vec3> vertices;
	float mass = 0.0f;
};

struct pre_compound_t {
	std::vector<pre_hull_t> hulls;
};

struct bsp_collision_s {    
	std::vector<pre_compound_t> prehullgroups;
    std::vector<collision_body_t> bodies;
    std::vector<collision_constraint_t> constraints;
	Body* worldFixed;
};

namespace {
    TempAllocatorImpl* temp_allocator = nullptr;
    JobSystemThreadPool* job_system = nullptr;
    BPLayerInterfaceImpl* broad_phase_layer_interface = nullptr;
    PhysicsSystem* physics_system = nullptr;
    MyBodyActivationListener* body_activation_listener = nullptr;
    MyContactListener* contact_listener = nullptr;
	BodyInterface* interface = nullptr;

	QuakeDebugRenderer* debug_renderer = nullptr;

// This is the max amount of rigid bodies that you can add to the physics system. If you try to add more you'll get an error.
	// Note: This value is low because this is a simple test. For a real project use something in the order of 65536.
	const uint cMaxBodies = 1024;

	// This determines how many mutexes to allocate to protect rigid bodies from concurrent access. Set it to 0 for the default settings.
	const uint cNumBodyMutexes = 0;

	// This is the max amount of body pairs that can be queued at any time (the broad phase will detect overlapping
	// body pairs based on their bounding boxes and will insert them into a queue for the narrowphase). If you make this buffer
	// too small the queue will fill up and the broad phase jobs will start to do narrow phase work. This is slightly less efficient.
	// Note: This value is low because this is a simple test. For a real project use something in the order of 65536.
	const uint cMaxBodyPairs = 1024;

	// This is the maximum size of the contact constraint buffer. If more contacts (collisions between bodies) are detected than this
	// number then these contacts will be ignored and bodies will start interpenetrating / fall through the world.
	// Note: This value is low because this is a simple test. For a real project use something in the order of 10240.
	const uint cMaxContactConstraints = 1024;

	const int cCollisionSteps = 1;

	const int cIntegrationSubSteps = 3;

	const float cDeltaTime = 1.0f / 60.0f;

	//Utils from bullet

#ifdef _DEBUG
	struct planeVectorPair
	{
		Vec4 vector;
		cplane_t* plane;
	};
	using planeVector = std::vector<planeVectorPair>;
#define STORE_PLANE_VEC(equation) planeVectorPair {equation, plane}
#define LOAD_PLANE_VEC(vect) vect.vector
#else
	using planeVector = std::vector<Vec4>;
#define STORE_PLANE_VEC(equation) equation
#define LOAD_PLANE_VEC(vect) vect
#endif

	bool isPointInsidePlanes(const planeVector& planeEquations, const Vec3& point, float margin)
	{
		int numbrushes = planeEquations.size();
		for (int i = 0; i < numbrushes; i++)
		{
			const Vec4& planeValue = LOAD_PLANE_VEC(planeEquations[i]);
			const Vec3 N1(planeValue.GetX(), planeValue.GetY(), planeValue.GetZ());
			float dist = N1.Dot(point) + planeValue.GetW() - margin;
			if (dist > 0.f)
			{
				return false;
			}
		}
		return true;
	}

	Vec3 CrossXYZ(const Vec4& a, const Vec4& b)
	{
		return Vec3(a.GetX(), a.GetY(), a.GetZ()).Cross(Vec3(b.GetX(), b.GetY(), b.GetZ()));
	}

#ifdef _DEBUG
	struct corner_t
	{
		float x, y, z;
		corner_t(const Vec3& v) : x(v.GetX()), y(v.GetY()), z(v.GetZ()) {}
	};
	std::map<const cplane_t*, std::vector<corner_t>> _CachedPlaneCorners;
	std::vector<std::vector<corner_t>> _CachedPlaneCornersById;
#endif

	void GetVerticesFromPlaneEquations(const planeVector& planeEquations, pre_hull_t& preHull)
	{
		Array<Vec3>& verticesOut = preHull.vertices;

		const int numbrushes = planeEquations.size();
		// brute force:
		for (int i = 0; i < numbrushes; i++)
		{
			const Vec4& N1 = LOAD_PLANE_VEC(planeEquations[i]);

			for (int j = i + 1; j < numbrushes; j++)
			{
				const Vec4& N2 = LOAD_PLANE_VEC(planeEquations[j]);

				for (int k = j + 1; k < numbrushes; k++)
				{
					const Vec4& N3 = LOAD_PLANE_VEC(planeEquations[k]);

					Vec3 n2n3 = CrossXYZ(N2, N3);
					Vec3 n3n1 = CrossXYZ(N3, N1);
					Vec3 n1n2 = CrossXYZ(N1, N2);

					const float ln2n3sq = n2n3.LengthSq();
					const float ln3n1sq = n3n1.LengthSq();
					const float ln1n2sq = n1n2.LengthSq();

					if ((ln2n3sq > float(0.0001)) &&
						(ln3n1sq > float(0.0001)) &&
						(ln1n2sq > float(0.0001)))
					{
						//point P out of 3 plane equations:

						//	d1 ( N2 * N3 ) + d2 ( N3 * N1 ) + d3 ( N1 * N2 )
						//P =  -------------------------------------------------------------------------
						//   N1 . ( N2 * N3 )

						float quotient = (Vec3(N1.GetX(), N1.GetY(), N1.GetZ()).Dot(n2n3));
						if (fabs(quotient) > float(0.000001))
						{
							quotient = float(-1.) / quotient;
							n2n3 *= N1.GetW();
							n3n1 *= N2.GetW();
							n1n2 *= N3.GetW();
							Vec3 potentialVertex = n2n3;
							potentialVertex += n3n1;
							potentialVertex += n1n2;
							potentialVertex *= quotient;

							//check if inside, and replace supportingVertexOut if needed
							if (isPointInsidePlanes(planeEquations, potentialVertex, float(0.01)))
							{
								verticesOut.push_back(potentialVertex);
#ifdef _DEBUG
								//Also connect vertex to cplane_t ptr
								_CachedPlaneCorners[planeEquations[i].plane].push_back(potentialVertex);
								_CachedPlaneCorners[planeEquations[j].plane].push_back(potentialVertex);
								_CachedPlaneCorners[planeEquations[k].plane].push_back(potentialVertex);
#endif
							}
						}
					}
				}
			}
		}
	}

	Vec3 GetEulerYPR(const Mat44& rotation)
	{
		// first use the normal calculus
		Mat44 rott = rotation;
		float yaw = float(atan2(rott.GetAxisY().GetX(), rott.GetAxisX().GetX()));
		float pitch = float(asin(-rott.GetAxisZ().GetX()));
		float roll = float(atan2(rott.GetAxisZ().GetY(), rott.GetAxisZ().GetZ()));

		// on pitch = +/-HalfPI
		static float HALF_PI = JPH_PI * 0.5f;
		if (fabs(pitch) == HALF_PI)
		{
			if (yaw > 0)
				yaw -= JPH_PI;
			else
				yaw += JPH_PI;

			if (roll > 0)
				roll -= JPH_PI;
			else
				roll += JPH_PI;
		}

		return Vec3(yaw, pitch, roll);
	};

	void GetEulerSlartibarty(const Mat44& rotation, float angles[3])
	{
		Vec3 flatForward = rotation.GetAxisX();
		flatForward.SetZ(0.0f);
		
		float xyDist = flatForward.Length();

		// enough here to get angles?
		if ( xyDist > 0.001f )
		{
			angles[1] = RAD2DEG( atan2f( flatForward.GetY(), flatForward.GetX() ) );
			angles[0] = RAD2DEG( atan2f( -rotation.GetAxisX().GetZ(), xyDist ) );
			angles[2] = RAD2DEG( atan2f( rotation.GetAxisY().GetZ(), rotation.GetAxisZ().GetZ() ) );
		}
		else	// forward is mostly Z, gimbal lock-
		{
			angles[1] = RAD2DEG( atan2f( -rotation.GetAxisY().GetX(), rotation.GetAxisY().GetY() ) );
			angles[0] = RAD2DEG( atan2f( -rotation.GetAxisX().GetZ(), xyDist ) );
			angles[2] = 0.0f;
		}
	}

	static bsp_collision_t bsp_col;
}

void COL_OneTimeInit()
{
	// Register allocation hook
    RegisterDefaultAllocator();

	// Install callbacks
    JPH::Trace = TraceImpl;
    JPH_IF_ENABLE_ASSERTS(AssertFailed = AssertFailedImpl;)

	// Setup spaces transformation
	physicsToWorldTransform = Mat44::sIdentity();
	physicsToWorldTransform.SetAxisX(Vec3::sAxisX() * 32.0f);
	physicsToWorldTransform.SetAxisY(Vec3::sAxisY() * 32.0f);
	physicsToWorldTransform.SetAxisZ(Vec3::sAxisZ() * 32.0f);
	worldToPhysicsTransform = physicsToWorldTransform.Inversed();

	// Create a factory
	Factory::sInstance = new Factory();

	// Register all Jolt physics types
	RegisterTypes();

    temp_allocator = new TempAllocatorImpl(10 * 1024 * 1024);

    job_system = new JobSystemThreadPool(cMaxPhysicsJobs, cMaxPhysicsBarriers, thread::hardware_concurrency() - 1);

    broad_phase_layer_interface = new BPLayerInterfaceImpl();

    physics_system = new PhysicsSystem();
    physics_system->Init(
        cMaxBodies, cNumBodyMutexes, cMaxBodyPairs, cMaxContactConstraints, 
        *broad_phase_layer_interface, 
        MyBroadPhaseCanCollide, 
        MyObjectCanCollide
    );

    body_activation_listener = new MyBodyActivationListener();
    contact_listener = new MyContactListener();

    physics_system->SetBodyActivationListener(body_activation_listener);
    physics_system->SetContactListener(contact_listener);

	interface = &physics_system->GetBodyInterface();

#if 0
	debug_renderer = new QuakeDebugRenderer();
#endif

	Com_LPrintf(PRINT_NOTICE, "Collision initialized\n");
}


JPH::Vec3 LocalizeVerticesNewOrigin(Array<JPH::Vec3>& vertices)
{
	JPH::Vec3 computeMin = vertices.at(0);
	JPH::Vec3 computeMax = vertices.at(0);
	for(size_t i = 1; i < vertices.size(); ++i)
	{
		computeMin.SetX(computeMin.GetX() < vertices[i].GetX() ? computeMin.GetX() : vertices[i].GetX());
		computeMin.SetY(computeMin.GetY() < vertices[i].GetY() ? computeMin.GetY() : vertices[i].GetY());
		computeMin.SetZ(computeMin.GetZ() < vertices[i].GetZ() ? computeMin.GetZ() : vertices[i].GetZ());
		computeMax.SetX(computeMax.GetX() > vertices[i].GetX() ? computeMax.GetX() : vertices[i].GetX());
		computeMax.SetY(computeMax.GetY() > vertices[i].GetY() ? computeMax.GetY() : vertices[i].GetY());
		computeMax.SetZ(computeMax.GetZ() > vertices[i].GetZ() ? computeMax.GetZ() : vertices[i].GetZ());
	}
	JPH::Vec3 computeCenter = (computeMax - computeMin) * 0.5f + computeMin;
	for(size_t i = 0; i < vertices.size(); ++i)
	{
		vertices[i] = vertices[i] - computeCenter;
	}
	return computeCenter;
}

EXPORT void COL_Init(bsp_t* bsp)
{
	if(!bsp)
		return;

    if(!physics_system)
    {
        COL_OneTimeInit();
    }

	physics_system->SetGravity(Vec3(0, 0, -640));

    //Iterate Map brush planes and collect their vertices
	struct BrushPlaneContext_T {
		std::map<mmodel_t*, std::map<mbrush_t*, planeVector > > planeEquations;
	};
	BrushPlaneContext_T context;

	gi.BSP_RecurseBrushPlanes(
		bsp, &context, (CONTENTS_SOLID|CONTENTS_WINDOW),
		+[](bsp_t* bsp, mmodel_t* model, mbrush_t* brush, cplane_t* plane, float plane_nx, float plane_ny, float plane_nz, float plane_d, void* context)
		{
			BrushPlaneContext_T& brushPlaneContext = *(BrushPlaneContext_T*)context;

			Vec4 planeEquation(plane_nx, plane_ny, plane_nz, plane_d);

			brushPlaneContext.planeEquations[model][brush].push_back(STORE_PLANE_VEC(planeEquation));
		}
	);

	bsp_col.prehullgroups.reserve(context.planeEquations.size());
	bsp_col.bodies.reserve(context.planeEquations.size());

	using PreparedHullGroupVector = std::vector<pre_hull_t>;

    for(const std::pair<mmodel_t*, std::map<mbrush_t*, planeVector > >& modelBrushPlanesMap : context.planeEquations)
    {
        Com_LPrintf(PRINT_DEVELOPER, "Collecting Collision MapModel\n");
		if(modelBrushPlanesMap.second.size() == 0)
			continue;

		pre_compound_t& preCompound = bsp_col.prehullgroups.emplace_back();
		bsp_col.bodies.push_back(collision_body_t{nullptr, Vec3::sZero(), nullptr});
		preCompound.hulls.reserve(modelBrushPlanesMap.second.size());

		for(const std::pair<mbrush_t*, planeVector >& brushPlanesMap : modelBrushPlanesMap.second)
		{
			pre_hull_t& hull = preCompound.hulls.emplace_back();

			GetVerticesFromPlaneEquations(brushPlanesMap.second, hull);
		}
    }

#ifdef _DEBUG
	//Now sort the plane corners, so drawing is easy
	std::vector<const cplane_t*> planes;
	for(auto& planeCorners : _CachedPlaneCorners)
	{
		Vec3 centroid(0.f, 0.f, 0.f);
		for(auto& corner : planeCorners.second)
		{
			centroid += Vec3(corner.x, corner.y, corner.z);
		}
		centroid /= planeCorners.second.size();

		//squash vertices to 2D
		struct PreSortedVec {
			Vec3 corner;
			Vec3 originalCorner;
			float angle;
			int index;
		};
		std::vector<PreSortedVec> cornersCentroidNormalizedProjected;
		Vec3 normal {planeCorners.first->normal[0], planeCorners.first->normal[1], planeCorners.first->normal[2]};
		Vec3 normal_d {planeCorners.first->dist, planeCorners.first->dist, planeCorners.first->dist};
		cornersCentroidNormalizedProjected.reserve(planeCorners.second.size());
		int i = 0;
		for(auto& corner : planeCorners.second)
		{
			Vec3 cv(corner.x, corner.y, corner.z);
			//Vec3 cpj = (cv - centroid);
			Vec3 cpj = (cv - (normal*cv+normal_d).Cross(normal)) - centroid;
			cornersCentroidNormalizedProjected.push_back(
				{
					cpj,
					cv,
					0.0f,
					i++
				}
			);
		}

		//build a matrix, which allows us to transform our 3D to 2D
		Mat44 mat3to2 = Mat44::sIdentity();
		mat3to2.SetAxisZ(normal); //our new UP
		Vec3 side = (Vec3(planeCorners.second.front().x, planeCorners.second.front().y, planeCorners.second.front().z) - centroid).Normalized();
		mat3to2.SetAxisX(side); //our new FWD
		mat3to2.SetAxisY(normal.Cross(side));

		//transform our points to 2D (note, this simply transforms the 3D point in such a way that Z becomes 0)
		for(auto& pt : cornersCentroidNormalizedProjected)
		{
			pt.corner = mat3to2 * pt.corner;
			//JPH_ASSERT(fabs(pt.corner.GetZ()) < 0.000001f, "Expected point to be 2D, it's not!");

			//find angle to centroid with atan2, while we're here anyways
			pt.angle = atan2(pt.corner.GetY(), pt.corner.GetX());
		}

		//sort by angle
		std::sort(
			cornersCentroidNormalizedProjected.begin(), 
			cornersCentroidNormalizedProjected.end(), 
			[](PreSortedVec& left, PreSortedVec& right)
			{
				return left.angle < right.angle;
			}
		);

		//apply & profit
		for(int i = 0; i < planeCorners.second.size(); ++i)
		{
			planeCorners.second[i] = cornersCentroidNormalizedProjected[i].originalCorner;
		}

		planes.push_back(planeCorners.first);
	}

	std::sort(planes.begin(), planes.end(), [](const cplane_t* left, const cplane_t* right){
		return left->id < right->id;
	});

	_CachedPlaneCornersById.reserve(planes.size());
	for(const cplane_t* plane : planes)
	{
		while(plane->id > _CachedPlaneCornersById.size())
		{
			_CachedPlaneCornersById.push_back({});
		}
		if(plane->id == _CachedPlaneCornersById.size())
		{
			_CachedPlaneCornersById.push_back(_CachedPlaneCorners[plane]);
		}
		else
		{
			//Should not happen! Recover anyways
			JPH_ASSERT(_CachedPlaneCornersById[plane->id].empty(), "Unexpected overwrite of a _CachedPlaneCorners item.");
			_CachedPlaneCornersById[plane->id] = _CachedPlaneCorners[plane];
		}
	}

	_CachedPlaneCorners.clear();
#endif
}

EXTERN void COL_GetPlaneCorners(unsigned short planeId, int* outSize, float** outFloat)
{
	if(planeId >= _CachedPlaneCornersById.size())
	{
		(*outSize) = 0;
		return;
	}

	(*outSize) = static_cast<int>(_CachedPlaneCornersById.at(planeId).size());
	(*outFloat) = &_CachedPlaneCornersById.at(planeId).data()->x;
}

EXPORT void COL_Think()
{
	if(!physics_system)
		return;

	physics_system->Update(cDeltaTime, cCollisionSteps, cIntegrationSubSteps, temp_allocator, job_system);

	//Apply simulation to entities
	for (size_t i = 0; i < bsp_col.bodies.size(); ++i)
	{
		Body* body = bsp_col.bodies[i].body;
		if(!body)
		{
		    // Com_LPrintf(PRINT_ERROR, "Body missing body pointer!\n");
			continue;
		}

		edict_t* ent = bsp_col.bodies[i].ent;
		if(!ent)
		{
			continue;
		}

		if(ent->mass == 0)
		{
			continue;
		}

		Mat44 transform = body->GetWorldTransform();
		Mat44 ltransform = body->GetCenterOfMassTransform();

		Quat rotation = transform.GetRotation().GetQuaternion();

		Vec3 vec = transform.GetTranslation();

		ent->s.origin[0] = vec.GetX();
		ent->s.origin[1] = vec.GetY();
		ent->s.origin[2] = vec.GetZ();

		GetEulerSlartibarty(transform, ent->s.angles);

        gi.linkentity(ent);
	}
}

EXPORT void COL_Destroy(bsp_t* bsp)
{
	if(!bsp)
	{
		return;
	}

    Com_LPrintf(PRINT_NOTICE, "Destroying Collision\n");

	//delete constraints
	for (size_t c = bsp_col.constraints.size() - 1; c < bsp_col.constraints.size(); --c)
	{
		physics_system->RemoveConstraint(bsp_col.constraints[c].constraint);
		delete bsp_col.constraints[c].constraint;
		// physics_system->DestroyConstraint(bsp_col.constraints[c].constraint);
	}
	bsp_col.constraints.clear();

	//delete collision shapes
	for (size_t j = bsp_col.bodies.size() - 1; j < bsp_col.bodies.size(); --j)
	{
        Body* body = bsp_col.bodies[j].body;
		interface->RemoveBody(body->GetID());
		interface->DestroyBody(body->GetID());
	}
    bsp_col.bodies.clear();

	physics_system->OptimizeBroadPhase();

#ifdef _DEBUG
	_CachedPlaneCorners.clear();
#endif
}

EXPORT void COL_DoSpawnConstraint(collision_prepared_constraint_t& constraintEnt, collision_body_t& bodyEnt1)
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

	collision_body_t& bodyEnt2 = bsp_col.bodies.front();

	Vec3 frame1(0, 0, 0);
	Vec3 world1 = bodyEnt1.spawnOffset;

	if(!(constraintEnt.ent->spawnflags & 4))
	{
		Vec3 origin(constraintEnt.ent->s.origin[0], constraintEnt.ent->s.origin[1], constraintEnt.ent->s.origin[2]);
		frame1 = origin - world1;
	}

	Vec3 frame2 = bodyEnt2.body->GetCenterOfMassPosition();

	using EAxis = SixDOFConstraintSettings::EAxis;

	SixDOFConstraintSettings constrainSettings;
	constrainSettings.SetEmbedded(); //Tell Jolt we're owning it

	constrainSettings.mSpace = EConstraintSpace::LocalToBodyCOM;
	constrainSettings.mPosition2 = frame1;
	constrainSettings.mPosition1 = frame2;

	//Limits
	for(int i = 0; i < 6; ++i)
	{
		float minv = (i < 3 ? constraintEnt.minaxis : constraintEnt.minangle)[i % 3];
		float maxv = (i < 3 ? constraintEnt.maxaxis : constraintEnt.maxangle)[i % 3];
		if(minv == maxv)
		{
			if(minv == -1.0f)
			{
				constrainSettings.MakeFreeAxis((EAxis)i);
			}
			else
			{
				constrainSettings.MakeFixedAxis((EAxis)i);
			}
		}
		else
		{
			if(i >= 3)
			{
				minv = DEG2RAD(minv);
				maxv = DEG2RAD(maxv);
			}
			constrainSettings.SetLimitedAxis((EAxis)i, minv, maxv);
		}

		constrainSettings.mMotorSettings[i] = MotorSettings(
			constraintEnt.ent->wait, 2.0f, 
			bodyEnt1.body->GetMotionProperties()->GetInverseMass() / constraintEnt.ent->delay, 
			0.0f
		);
	}

	//Motors
	// spawnflags & 1 enable motor
	// spawnflags & 2 start activated
	// style -> axis
	// target velocity -> wait
	// max force -> delay
	if(constraintEnt.ent->spawnflags & 1)
	{
		constraintEnt.ent->style = abs(constraintEnt.ent->style) % 6;
	}

	SixDOFConstraint* constraint = static_cast<SixDOFConstraint *>(constrainSettings.Create(*bodyEnt2.body, *bodyEnt1.body));

	if(constraintEnt.ent->spawnflags & (1 | 2))
	{
		constraint->SetMotorState((EAxis)constraintEnt.ent->style, EMotorState::Velocity);
	}

	physics_system->AddConstraint(constraint);

	bsp_col.constraints.push_back({constraintEnt.ent, constraint});
}

std::vector<collision_prepared_constraint_t> PreparedConstraintSettings;

EXPORT void COL_SpawnEntity(edict_t* ent)
{
	if(!ent->inuse)
	{
		return;
	}

	if(ent->s.modelindex > bsp_col.bodies.size())
	{
        Com_LPrintf(PRINT_ERROR, "Unable to link entity to collision body: modelindex %d out of bounds %zu!\n", ent->s.modelindex, bsp_col.bodies.size());
		return;
	}

	if(ent->mass > 0)
	{
		pre_compound_t& pre_body = bsp_col.prehullgroups[ent->s.modelindex - 1];
		collision_body_t& col_body = bsp_col.bodies[ent->s.modelindex - 1];

		Ref<StaticCompoundShapeSettings> compound = new StaticCompoundShapeSettings();
		for(pre_hull_t& hull : pre_body.hulls)
		{
			RefConst<Shape> shape = ConvexHullShapeSettings(hull.vertices).Create().Get();
			compound->AddShape(Vec3::sZero(), Quat::sIdentity(), shape);
		}

		col_body.body = interface->CreateBody(
			BodyCreationSettings(
				compound, Vec3::sZero(), Quat::sIdentity(), 
				EMotionType::Dynamic, Layers::MOVING
			)
		);
		col_body.ent = ent;


		if (ent->targetname && Q_stricmp(ent->targetname, "storecenter") == 0)
		{
			collision_body_t& col_body = bsp_col.bodies[ent->s.modelindex - 1];

			Vec3 center = col_body.body->GetWorldTransform().GetTranslation();
			ent->s.origin[0] = center.GetX();
			ent->s.origin[1] = center.GetY();
			ent->s.origin[2] = center.GetZ();

			Com_LPrintf(PRINT_DEVELOPER, "Storecenter %d Angles: %.2f, %.2f, %.2f, origin: %.2f, %.2f, %.2f\n", col_body.body->GetID(), ent->s.angles[0], ent->s.angles[1], ent->s.angles[2], ent->s.origin[0], ent->s.origin[1], ent->s.origin[2]);

			interface->DestroyBody(col_body.body->GetID());
			// delete col_body.body;
			col_body.body = nullptr;
			ent->mass = 0;
		}
		else
		{
			interface->AddBody(col_body.body->GetID(), EActivation::Activate);
			ent->movetype = MOVETYPE_BULLETPHYSICS;
		}

	}
	else
	{
		//Kinematic
		Com_LPrintf(PRINT_DEVELOPER, "Potential kinematic: modelidx %d\n", ent->s.modelindex);

	}

	gi.linkentity(ent);
}

EXPORT void COL_LoadComplete()
{
	if(!physics_system)
		return;

	//Create the static bodies who didn't had entities attached to them
	for(size_t i = 0; i < bsp_col.bodies.size(); ++i)
	{
		collision_body_t& col_body = bsp_col.bodies[i];

		if(col_body.body)
			continue;

		pre_compound_t& pre_body = bsp_col.prehullgroups[i];
		Ref<StaticCompoundShapeSettings> compound = new StaticCompoundShapeSettings();
		for(pre_hull_t& hull : pre_body.hulls)
		{
			ConvexHullShapeSettings shapeSettings(hull.vertices);
			ShapeSettings::ShapeResult shapeResult = shapeSettings.Create();
			if(!shapeResult.IsValid())
			{
				Com_LPrintf(PRINT_DEVELOPER, "Skipping invalid shape!\n");
				continue;
			}
			if(shapeResult.IsEmpty())
			{
				Com_LPrintf(PRINT_DEVELOPER, "Skipping empty shape!\n");
				continue;
			}

			RefConst<Shape> shape = shapeResult.Get();
			compound->AddShape(Vec3::sZero(), Quat::sIdentity(), shape);
		}

		if(compound->mSubShapes.size() == 0)
			continue;

		col_body.body = interface->CreateBody(
			BodyCreationSettings(
				compound, Vec3::sZero(), Quat::sIdentity(), 
				EMotionType::Static, Layers::NON_MOVING
			)
		);

		interface->AddBody(col_body.body->GetID(), EActivation::Activate);
	}

	if(bsp_col.bodies.front().body)
	{
		PreparedConstraintSettings.erase(
			std::remove_if(
				PreparedConstraintSettings.begin(), 
				PreparedConstraintSettings.end(), 
				[](collision_prepared_constraint_t& waitingEnt){
					for(collision_body_t& col_body : bsp_col.bodies)
					{
						if(!col_body.ent)
						{
							continue;
						}
						if(!col_body.body)
						{
							continue;
						}
						if(!col_body.ent->targetname)
						{
							continue;
						}
						if (Q_stricmp(col_body.ent->targetname, waitingEnt.ent->target) == 0)
						{
							COL_DoSpawnConstraint(waitingEnt, col_body);
							return true;
						}
					}
					return false;
				}
			),
			PreparedConstraintSettings.end()
		);
	}

	physics_system->OptimizeBroadPhase();

	if(!PreparedConstraintSettings.empty())
	{
		Com_LPrintf(PRINT_ERROR, "Unable to attach all constraints!\n");
#ifdef _DEBUG
		for(auto& missedConstraint : PreparedConstraintSettings)
		{
			Com_LPrintf(PRINT_ERROR, "    No body found with targetname: %s\n", missedConstraint.ent->target);
		}
#endif
	}
}

BodyManager::DrawSettings GetDefaultSettings()
{
	using EShapeColor = BodyManager::EShapeColor;

	return BodyManager::DrawSettings {
		.mDrawGetSupportFunction = false,					///< Draw the GetSupport() function, used for convex collision detection	
		.mDrawSupportDirection = false,						///< When drawing the support function, also draw which direction mapped to a specific support point
		.mDrawGetSupportingFace = false,					///< Draw the faces that were found colliding during collision detection
		.mDrawShape = true,									///< Draw the shapes of all bodies
		.mDrawShapeWireframe = true,						///< When mDrawShape is true and this is true, the shapes will be drawn in wireframe instead of solid.
		.mDrawShapeColor = EShapeColor::MotionTypeColor,	///< Coloring scheme to use for shapes
		.mDrawBoundingBox = false,							///< Draw a bounding box per body
		.mDrawCenterOfMassTransform = false,				///< Draw the center of mass for each body
		.mDrawWorldTransform = false,						///< Draw the world transform (which can be different than the center of mass) for each body
		.mDrawVelocity = false,								///< Draw the velocity vector for each body
		.mDrawMassAndInertia = false,						///< Draw the mass and inertia (as the box equivalent) for each body
		.mDrawSleepStats = false,							///< Draw stats regarding the sleeping algorithm of each body
	};
}

unsigned short DebugVisualizePlaneId = 0;

EXPORT void COL_DebugDraw()
{
	if(debug_renderer)
	{
		static BodyManager::DrawSettings drawSettings = GetDefaultSettings();
		
		physics_system->DrawBodies(drawSettings, debug_renderer);
		physics_system->DrawConstraints(debug_renderer);
		physics_system->DrawConstraintLimits(debug_renderer);
		physics_system->DrawConstraintReferenceFrame(debug_renderer);

		debug_renderer->FlushDraw();
	}

	if(DebugVisualizePlaneId)
	{
        int vCount = 0;
        float* v = NULL;

        COL_GetPlaneCorners(DebugVisualizePlaneId, &vCount, &v);
        if(vCount)
        {
            float color[4] = {1.0f, 0.0f, 0.0f, 1.0f};
            gi.R_DrawLines(v, vCount, NULL, 0, color);
        }
	}
}

EXPORT void COL_SpawnConstraint(edict_t* ent)
{
	if(!ent->target)
	{
		return;
	}

	collision_prepared_constraint_t preparedConstraint {
		ent, 
		{st.minaxis[0], st.minaxis[1], st.minaxis[2]}, 
		{st.maxaxis[0], st.maxaxis[1], st.maxaxis[2]}, 
		{st.minangle[0], st.minangle[1], st.minangle[2]}, 
		{st.maxangle[0], st.maxangle[1], st.maxangle[2]}
	};

	PreparedConstraintSettings.push_back(preparedConstraint);
}

EXPORT void COL_DebugVisualizePlane(unsigned short planeId)
{
	DebugVisualizePlaneId = planeId;
}

EXPORT void COL_Trace()
{

}

EXPORT void COL_BakeEntityStates()
{

}