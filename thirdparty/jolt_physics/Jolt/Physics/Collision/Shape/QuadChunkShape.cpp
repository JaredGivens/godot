#include "QuadChunkShape.h"
#include "modules/shardscape/QuadChunk16.h"
#include "modules/shardscape/vector3u4.h"

#include <Jolt/Core/Profiler.h>
#include <Jolt/Core/StreamIn.h>
#include <Jolt/Core/StreamOut.h>
#include <Jolt/Geometry/OrientedBox.h>
#include <Jolt/Geometry/RayAABox.h>
#include <Jolt/Geometry/RayTriangle.h>
#include <Jolt/Math/UVec4.h>
#include <Jolt/Physics/Collision/CastConvexVsTriangles.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CastSphereVsTriangles.h>
#include <Jolt/Physics/Collision/CollideConvexVsTriangles.h>
#include <Jolt/Physics/Collision/CollideSoftBodyVerticesVsTriangles.h>
#include <Jolt/Physics/Collision/CollideSphereVsTriangles.h>
#include <Jolt/Physics/Collision/CollisionDispatch.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/Shape/ConvexShape.h>
#include <Jolt/Physics/Collision/Shape/ScaleHelpers.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/ShapeFilter.h>
#include <Jolt/Physics/Collision/TransformedShape.h>
#include <cfloat>
#include <cstdint>
#include <cstring>

JPH_NAMESPACE_BEGIN

using QC = SSK::QuadChunk16;

static constexpr float kDensity = 1000.0f;
static constexpr int32_t kOwnerMax = QC::kSize1D - 2;

class FaceCursor {
public:
	FaceCursor() = default;

	explicit FaceCursor(const AABox &inLocalBox) {
		Vec3 lo = Vec3::sClamp(inLocalBox.mMin, Vec3::sReplicate(-2.0f), Vec3::sReplicate(QC::kSize1D + 1.0f));
		Vec3 hi = Vec3::sClamp(inLocalBox.mMax, Vec3::sReplicate(-2.0f), Vec3::sReplicate(QC::kSize1D + 1.0f));
		for (int32_t i = 0; i < 3; ++i) {
			mLo[i] = MAX((int32_t)floorf(lo[i]) - 1, 0);
			mHi[i] = MIN((int32_t)floorf(hi[i]), kOwnerMax);
			if (mLo[i] > mHi[i]) {
				return;
			}
		}
		mLaneMask = ((1u << (2 * mHi[0] + 2)) - 1) & ~((1u << (2 * mLo[0])) - 1);
		mY = mLo[1];
		mZ = mLo[2];
		mAxis = -1;
		mDone = false;
	}

	JPH_INLINE bool IsDone() const { return mDone; }

	template <class Visitor>
	void Walk(const QC *inQuadChunk, Visitor &ioVisitor) {
		if (mAxis < 0) {
			mAxis = 0;
			mBits = inQuadChunk->face_row(0, mY, mZ) & mLaneMask;
		}
		while (!mDone) {
			while (mBits != 0) {
				if (ioVisitor.ShouldAbort()) {
					return;
				}
				uint32_t cell = (uint32_t)(__builtin_ctz(mBits) >> 1) | (uint32_t)mY << 4 | (uint32_t)mZ << 8;
				mBits &= mBits - 1;
				ioVisitor.VisitTriangle(cell, mAxis * 2);
				ioVisitor.VisitTriangle(cell, mAxis * 2 + 1);
			}
			if (++mY > mHi[1]) {
				mY = mLo[1];
				if (++mZ > mHi[2]) {
					mZ = mLo[2];
					if (++mAxis == 3) {
						mDone = true;
						return;
					}
				}
			}
			mBits = inQuadChunk->face_row(mAxis, mY, mZ) & mLaneMask;
		}
	}

private:
	int32_t mLo[3] = {};
	int32_t mHi[3] = {};
	int32_t mY = 0;
	int32_t mZ = 0;
	int32_t mAxis = 0;
	uint32_t mBits = 0;
	uint32_t mLaneMask = 0;
	bool mDone = true;
};

template <class Visitor>
static JPH_INLINE void WalkBox(const QC *inQuadChunk, const AABox &inLocalBox, Visitor &ioVisitor) {
	JPH_PROFILE_FUNCTION();
	RWLockRead guard(inQuadChunk->lock_);
	FaceCursor cursor(inLocalBox);
	cursor.Walk(inQuadChunk, ioVisitor);
}

static JPH_INLINE AABox sChunkBox() {
	return AABox(Vec3::sZero(), Vec3::sReplicate((float)QC::kSize1D));
}

ShapeSettings::ShapeResult QuadChunkShapeSettings::Create() const {
	if (mCachedResult.IsEmpty()) {
		Ref<Shape> shape = new QuadChunkShape(*this, mCachedResult);
	}
	return mCachedResult;
}

QuadChunkShape::QuadChunkShape(const QuadChunkShapeSettings &inSettings, ShapeResult &outResult) :
		Shape(EShapeType::User3, EShapeSubType::User3, inSettings, outResult),
		mQuadChunk(inSettings.mQuadChunk) {
	if (inSettings.mQuadChunk == nullptr) {
		outResult.SetError("Invalid faces");
		return;
	}
	outResult.Set(this);
}

MassProperties QuadChunkShape::GetMassProperties() const {
	RWLockRead guard(mQuadChunk->lock_);
	Vector3i lo;
	Vector3i hi;
	int32_t count = mQuadChunk->occupied_bounds(lo, hi);
	MassProperties p;
	p.SetMassAndInertiaOfSolidBox(Vec3(float(hi.x - lo.x + 1), float(hi.y - lo.y + 1), float(hi.z - lo.z + 1)), kDensity);
	p.ScaleToMass(float(MAX(count, 1)) * kDensity);
	return p;
}

const PhysicsMaterial *QuadChunkShape::GetMaterial(const SubShapeID &inSubShapeID) const {
	return JPH::PhysicsMaterial::sDefault;
}

SubShapeID QuadChunkShape::EncodeSubShapeID(const SubShapeIDCreator &inCreator, uint32_t inHex, uint32_t inTriangle) const {
	return inCreator.PushID(inHex | inTriangle << 12, cSubShapeIDBits).GetID();
}

int QuadChunkShape::DecodeTriangle(uint32_t inHexi, uint32_t inTrii, Vec3 outVerts[3]) const {
	float floats[9];
	if (mQuadChunk->get_tri(inHexi, inTrii, floats)) {
		return 1;
	}
	for (int i = 0; i < 3; ++i) {
		int vi = i * 3;
		outVerts[i] = Vec3(floats[vi], floats[vi + 1], floats[vi + 2]);
	}
	return 0;
}

void QuadChunkShape::DecodeSubShapeID(const SubShapeID &inSubShapeID, uint32_t &outHex, uint32_t &outTriangle) const {
	SubShapeID remainder;
	uint32_t id = inSubShapeID.PopID(cSubShapeIDBits, remainder);
	JPH_ASSERT(remainder.IsEmpty(), "Invalid subshape ID");

	outTriangle = id >> 12;
	outHex = id & ((1 << 12) - 1);
}

Vec3 QuadChunkShape::GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const {
	uint32_t face;
	uint32_t triangle_idx;
	DecodeSubShapeID(inSubShapeID, face, triangle_idx);

	Vec3 verts[3];
	RWLockRead guard(mQuadChunk->lock_);
	if (DecodeTriangle(face, triangle_idx, verts)) {
		return Vec3::sAxisY();
	}

	return (verts[0] - verts[1]).Cross(verts[2] - verts[1]).Normalized();
}

void QuadChunkShape::GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const {
	uint32_t face;
	uint32_t triangle_idx;
	DecodeSubShapeID(inSubShapeID, face, triangle_idx);

	outVertices.resize(3);
	{
		RWLockRead guard(mQuadChunk->lock_);
		if (DecodeTriangle(face, triangle_idx, &outVertices[0])) {
			return;
		}
	}

	if (ScaleHelpers::IsInsideOut(inScale)) {
		std::swap(outVertices[1], outVertices[2]);
	}

	Mat44 transform = inCenterOfMassTransform.PreScaled(inScale);

	for (Vec3 &v : outVertices) {
		v = transform * v;
	}
}

#ifdef JPH_DEBUG_RENDERER
void QuadChunkShape::Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const {
	return;
}
#endif

template <typename Visit>
static JPH_INLINE void VisitCellOwners(const QC *inQuadChunk, uint32_t inCell, uint32_t *ioVisited, Visit &inVisit) {
	int32_t c[3] = { (int32_t)(inCell & 15), (int32_t)((inCell >> 4) & 15), (int32_t)((inCell >> 8) & 15) };
	for (int32_t a = 0; a < 3; ++a) {
		int32_t u = (a + 1) % 3;
		int32_t w = (a + 2) % 3;
		for (int32_t d = 0; d < 4; ++d) {
			int32_t b[3] = { c[0], c[1], c[2] };
			b[u] -= d & 1;
			b[w] -= d >> 1;
			if (b[0] < 0 || b[1] < 0 || b[2] < 0 || b[0] > kOwnerMax || b[1] > kOwnerMax || b[2] > kOwnerMax) {
				continue;
			}
			if (((inQuadChunk->face_row(a, b[1], b[2]) >> (b[0] * 2)) & 1) == 0) {
				continue;
			}
			uint32_t owner = (uint32_t)(b[0] | b[1] << 4 | b[2] << 8);
			uint32_t bit = owner * 3 + (uint32_t)a;
			if ((ioVisited[bit >> 5] >> (bit & 31)) & 1) {
				continue;
			}
			ioVisited[bit >> 5] |= 1u << (bit & 31);
			inVisit(owner, (uint32_t)a * 2);
			inVisit(owner, (uint32_t)a * 2 + 1);
		}
	}
}

template <typename OnHit>
static void CastRayT(const QuadChunkShape *inShape, Vec3Arg inOrigin, Vec3Arg inDirection, EBackFaceMode inBackFaceMode, OnHit &&inOnHit) {
	JPH_PROFILE_FUNCTION();
	const QC *qc = inShape->mQuadChunk;
	RWLockRead guard(qc->lock_);
	auto minBounds = Vec3::sReplicate(0);
	auto maxBounds = Vec3::sReplicate(QC::kSize1D - 0.01);
	auto rayInvDir = RayInvDirection(inDirection);
	float distance = RayAABox(inOrigin, rayInvDir, minBounds, maxBounds);
	if (distance == FLT_MAX) {
		return;
	}

	float lanes[9][4] = {};
	uint32_t ids[4] = {};
	int32_t count = 0;
	auto flush = [&]() {
		Vec4 fractions = RayTriangle4(inOrigin, inDirection,
				Vec4(lanes[0][0], lanes[0][1], lanes[0][2], lanes[0][3]),
				Vec4(lanes[1][0], lanes[1][1], lanes[1][2], lanes[1][3]),
				Vec4(lanes[2][0], lanes[2][1], lanes[2][2], lanes[2][3]),
				Vec4(lanes[3][0], lanes[3][1], lanes[3][2], lanes[3][3]),
				Vec4(lanes[4][0], lanes[4][1], lanes[4][2], lanes[4][3]),
				Vec4(lanes[5][0], lanes[5][1], lanes[5][2], lanes[5][3]),
				Vec4(lanes[6][0], lanes[6][1], lanes[6][2], lanes[6][3]),
				Vec4(lanes[7][0], lanes[7][1], lanes[7][2], lanes[7][3]),
				Vec4(lanes[8][0], lanes[8][1], lanes[8][2], lanes[8][3]));
		for (int32_t i = 0; i < count; ++i) {
			if (fractions[i] != FLT_MAX) {
				inOnHit(fractions[i], ids[i] & 0xFFF, ids[i] >> 12);
			}
		}
		memset(lanes, 0, sizeof(lanes));
		count = 0;
	};
	auto visit = [&](uint32_t inCell, uint32_t inTri) {
		Vec3 v[3];
		if (inShape->DecodeTriangle(inCell, inTri, v)) {
			return;
		}
		if (inBackFaceMode == EBackFaceMode::IgnoreBackFaces && (v[2] - v[0]).Cross(v[1] - v[0]).Dot(inDirection) < 0) {
			return;
		}
		for (int32_t k = 0; k < 3; ++k) {
			lanes[k * 3 + 0][count] = v[k].GetX();
			lanes[k * 3 + 1][count] = v[k].GetY();
			lanes[k * 3 + 2][count] = v[k].GetZ();
		}
		ids[count++] = inCell | inTri << 12;
		if (count == 4) {
			flush();
		}
	};

	uint32_t visited[QC::kSize3D * 3 / 32] = {};

	auto end = inOrigin + inDirection;
	auto end_v3u4 = Vector3u4(floor(end.GetX()), floor(end.GetY()), floor(end.GetZ()));
	Vec3 startPos = inOrigin;
	if (distance > 0) {
		startPos += inDirection * distance;
	}

	UVec4 startPosU = startPos.ToInt();
	Vector3u4 xyz(startPosU.GetX(), startPosU.GetY(), startPosU.GetZ());

	Vec3 step = inDirection.GetSign();
	constexpr float LARGE = float(1 << 16);
	auto tDelta = UVec4::sSelect((LARGE * rayInvDir.mInvDirection.Abs()).ToInt(), UVec4::sReplicate(LARGE), rayInvDir.mIsParallel);
	Vec3 startPosF = (startPos - Vec3(startPosU.ToFloat())) * -step + Vec3::sMax(step, Vec3::sZero());
	auto tMax = UVec4::sSelect((startPosF * Vec3(tDelta.ToFloat())).ToInt(), UVec4::sReplicate(LARGE), rayInvDir.mIsParallel);

	while (!xyz.is_invalid()) {
		VisitCellOwners(qc, xyz.packed, visited, visit);
		auto ax = tMax[0] < tMax[1] && tMax[0] < tMax[2] ? 0 : (tMax[1] < tMax[2] ? 1 : 2);
		if (end_v3u4.packed == xyz.packed) {
			break;
		}
		xyz.try_add(ax, step[ax]);
		tMax[ax] += tDelta[ax];
	}
	if (count != 0) {
		flush();
	}
}

bool QuadChunkShape::CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const {
	JPH_PROFILE_FUNCTION();
	bool hit = false;
	CastRayT(this, inRay.mOrigin, inRay.mDirection, EBackFaceMode::IgnoreBackFaces, [&](float inFraction, uint32_t inCell, uint32_t inTri) {
		if (inFraction < ioHit.mFraction) {
			ioHit.mFraction = inFraction;
			ioHit.mSubShapeID2 = EncodeSubShapeID(inSubShapeIDCreator, inCell, inTri);
			hit = true;
		}
	});
	return hit;
}

void QuadChunkShape::CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter) const {
	JPH_PROFILE_FUNCTION();

	if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID())) {
		return;
	}

	CastRayT(this, inRay.mOrigin, inRay.mDirection, inRayCastSettings.mBackFaceModeTriangles, [&](float inFraction, uint32_t inCell, uint32_t inTri) {
		if (inFraction < ioCollector.GetEarlyOutFraction()) {
			RayCastResult hit;
			hit.mBodyID = TransformedShape::sGetBodyID(ioCollector.GetContext());
			hit.mFraction = inFraction;
			hit.mSubShapeID2 = EncodeSubShapeID(inSubShapeIDCreator, inCell, inTri);
			ioCollector.AddHit(hit);
		}
	});
}

void QuadChunkShape::CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter) const {
	sCollidePointUsingRayCast(*this, inPoint, inSubShapeIDCreator, ioCollector, inShapeFilter);
}

void QuadChunkShape::CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint32_t inNumVertices, int32_t inCollidingShapeIndex) const {
	JPH_PROFILE_FUNCTION();

	struct Visitor : public CollideSoftBodyVerticesVsTriangles {
		JPH_INLINE explicit Visitor(const QuadChunkShape *inShape, Mat44Arg inCOM, Vec3Arg inScale) :
				CollideSoftBodyVerticesVsTriangles(inCOM, inScale), mShape(inShape) {}

		JPH_INLINE bool ShouldAbort() const {
			return false;
		}

		JPH_INLINE void VisitTriangle(uint32_t inHex, uint32_t inTriangle) {
			Vec3 verts[3];
			if (mShape->DecodeTriangle(inHex, inTriangle, verts)) {
				return;
			}
			ProcessTriangle(verts[0], verts[1], verts[2]);
		}

		const QuadChunkShape *mShape;
	};

	Visitor visitor(this, inCenterOfMassTransform, inScale);

	for (CollideSoftBodyVertexIterator v = inVertices, sbv_end = inVertices + inNumVertices; v != sbv_end; ++v) {
		if (v.GetInvMass() > 0.0f) {
			visitor.StartVertex(v);
			WalkBox(mQuadChunk, sChunkBox(), visitor);
			visitor.FinishVertex(v, inCollidingShapeIndex);
		}
	}
}

struct QuadChunkShape::QSGetTrianglesContext {
	JPH_INLINE QSGetTrianglesContext(const QuadChunkShape *inShape, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) :
			mShape(inShape),
			mLocalToWorld(Mat44::sRotationTranslation(inRotation, inPositionCOM) * Mat44::sScale(inScale)) {
		OrientedBox local_box(Mat44::sInverseRotationTranslation(inRotation, inPositionCOM), inBox);
		AABox local_aabb = AABox(-local_box.mHalfExtents, local_box.mHalfExtents).Transformed(local_box.mOrientation);
		mCursor = FaceCursor(local_aabb.Scaled(inScale.Reciprocal()));
	}

	JPH_INLINE bool ShouldAbort() const {
		return mNumTrianglesFound + 2 > mMaxTrianglesRequested;
	}

	JPH_INLINE void VisitTriangle(uint32_t inHex, uint32_t inTriangle) {
		Vec3 verts[3];
		if (mShape->DecodeTriangle(inHex, inTriangle, verts)) {
			return;
		}
		(mLocalToWorld * verts[0]).StoreFloat3(mTriangleVertices++);
		(mLocalToWorld * verts[1]).StoreFloat3(mTriangleVertices++);
		(mLocalToWorld * verts[2]).StoreFloat3(mTriangleVertices++);
		if (mMaterials != nullptr) {
			*mMaterials++ = PhysicsMaterial::sDefault;
		}
		mNumTrianglesFound++;
	}

	FaceCursor mCursor;
	const QuadChunkShape *mShape;
	Mat44 mLocalToWorld;
	int32_t mMaxTrianglesRequested = 0;
	Float3 *mTriangleVertices = nullptr;
	int32_t mNumTrianglesFound = 0;
	const PhysicsMaterial **mMaterials = nullptr;
};

void QuadChunkShape::GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const {
	static_assert(sizeof(QSGetTrianglesContext) <= sizeof(GetTrianglesContext), "GetTrianglesContext too small");
	JPH_ASSERT(IsAligned(&ioContext, alignof(QSGetTrianglesContext)));
	new (&ioContext) QSGetTrianglesContext(this, inBox, inPositionCOM, inRotation, inScale);
}

int32_t QuadChunkShape::GetTrianglesNext(GetTrianglesContext &ioContext, int32_t inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials) const {
	RWLockRead guard(mQuadChunk->lock_);
	JPH_ASSERT(inMaxTrianglesRequested >= cGetTrianglesMinTrianglesRequested);

	QSGetTrianglesContext &context = (QSGetTrianglesContext &)ioContext;
	if (context.mCursor.IsDone()) {
		return 0;
	}

	context.mMaxTrianglesRequested = inMaxTrianglesRequested;
	context.mTriangleVertices = outTriangleVertices;
	context.mMaterials = outMaterials;
	context.mNumTrianglesFound = 0;

	context.mCursor.Walk(mQuadChunk, context);
	return context.mNumTrianglesFound;
}

void QuadChunkShape::SaveBinaryState(StreamOut &inStream) const {
}

void QuadChunkShape::RestoreBinaryState(StreamIn &inStream) {
}

void QuadChunkShape::RestoreMaterialState(const PhysicsMaterialRefC *inMaterials, uint32_t inNumMaterials) {}

Shape::Stats QuadChunkShape::GetStats() const {
	return Stats(sizeof(*this), mQuadChunk->count_ * 2);
}

uint8 QuadChunkShape::GetEdgeFlags(uint32_t inHex, uint32_t inTriangle) const {
	return 0b111;
}

void QuadChunkShape::sCollideConvexVsQuadChunk(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter) {
	JPH_PROFILE_FUNCTION();

	JPH_ASSERT(inShape1->GetType() == EShapeType::Convex);
	JPH_ASSERT(inShape2->GetType() == EShapeType::User3);
	const ConvexShape *shape1 = static_cast<const ConvexShape *>(inShape1);
	const QuadChunkShape *shape2 = static_cast<const QuadChunkShape *>(inShape2);

	struct Visitor : public CollideConvexVsTriangles {
		using CollideConvexVsTriangles::CollideConvexVsTriangles;

		JPH_INLINE AABox LocalBox() const {
			return mBoundsOf1InSpaceOf2;
		}

		JPH_INLINE bool ShouldAbort() const {
			return mCollector.ShouldEarlyOut();
		}

		JPH_INLINE void VisitTriangle(uint32_t inHex, uint32_t inTriangle) {
			Vec3 verts[3];
			if (mShape->DecodeTriangle(inHex, inTriangle, verts)) {
				return;
			}
			Collide(verts[0], verts[1], verts[2], mShape->GetEdgeFlags(inHex, inTriangle), mShape->EncodeSubShapeID(mSubShapeIDCreator2, inHex, inTriangle));
		}

		const QuadChunkShape *mShape;
		SubShapeIDCreator mSubShapeIDCreator2;
	};

	Visitor visitor(shape1, inScale1, inScale2, inCenterOfMassTransform1, inCenterOfMassTransform2, inSubShapeIDCreator1.GetID(), inCollideShapeSettings, ioCollector);
	visitor.mShape = shape2;
	visitor.mSubShapeIDCreator2 = inSubShapeIDCreator2;
	WalkBox(shape2->mQuadChunk, visitor.LocalBox().Scaled(inScale2.Reciprocal()), visitor);
}

void QuadChunkShape::sCollideSphereVsQuadChunk(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter) {
	JPH_PROFILE_FUNCTION();

	JPH_ASSERT(inShape1->GetSubType() == EShapeSubType::Sphere);
	JPH_ASSERT(inShape2->GetType() == EShapeType::User3);
	const SphereShape *shape1 = static_cast<const SphereShape *>(inShape1);
	const QuadChunkShape *shape2 = static_cast<const QuadChunkShape *>(inShape2);

	struct Visitor : public CollideSphereVsTriangles {
		using CollideSphereVsTriangles::CollideSphereVsTriangles;

		JPH_INLINE AABox LocalBox() const {
			Vec3 reach = Vec3::sReplicate(sqrtf(mRadiusPlusMaxSeparationSq));
			return AABox(mSphereCenterIn2 - reach, mSphereCenterIn2 + reach);
		}

		JPH_INLINE bool ShouldAbort() const {
			return mCollector.ShouldEarlyOut();
		}

		JPH_INLINE void VisitTriangle(uint32_t inHex, uint32_t inTriangle) {
			Vec3 verts[3];
			if (mShape->DecodeTriangle(inHex, inTriangle, verts)) {
				return;
			}
			Collide(verts[0], verts[1], verts[2], mShape->GetEdgeFlags(inHex, inTriangle), mShape->EncodeSubShapeID(mSubShapeIDCreator2, inHex, inTriangle));
		}

		const QuadChunkShape *mShape;
		SubShapeIDCreator mSubShapeIDCreator2;
	};

	Visitor visitor(shape1, inScale1, inScale2, inCenterOfMassTransform1, inCenterOfMassTransform2, inSubShapeIDCreator1.GetID(), inCollideShapeSettings, ioCollector);
	visitor.mShape = shape2;
	visitor.mSubShapeIDCreator2 = inSubShapeIDCreator2;
	WalkBox(shape2->mQuadChunk, visitor.LocalBox().Scaled(inScale2.Reciprocal()), visitor);
}

void QuadChunkShape::sCastConvexVsQuadChunk(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector) {
	JPH_PROFILE_FUNCTION();

	JPH_ASSERT(inShape->GetSubType() == EShapeSubType::User3);
	const QuadChunkShape *shape = static_cast<const QuadChunkShape *>(inShape);

	struct Visitor : public CastConvexVsTriangles {
		using CastConvexVsTriangles::CastConvexVsTriangles;

		JPH_INLINE bool ShouldAbort() const {
			return mCollector.ShouldEarlyOut();
		}

		JPH_INLINE void VisitTriangle(uint32_t inHex, uint32_t inTriangle) {
			Vec3 verts[3];
			if (mShape->DecodeTriangle(inHex, inTriangle, verts)) {
				return;
			}
			Cast(verts[0], verts[1], verts[2], mShape->GetEdgeFlags(inHex, inTriangle), mShape->EncodeSubShapeID(mSubShapeIDCreator2, inHex, inTriangle));
		}

		const QuadChunkShape *mShape;
		SubShapeIDCreator mSubShapeIDCreator2;
	};

	Visitor visitor(inShapeCast, inShapeCastSettings, inScale, inCenterOfMassTransform2, inSubShapeIDCreator1, ioCollector);
	visitor.mShape = shape;
	visitor.mSubShapeIDCreator2 = inSubShapeIDCreator2;
	AABox swept = inShapeCast.mShapeWorldBounds;
	swept.Encapsulate(AABox(swept.mMin + inShapeCast.mDirection, swept.mMax + inShapeCast.mDirection));
	WalkBox(shape->mQuadChunk, swept.Scaled(inScale.Reciprocal()), visitor);
}

void QuadChunkShape::sCastSphereVsQuadChunk(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector) {
	JPH_PROFILE_FUNCTION();

	JPH_ASSERT(inShape->GetSubType() == EShapeSubType::User3);
	const QuadChunkShape *shape = static_cast<const QuadChunkShape *>(inShape);

	struct Visitor : public CastSphereVsTriangles {
		using CastSphereVsTriangles::CastSphereVsTriangles;

		JPH_INLINE AABox LocalBox() const {
			Vec3 r = Vec3::sReplicate(mRadius);
			AABox swept(mStart - r, mStart + r);
			swept.Encapsulate(AABox(mStart + mDirection - r, mStart + mDirection + r));
			return swept;
		}

		JPH_INLINE bool ShouldAbort() const {
			return mCollector.ShouldEarlyOut();
		}

		JPH_INLINE void VisitTriangle(uint32_t inHex, uint32_t inTriangle) {
			Vec3 verts[3];
			if (mShape->DecodeTriangle(inHex, inTriangle, verts)) {
				return;
			}
			Cast(verts[0], verts[1], verts[2], mShape->GetEdgeFlags(inHex, inTriangle), mShape->EncodeSubShapeID(mSubShapeIDCreator2, inHex, inTriangle));
		}

		const QuadChunkShape *mShape;
		SubShapeIDCreator mSubShapeIDCreator2;
	};

	Visitor visitor(inShapeCast, inShapeCastSettings, inScale, inCenterOfMassTransform2, inSubShapeIDCreator1, ioCollector);
	visitor.mShape = shape;
	visitor.mSubShapeIDCreator2 = inSubShapeIDCreator2;
	WalkBox(shape->mQuadChunk, visitor.LocalBox().Scaled(inScale.Reciprocal()), visitor);
}

void QuadChunkShape::sRegister() {
	ShapeFunctions &f = ShapeFunctions::sGet(EShapeSubType::Mesh);
	f.mConstruct = []() -> Shape * { return new QuadChunkShape; };
	f.mColor = Color::sRed;

	for (EShapeSubType s : sConvexSubShapeTypes) {
		CollisionDispatch::sRegisterCollideShape(s, EShapeSubType::User3, sCollideConvexVsQuadChunk);
		CollisionDispatch::sRegisterCastShape(s, EShapeSubType::User3, sCastConvexVsQuadChunk);

		CollisionDispatch::sRegisterCastShape(EShapeSubType::User3, s, CollisionDispatch::sReversedCastShape);
		CollisionDispatch::sRegisterCollideShape(EShapeSubType::User3, s, CollisionDispatch::sReversedCollideShape);
	}

	CollisionDispatch::sRegisterCollideShape(EShapeSubType::Sphere, EShapeSubType::User3, sCollideSphereVsQuadChunk);
	CollisionDispatch::sRegisterCastShape(EShapeSubType::Sphere, EShapeSubType::User3, sCastSphereVsQuadChunk);
}

JPH_NAMESPACE_END
