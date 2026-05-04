#include "QuadChunkShape.h"
#include "modules/shardscape/QuadChunk16.h"
#include "modules/shardscape/vector3u4.h"

#include <Jolt/AABBTree/AABBTreeBuilder.h>
#include <Jolt/AABBTree/AABBTreeToBuffer.h>
#include <Jolt/AABBTree/NodeCodec/NodeCodecQuadTreeHalfFloat.h>
#include <Jolt/AABBTree/TriangleCodec/TriangleCodecIndexed8BitPackSOA4Flags.h>
#include <Jolt/Core/Profiler.h>
#include <Jolt/Core/StreamIn.h>
#include <Jolt/Core/StreamOut.h>
#include <Jolt/Core/StringTools.h>
#include <Jolt/Core/UnorderedMap.h>
#include <Jolt/Geometry/AABox4.h>
#include <Jolt/Geometry/Indexify.h>
#include <Jolt/Geometry/OrientedBox.h>
#include <Jolt/Geometry/Plane.h>
#include <Jolt/Geometry/RayAABox.h>
#include <Jolt/Math/UVec4.h>
#include <Jolt/ObjectStream/TypeDeclarations.h>
#include <Jolt/Physics/Collision/ActiveEdges.h>
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
#include <Jolt/TriangleSplitter/TriangleSplitterBinning.h>
#include <cfloat>
#include <climits>
#include <cstdint>

JPH_NAMESPACE_BEGIN

static constexpr uint32_t cStackSize = 256; // Stack size for blocks
using QC = SSK::QuadChunk16;

/// This function will sort values from high to low and only keep the ones that are less than inMaxValue
/// @param inValues Values to be sorted
/// @param inMaxValue Values need to be less than this to keep them
/// @param ioIdentifiers 8 identifiers that will be sorted in the same way as the values
/// @param outValues The values are stored here from high to low
/// @return The number of values that were kept
JPH_INLINE int32_t SortReverseAndStore8(float inValues[8], float inMaxValue, uint32_t ioIdentifiers[8], float *outValues)
{
	// Sort so that highest values are first (we want to first process closer hits and we process stack top to bottom)
	for (int32_t i = 0; i < 7; ++i) {
		int32_t maxIndex = i;
		for (int32_t j = i + 1; j < 8; ++j) {
			if (inValues[j] > inValues[maxIndex]) {
				maxIndex = j;
			}
		}
		// swap
		float ftemp = inValues[i];
		inValues[i] = inValues[maxIndex];
		inValues[maxIndex] = ftemp;
		int32_t itemp = ioIdentifiers[i];
		ioIdentifiers[i] = ioIdentifiers[maxIndex];
		ioIdentifiers[maxIndex] = itemp;

	}

	// Keep only results < inMaxValue
	int32_t num_results = 0;
	for (int32_t i = 0; i < 8; ++i) {
		if (inValues[i] < inMaxValue) {
			outValues[num_results] = inValues[i];
			ioIdentifiers[num_results] = ioIdentifiers[i];
			++num_results;
		}
	}

	return num_results;
}

/// Shift the elements so that the identifiers that correspond with the trues in inValue come first
/// @param inValue Values to test for true or false
/// @param ioIdentifiers the identifiers that are shifted, on return they are shifted
/// @return The number of trues
 JPH_INLINE int CountAndSortTrues8(uint32_t inValues[8], uint32_t ioIdentifiers[8])
{
    uint32_t tmp_ids[8];
    int n_true = 0, n_false = 0;

    // First pass: stable partition into tmp_ids (trues at front)
    for (int i = 0; i < 8; ++i)
    {
        if (inValues[i])
            tmp_ids[n_true++] = ioIdentifiers[i];
        else
            tmp_ids[7 - (n_false++)] = ioIdentifiers[i]; // store falses from the back
    }

    // Write back: trues first, then falses (contiguous)
    for (int i = 0; i < n_true; ++i) { ioIdentifiers[i] = tmp_ids[i]; inValues[i] = 1; }
    for (int i = 0; i < 8 - n_true; ++i) { ioIdentifiers[n_true + i] = tmp_ids[7 - i]; inValues[n_true + i] = 0; }

    return n_true;
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
	// We cannot calculate the volume for an arbitrary mesh, so we return invalid mass properties.
	// If you want your mesh to be dynamic, then you should provide the mass properties yourself when
	// creating a Body:
	//
	// BodyCreationSettings::mOverrideMassProperties = EOverrideMassProperties::MassAndInertiaProvided;
	// BodyCreationSettings::mMassPropertiesOverride.SetMassAndInertiaOfSolidBox(Vec3::sReplicate(1.0f), 1000.0f);
	//
	// Note that for a mesh shape to simulate properly, it is best if the mesh is manifold
	// (i.e. closed, all edges shared by only two triangles, consistent winding order).
	return MassProperties();
}
const PhysicsMaterial *QuadChunkShape::GetMaterial(const SubShapeID &inSubShapeID) const {
	return JPH::PhysicsMaterial::sDefault;
}

SubShapeID QuadChunkShape::EncodeSubShapeID(const SubShapeIDCreator &inCreator, uint32_t inHex, uint32_t inTriangle) const {
	return inCreator.PushID(inHex | inTriangle << 12, cSubShapeIDBits).GetID();
}
// returns 0 on success and 1 on fail
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
	// Get block
	SubShapeID remainder;
	uint32_t id = inSubShapeID.PopID(cSubShapeIDBits, remainder);
	JPH_ASSERT(remainder.IsEmpty(), "Invalid subshape ID");

	outTriangle = id >> 12;
	outHex = id & ((1 << 12) - 1);
}

Vec3 QuadChunkShape::GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const {
	// Decode ID
	uint32_t face;
	uint32_t triangle_idx;
	DecodeSubShapeID(inSubShapeID, face, triangle_idx);

	// Decode triangle
	Vec3 verts[3];
	if (DecodeTriangle(face, triangle_idx, verts)) {
		return Vec3::sAxisY();
	}

	// Calculate normal
	return (verts[2] - verts[1]).Cross(verts[0] - verts[1]).Normalized();
}

void QuadChunkShape::GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const {
	// Decode ID
	uint32_t face;
	uint32_t triangle_idx;
	DecodeSubShapeID(inSubShapeID, face, triangle_idx);

	// Decode triangle
	outVertices.resize(3);
	if (DecodeTriangle(face, triangle_idx, &outVertices[0])) {
		return;
	}

	// Flip triangle if scaled inside out
	if (ScaleHelpers::IsInsideOut(inScale)) {
		std::swap(outVertices[1], outVertices[2]);
	}

	// Calculate transform with scale
	Mat44 transform = inCenterOfMassTransform.PreScaled(inScale);

	// Transform to world space
	for (Vec3 &v : outVertices) {
		v = transform * v;
	}
}

template <typename Visitor>
void VisitStarT(Visitor &ioVisitor, int32_t xyz) {
	for (int32_t i = 0; i < 12; ++i) {
		auto ax = i >> 2;
		auto v3u4 = Vector3u4(xyz);
		v3u4.try_add(~ax & 1, -((~i >> 0) & 1));
		v3u4.try_add(~ax & 2, -((~i >> 1) & 1));
		if (v3u4.is_invalid())
			continue;
		ioVisitor.VisitTriangle(v3u4.packed, ax * 2);
		if (ioVisitor.ShouldAbort())
			return;
		ioVisitor.VisitTriangle(v3u4.packed, ax * 2 + 1);
		if (ioVisitor.ShouldAbort())
			return;
	}
}

#ifdef JPH_DEBUG_RENDERER
void QuadChunkShape::Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const {
	return;
}
#endif

class DecodingContext {
public:
	static constexpr uint32_t cNumBitsXYZ = 4; // 4 bits each for x, y, z (32 blocks per dimension)
	static constexpr uint32_t cMaskBitsXYZ = (1 << cNumBitsXYZ) - 1;
	static constexpr uint32_t cLevelShift = 3 * cNumBitsXYZ; // 12 bits total for xyz

	JPH_INLINE explicit DecodingContext() : mTop(7) {
		// Initialize stack with root block (level 0, x=0, y=0, z=0)
		for (uint32_t i = 0; i < 8; ++i) {
			int32_t offset_x = ((i >> 0) & 1) * (QC::SIZE_1D >> 1);
			int32_t offset_y = ((i >> 1) & 1) * (QC::SIZE_1D >> 1);
			int32_t offset_z = ((i >> 2) & 1) * (QC::SIZE_1D >> 1);
			mPropertiesStack[i] = (1 << cLevelShift)
				| (offset_z << (cNumBitsXYZ * 2))
				| (offset_y << (cNumBitsXYZ * 1))
				| (offset_x << (cNumBitsXYZ * 0));
		}
	}

	template <class Visitor>
	JPH_INLINE void WalkQuads(Visitor &ioVisitor) {
		JPH_PROFILE_FUNCTION();

		// Early out if there's no collision
		if (ioVisitor.mShape->mQuadChunk->get_count() == 0) {
			return;
		}

		do {
			uint32_t properties = mPropertiesStack[mTop];
			uint32_t xyz = properties & ((1 << cLevelShift) - 1); // Extract xyz from properties
			uint32_t level = properties >> cLevelShift;
			if (level == cNumBitsXYZ) {
				// Leaf node: Process faces in this block
				VisitStarT(ioVisitor, xyz);
			} else {
				// Non-leaf node: Subdivide and push child blocks
				ProcessBranchBlock(ioVisitor, xyz, level);
			}

			// Check for early termination
			if (ioVisitor.ShouldAbort()) {
				return;
			}

			// Fetch next block
			do {
				--mTop;
			} while (mTop >= 0 && !ioVisitor.ShouldVisitRangeBlock(mTop));
		} while (mTop >= 0);
	}

	JPH_INLINE bool IsDoneWalking() const {
		return mTop < 0;
	}


private:
	int32_t mTop;
	uint32_t mPropertiesStack[cStackSize];

private:
	// Process a non-leaf block by subdividing
	template <class Visitor>
	JPH_INLINE void ProcessBranchBlock(Visitor &ioVisitor, int32_t xyz, uint32_t level) {
		int32_t x = (xyz >> (cNumBitsXYZ * 0)) & cMaskBitsXYZ;
		int32_t y = (xyz >> (cNumBitsXYZ * 1)) & cMaskBitsXYZ;
		int32_t z = (xyz >> (cNumBitsXYZ * 2)) & cMaskBitsXYZ;
		// Calculate block size at this level
		int32_t block_size = QC::SIZE_1D >> (level + 1);

		// Calculate base position
		Vec3 base_pos = Vec3(x, y, z);

		// Define 8 child blocks
		Mat44 block_min0, block_min1, block_max0, block_max1;
		uint32_t properties[8];
		for (uint32_t i = 0; i < 8; ++i) {
			int32_t offset_x = ((i >> 0) & 1) * block_size;
			int32_t offset_y = ((i >> 1) & 1) * block_size;
			int32_t offset_z = ((i >> 2) & 1) * block_size;
			auto offset = Vec3(offset_x, offset_y, offset_z);
			if (i < 4) {
				block_min0.SetColumn4(i, Vec4(base_pos + offset));
				block_max0.SetColumn4(i, Vec4(base_pos + offset + Vec3::sReplicate(block_size)));
			} else {
				block_min1.SetColumn4(i - 4, Vec4(base_pos + offset));
				block_max1.SetColumn4(i - 4, Vec4(base_pos + offset + Vec3::sReplicate(block_size)));
			}
			properties[i] = ((level + 1) << cLevelShift)
				| ((z + offset_z) << (cNumBitsXYZ * 2))
				| ((y + offset_y) << (cNumBitsXYZ * 1))
				| ((x + offset_x) << (cNumBitsXYZ * 0));
		}

		Mat44 transposed_min0 = block_min0.Transposed();
		Mat44 transposed_max0 = block_max0.Transposed();
		Mat44 transposed_min1 = block_min1.Transposed();
		Mat44 transposed_max1 = block_max1.Transposed();
		// Test child blocks
		uint32_t colliding_blocks[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
		int32_t num_results = ioVisitor.VisitRangeBlock(
				transposed_min0, transposed_max0,
                transposed_min1, transposed_max1,
				colliding_blocks, mTop);

		// Push colliding blocks onto stack
		JPH_ASSERT(mTop + num_results < cStackSize);
		for (int32_t i = num_results - 1; i >= 0; --i) {
			mPropertiesStack[mTop + i] = properties[colliding_blocks[i]];
		}
		mTop += num_results;
	}
};

template <typename Visitor>
void CastRayT(Visitor &ioVisitor) {
	JPH_PROFILE_FUNCTION();
	auto minBounds = Vec3::sReplicate(0);
	auto maxBounds = Vec3::sReplicate(QC::SIZE_1D - 0.01);
	Vec3 rayOrigin = ioVisitor.mRayOrigin;
	Vec3 rayDir = ioVisitor.mRayDirection;
	auto rayInvDir = RayInvDirection(rayDir);
	float distance = RayAABox(rayOrigin, rayInvDir, minBounds, maxBounds);
	if (distance == FLT_MAX) {
		return; // Ray misses chunk or is beyond early-out distance
	}

	auto end = rayOrigin + rayDir;
	auto end_v3u4 = Vector3u4(floor(end.GetX()), floor(end.GetY()), floor(end.GetZ()));
	Vec3 startPos = rayOrigin;
	if (distance > 0) {
		startPos += rayDir * distance;
	}

	UVec4 startPosU = startPos.ToInt();
	Vector3u4 xyz(startPosU.GetX(), startPosU.GetY(), startPosU.GetZ());

	Vec3 step =  rayDir.GetSign();
	// get the fractional portions of the float of the starting position
	// 1 - n for negetive components
	constexpr float LARGE = float(1 << 16);
	auto tDelta = UVec4::sSelect((LARGE * rayInvDir.mInvDirection.Abs()).ToInt(), UVec4::sReplicate(LARGE), rayInvDir.mIsParallel);
	Vec3 startPosF = (startPos - Vec3(startPosU.ToFloat())) * -step + Vec3::sMax(step, Vec3::sZero());
	auto tMax = UVec4::sSelect((startPosF * Vec3(tDelta.ToFloat())).ToInt(),UVec4::sReplicate(LARGE), rayInvDir.mIsParallel);

	// step through vert grid and check attached vert stars(cell)
	while (!xyz.is_invalid()) {
		VisitStarT(ioVisitor, xyz.packed);
		if (ioVisitor.ShouldAbort()) {
			break;
		}
		auto ax = tMax[0] < tMax[1] && tMax[0] < tMax[2] ? 0 : (tMax[1] < tMax[2] ? 1 : 2);
		if (end_v3u4.packed == xyz.packed) {
			break;
		}
		xyz.try_add(ax, step[ax]);
		tMax[ax] += tDelta[ax];
	}
}

bool QuadChunkShape::CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const {
	JPH_PROFILE_FUNCTION();

	struct Visitor {
		JPH_INLINE explicit Visitor(const QuadChunkShape *inShape, const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) :
				mHit(ioHit),
				mRayOrigin(inRay.mOrigin),
				mRayDirection(inRay.mDirection),
				mRayInvDirection(inRay.mDirection),
				mShape(inShape),
				mSubShapeIDCreator(inSubShapeIDCreator) {}

		JPH_INLINE bool ShouldAbort() const {
			return mHit.mFraction <= 0.0f;
		}

		JPH_INLINE void VisitTriangle(uint32_t inHex, uint32_t inTriangle) {
			Vec3 verts[3];
			if (mShape->DecodeTriangle(inHex, inTriangle, verts)) {
				return;
			}
			float fraction = RayTriangle(mRayOrigin, mRayDirection, verts[0], verts[1], verts[2]);
			if (fraction < mHit.mFraction) {
				mHit.mFraction = fraction;
				mHit.mSubShapeID2 = mShape->EncodeSubShapeID(mSubShapeIDCreator, inHex, inTriangle);
				mReturnValue = true;
			}
		}

		RayCastResult &mHit;
		Vec3 mRayOrigin;
		Vec3 mRayDirection;
		RayInvDirection mRayInvDirection;
		const QuadChunkShape *mShape;
		const SubShapeIDCreator &mSubShapeIDCreator;
		bool mReturnValue = false;
		float mDistanceStack[cStackSize];
	};

	Visitor visitor(this, inRay, inSubShapeIDCreator, ioHit);
	CastRayT(visitor);

	return visitor.mReturnValue;
}

void QuadChunkShape::CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter) const {
	JPH_PROFILE_FUNCTION();

	// Test shape filter
	if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID())) {
		return;
	}

	struct Visitor {
		JPH_INLINE explicit Visitor(const QuadChunkShape *inShape, const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector) :
				mRayOrigin(inRay.mOrigin),
				mRayDirection(inRay.mDirection),
				mShape(inShape),
				mSubShapeIDCreator(inSubShapeIDCreator),
				mCollector(ioCollector) {}
		JPH_INLINE bool ShouldAbort() const {
			return mCollector.ShouldEarlyOut();
		}

		JPH_INLINE void VisitTriangle(uint32_t inHex, uint32_t inTriangle) {
			Vec3 verts[3];
			if (mShape->DecodeTriangle(inHex, inTriangle, verts)) {
				return;
			}
			// Back facing check
			if (mBackFaceMode == EBackFaceMode::IgnoreBackFaces && (verts[2] - verts[0]).Cross(verts[1] - verts[0]).Dot(mRayDirection) < 0) {
				return;
			}

			// Check the triangle
			float fraction = RayTriangle(mRayOrigin, mRayDirection, verts[0], verts[1], verts[2]);
			if (fraction < mCollector.GetEarlyOutFraction()) {
				RayCastResult hit;

				hit.mBodyID = TransformedShape::sGetBodyID(mCollector.GetContext());
				hit.mFraction = fraction;
				hit.mSubShapeID2 = mShape->EncodeSubShapeID(mSubShapeIDCreator, inHex, inTriangle);
				mCollector.AddHit(hit);
			}
		}

		Vec3 mRayOrigin;
		Vec3 mRayDirection;
		const QuadChunkShape *mShape;
		const SubShapeIDCreator &mSubShapeIDCreator;
		CastRayCollector &mCollector;
		EBackFaceMode mBackFaceMode;
	};

	Visitor visitor(this, inRay, inSubShapeIDCreator, ioCollector);
	CastRayT(visitor);
}

void QuadChunkShape::CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter) const {
	sCollidePointUsingRayCast(*this, inPoint, inSubShapeIDCreator, ioCollector, inShapeFilter);
}

void QuadChunkShape::CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint32_t inNumVertices, int32_t inCollidingShapeIndex) const {
	JPH_PROFILE_FUNCTION();

	struct Visitor : public CollideSoftBodyVerticesVsTriangles {
		using CollideSoftBodyVerticesVsTriangles::CollideSoftBodyVerticesVsTriangles;
		JPH_INLINE explicit Visitor(const QuadChunkShape *inShape, Mat44Arg inCOM, Vec3Arg inScale) :
				CollideSoftBodyVerticesVsTriangles(inCOM, inScale), mShape(inShape) {}

		JPH_INLINE bool ShouldAbort() const {
			return false;
		}
		JPH_INLINE bool ShouldVisitRangeBlock([[maybe_unused]] int32_t inStackTop) const {
			return mDistanceStack[inStackTop] < mClosestDistanceSq;
		}

		JPH_INLINE int32_t VisitRangeBlock(const Mat44 &inBoundsMin0, const Mat44 &inBoundsMax0, const Mat44 &inBoundsMin1, const Mat44 &inBoundsMax1, uint32_t ioProperties[8], int32_t inStackTop) {
			// Get distance to vertex
			Vec4 dist_sq0 = AABox4DistanceSqToPoint(mLocalPosition,
					inBoundsMin0.GetColumn4(0), inBoundsMin0.GetColumn4(1), inBoundsMin0.GetColumn4(2),
					inBoundsMax0.GetColumn4(0), inBoundsMax0.GetColumn4(1), inBoundsMax0.GetColumn4(2));
			Vec4 dist_sq1 = AABox4DistanceSqToPoint(mLocalPosition,
					inBoundsMin1.GetColumn4(0), inBoundsMin1.GetColumn4(1), inBoundsMin1.GetColumn4(2),
					inBoundsMax1.GetColumn4(0), inBoundsMax1.GetColumn4(1), inBoundsMax1.GetColumn4(2));

			float dist_sq[8]{
				dist_sq0.GetX(), dist_sq0.GetY(), dist_sq0.GetZ(), dist_sq0.GetW(),
				dist_sq1.GetX(), dist_sq1.GetY(), dist_sq1.GetZ(), dist_sq1.GetW()
			};

			// Sort so that highest values are first (we want to first process closer hits and we process stack top to bottom)
			return SortReverseAndStore8(dist_sq, mClosestDistanceSq, ioProperties, &mDistanceStack[inStackTop]);
		}
		JPH_INLINE void VisitTriangle(uint32_t inHex, uint32_t inTriangle) {
			Vec3 verts[3];
			if (mShape->DecodeTriangle(inHex, inTriangle, verts)) {
				return;
			}
			ProcessTriangle(verts[0], verts[1], verts[2]);
		}

		const QuadChunkShape *mShape;
		float mDistanceStack[cStackSize];
	};

	Visitor visitor(this, inCenterOfMassTransform, inScale);

	for (CollideSoftBodyVertexIterator v = inVertices, sbv_end = inVertices + inNumVertices; v != sbv_end; ++v) {
		if (v.GetInvMass() > 0.0f) {
			visitor.StartVertex(v);
			auto ctx = DecodingContext();
			ctx.WalkQuads(visitor);
			visitor.FinishVertex(v, inCollidingShapeIndex);
		}
	}
}

struct QuadChunkShape::QSGetTrianglesContext {
	JPH_INLINE QSGetTrianglesContext(const QuadChunkShape *inShape, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) :
			mShape(inShape),
			mLocalBox(Mat44::sInverseRotationTranslation(inRotation, inPositionCOM), inBox),
			mChunkScale(inScale),
			mLocalToWorld(Mat44::sRotationTranslation(inRotation, inPositionCOM) * Mat44::sScale(inScale)),
			mIsInsideOut(ScaleHelpers::IsInsideOut(inScale)) {}
	bool ShouldAbort() const {
		return mShouldAbort;
	}

	bool ShouldVisitRangeBlock([[maybe_unused]] int32_t inTop) const {
		// Always visit all blocks
		return true;
	}

	JPH_INLINE int32_t VisitRangeBlock(const Mat44 &inBoundsMin0, const Mat44 &inBoundsMax0, const Mat44 &inBoundsMin1, const Mat44 &inBoundsMax1, uint32_t ioProperties[8], int32_t inStackTop) {
		// Scale the bounding boxes of this node
		Vec4 bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z;
		AABox4Scale(mChunkScale,
				inBoundsMin0.GetColumn4(0), inBoundsMin0.GetColumn4(1), inBoundsMin0.GetColumn4(2),
				inBoundsMax0.GetColumn4(0), inBoundsMax0.GetColumn4(1), inBoundsMax0.GetColumn4(2),
				bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);
		// Test which nodes collide
		UVec4 collides0 = AABox4VsBox(mLocalBox, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);
		AABox4Scale(mChunkScale,
				inBoundsMin1.GetColumn4(0), inBoundsMin1.GetColumn4(1), inBoundsMin1.GetColumn4(2),
				inBoundsMax1.GetColumn4(0), inBoundsMax1.GetColumn4(1), inBoundsMax1.GetColumn4(2),
				bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);
		UVec4 collides1 = AABox4VsBox(mLocalBox, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

		uint32_t collides[8] = {
			collides0.GetX(), collides0.GetY(), collides0.GetZ(), collides0.GetW(),
			collides1.GetX(), collides1.GetY(), collides1.GetZ(), collides1.GetW()
		};

		return CountAndSortTrues8(collides, ioProperties);
	}

	void VisitTriangle(uint32_t inHex, uint32_t inTriangle) {
		// When the buffer is full and we cannot process the triangles, abort the height field walk. The next time GetTrianglesNext is called we will continue here.
		if (mNumTrianglesFound + 1 > mMaxTrianglesRequested) {
			mShouldAbort = true;
			return;
		}

		Vec3 verts[3];
		if(mShape->DecodeTriangle(inHex, inTriangle, verts)) {
			return;
		}
		// Reverse vertices
		(mLocalToWorld * verts[0]).StoreFloat3(mTriangleVertices++);
		(mLocalToWorld * verts[1]).StoreFloat3(mTriangleVertices++);
		(mLocalToWorld * verts[2]).StoreFloat3(mTriangleVertices++);

		// Accumulate triangles found
		mNumTrianglesFound++;
	}

	DecodingContext mDecodeCtx;
	const QuadChunkShape *mShape;
	OrientedBox mLocalBox;
	Vec3 mChunkScale;
	Mat44 mLocalToWorld;
	int32_t mMaxTrianglesRequested;
	Float3 *mTriangleVertices;
	int32_t mNumTrianglesFound;
	const PhysicsMaterial **mMaterials;
	bool mShouldAbort;
	bool mIsInsideOut;
};

void QuadChunkShape::GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const {
	static_assert(sizeof(QSGetTrianglesContext) <= sizeof(GetTrianglesContext), "GetTrianglesContext too small");
	JPH_ASSERT(IsAligned(&ioContext, alignof(QSGetTrianglesContext)));
	new (&ioContext) QSGetTrianglesContext(this, inBox, inPositionCOM, inRotation, inScale);
}
int32_t QuadChunkShape::GetTrianglesNext(GetTrianglesContext &ioContext, int32_t inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials) const {
	JPH_ASSERT(inMaxTrianglesRequested >= cGetTrianglesMinTrianglesRequested);

	QSGetTrianglesContext &context = (QSGetTrianglesContext &)ioContext;
	if (context.mDecodeCtx.IsDoneWalking()) {
		return 0;
	}

	context.mMaxTrianglesRequested = inMaxTrianglesRequested;
	context.mTriangleVertices = outTriangleVertices;
	context.mMaterials = outMaterials;
	context.mShouldAbort = false;
	context.mNumTrianglesFound = 0;

	context.mDecodeCtx.WalkQuads(context);
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
	// Dummy implementation - return all edges active
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

		JPH_INLINE bool ShouldAbort() const {
			return mCollector.ShouldEarlyOut();
		}

		JPH_INLINE bool ShouldVisitRangeBlock([[maybe_unused]] int inStackTop) const {
			return true;
		}

		JPH_INLINE int32_t VisitRangeBlock(const Mat44 &inBoundsMin0, const Mat44 &inBoundsMax0, const Mat44 &inBoundsMin1, const Mat44 &inBoundsMax1, uint32_t ioProperties[8], [[maybe_unused]] int inStackTop) {
			// First batch: process inBoundsMin0/inBoundsMax0
			Vec4 bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z;
			AABox4Scale(mScale2,
					inBoundsMin0.GetColumn4(0), inBoundsMin0.GetColumn4(1), inBoundsMin0.GetColumn4(2),
					inBoundsMax0.GetColumn4(0), inBoundsMax0.GetColumn4(1), inBoundsMax0.GetColumn4(2),
					bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			UVec4 collides0 = AABox4VsBox(mBoundsOf1InSpaceOf2, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			// Second batch: process inBoundsMin1/inBoundsMax1
			AABox4Scale(mScale2,
					inBoundsMin1.GetColumn4(0), inBoundsMin1.GetColumn4(1), inBoundsMin1.GetColumn4(2),
					inBoundsMax1.GetColumn4(0), inBoundsMax1.GetColumn4(1), inBoundsMax1.GetColumn4(2),
					bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			UVec4 collides1 = AABox4VsBox(mBoundsOf1InSpaceOf2, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			uint32_t collides[8] = {
				collides0.GetX(), collides0.GetY(), collides0.GetZ(), collides0.GetW(),
				collides1.GetX(), collides1.GetY(), collides1.GetZ(), collides1.GetW()
			};

			return CountAndSortTrues8(collides, ioProperties);
		}

		JPH_INLINE void VisitTriangle(uint32_t inHex, uint32_t inTriangle) {
			Vec3 verts[3];
			if (mShape->DecodeTriangle(inHex, inTriangle, verts)) {
				return;
			}
			SubShapeID quad_sub_shape_id = mShape->EncodeSubShapeID(mSubShapeIDCreator2, inHex, inTriangle);
			uint8 active_edges = mShape->GetEdgeFlags(inHex, inTriangle);

			Collide(verts[0], verts[1], verts[2], active_edges, quad_sub_shape_id);
		}

		const QuadChunkShape *mShape;
		SubShapeIDCreator mSubShapeIDCreator2;
	};

	Visitor visitor(shape1, inScale1, inScale2, inCenterOfMassTransform1, inCenterOfMassTransform2, inSubShapeIDCreator1.GetID(), inCollideShapeSettings, ioCollector);
	visitor.mShape = shape2;
	visitor.mSubShapeIDCreator2 = inSubShapeIDCreator2;
	auto ctx = DecodingContext();
	ctx.WalkQuads(visitor);
}

void QuadChunkShape::sCollideSphereVsQuadChunk(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter) {
	JPH_PROFILE_FUNCTION();

	JPH_ASSERT(inShape1->GetSubType() == EShapeSubType::Sphere);
	JPH_ASSERT(inShape2->GetType() == EShapeType::User3);
	const SphereShape *shape1 = static_cast<const SphereShape *>(inShape1);
	const QuadChunkShape *shape2 = static_cast<const QuadChunkShape *>(inShape2);

	struct Visitor : public CollideSphereVsTriangles {
		using CollideSphereVsTriangles::CollideSphereVsTriangles;

		JPH_INLINE bool ShouldAbort() const {
			return mCollector.ShouldEarlyOut();
		}

		JPH_INLINE bool ShouldVisitRangeBlock([[maybe_unused]] int inStackTop) const {
			return true;
		}

		JPH_INLINE int32_t VisitRangeBlock(const Mat44 &inBoundsMin0, const Mat44 &inBoundsMax0, const Mat44 &inBoundsMin1, const Mat44 &inBoundsMax1, uint32_t ioProperties[8], [[maybe_unused]] int inStackTop) {
			// First batch: process inBoundsMin0/inBoundsMax0
			Vec4 bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z;
			AABox4Scale(mScale2,
					inBoundsMin0.GetColumn4(0), inBoundsMin0.GetColumn4(1), inBoundsMin0.GetColumn4(2),
					inBoundsMax0.GetColumn4(0), inBoundsMax0.GetColumn4(1), inBoundsMax0.GetColumn4(2),
					bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			UVec4 collides0 = AABox4VsSphere(mSphereCenterIn2, mRadiusPlusMaxSeparationSq, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			// Second batch: process inBoundsMin1/inBoundsMax1
			AABox4Scale(mScale2,
					inBoundsMin1.GetColumn4(0), inBoundsMin1.GetColumn4(1), inBoundsMin1.GetColumn4(2),
					inBoundsMax1.GetColumn4(0), inBoundsMax1.GetColumn4(1), inBoundsMax1.GetColumn4(2),
					bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			UVec4 collides1 = AABox4VsSphere(mSphereCenterIn2, mRadiusPlusMaxSeparationSq, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			uint32_t collides[8] = {
				collides0.GetX(), collides0.GetY(), collides0.GetZ(), collides0.GetW(),
				collides1.GetX(), collides1.GetY(), collides1.GetZ(), collides1.GetW()
			};

			return CountAndSortTrues8(collides, ioProperties);
		}

		JPH_INLINE void VisitTriangle(uint32_t inHex, uint32_t inTriangle) {
			Vec3 verts[3];
			if (mShape->DecodeTriangle(inHex, inTriangle, verts)) {
				return;
			}
			SubShapeID quad_sub_shape_id = mShape->EncodeSubShapeID(mSubShapeIDCreator2, inHex, inTriangle);
			uint8 active_edges = mShape->GetEdgeFlags(inHex, inTriangle);

			Collide(verts[0], verts[1], verts[2], active_edges, quad_sub_shape_id);
		}

		const QuadChunkShape *mShape;
		SubShapeIDCreator mSubShapeIDCreator2;
	};

	Visitor visitor(shape1, inScale1, inScale2, inCenterOfMassTransform1, inCenterOfMassTransform2, inSubShapeIDCreator1.GetID(), inCollideShapeSettings, ioCollector);
	visitor.mShape = shape2;
	visitor.mSubShapeIDCreator2 = inSubShapeIDCreator2;
	auto ctx = DecodingContext();
	ctx.WalkQuads(visitor);
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

		JPH_INLINE bool ShouldVisitRangeBlock(int inStackTop) const {
			return mDistanceStack[inStackTop] < mCollector.GetPositiveEarlyOutFraction();
		}

		JPH_INLINE int32_t VisitRangeBlock(const Mat44 &inBoundsMin0, const Mat44 &inBoundsMax0, const Mat44 &inBoundsMin1, const Mat44 &inBoundsMax1, uint32_t ioProperties[8], int inStackTop) {
			// First batch: process inBoundsMin0/inBoundsMax0
			Vec4 bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z;
			AABox4Scale(mScale,
					inBoundsMin0.GetColumn4(0), inBoundsMin0.GetColumn4(1), inBoundsMin0.GetColumn4(2),
					inBoundsMax0.GetColumn4(0), inBoundsMax0.GetColumn4(1), inBoundsMax0.GetColumn4(2),
					bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			AABox4EnlargeWithExtent(mScale, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			Vec4 distance0 = RayAABox4(mBoxCenter, mInvDirection, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			// Second batch: process inBoundsMin1/inBoundsMax1
			AABox4Scale(mScale,
					inBoundsMin1.GetColumn4(0), inBoundsMin1.GetColumn4(1), inBoundsMin1.GetColumn4(2),
					inBoundsMax1.GetColumn4(0), inBoundsMax1.GetColumn4(1), inBoundsMax1.GetColumn4(2),
					bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			AABox4EnlargeWithExtent(mScale, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			Vec4 distance1 = RayAABox4(mBoxCenter, mInvDirection, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			float distances[8] = {
				distance0.GetX(), distance0.GetY(), distance0.GetZ(), distance0.GetW(),
				distance1.GetX(), distance1.GetY(), distance1.GetZ(), distance1.GetW()
			};

			return SortReverseAndStore8(distances, mCollector.GetPositiveEarlyOutFraction(), ioProperties, &mDistanceStack[inStackTop]);
		}

		JPH_INLINE void VisitTriangle(uint32_t inHex, uint32_t inTriangle) {
			Vec3 verts[3];
			if (mShape->DecodeTriangle(inHex, inTriangle, verts)) {
				return;
			}
			SubShapeID quad_sub_shape_id = mShape->EncodeSubShapeID(mSubShapeIDCreator2, inHex, inTriangle);
			uint8 active_edges = mShape->GetEdgeFlags(inHex, inTriangle);

			Cast(verts[0], verts[1], verts[2], active_edges, quad_sub_shape_id);
		}

		const QuadChunkShape *mShape;
		RayInvDirection mInvDirection;
		Vec3 mBoxCenter;
		Vec3 mBoxExtent;
		SubShapeIDCreator mSubShapeIDCreator2;
		float mDistanceStack[cStackSize];
	};

	Visitor visitor(inShapeCast, inShapeCastSettings, inScale, inCenterOfMassTransform2, inSubShapeIDCreator1, ioCollector);
	visitor.mShape = shape;
	visitor.mInvDirection.Set(inShapeCast.mDirection);
	visitor.mBoxCenter = inShapeCast.mShapeWorldBounds.GetCenter();
	visitor.mBoxExtent = inShapeCast.mShapeWorldBounds.GetExtent();
	visitor.mSubShapeIDCreator2 = inSubShapeIDCreator2;
	auto ctx = DecodingContext();
	ctx.WalkQuads(visitor);
}

void QuadChunkShape::sCastSphereVsQuadChunk(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector) {
	JPH_PROFILE_FUNCTION();

	JPH_ASSERT(inShape->GetSubType() == EShapeSubType::User3);
	const QuadChunkShape *shape = static_cast<const QuadChunkShape *>(inShape);

	struct Visitor : public CastSphereVsTriangles {
		using CastSphereVsTriangles::CastSphereVsTriangles;

		JPH_INLINE bool ShouldAbort() const {
			return mCollector.ShouldEarlyOut();
		}

		JPH_INLINE bool ShouldVisitRangeBlock(int inStackTop) const {
			return mDistanceStack[inStackTop] < mCollector.GetPositiveEarlyOutFraction();
		}

		JPH_INLINE int32_t VisitRangeBlock(const Mat44 &inBoundsMin0, const Mat44 &inBoundsMax0, const Mat44 &inBoundsMin1, const Mat44 &inBoundsMax1, uint32_t ioProperties[8], int inStackTop) {
			// First batch: process inBoundsMin0/inBoundsMax0
			Vec4 bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z;
			AABox4Scale(mScale,
					inBoundsMin0.GetColumn4(0), inBoundsMin0.GetColumn4(1), inBoundsMin0.GetColumn4(2),
					inBoundsMax0.GetColumn4(0), inBoundsMax0.GetColumn4(1), inBoundsMax0.GetColumn4(2),
					bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			AABox4EnlargeWithExtent(Vec3::sReplicate(mRadius), bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			Vec4 distance0 = RayAABox4(mStart, mInvDirection, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			// Second batch: process inBoundsMin1/inBoundsMax1
			AABox4Scale(mScale,
					inBoundsMin1.GetColumn4(0), inBoundsMin1.GetColumn4(1), inBoundsMin1.GetColumn4(2),
					inBoundsMax1.GetColumn4(0), inBoundsMax1.GetColumn4(1), inBoundsMax1.GetColumn4(2),
					bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			AABox4EnlargeWithExtent(Vec3::sReplicate(mRadius), bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			Vec4 distance1 = RayAABox4(mStart, mInvDirection, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			float distances[8] = {
				distance0.GetX(), distance0.GetY(), distance0.GetZ(), distance0.GetW(),
				distance1.GetX(), distance1.GetY(), distance1.GetZ(), distance1.GetW()
			};

			return SortReverseAndStore8(distances, mCollector.GetPositiveEarlyOutFraction(), ioProperties, &mDistanceStack[inStackTop]);
		}

		JPH_INLINE void VisitTriangle(uint32_t inHex, uint32_t inTriangle) {
			Vec3 verts[3];
			if (mShape->DecodeTriangle(inHex, inTriangle, verts)) {
				return;
			}
			SubShapeID quad_sub_shape_id = mShape->EncodeSubShapeID(mSubShapeIDCreator2, inHex, inTriangle);
			uint8 active_edges = mShape->GetEdgeFlags(inHex, inTriangle);

			Cast(verts[0], verts[1], verts[2], active_edges, quad_sub_shape_id);
		}

		const QuadChunkShape *mShape;
		RayInvDirection mInvDirection;
		SubShapeIDCreator mSubShapeIDCreator2;
		float mDistanceStack[cStackSize];
	};

	Visitor visitor(inShapeCast, inShapeCastSettings, inScale, inCenterOfMassTransform2, inSubShapeIDCreator1, ioCollector);
	visitor.mShape = shape;
	visitor.mInvDirection.Set(inShapeCast.mDirection);
	visitor.mSubShapeIDCreator2 = inSubShapeIDCreator2;
	auto ctx = DecodingContext();
	ctx.WalkQuads(visitor);
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

	// Specialized collision functions
	CollisionDispatch::sRegisterCollideShape(EShapeSubType::Sphere, EShapeSubType::User3, sCollideSphereVsQuadChunk);
	CollisionDispatch::sRegisterCastShape(EShapeSubType::Sphere, EShapeSubType::User3, sCastSphereVsQuadChunk);
}

JPH_NAMESPACE_END
