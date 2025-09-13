#include "FaceChunkShape.h"

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
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/ShapeFilter.h>
#include <Jolt/Physics/Collision/TransformedShape.h>
#include <Jolt/TriangleSplitter/TriangleSplitterBinning.h>

JPH_NAMESPACE_BEGIN

#ifdef JPH_DEBUG_RENDERER
bool MeshShape::sDrawTriangleGroups = false;
bool MeshShape::sDrawTriangleOutlines = false;
#endif // JPH_DEBUG_RENDERER

static constexpr uint32_t cStackSize = 256; // Stack size for blocks

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


ShapeSettings::ShapeResult FaceChunkShapeSettings::Create() const {
	if (mCachedResult.IsEmpty()) {
		Ref<Shape> shape = new FaceChunkShape(*this, mCachedResult);
	}
	return mCachedResult;
}

FaceChunkShape::FaceChunkShape(const FaceChunkShapeSettings &inSettings, ShapeResult &outResult) :
		Shape(EShapeType::User3, EShapeSubType::User3, inSettings, outResult),
		mFaces(inSettings.mFaces) {
	if (inSettings.mFaces.is_null()) {
		outResult.SetError("Invalid faces");
		return;
	}
	outResult.Set(this);
}

MassProperties FaceChunkShape::GetMassProperties() const {
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
const PhysicsMaterial *FaceChunkShape::GetMaterial(const SubShapeID &inSubShapeID) const {
	return JPH::PhysicsMaterial::sDefault;
}

SubShapeID FaceChunkShape::EncodeSubShapeID(const SubShapeIDCreator &inCreator, uint32_t inFace, uint32_t inTriangle) const {
	return inCreator.PushID(inFace * 2 + inTriangle, cSubShapeIDBits).GetID();
}
void FaceChunkShape::DecodeTriangle(uint32_t inFace, uint32_t inTriangle, Vec3 outVerts[3]) const {
	SSK::Face face = mFaces->data_[inFace];
	int32_t axis = inFace % 3;
	auto v0 = Vec3(inFace / 32 / 32, inFace / 32 % 32, inFace % 32);
	auto v1 = Vec3(axis == 0, axis == 2, axis == 1);
	auto v2 = Vec3(axis == 1, axis == 0, axis == 2);
	if (inTriangle) {
		v1 += v2;
	} else {
		v2 += v1;
	}
	v1 += v0;
	v2 += v0;
	int32_t i0 = 0;
	int32_t i1 = inTriangle ? 2 : 1;
	int32_t i2 = inTriangle ? 3 : 2;
	if ((face.enabled_flip_tx & 2) == 2) {
		auto tmp = v1;
		v1 = v2;
		v2 = tmp;
		i1 = i1 ^ i2;
		i2 = i1 ^ i2;
		i1 = i1 ^ i2;
	}
	outVerts[0] = Vec3(v0[0] + 1.0f / (face.pos[i0] & 7),
			v0[1] + 1.0f / ((face.pos[i0] << 3) & 7),
			v0[2] + 1.0f / (face.pos[i0] << 6 | (face.pos_ends & 3)));
	outVerts[1] = Vec3(v1[0] + 1.0f / (face.pos[1] & 7),
			v1[1] + 1.0f / ((face.pos[i1] << 3) & 7),
			v1[2] + 1.0f / (face.pos[i1] << 6 | ((face.pos_ends << (i1 * 2)) & 3)));
	outVerts[2] = Vec3(v2[0] + 1.0f / (face.pos[i2] & 7),
			v2[1] + 1.0f / ((face.pos[i2] << 3) & 7),
			v2[2] + 1.0f / (face.pos[i2] << 6 | ((face.pos_ends << (i2 * 2)) & 3)));
}

void FaceChunkShape::DecodeSubShapeID(const SubShapeID &inSubShapeID, uint32_t &outFace, uint32_t &outTriangle) const {
	// Get block
	SubShapeID remainder;
	uint32_t id = inSubShapeID.PopID(cSubShapeIDBits, remainder);
	JPH_ASSERT(remainder.IsEmpty(), "Invalid subshape ID");

	outTriangle = id & 1;
	outFace = id >> 1;
}

Vec3 FaceChunkShape::GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const {
	// Decode ID
	uint32_t face;
	uint32_t triangle_idx;
	DecodeSubShapeID(inSubShapeID, face, triangle_idx);

	// Decode triangle
	Vec3 verts[3];
	DecodeTriangle(face, triangle_idx, verts);

	// Calculate normal
	return (verts[2] - verts[1]).Cross(verts[0] - verts[1]).Normalized();
}

void FaceChunkShape::GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const {
	// Decode ID
	uint32_t face;
	uint32_t triangle_idx;
	DecodeSubShapeID(inSubShapeID, face, triangle_idx);

	// Decode triangle
	outVertices.resize(3);
	DecodeTriangle(face, triangle_idx, &outVertices[0]);

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
#ifdef JPH_DEBUG_RENDERER
void HeightFieldShape::Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const {
	return
}
#endif

class DecodingContext {
public:
	static constexpr uint32_t cNumBitsXYZ = 5; // 5 bits each for x, y, z (32 blocks per dimension)
	static constexpr uint32_t cMaskBitsXYZ = (1 << cNumBitsXYZ) - 1;
	static constexpr uint32_t cLevelShift = 3 * cNumBitsXYZ; // 15 bits total for xyz
	static constexpr uint32_t cMaxLevel = 5; // Max subdivision levels log2(32)

	JPH_INLINE explicit DecodingContext(const FaceChunkShape *inShape) :
			mShape(inShape),
			mTop(0) {
		JPH_ASSERT(inShape != nullptr, "FaceChunkShape cannot be null");
		JPH_ASSERT(!inShape->GetFaces().is_null(), "Faces array cannot be null");

		// Initialize stack with root block (level 0, x=0, y=0, z=0)
		mPropertiesStack[0] = 0;
	}

	template <class Visitor>
	JPH_INLINE void WalkFaces(Visitor &ioVisitor) {
		JPH_PROFILE_FUNCTION();

		// Early out if there's no collision
		if (mShape->mFaces->count_ == 0) {
			return;
		}

		do {
			uint32_t properties = mPropertiesStack[mTop];
			uint32_t xyz = properties & (1 << cLevelShift) - 1; // Extract xyz from properties
			uint32_t level = properties >> cLevelShift;
			if (level >= cMaxLevel) {
				// Leaf node: Process faces in this block
				for (uint32_t axis = 0; axis < 3 && !ioVisitor.ShouldAbort(); ++axis) {
					uint32_t face_idx = xyz * 3 + axis;
					VisitFaceT(ioVisitor, face_idx);
				}
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
	const FaceChunkShape *mShape;
	int32_t mTop;
	uint32_t mPropertiesStack[cStackSize];

private:
	// Process a non-leaf block by subdividing
	template <class Visitor>
	JPH_INLINE void ProcessBranchBlock(Visitor &ioVisitor, int32_t xyz, uint32_t level) {
		int32_t x = xyz & cMaskBitsXYZ;
		int32_t y = (xyz >> cNumBitsXYZ) & cMaskBitsXYZ;
		int32_t z = (xyz >> (cNumBitsXYZ * 2)) & cMaskBitsXYZ;
		// Calculate block size at this level
		int32_t block_size = (SSK::kSize >> (level + 1)) + 1;

		// Calculate base position
		Vec3 base_pos = Vec3(x, y, z);

		// Define 8 child blocks
		Mat44 block_min0, block_min1, block_max0, block_max1;
		uint32_t properties[8];
		for (uint32_t i = 0; i < 8; ++i) {
			int32_t offset_x = (i & 1) * block_size;
			int32_t offset_y = ((i & 2) << 1) * block_size;
			int32_t offset_z = ((i & 4) << 2) * block_size;
			auto offset = Vec3(
					(i & 1) ? block_size : 0.0,
					(i & 2) ? block_size : 0.0,
					(i & 4) ? block_size : 0.0 );
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
		Mat44 transposed_min1 = block_min1.Transposed();
		Mat44 transposed_max0 = block_max0.Transposed();
		Mat44 transposed_max1 = block_max1.Transposed();
		// Test child blocks
		uint32_t colliding_blocks[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
		int32_t num_results = ioVisitor.VisitRangeBlock(
				transposed_min0, transposed_min1,
				transposed_max0, transposed_max1,
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
void VisitFaceT(Visitor &ioVisitor, uint32_t inFace) {
	auto face = ioVisitor.mShape->mFaces->data_[inFace];
	if ((face.enabled_flip_tx & 1) == 0) {
		return;
	}
	ioVisitor.VisitTriangle(inFace, 0);
	ioVisitor.VisitTriangle(inFace, 1);
}

template <typename Visitor>
void VisitCellT(Visitor &ioVisitor, int32_t x, int32_t y, int32_t z) {
	for (int32_t axis = 0; axis < 3; ++axis) {
		VisitFaceT(ioVisitor, (x * SSK::kSize2 + y * SSK::kSize + z) * 3 + axis);
	}
	// Check -x cell (x-1, y, z)
	if (x > 0) {
		VisitFaceT(ioVisitor, ((x - 1) * SSK::kSize2 + y * SSK::kSize + z) * 3 + 1);
		VisitFaceT(ioVisitor, ((x - 1) * SSK::kSize2 + y * SSK::kSize + z) * 3 + 2);
	}

	// Check -y cell (x, y-1, z)
	if (y > 0) {
		VisitFaceT(ioVisitor, (x * SSK::kSize2 + (y - 1) * SSK::kSize + z) * 3 + 0);
		VisitFaceT(ioVisitor, (x * SSK::kSize2 + (y - 1) * SSK::kSize + z) * 3 + 2);
	}

	// Check -z cell (x, y, z-1)
	if (z > 0) {
		VisitFaceT(ioVisitor, (x * SSK::kSize2 + y * SSK::kSize + (z - 1)) * 3 + 0);
		VisitFaceT(ioVisitor, (x * SSK::kSize2 + y * SSK::kSize + (z - 1)) * 3 + 1);
	}

	// Check -xy cell (x-1, y-1, z)
	if (x > 0 && y > 0) {
		VisitFaceT(ioVisitor, ((x - 1) * SSK::kSize2 + (y - 1) * SSK::kSize + z) * 3 + 2);
	}

	// Check -xz cell (x-1, y, z-1)
	if (x > 0 && z > 0) {
		VisitFaceT(ioVisitor, ((x - 1) * SSK::kSize2 + y * SSK::kSize + (z - 1)) * 3 + 1);
	}

	// Check -yz cell (x, y-1, z-1)
	if (y > 0 && z > 0) {
		VisitFaceT(ioVisitor, (x * SSK::kSize2 + (y - 1) * SSK::kSize + (z - 1)) * 3 + 0);
	}
}

template <typename Visitor>
void CastRayT(Visitor &ioVisitor) {
	JPH_PROFILE_FUNCTION();

	auto tMin = Vec3::sReplicate(0.0f);
	auto tMax = Vec3::sReplicate(SSK::kSize);
	Vec3 rayOrigin = ioVisitor.mRayOrigin;
	Vec3 rayDir = ioVisitor.mRayDirection;
	auto rayInvDir = RayInvDirection(rayDir);
	float distance = RayAABox(rayOrigin, rayInvDir, tMin, tMax);
	if (distance == FLT_MAX) {
		return; // Ray misses chunk or is beyond early-out distance
	}

	auto invDir = rayInvDir.mInvDirection;
	Vec3 startPos = rayOrigin + rayDir * distance;
	int32_t x = static_cast<int32_t>(floorf(startPos.GetX()));
	int32_t y = static_cast<int32_t>(floorf(startPos.GetY()));
	int32_t z = static_cast<int32_t>(floorf(startPos.GetZ()));

	int32_t stepX = rayDir.GetX() >= 0 ? 1 : -1;
	int32_t stepY = rayDir.GetY() >= 0 ? 1 : -1;
	int32_t stepZ = rayDir.GetZ() >= 0 ? 1 : -1;

	float tMaxX = x +(stepX == 1)- startPos.GetX();
	float tMaxY = y +(stepX == 1)- startPos.GetY();
	float tMaxZ = z +(stepX == 1)- startPos.GetZ();
	tMaxX = tMin.GetX() + tMaxX * invDir.GetX();
	tMaxY = tMin.GetY() + tMaxY * invDir.GetY();
	tMaxZ = tMin.GetZ() + tMaxZ * invDir.GetZ();

	float tDeltaX = stepX * invDir.GetX();
	float tDeltaY = stepY * invDir.GetY();
	float tDeltaZ = stepZ * invDir.GetZ();

	while (x >= 0 && x < SSK::kSize && y >= 0 && y < SSK::kSize && z >= 0 && z < SSK::kSize) {
		VisitCellT(ioVisitor, x, y, z);
		if (ioVisitor.ShouldAbort()) {
			break;
		}

		if (tMaxX < tMaxY && tMaxX < tMaxZ) {
			x += stepX;
			tMaxX += tDeltaX;
		} else if (tMaxY < tMaxZ) {
			y += stepY;
			tMaxY += tDeltaY;
		} else {
			z += stepZ;
			tMaxZ += tDeltaZ;
		}
	}
}

bool FaceChunkShape::CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const {
	JPH_PROFILE_FUNCTION();

	struct Visitor {
		JPH_INLINE explicit Visitor(const FaceChunkShape *inShape, const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) :
				mHit(ioHit),
				mRayOrigin(inRay.mOrigin),
				mRayDirection(inRay.mDirection),
				mRayInvDirection(inRay.mDirection),
				mShape(inShape),
				mSubShapeIDCreator(inSubShapeIDCreator) {}

		JPH_INLINE bool ShouldAbort() const {
			return mHit.mFraction <= 0.0f;
		}

		JPH_INLINE void VisitTriangle(uint inFace, uint inTriangle) {
			Vec3 verts[3];
			mShape->DecodeTriangle(inFace, inTriangle, verts);
			float fraction = RayTriangle(mRayOrigin, mRayDirection, verts[0], verts[1], verts[2]);
			if (fraction < mHit.mFraction) {
				mHit.mFraction = fraction;
				mHit.mSubShapeID2 = mShape->EncodeSubShapeID(mSubShapeIDCreator, inFace, inTriangle);
				mReturnValue = true;
			}
		}

		RayCastResult &mHit;
		Vec3 mRayOrigin;
		Vec3 mRayDirection;
		RayInvDirection mRayInvDirection;
		const FaceChunkShape *mShape;
		const SubShapeIDCreator &mSubShapeIDCreator;
		bool mReturnValue = false;
		float						mDistanceStack[cStackSize];
	};

	Visitor visitor(this, inRay, inSubShapeIDCreator, ioHit);
	CastRayT(visitor);

	return visitor.mReturnValue;
}

void FaceChunkShape::CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter) const {
	JPH_PROFILE_FUNCTION();

	// Test shape filter
	if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID())) {
		return;
	}

	struct Visitor {
		JPH_INLINE explicit Visitor(const FaceChunkShape *inShape, const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector) :
				mRayOrigin(inRay.mOrigin),
				mRayDirection(inRay.mDirection),
				mShape(inShape),
				mSubShapeIDCreator(inSubShapeIDCreator),
				mCollector(ioCollector) {}
		JPH_INLINE bool ShouldAbort() const {
			return mCollector.ShouldEarlyOut();
		}

		JPH_INLINE void VisitTriangle(uint32_t inFace, uint32_t inTriangle) {
			Vec3 verts[3];
			mShape->DecodeTriangle(inFace, inTriangle, verts);
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
				hit.mSubShapeID2 = mShape->EncodeSubShapeID(mSubShapeIDCreator, inFace, inTriangle);
				mCollector.AddHit(hit);
			}
		}

		Vec3 mRayOrigin;
		Vec3 mRayDirection;
		const FaceChunkShape *mShape;
		const SubShapeIDCreator &mSubShapeIDCreator;
		CastRayCollector &mCollector;
		EBackFaceMode mBackFaceMode;
	};

	Visitor visitor(this, inRay, inSubShapeIDCreator, ioCollector);
	CastRayT(visitor);
}

void FaceChunkShape::CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter) const {
	sCollidePointUsingRayCast(*this, inPoint, inSubShapeIDCreator, ioCollector, inShapeFilter);
}

void FaceChunkShape::CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint32_t inNumVertices, int32_t inCollidingShapeIndex) const {
	JPH_PROFILE_FUNCTION();

	struct Visitor : public CollideSoftBodyVerticesVsTriangles {
		using CollideSoftBodyVerticesVsTriangles::CollideSoftBodyVerticesVsTriangles;
		JPH_INLINE explicit Visitor(const FaceChunkShape *inShape, Mat44Arg inCOM, Vec3Arg inScale) :
				CollideSoftBodyVerticesVsTriangles(inCOM, inScale), mShape(inShape) {}

		JPH_INLINE bool ShouldAbort() const {
			return false;
		}
		JPH_INLINE bool	ShouldVisitRangeBlock([[maybe_unused]] int32_t inStackTop) const
		{
			return mDistanceStack[inStackTop] < mClosestDistanceSq;
		}

		JPH_INLINE int32_t VisitRangeBlock(const Mat44 &inBoundsMin0, const Mat44 &inBoundsMax0, const Mat44 &inBoundsMin1, const Mat44 &inBoundsMax1, uint32_t ioProperties[8], int32_t inStackTop)
		{
			// Get distance to vertex
			Vec4 dist_sq0 = AABox4DistanceSqToPoint(mLocalPosition, 
					inBoundsMin0.GetColumn4(0), inBoundsMin0.GetColumn4(1), inBoundsMin0.GetColumn4(2), 
					inBoundsMax0.GetColumn4(0), inBoundsMax0.GetColumn4(1), inBoundsMax0.GetColumn4(2));
			Vec4 dist_sq1 = AABox4DistanceSqToPoint(mLocalPosition, 
					inBoundsMin1.GetColumn4(1), inBoundsMin1.GetColumn4(1), inBoundsMin1.GetColumn4(2), 
					inBoundsMax1.GetColumn4(1), inBoundsMax1.GetColumn4(1), inBoundsMax1.GetColumn4(2));

			float dist_sq[8] {
				dist_sq0.GetX(), dist_sq0.GetY(), dist_sq0.GetZ(), dist_sq0.GetW(),
				dist_sq1.GetX(), dist_sq1.GetY(), dist_sq1.GetZ(), dist_sq1.GetW()	
			};

			// Sort so that highest values are first (we want to first process closer hits and we process stack top to bottom)
			return SortReverseAndStore8(dist_sq, mClosestDistanceSq, ioProperties, &mDistanceStack[inStackTop]);
		}
		JPH_INLINE void VisitTriangle(uint32_t inFace, uint32_t inTriangle) {
			Vec3 verts[3];
			mShape->DecodeTriangle(inFace, inTriangle, verts);
			ProcessTriangle(verts[0], verts[1], verts[2]);
		}

		const FaceChunkShape *mShape;
		float						mDistanceStack[cStackSize];
	};

	Visitor visitor(this, inCenterOfMassTransform, inScale);

	for (CollideSoftBodyVertexIterator v = inVertices, sbv_end = inVertices + inNumVertices; v != sbv_end; ++v) {
		if (v.GetInvMass() > 0.0f) {
			visitor.StartVertex(v);

			// Get vertex in local chunk space
			Vec3 local_pos = v.GetPosition();

			// Determine cell containing the vertex
			int32_t x = static_cast<int32_t>(floorf(local_pos.GetX()));
			int32_t y = static_cast<int32_t>(floorf(local_pos.GetY()));
			int32_t z = static_cast<int32_t>(floorf(local_pos.GetZ()));

			// Visit the cell (and optionally neighbors if soft body radius requires it)
			VisitCellT(visitor, x, y, z);

			visitor.FinishVertex(v, inCollidingShapeIndex);
		}
	}
}

struct FaceChunkShape::MSGetTrianglesContext {
	JPH_INLINE MSGetTrianglesContext(const FaceChunkShape *inShape, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) :
			mDecodeCtx(inShape),
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

	JPH_INLINE int32_t VisitRangeBlock(const Mat44 &inBoundsMin0, const Mat44 &inBoundsMax0, const Mat44 &inBoundsMin1, const Mat44 &inBoundsMax1, uint32_t ioProperties[8], int32_t inStackTop)
	{
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

	void	VisitTriangle(uint inFace, uint inTriangle)
	{
		// When the buffer is full and we cannot process the triangles, abort the height field walk. The next time GetTrianglesNext is called we will continue here.
		if (mNumTrianglesFound + 1 > mMaxTrianglesRequested)
		{
			mShouldAbort = true;
			return;
		}

		Vec3 verts[3];
		mShape->DecodeTriangle(inFace, inTriangle, verts);
		// Reverse vertices
		(mLocalToWorld * verts[0]).StoreFloat3(mTriangleVertices++);
		(mLocalToWorld * verts[1]).StoreFloat3(mTriangleVertices++);
		(mLocalToWorld * verts[2]).StoreFloat3(mTriangleVertices++);

		// Accumulate triangles found
		mNumTrianglesFound++;
	}

	DecodingContext	mDecodeCtx;
	const FaceChunkShape *mShape;
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

void FaceChunkShape::GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const {
	static_assert(sizeof(MSGetTrianglesContext) <= sizeof(GetTrianglesContext), "GetTrianglesContext too small");
	JPH_ASSERT(IsAligned(&ioContext, alignof(MSGetTrianglesContext)));
	new (&ioContext) MSGetTrianglesContext(this, inBox, inPositionCOM, inRotation, inScale);
}
int32_t FaceChunkShape::GetTrianglesNext(GetTrianglesContext &ioContext, int32_t inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials) const {
	JPH_ASSERT(inMaxTrianglesRequested >= cGetTrianglesMinTrianglesRequested);

	MSGetTrianglesContext &context = (MSGetTrianglesContext &)ioContext;
	if (context.ShouldAbort()) {
		return 0;
	}

	context.mMaxTrianglesRequested = inMaxTrianglesRequested;
	context.mTriangleVertices = outTriangleVertices;
	context.mMaterials = outMaterials;
	context.mShouldAbort = false; 
	context.mNumTrianglesFound = 0;

	context.mDecodeCtx.WalkFaces(context);
	return context.mNumTrianglesFound;
}

void FaceChunkShape::SaveBinaryState(StreamOut &inStream) const {
}

void FaceChunkShape::RestoreBinaryState(StreamIn &inStream) {
}

void FaceChunkShape::RestoreMaterialState(const PhysicsMaterialRefC *inMaterials, uint32_t inNumMaterials) {}

Shape::Stats FaceChunkShape::GetStats() const {
	uint32_t num_triangles = 0;
	for (int32_t x = 0; x < SSK::kSize; ++x) {
		for (int32_t y = 0; y < SSK::kSize; ++y) {
			for (int32_t z = 0; z < SSK::kSize; ++z) {
				num_triangles += 6; // 2 triangles per face, 3 faces per cell
			}
		}
	}
	return Stats(sizeof(*this) + sizeof(SSK::Faces), num_triangles);
}

void FaceChunkShape::sRegister() {
	ShapeFunctions &f = ShapeFunctions::sGet(EShapeSubType::Mesh);
	f.mConstruct = []() -> Shape * { return new FaceChunkShape; };
	f.mColor = Color::sRed;

	for (EShapeSubType s : sConvexSubShapeTypes) {
		//CollisionDispatch::sRegisterCollideShape(s, EShapeSubType::User3, sCollideConvexVsFaceChunk);
		//CollisionDispatch::sRegisterCastShape(s, EShapeSubType::User3, sCastConvexVsFaceChunk);

		CollisionDispatch::sRegisterCastShape(EShapeSubType::User3, s, CollisionDispatch::sReversedCastShape);
		CollisionDispatch::sRegisterCollideShape(EShapeSubType::User3, s, CollisionDispatch::sReversedCollideShape);
	}

	// Specialized collision functions
	//CollisionDispatch::sRegisterCollideShape(EShapeSubType::Sphere, EShapeSubType::User3, sCollideSphereVsFaceChunk);
	//CollisionDispatch::sRegisterCastShape(EShapeSubType::Sphere, EShapeSubType::User3, sCastSphereVsFaceChunk);
}

JPH_NAMESPACE_END
