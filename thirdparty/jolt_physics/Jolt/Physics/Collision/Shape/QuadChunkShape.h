#ifndef SSK_JOLT_CHUNK_SHAPE_H
#define SSK_JOLT_CHUNK_SHAPE_H

#include <modules/shardscape/QuadChunk.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

JPH_NAMESPACE_BEGIN


class CollideShapeSettings;

class JPH_EXPORT QuadChunkShapeSettings final : public ShapeSettings {
public:
	QuadChunkShapeSettings(const SSK::QuadChunk *pQuadChunk) :
			mQuadChunk(pQuadChunk) {}

	virtual ShapeResult Create() const override;
	const SSK::QuadChunk *mQuadChunk;
};

class JPH_EXPORT QuadChunkShape final : public Shape {
public:
	JPH_OVERRIDE_NEW_DELETE

	/// Constructor
	QuadChunkShape() :
			Shape(EShapeType::User3, EShapeSubType::User3) {}
	QuadChunkShape(const QuadChunkShapeSettings &inSettings, ShapeResult &outResult);
	const SSK::QuadChunk *mQuadChunk;

	virtual bool MustBeStatic() const override { return true; }
	// See Shape::GetLocalBounds
	virtual AABox GetLocalBounds() const override { return AABox(Vec3::sReplicate(-2), Vec3::sReplicate(SSK::QuadChunk::SIZE_1D + 2)); }
	// See Shape::GetSubShapeIDBitsRecursive
	virtual uint GetSubShapeIDBitsRecursive() const override { return cSubShapeIDBits; }
	// See Shape::GetInnerRadius
	virtual float GetInnerRadius() const override { return SSK::QuadChunk::SIZE_1D / 2.0F; }
	// See Shape::GetMassProperties
	virtual MassProperties GetMassProperties() const override;

	virtual void RestoreMaterialState(const PhysicsMaterialRefC *inMaterials, uint inNumMaterials) override;
	// See Shape::GetMaterial
	virtual const PhysicsMaterial *GetMaterial(const SubShapeID &inSubShapeID) const override;
	// See Shape::GetSurfaceNormal
	virtual Vec3 GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const override;
	// See Shape::GetSupportingQuad
	virtual void GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const override;
	// See Shape::CastRay
	virtual bool CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const override;
	// See: Shape::CollidePoint
	virtual void CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter = {}) const override;
	// See: Shape::CollideSoftBodyVertices
	virtual void CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const override;
	// See Shape::GetSubmergedVolume
	virtual void GetSubmergedVolume(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const Plane &inSurface, float &outTotalVolume, float &outSubmergedVolume, Vec3 &outCenterOfBuoyancy JPH_IF_DEBUG_RENDERER(, RVec3Arg inBaseOffset)) const override { JPH_ASSERT(false, "Not supported"); }
#ifdef JPH_DEBUG_RENDERER
	// See Shape::Draw
	virtual void Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const override;
#endif // JPH_DEBUG_RENDERER
	// See Shape::GetTrianglesStart
	virtual void GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const override;
	// See Shape::GetTrianglesNext
	virtual int GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials = nullptr) const override;
	// See Shape
	virtual void SaveBinaryState(StreamOut &inStream) const override;
	// See Shape::GetStats
	virtual Stats GetStats() const override;
	// See Shape::GetVolume
	virtual float GetVolume() const override { return GetLocalBounds().GetVolume(); }
	/// Get the convex radius of this box
	float GetConvexRadius() const { return 0.0f; }
	// Register shape functions with the registry
	static void sRegister();
	inline int DecodeTriangle(uint inQuad, uint inTriangle, Vec3 outTriangle[3]) const;

	/// Get the edge flags for a triangle
	inline uint8 GetEdgeFlags(uint inQuad, uint inTriangle) const;

	// Helper functions called by CollisionDispatch
	static void	sCollideConvexVsQuadChunk(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter);
	static void	sCollideSphereVsQuadChunk(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter);
	static void	sCastConvexVsQuadChunk(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector);
	static void	sCastSphereVsQuadChunk(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector);

protected:
	// See: Shape::RestoreBinaryState
	virtual void RestoreBinaryState(StreamIn &inStream) override;

private:
	struct QSGetTrianglesContext; ///< Context class for GetTrianglesStart/Next
	void CastRay(const RayCast &, const RayCastSettings &, const SubShapeIDCreator &, CastRayCollector &, const ShapeFilter &) const override;
	/// En/decode a sub shape ID. inX and inY specify the coordinate of the triangle. inTriangle == 0 is the lower triangle, inTriangle == 1 is the upper triangle.
	inline SubShapeID EncodeSubShapeID(const SubShapeIDCreator &inCreator, uint inQuad, uint inTriangle) const;
	inline void DecodeSubShapeID(const SubShapeID &inSubShapeID, uint &outQuad, uint &outTriangle) const;

	static constexpr uint cSubShapeIDBits = 18; // 15 bits for 32 * 32 * 32 hexes and 3 for tri
};

JPH_NAMESPACE_END
#endif
