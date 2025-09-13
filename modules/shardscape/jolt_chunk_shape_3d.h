#ifndef SSK_CHUNK_SHAPE_H
#define SSK_CHUNK_SHAPE_H
#include "FaceChunkShape.h"
#include <modules/jolt_physics/shapes/jolt_shape_3d.h>

class JoltFaceChunkShape3D final : public JoltShape3D {
	const AABB aabb = AABB(Vector3(0, 0, 0), Vector3(32, 32, 32));
	Ref<SSK::Faces> faces;
	bool back_face_collision = false;
virtual JPH::ShapeRefC _build() const override;

public:
	virtual ShapeType get_type() const override { return ShapeType::SHAPE_CONCAVE_POLYGON; }
	virtual bool is_convex() const override { return false; }

	virtual Variant get_data() const override;
	virtual void set_data(const Variant &p_data) override;

	virtual float get_margin() const override { return 0.0f; }
	virtual void set_margin(float p_margin) override {}

	virtual AABB get_aabb() const override { return aabb; }

	String to_string() const;
};
#endif
