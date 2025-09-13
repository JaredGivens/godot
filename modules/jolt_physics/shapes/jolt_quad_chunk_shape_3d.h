#pragma once
#include <modules/shardscape/QuadChunk.h>
#include <modules/jolt_physics/shapes/jolt_shape_3d.h>

class JoltQuadChunkShape3D final : public JoltShape3D {
	const AABB aabb = AABB(Vector3(0, 0, 0), Vector3(32, 32, 32));
	SSK::QuadChunk *quad_chunk = nullptr;
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
