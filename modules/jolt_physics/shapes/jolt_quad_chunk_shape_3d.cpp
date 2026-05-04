#include "core/variant/variant.h"
#include "jolt_quad_chunk_shape_3d.h"

#include "../jolt_project_settings.h"
#include "../misc/jolt_type_conversions.h"

#include <Jolt/Physics/Collision/Shape/QuadChunkShape.h>

JPH::ShapeRefC JoltQuadChunkShape3D::_build() const {
	ERR_FAIL_COND_V_MSG(quad_chunk == nullptr, nullptr, vformat("Failed to build Jolt Physics chunk shape. faces is null"));

	JPH::QuadChunkShapeSettings shape_settings(quad_chunk);
	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();
	return JoltShape3D::with_double_sided(shape_result.Get(), true);
}

Variant JoltQuadChunkShape3D::get_data() const {
	Dictionary data;
	data["quad_chunk"] = PackedInt64Array({static_cast<int64_t>(reinterpret_cast<intptr_t>(quad_chunk))});
	return data;
}

void JoltQuadChunkShape3D::set_data(const Variant &p_data) {
	ERR_FAIL_COND(p_data.get_type() != Variant::PACKED_INT64_ARRAY);

	const Dictionary data = p_data;

	quad_chunk = reinterpret_cast<SSK::QuadChunk16 *>(static_cast<intptr_t>(static_cast<PackedInt64Array>(p_data)[0]));

	destroy();
}

String JoltQuadChunkShape3D::to_string() const {
    return vformat("{faces_count=%d}", quad_chunk == nullptr ? 0 : quad_chunk->get_count());
}
