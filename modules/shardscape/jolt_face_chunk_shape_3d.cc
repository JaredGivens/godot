
#include "jolt_chunk_shape_3d.h"
#include "FaceChunkShape.h"

#include <jolt_project_settings.h>
#include <misc/jolt_type_conversions.h>


namespace {

bool _is_vertex_hole(const JPH::VertexList &p_vertices, int p_index) {
	const float height = p_vertices[(size_t)p_index].y;
	return height == FLT_MAX || Math::is_nan(height);
}

bool _is_triangle_hole(const JPH::VertexList &p_vertices, int p_index0, int p_index1, int p_index2) {
	return _is_vertex_hole(p_vertices, p_index0) || _is_vertex_hole(p_vertices, p_index1) || _is_vertex_hole(p_vertices, p_index2);
}

} // namespace

JPH::ShapeRefC JoltFaceChunkShape3D::_build() const {
	ERR_FAIL_COND_V_MSG(faces.is_null(), nullptr, vformat("Failed to build Jolt Physics chunk shape. faces is null"));;

	JPH::FaceChunkShapeSettings shape_settings(faces);
	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();
	return JoltShape3D::with_double_sided(shape_result.Get(), true);
}

Variant JoltFaceChunkShape3D::get_data() const {
	Dictionary data;
	data["faces"] = faces;
	return data;
}

void JoltFaceChunkShape3D::set_data(const Variant &p_data) {
	ERR_FAIL_COND(p_data.get_type() != Variant::DICTIONARY);

	const Dictionary data = p_data;

	const Variant maybe_faces = data.get("faces", Variant());

	ERR_FAIL_COND(maybe_faces.get_type() != Variant::OBJECT);
	faces = maybe_faces;

	destroy();
}

String JoltFaceChunkShape3D::to_string() const {
	return vformat("{faces_count=%d}", 32 * 32 * 32 * 3);
}
