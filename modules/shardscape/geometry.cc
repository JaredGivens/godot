#include "geometry.h"
#include "core/os/memory.h"
namespace SSK {
Geometry::Geometry(RID scenario, RID space, RID navMap) {
	_faces = memnew(Faces);
	_navigation = memnew(Navigation(navMap));
	_rendering = memnew(Rendering(scenario));
	_collider = memnew(Collider(space));
	//_boundtryVertBufs.Resize((int32_t)Mesh.ArrayType.Max);
	//_normalVertBufs.Resize((int32_t)Mesh.ArrayType.Max);
	_body = PhysicsServer3D::get_singleton()->body_create();
	PhysicsServer3D::get_singleton()->body_set_space(_body, space);
	_body.CollisionMask = 0;
	PhysicsServer3D.BodySetSpace(_body.GetRid(), space);
}
Geometry::~Geometry() {
	PhysicsServer3D::get_singleton()->free(_body);
	memdelete(_faces);
	memdelete(_navigation);
	memdelete(_collider);
	memdelete(_rendering);
}
//Cell GetCell(Vector3I dcell) {
//return (Cell)_cacheMap[Glob.ModFlat2(Skey(Gkey), Cache.MapDimLen)]
//.Durable.Cells[Glob.Flat(dcell, DimLen)];
//}
void Geometry::add_vertices(Chunk cache) {
	for (int32_t i = 0; i < Chunk::kDimLen - 1; ++i) {
		for (int32_t j = 0; j < Chunk::kDimLen - 1; ++j) {
			for (int32_t k = 0; k < Chunk::kDimLen - 1; ++k) {
				auto dcell = new Vector3i(i, j, k);
				auto res = _dual_contour.EvaluateCell(cache, dcell);
				//if (Options.HasFlag(DisplayOptions.Normals)) {
				//}
				if (res) {
					_vert_flats.Add(i * Chunk::kDimLen2 + j * Chunk::kDimLen + k);
					_cellVerts.Add(_dualContour.QEF());
					_cellNorms.Add(_dualContour.Norm());
					_dualContour.AddDebug(normVerts);
				}
			}
		}
	}
	//if (Options.HasFlag(DisplayOptions.Normals) && verts.Count != 0) {
	_normalVertBufs[int32_t(Mesh.ArrayType.Vertex)] = normVerts.ToArray();
	//}
}
void Geometry::rebuild(Chunk chunk) {
	_tsf = new Transform3D(
			Basis.FromScale(Vector3.One * Scale),
			((Vector3)chunk.ChunkI * Size - Vector3.One * 2) * Scale);
	var color = (Vector3)chunk.ChunkI % 4 / 4;
	_vertFlats.Clear();
	_cellVerts.Clear();
	_cellNorms.Clear();
	AddVertices(chunk);
	if (_vertFlats.Count == 0) {
		Disable();
		return;
	}
	StitchQuads(chunk);
	Update(chunk);
}
void Geometry::update(Chunk chunk) {
	if (!_stitched) {
		Disable();
		return;
	}
	_vertBufs[(Int32)Mesh.ArrayType.Normal] = _norms.ToArray();
	_vertBufs[(Int32)Mesh.ArrayType.Vertex] = _verts.ToArray();
	_vertBufs[(Int32)Mesh.ArrayType.TexUV] = _uvs.ToArray();
	_rendering.Update(chunk, _vertBufs);
	_collider.Update(chunk, _rendering.ArrMesh);
	_navigation.Update(chunk, _rendering.ArrMesh);
}
public
void Dispose() {
	_rendering.Dispose();
	_navigation.Dispose();
	_collider.Dispose();
}
} //namespace SSK
