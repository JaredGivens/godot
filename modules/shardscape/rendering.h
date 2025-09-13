#ifndef SSK_CHUNK_RENDERING
#define SSK_CHUNK_RENDERING
#include "scene/resources/multimesh.h"
namespace SSK {
enum DisplayOptions {
	None = 0,
	Boundries = 1,
	Normals = 2,
	Colors = 4,
}

class Rendering {
	static DisplayOptions Options;
	RID _quadInst;
	RID _quadMulti;
	StandardMaterial3D _debugMat;
	ArrayMesh _debugArrMesh;
	RID _debugInst;
	Rendering(RID scenario);
	//Cell GetCell(Vector3I dcell) {
	//return (Cell)_cacheMap[Glob.ModFlat2(Skey(Gkey), Cache.MapDimLen)]
	//.Durable.Cells[Glob.Flat(dcell, DimLen)];
	//}
	~Rendering();
	void Rendering::update(Cache cache, Godot.Collections.Array vertBufs);
public
	void Rendering::update_grass(Vector3 pos) {
		//if (pos.DistanceTo(Gkey * Size) > _grassRadius) {
		//RenderingServer.InstanceSetVisible(_grassMultiInst, false);
		//return;
		//}
	}
	void AddBoundries(Cache cache) {
		_boundtryVertBufs[(Int32)Mesh.ArrayType.Vertex] = new Vector3[]{
			new Vector3(0, 0, 0) * Geometry.Size,
			new Vector3(1, 0, 0) * Geometry.Size,
			new Vector3(0, 1, 0) * Geometry.Size,
			new Vector3(1, 1, 0) * Geometry.Size,
			new Vector3(0, 0, 1) * Geometry.Size,
			new Vector3(1, 0, 1) * Geometry.Size,
			new Vector3(0, 1, 1) * Geometry.Size,
			new Vector3(1, 1, 1) * Geometry.Size
		};
		_boundtryVertBufs[(Int32)Mesh.ArrayType.Index] = new Int32[]{
			0, 1,
			0, 2,
			0, 4,
			1, 3,
			2, 3,
			4, 5,
			4, 6,
			7, 6,
			7, 5,
			7, 3
		};
		var i = _debugArrMesh.GetSurfaceCount();
		_debugArrMesh.AddSurfaceFromArrays(
				Mesh.PrimitiveType.Lines, _boundtryVertBufs);
		RenderingServer
				.InstanceSetSurfaceOverrideMaterial(_debugInst, i, _debugMat.GetRid());
		RenderingServer.InstanceSetTransform(_debugInst, cache.GetTransform());
		RenderingServer.InstanceSetVisible(_debugInst, true);
	}
public
	void TryAddGrass(Cache cache, Vector3[] verts, Int32 vertsOffset, Vector3 normal, Int32 seed) {
		var a = verts[vertsOffset];
		var b = verts[vertsOffset + 1];
		var c = verts[vertsOffset + 2];
		// Compute two basis vectors for the plane
		Vector3 u = b - a;
		Vector3 v = c - a;

		// Generate random barycentric coordinates
		seed += Glob.Flat((Vector3I)(a * Geometry.Size), Geometry.Size) + Glob.ModFlat2(cache.ChunkI, 256);
		var rng = new Random(seed);

		var r1 = rng.NextSingle();
		var r2 = rng.NextSingle();

		// Ensure the random values sum to less than or equal to 1
		if (r1 + r2 > 1.0f) {
			r1 = 1.0f - r1;
			r2 = 1.0f - r2;
		}

		// Calculate the random point on the plane
		var tangent = Vector3.Up.Cross(normal);
		var pos = a + r1 * u + r2 * v;
		var basis = new Basis(tangent, MathF.Asin(tangent.Length()));
		basis = basis.Rotated(normal, rng.NextSingle() * MathF.Tau);
		basis = basis.Scaled(Vector3.One * ((rng.NextSingle() * 0.5f) + 0.5f));
		//GD.PrintS(pos);
		var tsf = new Transform3D(basis, pos);
		for (Int32 i = 0; i < 3; ++i) {
			for (Int32 j = 0; j < 4; ++j) {
				_grassTsfs.Add(tsf[j][i]);
			}
		}
	}
	Vector3 Normal(Vector3 a, Vector3 b, Vector3 c) {
		return (a - b).Cross(c - b).Normalized();
	}
public
	void Dispose() {
		ArrMesh.ClearSurfaces();
		RenderingServer.FreeRid(_inst);
		RenderingServer.FreeRid(_grassMultiInst);
	}
}
} //namespace SSK
#endif
