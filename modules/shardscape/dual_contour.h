#ifndef SSK_CHUNK_DUAL_CONTOUR_H
#define SSK_CHUNK_DUAL_CONTOUR_H
#include "core/math/vector3.h"
#include "core/math/vector3i.h"
#include "geometry.h"
#include "shardscape.h"
namespace ssk {
class DualContour {
	static constexpr int32_t kMaxSamples = 12;
	Vector3i _cellv;
	Vector3 _normals[kMaxSamples];
	Vector3 _positions[kMaxSamples];
	int32_t _num_samples;

public:
	Vector3i _cell;
	void add_bias();
	void boundries(Chunk *);
	void compute_faces(Chunk *, Faces *);
	void compute_vertex(Faces *);
	//void AddDebug(List<Vector3> vertBuf);
};
} //namespace ssk
#endif
