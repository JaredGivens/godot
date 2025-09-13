#include "dual_contour.h"
#include "modules/shardscape/Eigen/Dense"

namespace ssk {
Vector3 DualContour::compute_vertex(Faces *faces) {
	// Map C arrays to Eigen matrices (row-major for contiguous XYZ triples)
	using MatrixX3f_RM = Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>;
	Eigen::Map<const MatrixX3f_RM> A(_normals, _num_samples, 3); // Normals matrix (Nx3)
	Eigen::Map<const MatrixX3f_RM> B(_positions, _num_samples, 3); // Positions matrix (Nx3)

	// Compute b = (A * B).rowwise().sum()
	Eigen::VectorXf b = (A.array() * B.array()).rowwise().sum();

	// Construct normal equations: A^T A x = A^T b
	Eigen::Matrix3f ATA = A.transpose() * A;
	Eigen::Vector3f ATb = A.transpose() * b;

	// Solve using SVD (fallback for ill-conditioned systems)
	Eigen::JacobiSVD<Eigen::Matrix3f> svd(ATA, ComputeFullU | ComputeFullV);
	float cond = svd.singularValues()(0) / svd.singularValues().tail(1)(0);

	Vector3 output_vertex();
	if (cond < 1e6) {
		Vector3f x = svd.solve(ATb);
		output_vertex[0] = x(0);
		output_vertex[1] = x(1);
		output_vertex[2] = x(2);
	} else {
		// Fallback: Average positions
		output_vertex[0] = B.col(0).mean();
		output_vertex[1] = B.col(1).mean();
		output_vertex[2] = B.col(2).mean();
	}
	if (_cellv.x != Geometry::kSize && _cellv.y != Geometry::kSize && _cellv.z != Geometry::kSize) {
		int32_t flat0 = Glob::flat(_cellv, Geometry::kSize) * 3;
		faces->set_face_vert(flat0, 0, output_vertex);
		faces->set_face_vert(flat0 + 1, 0, output_vertex);
		faces->set_face_vert(flat0 + 2, 0, output_vertex);
	}
	if (_cellv.x != 0) {
		auto flatnx = Glob::flat(_cellv - Vector3i(1, 0, 0));
		faces->set_face_vert(flat0 + 1, 3, output_vertex);
		faces->set_face_vert(flat0 + 2, 1, output_vertex);
	}
	if (_cellv.y != 0) {
		auto flatny = Glob::flat(_cellv - Vector3i(1, 0, 0));
		faces->set_face_vert(flat0 + 1, 3, output_vertex);
		faces->set_face_vert(flat0 + 2, 1, output_vertex);
	}
}
void AddBias() {
	var massPoint = _cell + Vector3.One * 0.5f;
	var biasStrength = 0.25f;
	_mids.AddRange(Enumerable.Repeat(massPoint, 3));
	_norms.Add(new Vector3(biasStrength, 0, 0));
	_norms.Add(new Vector3(0, biasStrength, 0));
	_norms.Add(new Vector3(0, 0, biasStrength));
}

void boundries(Chunk chunk) {
	// iterate over edges of cube and compare vert distances
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 2; ++j) {
			for (int k = 0; k < 2; ++k) {
				auto cellv0 = _cellv;
				cell0[(i + 1) % 3] += j;
				cell0[(i + 2) % 3] += k;
				auto c0 = chunk.get_cell(cellv0);
				auto cell1 = cell0;
				cell1[i] += 1;
				auto c1 = chunk.get_cell(cellv1);
				if (c0.Dist == -128 || c1.Dist == -128) {
					continue;
				}
				if (c0.Dist <= 0 && c1.Dist >= 0) {
					Vector3 mid(1, 1, 1);
					mid[i] -= (real_t)c0.Dist / Glob.DistFac;
					_mids.Add(mid);
					var norm = c0.get_normal();
					if (abs(norm[i]) < CMP_EPSILON) {
						norm = Vector3.Zero;
						norm[i] += 1;
					} else {
						norm[i] += 1;
						norm.Normalized();
					}
					_norms.Add(norm);
				} else if (c0.Dist >= 0 && c1.Dist <= 0) {
					Vector3 mid();
					mid[i] += (real_t)c1.Dist / Glob.DistFac;
					_mids.Add(mid);
					var norm = c1.get_normal();
					if (MathF.Abs(norm[i]) < CMP_EPSILON) {
						norm = Vector3.Zero;
						norm[i] = -1;
					} else {
						norm[i] += 1;
						norm.Normalized();
					}
					_norms.Add(norm);
				}
			}
		}
	}
	if (_mids.Count != 0) {
		//var ad = Math.Abs(_getCell(dcell).Dist);
		//if (ad > Glob.DistFac * 4) {
		//for (int i = 0; i < 2; ++i) {
		//for (int j = 0; j < 2; ++j) {
		//for (int k = 0; k < 2; ++k) {
		//var dcell0 = dcell;
		//dcell0 += new Vector3I(i, j, k);
		//GD.PrintErr(dcell, dcell0, _getCell(dcell0).Dist, _getCell(dcell0).GetNormal());
		//}
		//}
		//}
		//}
		//AddBias(dcell);
	}
}

void DualContour::eval_verts(Chunk chunk, Faces *faces) {
	for (int32_t i = 0; i < Geometry::kDimLen - 1; ++i) {
		for (int32_t j = 0; j < Geometry::kDimLen - 1; ++j) {
			for (int32_t k = 0; k < Geometry::kDimLen - 1; ++k) {
				auto cell = new Vector3I(i, j, k);
				auto res = _dualContour.boundries(cell, chunk, faces);
				//if (Options.HasFlag(DisplayOptions.Normals)) {
				//}

				if (res) {
					_vertFlats.Add(i * DimLen2 + j * DimLen + k);
					_cellVerts.Add(_dualContour.QEF());
					_cellNorms.Add(_dualContour.Norm());
					_dualContour.AddDebug(normVerts);
				}
			}
		}
	}
	_cell = cell;
	//_dkey = dkey;
	_mids.Clear();
	_norms.Clear();
	Boundries(cache);
	return _mids.Count != 0;
}
void AddDebug(List<Vector3> vertBuf) {
	for (int i = 0; i < _mids.Count; ++i) {
		vertBuf.Add(_cell);
		vertBuf.Add(_cell + Vector3I.Right);
		vertBuf.Add(_cell);
		vertBuf.Add(_cell + Vector3I.Left);
		vertBuf.Add(_cell);
		vertBuf.Add(_cell + Vector3I.Up);
		vertBuf.Add(_cell + new Vector3(0, 1, 1));
		vertBuf.Add(_cell + new Vector3(1, 1, 1));
		vertBuf.Add(_cell + new Vector3(1, 0, 1));
		vertBuf.Add(_cell + new Vector3(1, 1, 1));
		vertBuf.Add(_cell + new Vector3(1, 1, 0));
		vertBuf.Add(_cell + new Vector3(1, 1, 1));

		vertBuf.Add(_mids[i]);
		vertBuf.Add(_mids[i] + _norms[i]);
	}
}
} //namespace ssk
#endif
