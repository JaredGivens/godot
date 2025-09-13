#ifndef SSK_CELLS_H
#define SSK_CELLS_H

#include "core/math/vector3.h"
#include "core/object/ref_counted.h"
namespace SSK {
struct Cell {
	union {
		int32_t _int;
		struct {
			int8_t dist;
			int8_t id;
			int8_t theta;
			int8_t phi;
		};
	};
	void SetNormal(Vector3);
	Vector3 GetNormal();
};
} //namespace SSK
#endif
