#pragma once
#include <core/object/object.h>
#include <core/variant/binder_common.h>

namespace SSK {
class Global : public Object {
	GDCLASS(Global, Object);
protected:
    static void _bind_methods();
public:
	// Mirror of Env/chunk/MaskResource.cs flags for ease of use in C++
    enum Property {
        PROPERTY_SHARP        = 1 << 8,
        PROPERTY_LIQUID       = 1 << 10,
        PROPERTY_GAS          = 1 << 11,
    };
	LocalVector<int32_t> block_props;
	RID buffer_;
	RID get_block_props_buffer() const;
	void set_block_props(Vector<int32_t> block_properties);
	int32_t get_block_props(int32_t) const;
	static Global *singleton;
	static Global *get_singleton();
};
}
VARIANT_ENUM_CAST(SSK::Global::Property);
