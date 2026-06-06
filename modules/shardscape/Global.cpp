#include "Global.h"
#include "common/zstd_internal.h"
#include <core/object/class_db.h>
#include <servers/rendering/rendering_device.h>
namespace SSK {
void Global::set_block_props(Vector<int32_t> block_properties) {
	if (buffer_.is_valid()) {
		RenderingDevice::get_singleton()->free_rid(buffer_);
	}
	int32_t size = block_properties.size() * 4;
	Span<uint8_t> byte_span(reinterpret_cast<const uint8_t *>(block_properties.ptr()), size);
	buffer_ = RenderingDevice::get_singleton()->storage_buffer_create(size, byte_span);
	block_props = block_properties;
}
int32_t Global::get_block_props(int32_t index) const { return block_props[index]; }
Global *Global::singleton = nullptr;

Global *Global::get_singleton() {
	return singleton;
}

RID Global::get_block_props_buffer() const { return buffer_; }
void Global::_bind_methods() {
    BIND_ENUM_CONSTANT(PROPERTY_SHARP);
    BIND_ENUM_CONSTANT(PROPERTY_LIQUID);
    BIND_ENUM_CONSTANT(PROPERTY_GAS);
    ClassDB::bind_method(D_METHOD("set_block_props", "block_properties"), &Global::set_block_props);
    ClassDB::bind_method(D_METHOD("get_block_props", "bid"), &Global::get_block_props);
    ClassDB::bind_method(D_METHOD("get_block_props_buffer"), &Global::get_block_props_buffer);
}
}
