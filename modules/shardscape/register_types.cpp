/* register_types.cpp */

#include "register_types.h"

#include "core/object/class_db.h"
#include "QuadChunk.h"
#include "FastNoise.h"

void initialize_shardscape_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
	ClassDB::register_class<SSK::QuadChunk>();
	ClassDB::register_class<SSK::FastNoise>();
}

void uninitialize_shardscape_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
	// Nothing to do here in this example.
}
