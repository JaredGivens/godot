/* register_types.cpp */

#include "register_types.h"

#include "FastNoise.h"
#include "ChunkAtlas.h"
#include "QuadChunk16.h"
#include "Global.h"

#include <core/object/class_db.h>
#include <core/config/engine.h>

void initialize_shardscape_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
	ClassDB::register_class<SSK::QuadChunk16>();
	ClassDB::register_class<SSK::ChunkAtlas>();
	//ClassDB::register_class<SSK::QuadChunk<4>>();
	ClassDB::register_class<SSK::FastNoise>();
	SSK::Global::singleton = memnew(SSK::Global);
	Engine::get_singleton()->add_singleton(Engine::Singleton("Global", SSK::Global::get_singleton()));
	ClassDB::register_class<SSK::Global>();
}

void uninitialize_shardscape_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
	// Nothing to do here in this example.
}
