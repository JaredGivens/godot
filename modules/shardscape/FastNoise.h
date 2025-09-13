#ifndef SSK_FAST_NOISE_H
#define SSK_FAST_NOISE_H
#include <core/object/ref_counted.h>
#include <FastNoise/FastNoise_C.h>
#include <core/string/ustring.h>

namespace SSK {
class FastNoise : public RefCounted {
    GDCLASS(FastNoise, RefCounted);

public:
	// Node creation and management
    void from_encoded_node_tree(String encodedString) {
        node_ = fnNewFromEncodedNodeTree(encodedString.ascii().ptr(), 0);
    }

    void delete_node_ref() {
        fnDeleteNodeRef(node_);
    }

    // Node information
    unsigned get_simd_level() {
        return fnGetSIMDLevel(node_);
    }

    int get_metadata_id() {
        return fnGetMetadataID(node_);
    }

    // Uniform grid generation
    Vector<float> gen_uniform_grid_2d( int xStart, int yStart, int xSize, int ySize, float frequency, int seed) {
		noiseOut_.resize(xSize * ySize);
        fnGenUniformGrid2D(node_, noiseOut_.ptr(), xStart, yStart, xSize, ySize, frequency, seed, nullptr);
		return (Vector<float>)noiseOut_;
    }

    Vector<float> gen_uniform_grid_3d( int xStart, int yStart, int zStart, int xSize, int ySize, int zSize, float frequency, int seed) {
		noiseOut_.resize(xSize * ySize * zSize);
        fnGenUniformGrid3D(node_, noiseOut_.ptr(), xStart, yStart, zStart, xSize, ySize, zSize, frequency, seed, nullptr);
		return (Vector<float>)noiseOut_;
    }

    Vector<float> gen_uniform_grid_4d( int xStart, int yStart, int zStart, int wStart, int xSize, int ySize, int zSize, int wSize, float frequency, int seed) {
		noiseOut_.resize(xSize * ySize * zSize * wSize);
        fnGenUniformGrid4D(node_, noiseOut_.ptr(), xStart, yStart, zStart, wStart, xSize, ySize, zSize, wSize, frequency, seed, nullptr);
		return (Vector<float>)noiseOut_;
    }

     // Position array generation
    Vector<float> gen_position_array_2d( int count, const PackedFloat32Array &xPosArray, const PackedFloat32Array &yPosArray, float xOffset, float yOffset, int seed) {
		noiseOut_.resize(count);
        fnGenPositionArray2D(node_, noiseOut_.ptr(), count, xPosArray.ptr(), yPosArray.ptr(), xOffset, yOffset, seed, nullptr);
		return (Vector<float>)noiseOut_;
    }

    Vector<float> gen_position_array_3d( int count, const PackedFloat32Array &xPosArray, const PackedFloat32Array &yPosArray, const PackedFloat32Array &zPosArray, float xOffset, float yOffset, float zOffset, int seed) {
		noiseOut_.resize(count);
        fnGenPositionArray3D(node_, noiseOut_.ptr(), count, xPosArray.ptr(), yPosArray.ptr(), zPosArray.ptr(), xOffset, yOffset, zOffset, seed, nullptr);
		return (Vector<float>)noiseOut_;
    }

    Vector<float> gen_position_array_4d( int count, const PackedFloat32Array &xPosArray, const PackedFloat32Array &yPosArray, const PackedFloat32Array &zPosArray, const PackedFloat32Array &wPosArray, float xOffset, float yOffset, float zOffset, float wOffset, int seed) {
		noiseOut_.resize(count);
        fnGenPositionArray4D(node_, noiseOut_.ptr(), count, xPosArray.ptr(), yPosArray.ptr(), zPosArray.ptr(), wPosArray.ptr(), xOffset, yOffset, zOffset, wOffset, seed, nullptr);
		return (Vector<float>)noiseOut_;
    }

     // Tileable generation
    Vector<float> gen_tileable_2d( int xSize, int ySize, float frequency, int seed) {
		noiseOut_.resize(xSize * ySize);
        fnGenTileable2D(node_, noiseOut_.ptr(), xSize, ySize, frequency, seed, nullptr);
		return (Vector<float>)noiseOut_;
    }
 // Single value generation
    float gen_single_2d( float x, float y, int seed) {
        return fnGenSingle2D(node_, x, y, seed);
    }

    float gen_single_3d( float x, float y, float z, int seed) {
        return fnGenSingle3D(node_, x, y, z, seed);
    }

    float gen_single_4d( float x, float y, float z, float w, int seed) {
        return fnGenSingle4D(node_, x, y, z, w, seed);
    }

    // Metadata functions
    int get_metadata_count() {
        return fnGetMetadataCount();
    }

    String get_metadata_name(int id) {
		return String(fnGetMetadataName(id));
    }

	void from_metadata(int id, unsigned simdLevel) {
        node_ = fnNewFromMetadata(id, simdLevel);
    }

    // Metadata variable functions
    int get_metadata_variable_count(int id) {
        return fnGetMetadataVariableCount(id);
    }

    String get_metadata_variable_name(int id, int variableIndex) {
		return String(fnGetMetadataVariableName(id, variableIndex));
    }

    int get_metadata_variable_type(int id, int variableIndex) {
        return fnGetMetadataVariableType(id, variableIndex);
    }

    int get_metadata_variable_dimension_idx(int id, int variableIndex) {
        return fnGetMetadataVariableDimensionIdx(id, variableIndex);
    }

    int get_metadata_enum_count(int id, int variableIndex) {
        return fnGetMetadataEnumCount(id, variableIndex);
    }

    String get_metadata_enum_name(int id, int variableIndex, int enumIndex) {
        return String(fnGetMetadataEnumName(id, variableIndex, enumIndex));
    }

    bool set_variable_float( int variableIndex, float value) {
        return fnSetVariableFloat(node_, variableIndex, value);
    }

    bool set_variable_int_enum( int variableIndex, int value) {
        return fnSetVariableIntEnum(node_, variableIndex, value);
    }

    // Node lookup functions
    int get_metadata_node_lookup_count(int id) {
        return fnGetMetadataNodeLookupCount(id);
    }

    String get_metadata_node_lookup_name(int id, int nodeLookupIndex) {
        return String(fnGetMetadataNodeLookupName(id, nodeLookupIndex));
    }

    int get_metadata_node_lookup_dimension_idx(int id, int nodeLookupIndex) {
        return fnGetMetadataNodeLookupDimensionIdx(id, nodeLookupIndex);
    }

    // Hybrid functions
    int get_metadata_hybrid_count(int id) {
        return fnGetMetadataHybridCount(id);
    }

    String get_metadata_hybrid_name(int id, int hybridIndex) {
        return String(fnGetMetadataHybridName(id, hybridIndex));
    }

    int get_metadata_hybrid_dimension_idx(int id, int hybridIndex) {
        return fnGetMetadataHybridDimensionIdx(id, hybridIndex);
    }

    bool set_hybrid_float( int hybridIndex, float value) {
        return fnSetHybridFloat(node_, hybridIndex, value);
    }

protected:
// Implementation of _bind_methods
static void _bind_methods() {
	// Node creation and management
    ClassDB::bind_method(D_METHOD("from_encoded_node_tree", "encodedString"), &FastNoise::from_encoded_node_tree);
    ClassDB::bind_method(D_METHOD("delete_node_ref"), &FastNoise::delete_node_ref);

    // Node information
    ClassDB::bind_method(D_METHOD("get_simd_level"), &FastNoise::get_simd_level);
    ClassDB::bind_method(D_METHOD("get_metadata_id"), &FastNoise::get_metadata_id);

    // Uniform grid generation
    ClassDB::bind_method(D_METHOD("gen_uniform_grid_2d", "xStart", "yStart", "xSize", "ySize", "frequency", "seed"),
        &FastNoise::gen_uniform_grid_2d);
    ClassDB::bind_method(D_METHOD("gen_uniform_grid_3d", "xStart", "yStart", "zStart", "xSize", "ySize", "zSize", "frequency", "seed"),
        &FastNoise::gen_uniform_grid_3d);
    ClassDB::bind_method(D_METHOD("gen_uniform_grid_4d", "xStart", "yStart", "zStart", "wStart", "xSize", "ySize", "zSize", "wSize", "frequency", "seed"),
        &FastNoise::gen_uniform_grid_4d);

    // Position array generation
    ClassDB::bind_method(D_METHOD("gen_position_array_2d", "count", "xPosArray", "yPosArray", "xOffset", "yOffset", "seed"),
        &FastNoise::gen_position_array_2d);
    ClassDB::bind_method(D_METHOD("gen_position_array_3d", "count", "xPosArray", "yPosArray", "zPosArray", "xOffset", "yOffset", "zOffset", "seed"),
        &FastNoise::gen_position_array_3d);
    ClassDB::bind_method(D_METHOD("gen_position_array_4d", "count", "xPosArray", "yPosArray", "zPosArray", "wPosArray", "xOffset", "yOffset", "zOffset", "wOffset", "seed"),
        &FastNoise::gen_position_array_4d);

    // Tileable generation
    ClassDB::bind_method(D_METHOD("gen_tileable_2d", "xSize", "ySize", "frequency", "seed"),
        &FastNoise::gen_tileable_2d);

    // Single value generation
    ClassDB::bind_method(D_METHOD("gen_single_2d", "x", "y", "seed"), &FastNoise::gen_single_2d);
    ClassDB::bind_method(D_METHOD("gen_single_3d", "x", "y", "z", "seed"), &FastNoise::gen_single_3d);
    ClassDB::bind_method(D_METHOD("gen_single_4d", "x", "y", "z", "w", "seed"), &FastNoise::gen_single_4d);

    // Metadata functions
    ClassDB::bind_method(D_METHOD("get_metadata_count"), &FastNoise::get_metadata_count);
    ClassDB::bind_method(D_METHOD("get_metadata_name", "id"), &FastNoise::get_metadata_name);
    ClassDB::bind_method(D_METHOD("from_metadata", "id", "simdLevel"), &FastNoise::from_metadata);

    // Metadata variable functions
    ClassDB::bind_method(D_METHOD("get_metadata_variable_count", "id"), &FastNoise::get_metadata_variable_count);
    ClassDB::bind_method(D_METHOD("get_metadata_variable_name", "id", "variableIndex"), &FastNoise::get_metadata_variable_name);
    ClassDB::bind_method(D_METHOD("get_metadata_variable_type", "id", "variableIndex"), &FastNoise::get_metadata_variable_type);
    ClassDB::bind_method(D_METHOD("get_metadata_variable_dimension_idx", "id", "variableIndex"), &FastNoise::get_metadata_variable_dimension_idx);
    ClassDB::bind_method(D_METHOD("get_metadata_enum_count", "id", "variableIndex"), &FastNoise::get_metadata_enum_count);
    ClassDB::bind_method(D_METHOD("get_metadata_enum_name", "id", "variableIndex", "enumIndex"), &FastNoise::get_metadata_enum_name);
    ClassDB::bind_method(D_METHOD("set_variable_float", "variableIndex", "value"), &FastNoise::set_variable_float);
    ClassDB::bind_method(D_METHOD("set_variable_int_enum", "variableIndex", "value"), &FastNoise::set_variable_int_enum);

    // Node lookup functions
    ClassDB::bind_method(D_METHOD("get_metadata_node_lookup_count", "id"), &FastNoise::get_metadata_node_lookup_count);
    ClassDB::bind_method(D_METHOD("get_metadata_node_lookup_name", "id", "nodeLookupIndex"), &FastNoise::get_metadata_node_lookup_name);
    ClassDB::bind_method(D_METHOD("get_metadata_node_lookup_dimension_idx", "id", "nodeLookupIndex"), &FastNoise::get_metadata_node_lookup_dimension_idx);

    // Hybrid functions
    ClassDB::bind_method(D_METHOD("get_metadata_hybrid_count", "id"), &FastNoise::get_metadata_hybrid_count);
    ClassDB::bind_method(D_METHOD("get_metadata_hybrid_name", "id", "hybridIndex"), &FastNoise::get_metadata_hybrid_name);
    ClassDB::bind_method(D_METHOD("get_metadata_hybrid_dimension_idx", "id", "hybridIndex"), &FastNoise::get_metadata_hybrid_dimension_idx);
    ClassDB::bind_method(D_METHOD("set_hybrid_float", "hybridIndex", "value"), &FastNoise::set_hybrid_float);
}
private:
	void *node_;
	LocalVector<float> noiseOut_;
};
}
#endif
