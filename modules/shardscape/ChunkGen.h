#pragma once

#include <core/object/ref_counted.h>
#include <core/templates/local_vector.h>
#include <core/math/vector3i.h>
#include <core/string/ustring.h>
#include <core/variant/variant.h>
#include <cstdint>

namespace SSK {

// Compute core of Env/Gen.cs: the FastNoise-driven height/pattern loops, run over a pinned
// ChunkData (Cells + PaletteBids + PaletteCounts) so nothing is copied across the boundary.
// Config (layout, ecology/topology tables, pattern noises, block ids) is pushed in from C# once via
// the set_*/add_* setters; map generation, prefab stamping, and JSON loading stay in C#.
class ChunkGen : public RefCounted {
    GDCLASS(ChunkGen, RefCounted);

public:
    static constexpr int32_t kBandCount = 5;
    static constexpr int32_t kMaxLayerTypes = 4;
    static constexpr int32_t kMaxPatterns = 8;
    static constexpr int32_t kDimLen = 33;        // ChunkData.DimLen
    static constexpr int32_t kDimLen3 = kDimLen * kDimLen * kDimLen;
    static constexpr int32_t kNPad = 2;
    static constexpr int32_t kNDimLen = kDimLen + 2 * kNPad;
    static constexpr int32_t kNDimLen2 = kNDimLen * kNDimLen;
    static constexpr int32_t kNDimLen3 = kNDimLen2 * kNDimLen;
    static constexpr int32_t kChunkOffset = 30;   // Gen.ChunkOffset
    static constexpr int32_t kSkirt = 1;          // ChunkData.SkirtOffset
    static constexpr float kNoiseFreq = 0.005f;
    static constexpr int32_t kCellMaxDist = 4;    // Glob.CellMaxDist
    static constexpr int32_t kBlockDist = kCellMaxDist / 2;
    static constexpr int32_t kPaletteLen = (1 << 13) - 1;
    static constexpr int32_t kCidMask = 0x1FFF;
    static constexpr int32_t kDistMask = 0x7;
    static constexpr int32_t kDistBits = 3;
    static constexpr int32_t kDistXShift = 17;
    static constexpr int32_t kFieldShift = kDistXShift + kDistBits * 3;
    static constexpr int32_t kFieldMask = 0x3F;
    static constexpr int32_t kFieldMax = 2 * kCellMaxDist;

    // Ecology.FeatureUsage
    enum Usage {
        USAGE_CARVE = 0,
        USAGE_REPLACE = 1,
        USAGE_GEODE = 2,
    };

    // Width of a geode's block lining, in noise units above the feature's threshold.
    // ponytail: one global width; move onto Feat if an ecology ever needs its own.
    static constexpr float kGeodeShell = 0.1f;

    static constexpr float kBandDepths[3] = { 4.0f, 16.0f, 80.0f };

    ChunkGen();
    ~ChunkGen();

    void set_layout(int32_t layer_chunks, int32_t layer_types, int32_t surface_local_y,
            float carve_amp, int32_t magma_local_y);
    void set_fixed_noise(String palette_biome, String topology_biome, String topology_mask, String topology_base);
    void set_pattern_noise(int32_t index, String encoded, float frequency);
    void set_road(float mask_threshold, float depth);
    void add_topology(int32_t layer, float weight, String encoded_detail);
    int32_t add_ecology(int32_t layer, int32_t band, float weight, const PackedInt32Array &block_bids,
            const PackedFloat32Array &block_depths, int32_t road_bid);
    void add_ecology_feature(int32_t layer, int32_t band, int32_t eco_index, int32_t pattern,
            int32_t usage, int32_t block_bid, float threshold);

    void generate(int64_t cells_addr, int64_t palette_bids_addr, int64_t palette_counts_addr,
            int64_t noise3_addr, int64_t flatten_addr, Vector3i chunk_i, int32_t seed) const;
    // Exact world-space surface Y of a column, without generating its chunks.
    int32_t surface_height(int32_t world_x, int32_t world_z, int32_t level, int32_t seed,
            float flatten, int32_t flatten_y) const;
    int32_t surface_ecology(int32_t world_x, int32_t world_z, int32_t level, int32_t seed) const;

protected:
    static void _bind_methods();

private:
    struct Topo {
        float weight = 0.0f;
        void *detail_node = nullptr;
    };
    struct Feat {
        int32_t pattern = 0;
        int32_t usage = USAGE_REPLACE;
        int32_t block_bid = 0;
        float threshold = 0.0f;
    };
    struct Eco {
        float weight = 0.0f;
        int32_t road_bid = 0;
        LocalVector<int32_t> block_bids;
        LocalVector<float> block_depths;
        LocalVector<Feat> feats;
    };
    struct Pattern {
        void *node = nullptr;
        float frequency = kNoiseFreq;
    };

    struct Layout {
        int32_t layer_chunks = 16;
        int32_t layer_blocks = 16 * kChunkOffset;
        int32_t layer_types = 1;
        int32_t surface_local_y = 300;
        float carve_amp = 30.0f;
        int32_t magma_local_y = 30;
    };
    Layout layout_;

    void *biome_node_ = nullptr;        // 3D: picks the ecology (blocks + patterns) per cell
    void *topo_biome_node_ = nullptr;   // 2D: picks the topology (detail noise) per column
    void *mask_node_ = nullptr;         // 2D: how much topology detail a column gets
    void *base_node_ = nullptr;         // 2D: broad continental elevation
    Pattern patterns_[kMaxPatterns] = {};
    float road_threshold_ = 0.0f;
    float road_depth_ = 0.0f;

    // Per-instance scratch, sized once in the ctor and refilled per chunk. One ChunkGen lives per
    // load worker (not per chunk), so these are never touched concurrently.
    mutable LocalVector<float> base_;         // kNDimLen2
    mutable LocalVector<float> mask_;         // kNDimLen2
    mutable LocalVector<float> topo_biome_;   // kNDimLen2
    mutable LocalVector<float> detail_;       // topology_count * kNDimLen2
    mutable LocalVector<float> height_;       // kNDimLen2, layer-local blocks
    mutable LocalVector<float> biome_n_;      // kNDimLen3
    mutable LocalVector<uint8_t> solid_;
    mutable LocalVector<float> pattern_[kMaxPatterns]; // kNDimLen3 each, generated on demand

    LocalVector<Topo> topologies_[kMaxLayerTypes];
    float topology_weight_sum_[kMaxLayerTypes] = {};
    LocalVector<Eco> bands_[kMaxLayerTypes][kBandCount];
    float band_weight_sums_[kMaxLayerTypes][kBandCount] = {};
    uint32_t band_pattern_mask_[kMaxLayerTypes][kBandCount] = {};

    void gen_height(Vector3i pos, int32_t layer_type, int32_t seed, int32_t layer_y0, const float *flatten,
            float &min_h, float &max_h) const;
    void gen_patterns(Vector3i pos, uint32_t needed, int32_t seed) const;
    void gen_cells(int32_t *cells, uint16_t *pbids, uint16_t *pcounts, float *noise3,
            int32_t local_y0, int32_t layer_type, uint32_t patterns) const;
    void gen_dists(int32_t *cells, uint16_t *pbids, uint16_t *pcounts, const float *noise3) const;
    int32_t solid_field(int32_t ni0) const;

    // The one place the surface height is defined; both the per-chunk grid and the single-column
    // probe go through it, so a portal can never be stamped at a Y the terrain disagrees with.
    float column_height(float base, float mask, float detail) const {
        return (float)layout_.surface_local_y
                + base
                + mask * detail;
    }

    static float flattened(float h, float flatten, float local_target) {
        return h + (local_target - h) * flatten;
    }

    float probe_height(int32_t world_x, int32_t world_z, int32_t layer_type, int32_t lseed) const;
    int32_t select_topology(int32_t layer_type, float biome) const;
    int32_t select_ecology(int32_t layer_type, int32_t band, float biome) const;
    int32_t determine_band(float depth, int32_t local_y) const;
    uint32_t needed_patterns(int32_t layer_type, int32_t local_y0, float min_h, float max_h) const;

    static void set_cell_state(int32_t *cells, uint16_t *pbids, uint16_t *pcounts, int32_t idx, int32_t bid, Vector3i dist, int32_t field);

#ifdef DEBUG_ENABLED
    mutable uint32_t warned_ = 0;
    void check_range(int32_t id, const char *name, const float *v, int32_t n) const;
#endif
};

} // namespace SSK
