#include "ChunkGen.h"

#include <core/math/math_funcs.h>
#include <FastNoise/FastNoise_C.h>

namespace SSK {

static inline int32_t world_to_dist(double world_dist) {
    double v = Math::round((double)ChunkGen::kCellMaxDist * world_dist);
    if (v < 0) {
        v = 0;
    }
    if (v > ChunkGen::kCellMaxDist) {
        v = ChunkGen::kCellMaxDist;
    }
    return (int32_t)v;
}

struct FieldOffset {
    int32_t dn;
    int32_t d;
};

static constexpr int32_t kFieldBand = ChunkGen::kFieldMax / ChunkGen::kCellMaxDist;
static constexpr int32_t kFieldOffsetCount = (2 * kFieldBand + 1) * (2 * kFieldBand + 1) * (2 * kFieldBand + 1) - 1;

static const FieldOffset *field_offsets() {
    static FieldOffset table[kFieldOffsetCount];
    static bool built = [] {
        int32_t n = 0;
        for (int32_t z = -kFieldBand; z <= kFieldBand; ++z) {
            for (int32_t y = -kFieldBand; y <= kFieldBand; ++y) {
                for (int32_t x = -kFieldBand; x <= kFieldBand; ++x) {
                    if (x == 0 && y == 0 && z == 0) {
                        continue;
                    }
                    double qx = MAX(Math::abs(x) - 0.5, 0.0);
                    double qy = MAX(Math::abs(y) - 0.5, 0.0);
                    double qz = MAX(Math::abs(z) - 0.5, 0.0);
                    double v = Math::round(ChunkGen::kCellMaxDist * Math::sqrt(qx * qx + qy * qy + qz * qz));
                    table[n++] = { x + y * ChunkGen::kNDimLen + z * ChunkGen::kNDimLen2,
                        (int32_t)MIN(v, (double)ChunkGen::kFieldMax) };
                }
            }
        }
        for (int32_t i = 1; i < n; ++i) {
            FieldOffset f = table[i];
            int32_t j = i;
            for (; j > 0 && table[j - 1].d > f.d; --j) {
                table[j] = table[j - 1];
            }
            table[j] = f;
        }
        return true;
    }();
    (void)built;
    return table;
}

static inline int32_t pos_mod(int32_t a, int32_t b) {
    int32_t r = a % b;
    return r < 0 ? r + b : r;
}

static inline int32_t floor_div(int32_t a, int32_t b) {
    int32_t q = a / b;
    return (a % b != 0 && ((a < 0) != (b < 0))) ? q - 1 : q;
}

// A noise index runs x fastest then y then z; its column is the same index with y dropped.
static inline int32_t xz_index(int32_t i) {
    return (i % ChunkGen::kNDimLen) + (i / ChunkGen::kNDimLen2) * ChunkGen::kNDimLen;
}

static inline int32_t layer_seed(int32_t seed, int32_t level) {
    return (int32_t)((uint32_t)seed + (uint32_t)level * 0x9E3779B1u);
}

// Offset so that reusing one tree for both biome fields still yields independent fields — the
// topology a column gets must not be a function of the ecology its cells get. Every sampler of
// topo_biome_node_ must go through this or the probe and the terrain pick different topologies.
static inline int32_t topo_biome_seed(int32_t lseed) {
    return lseed ^ 0x5F356495;
}

// Noise trees output -1..1; every selection weight is authored as a positive share.
static inline float unit_to_ratio(float v) {
    return v * 0.5f + 0.5f;
}

ChunkGen::ChunkGen() {
    base_.resize(kNDimLen2);
    mask_.resize(kNDimLen2);
    topo_biome_.resize(kNDimLen2);
    height_.resize(kNDimLen2);
    biome_n_.resize(kNDimLen3);
    solid_.resize(kNDimLen3);
}

// Every fnNewFromEncodedNodeTree takes a SmartNode reference; without these releases the FastNoise
// pool asserts on a non-empty usedSlots at static destruction (SmartNode.cpp:64).
ChunkGen::~ChunkGen() {
    void *fixed[] = { biome_node_, topo_biome_node_, mask_node_, base_node_ };
    for (void *node : fixed) {
        if (node != nullptr) {
            fnDeleteNodeRef(node);
        }
    }
    for (const Pattern &p : patterns_) {
        if (p.node != nullptr) {
            fnDeleteNodeRef(p.node);
        }
    }
    for (int32_t l = 0; l < kMaxLayerTypes; ++l) {
        for (const Topo &t : topologies_[l]) {
            if (t.detail_node != nullptr) {
                fnDeleteNodeRef(t.detail_node);
            }
        }
    }
}

void ChunkGen::set_layout(int32_t layer_chunks, int32_t layer_types, int32_t surface_local_y,
        float carve_amp, int32_t magma_local_y) {
    layout_.layer_chunks = MAX(1, layer_chunks);
    layout_.layer_blocks = layout_.layer_chunks * kChunkOffset;
    layout_.layer_types = CLAMP(layer_types, 1, kMaxLayerTypes);
    layout_.surface_local_y = surface_local_y;
    layout_.carve_amp = carve_amp;
    layout_.magma_local_y = magma_local_y;
}

void ChunkGen::set_fixed_noise(String palette_biome, String topology_biome, String topology_mask, String topology_base) {
    biome_node_ = fnNewFromEncodedNodeTree(palette_biome.ascii().ptr(), 0);
    topo_biome_node_ = fnNewFromEncodedNodeTree(topology_biome.ascii().ptr(), 0);
    mask_node_ = fnNewFromEncodedNodeTree(topology_mask.ascii().ptr(), 0);
    base_node_ = fnNewFromEncodedNodeTree(topology_base.ascii().ptr(), 0);
}

void ChunkGen::set_pattern_noise(int32_t index, String encoded, float frequency) {
    ERR_FAIL_INDEX(index, kMaxPatterns);
    patterns_[index].node = fnNewFromEncodedNodeTree(encoded.ascii().ptr(), 0);
    patterns_[index].frequency = frequency;
    pattern_[index].resize(kNDimLen3);
}

void ChunkGen::set_road(float mask_threshold, float depth) {
    road_threshold_ = mask_threshold;
    road_depth_ = depth;
}

void ChunkGen::add_topology(int32_t layer, float weight, String encoded_detail) {
    ERR_FAIL_INDEX(layer, kMaxLayerTypes);
    Topo t;
    t.weight = weight;
    t.detail_node = fnNewFromEncodedNodeTree(encoded_detail.ascii().ptr(), 0);
    topologies_[layer].push_back(t);
    topology_weight_sum_[layer] += weight;
    int32_t widest = 0;
    for (int32_t l = 0; l < kMaxLayerTypes; ++l) {
        widest = MAX(widest, (int32_t)topologies_[l].size());
    }
    detail_.resize(widest * kNDimLen2);
}

int32_t ChunkGen::add_ecology(int32_t layer, int32_t band, float weight, const PackedInt32Array &block_bids,
        const PackedFloat32Array &block_depths, int32_t road_bid) {
    ERR_FAIL_INDEX_V(layer, kMaxLayerTypes, -1);
    ERR_FAIL_INDEX_V(band, kBandCount, -1);
    Eco e;
    e.weight = weight;
    e.road_bid = road_bid;
    e.block_bids.resize(block_bids.size());
    for (int32_t i = 0; i < block_bids.size(); ++i) {
        e.block_bids[i] = block_bids[i];
    }
    e.block_depths.resize(block_depths.size());
    for (int32_t i = 0; i < block_depths.size(); ++i) {
        e.block_depths[i] = block_depths[i];
    }
    bands_[layer][band].push_back(e);
    band_weight_sums_[layer][band] += weight;
    return (int32_t)bands_[layer][band].size() - 1;
}

void ChunkGen::add_ecology_feature(int32_t layer, int32_t band, int32_t eco_index, int32_t pattern,
        int32_t usage, int32_t block_bid, float threshold) {
    ERR_FAIL_INDEX(layer, kMaxLayerTypes);
    ERR_FAIL_INDEX(band, kBandCount);
    ERR_FAIL_INDEX(eco_index, (int32_t)bands_[layer][band].size());
    ERR_FAIL_INDEX(pattern, kMaxPatterns);
    Feat f;
    f.pattern = pattern;
    f.usage = usage;
    f.block_bid = block_bid;
    f.threshold = threshold;
    bands_[layer][band][eco_index].feats.push_back(f);
    band_pattern_mask_[layer][band] |= 1u << pattern;
}

// Mirror of ChunkData.SetCellState (automata/meta = 0): palette bookkeeping + packed write.
void ChunkGen::set_cell_state(int32_t *cells, uint16_t *pbids, uint16_t *pcounts, int32_t idx, int32_t bid, Vector3i dist, int32_t field) {
    int32_t old_cid = cells[idx] & kCidMask;
    pcounts[old_cid]--;
    int32_t new_cid = kPaletteLen;
    for (int32_t i = 0; i < kPaletteLen; ++i) {
        if (pbids[i] == (uint16_t)bid) {
            new_cid = i;
            break;
        }
    }
    if (new_cid == kPaletteLen) {
        for (int32_t i = 0; i < kPaletteLen; ++i) {
            if (pcounts[i] == 0) {
                new_cid = i;
                break;
            }
        }
    }
    pbids[new_cid] = (uint16_t)bid;
    pcounts[new_cid]++;
    cells[idx] = (new_cid & kCidMask)
            | ((dist.x & kDistMask) << kDistXShift)
            | ((dist.y & kDistMask) << (kDistXShift + kDistBits))
            | ((dist.z & kDistMask) << (kDistXShift + kDistBits * 2))
            | (((kFieldMax - CLAMP(field, -kFieldMax, kFieldMax)) & kFieldMask) << kFieldShift);
}

int32_t ChunkGen::select_topology(int32_t layer_type, float biome) const {
    const LocalVector<Topo> &topos = topologies_[layer_type];
    if (topos.is_empty()) {
        return -1;
    }
    float target = unit_to_ratio(biome) * topology_weight_sum_[layer_type];
    float weight = 0.0f;
    for (int32_t i = 0; i < (int32_t)topos.size() - 1; ++i) {
        weight += topos[i].weight;
        if (target < weight) {
            return i;
        }
    }
    return (int32_t)topos.size() - 1;
}

int32_t ChunkGen::select_ecology(int32_t layer_type, int32_t band, float biome) const {
    const LocalVector<Eco> &ecos = bands_[layer_type][band];
    if (ecos.is_empty()) {
        return -1;
    }
    float target = unit_to_ratio(biome) * band_weight_sums_[layer_type][band];
    float weight = 0.0f;
    for (int32_t i = 0; i < (int32_t)ecos.size() - 1; ++i) {
        weight += ecos[i].weight;
        if (target < weight) {
            return i;
        }
    }
    return (int32_t)ecos.size() - 1;
}

int32_t ChunkGen::determine_band(float depth, int32_t local_y) const {
    if (local_y < layout_.magma_local_y) {
        return 0;
    }
    if (depth < kBandDepths[0]) {
        return 4;
    }
    if (depth < kBandDepths[1]) {
        return 3;
    }
    if (depth < kBandDepths[2]) {
        return 2;
    }
    return 1;
}

uint32_t ChunkGen::needed_patterns(int32_t layer_type, int32_t local_y0, float min_h, float max_h) const {
    float depth_max = max_h - (float)local_y0;
    float depth_min = min_h - (float)(local_y0 + kNDimLen - 1);
    // Bands run 4 (shallowest) down to 1 (deepest), so the deepest sample gives the low bound.
    int32_t lo = determine_band(depth_max, layout_.magma_local_y);
    int32_t hi = determine_band(depth_min, layout_.magma_local_y);
    uint32_t mask = local_y0 < layout_.magma_local_y ? band_pattern_mask_[layer_type][0] : 0;
    for (int32_t band = lo; band <= hi; ++band) {
        mask |= band_pattern_mask_[layer_type][band];
    }
    // An ecology naming a pattern that was never registered would otherwise read an unsized buffer.
    uint32_t registered = 0;
    for (int32_t p = 0; p < kMaxPatterns; ++p) {
        if (patterns_[p].node != nullptr) {
            registered |= 1u << p;
        }
    }
    return mask & registered;
}

void ChunkGen::gen_height(Vector3i pos, int32_t layer_type, int32_t seed, int32_t layer_y0, const float *flatten,
        float &min_h, float &max_h) const {
    fnGenUniformGrid2D(base_node_, base_.ptr(), pos.x, pos.z, kNDimLen, kNDimLen, kNoiseFreq, seed, nullptr);
    fnGenUniformGrid2D(mask_node_, mask_.ptr(), pos.x, pos.z, kNDimLen, kNDimLen, kNoiseFreq, topo_biome_seed(seed), nullptr);
    fnGenUniformGrid2D(topo_biome_node_, topo_biome_.ptr(), pos.x, pos.z, kNDimLen, kNDimLen, kNoiseFreq, topo_biome_seed(seed), nullptr);
    const LocalVector<Topo> &topos = topologies_[layer_type];
    for (int32_t t = 0; t < (int32_t)topos.size(); ++t) {
        fnGenUniformGrid2D(topos[t].detail_node, detail_.ptr() + t * kNDimLen2, pos.x, pos.z,
                kNDimLen, kNDimLen, kNoiseFreq, seed, nullptr);
    }

#ifdef DEBUG_ENABLED
    check_range(1, "topology mask", mask_.ptr(), kNDimLen2);
    check_range(2, "topology biome", topo_biome_.ptr(), kNDimLen2);
#endif

    for (int32_t i = 0; i < kNDimLen2; ++i) {
        int32_t t = select_topology(layer_type, topo_biome_[i]);
        float detail = t < 0 ? 0.0f : detail_[t * kNDimLen2 + i];
        float h = flattened(column_height(base_[i], mask_[i], detail), flatten[2 * i], flatten[2 * i + 1] - (float)layer_y0);
        height_[i] = h;
        min_h = i == 0 ? h : MIN(min_h, h);
        max_h = i == 0 ? h : MAX(max_h, h);
    }
}

void ChunkGen::gen_patterns(Vector3i pos, uint32_t needed, int32_t seed) const {
    for (int32_t p = 0; p < kMaxPatterns; ++p) {
        if ((needed & (1u << p)) == 0 || patterns_[p].node == nullptr) {
            continue;
        }
        fnGenUniformGrid3D(patterns_[p].node, pattern_[p].ptr(), pos.x, pos.y, pos.z,
                kNDimLen, kNDimLen, kNDimLen, patterns_[p].frequency, seed + p * 7919, nullptr);
#ifdef DEBUG_ENABLED
        check_range(4, "pattern", pattern_[p].ptr(), kNDimLen3);
#endif
    }
}

void ChunkGen::gen_cells(int32_t *cells, uint16_t *pbids, uint16_t *pcounts, float *noise3,
        int32_t world_y0, int32_t layer_type, uint32_t patterns) const {
    for (int32_t i = 0; i < kNDimLen3; ++i) {
        int32_t nx = i % kNDimLen;
        int32_t ny = (i / kNDimLen) % kNDimLen;
        int32_t nz = i / kNDimLen2;
        float h = height_[xz_index(i)];
        // Layer-local Y is taken per sample, not per chunk, so two chunks meeting at a layer seam
        // agree on every cell of their shared skirt.
        int32_t local_y = pos_mod(world_y0 + ny, layout_.layer_blocks);
        float density = (float)local_y - h;
        if (local_y < layout_.magma_local_y) {
            density = MAX(density, (float)(layout_.magma_local_y - local_y));
        }

        bool interior = nx >= kNPad && nx < kNPad + kDimLen && ny >= kNPad && ny < kNPad + kDimLen
                && nz >= kNPad && nz < kNPad + kDimLen;
        int32_t cell_index = interior ? (nx - kNPad) + (ny - kNPad) * kDimLen + (nz - kNPad) * kDimLen * kDimLen : -1;
        solid_[i] = 0;
        if (density > 0) {
            noise3[i] = density;
            continue;
        }

        int32_t band = determine_band(h - (float)local_y, local_y);
        int32_t e_index = select_ecology(layer_type, band, biome_n_[i]);
        if (e_index < 0) {
            noise3[i] = density;
            continue;
        }
        const Eco &eco = bands_[layer_type][band][e_index];

        int32_t b_index = 0;
        for (float weight = 0.0f; b_index < (int32_t)eco.block_depths.size() - 1; ++b_index) {
            weight += eco.block_depths[b_index];
            if (-density < weight) {
                break;
            }
        }
        int32_t block_bid = eco.block_bids.is_empty()
                ? 0
                : eco.block_bids[MIN(b_index, (int32_t)eco.block_bids.size() - 1)];

        for (const Feat &f : eco.feats) {
            if ((patterns & (1u << f.pattern)) == 0) {
                continue;
            }
            float v = pattern_[f.pattern][i];
            switch (f.usage) {
                case USAGE_CARVE:
                    if (v > f.threshold) {
                        density = MAX(density, (v - f.threshold) * layout_.carve_amp);
                    }
                    break;
                case USAGE_REPLACE:
                    if (v > f.threshold) {
                        block_bid = f.block_bid;
                    }
                    break;
                case USAGE_GEODE:
                    if (v > f.threshold + kGeodeShell) {
                        density = MAX(density, (v - f.threshold - kGeodeShell) * layout_.carve_amp);
                    } else if (v > f.threshold) {
                        block_bid = f.block_bid;
                    }
                    break;
                default:
                    break;
            }
        }

        if (eco.road_bid != 0 && block_bid != 0 && -density < road_depth_ && mask_[xz_index(i)] < road_threshold_) {
            block_bid = eco.road_bid;
        }

        noise3[i] = density;
        if (density > 0) {
            block_bid = 0; // air is always bid/cid 0
        }
        solid_[i] = block_bid != 0;
        if (interior) {
            int32_t raw = cells[cell_index];
            Vector3i dist((raw >> kDistXShift) & kDistMask,
                    (raw >> (kDistXShift + kDistBits)) & kDistMask,
                    (raw >> (kDistXShift + kDistBits * 2)) & kDistMask);
            int32_t field = kFieldMax - ((raw >> kFieldShift) & kFieldMask);
            set_cell_state(cells, pbids, pcounts, cell_index, block_bid, dist, field);
        }
    }
}

void ChunkGen::gen_dists(int32_t *cells, uint16_t *pbids, uint16_t *pcounts, const float *noise3) const {
    for (int32_t i = 0; i < kDimLen3; ++i) {
        int32_t nx = i % kDimLen + kNPad;
        int32_t ny = (i / kDimLen) % kDimLen + kNPad;
        int32_t nz = i / (kDimLen * kDimLen) + kNPad;
        int32_t ni0 = nx + ny * kNDimLen + nz * kNDimLen2;
        float n0 = noise3[ni0];
        int32_t s0 = (n0 > 0) - (n0 < 0);
        int32_t raw = cells[i];
        int32_t bid = pbids[raw & kCidMask];
        Vector3i dist;
        for (int32_t j = 0; j < 3; ++j) {
            if (s0 == 0) {
                dist[j] = 0;
                continue;
            }
            int32_t d[3] = { 0, 0, 0 };
            d[j] = 1;
            float n1 = noise3[(nx + d[0]) + (ny + d[1]) * kNDimLen + (nz + d[2]) * kNDimLen2];
            int32_t s1 = (n1 > 0) - (n1 < 0);
            if (s0 != s1) {
                dist[j] = world_to_dist(n0 / (n0 - n1));
            } else {
                dist[j] = kBlockDist;
            }
        }
        set_cell_state(cells, pbids, pcounts, i, bid, dist, solid_field(ni0));
    }
}

int32_t ChunkGen::solid_field(int32_t ni0) const {
    const FieldOffset *offsets = field_offsets();
    uint8_t s = solid_[ni0];
    for (int32_t k = 0; k < kFieldOffsetCount; ++k) {
        if (solid_[ni0 + offsets[k].dn] != s) {
            return s ? -offsets[k].d : offsets[k].d;
        }
    }
    return s ? -kFieldMax : kFieldMax;
}

void ChunkGen::generate(int64_t cells_addr, int64_t palette_bids_addr, int64_t palette_counts_addr,
        int64_t noise3_addr, int64_t flatten_addr, Vector3i chunk_i, int32_t seed) const {
    int32_t *cells = reinterpret_cast<int32_t *>(cells_addr);
    const float *flatten = reinterpret_cast<const float *>(flatten_addr);
    uint16_t *pbids = reinterpret_cast<uint16_t *>(palette_bids_addr);
    uint16_t *pcounts = reinterpret_cast<uint16_t *>(palette_counts_addr);
    float *noise3 = reinterpret_cast<float *>(noise3_addr);

    Vector3i pos = chunk_i * kChunkOffset;
    Vector3i norigin = pos - Vector3i(kSkirt + kNPad, kSkirt + kNPad, kSkirt + kNPad);

    int32_t level = floor_div(pos.y + kChunkOffset / 2, layout_.layer_blocks);
    int32_t layer_type = pos_mod(level, layout_.layer_types);
    int32_t lseed = layer_seed(seed, level);

    float min_h = 0.0f, max_h = 0.0f;
    gen_height(norigin, layer_type, lseed, level * layout_.layer_blocks, flatten, min_h, max_h);

    int32_t local_y0 = pos_mod(norigin.y, layout_.layer_blocks);
    // Wholly above every column in this chunk: nothing but air, so no 3D noise is worth generating.
    if ((float)local_y0 > max_h && local_y0 + kNDimLen - 1 < layout_.layer_blocks) {
        for (int32_t i = 0; i < kNDimLen3; ++i) {
            noise3[i] = (float)(local_y0 + (i / kNDimLen) % kNDimLen) - height_[xz_index(i)];
            solid_[i] = 0;
        }
        gen_dists(cells, pbids, pcounts, noise3);
        return;
    }

    fnGenUniformGrid3D(biome_node_, biome_n_.ptr(), norigin.x, norigin.y, norigin.z,
            kNDimLen, kNDimLen, kNDimLen, kNoiseFreq, lseed, nullptr);
#ifdef DEBUG_ENABLED
    check_range(5, "palette biome", biome_n_.ptr(), kNDimLen3);
#endif

    uint32_t patterns = needed_patterns(layer_type, local_y0, min_h, max_h);
    gen_patterns(norigin, patterns, lseed);
    gen_cells(cells, pbids, pcounts, noise3, norigin.y, layer_type, patterns);
    gen_dists(cells, pbids, pcounts, noise3);
}

float ChunkGen::probe_height(int32_t world_x, int32_t world_z, int32_t layer_type, int32_t lseed) const {
    float base = 0.0f, mask = 0.0f, biome = 0.0f, detail = 0.0f;
    fnGenUniformGrid2D(base_node_, &base, world_x, world_z, 1, 1, kNoiseFreq, lseed, nullptr);
    fnGenUniformGrid2D(mask_node_, &mask, world_x, world_z, 1, 1, kNoiseFreq, topo_biome_seed(lseed), nullptr);
    fnGenUniformGrid2D(topo_biome_node_, &biome, world_x, world_z, 1, 1, kNoiseFreq, topo_biome_seed(lseed), nullptr);
    int32_t t = select_topology(layer_type, biome);
    if (t >= 0) {
        fnGenUniformGrid2D(topologies_[layer_type][t].detail_node, &detail, world_x, world_z, 1, 1, kNoiseFreq, lseed, nullptr);
    }
    return column_height(base, mask, detail);
}

int32_t ChunkGen::surface_height(int32_t world_x, int32_t world_z, int32_t level, int32_t seed,
        float flatten, int32_t flatten_y) const {
    int32_t layer_type = pos_mod(level, layout_.layer_types);
    int32_t layer_y0 = level * layout_.layer_blocks;
    float h = flattened(probe_height(world_x, world_z, layer_type, layer_seed(seed, level)), flatten, (float)(flatten_y - layer_y0));
    return layer_y0 + (int32_t)Math::round(h);
}

int32_t ChunkGen::surface_ecology(int32_t world_x, int32_t world_z, int32_t level, int32_t seed) const {
    int32_t layer_type = pos_mod(level, layout_.layer_types);
    int32_t lseed = layer_seed(seed, level);
    float h = probe_height(world_x, world_z, layer_type, lseed);
    int32_t local_y = (int32_t)Math::round(h) - 1;
    float biome = 0.0f;
    fnGenUniformGrid3D(biome_node_, &biome, world_x, level * layout_.layer_blocks + local_y, world_z,
            1, 1, 1, kNoiseFreq, lseed, nullptr);
    int32_t band = determine_band(h - (float)local_y, local_y);
    return band == kBandCount - 1 ? select_ecology(layer_type, band, biome) : -1;
}

#ifdef DEBUG_ENABLED
void ChunkGen::check_range(int32_t id, const char *name, const float *v, int32_t n) const {
    if (warned_ & (1u << id)) {
        return;
    }
    float mn = v[0], mx = v[0];
    for (int32_t i = 1; i < n; ++i) {
        mn = MIN(mn, v[i]);
        mx = MAX(mx, v[i]);
    }
    if (mn < -1.5f || mx > 1.5f) {
        warned_ |= 1u << id;
        ERR_PRINT(vformat("ChunkGen: noise '%s' outputs [%f, %f]; trees must be authored to -1..1 and scaled here.",
                String(name), mn, mx));
    }
}
#endif

void ChunkGen::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_layout", "layer_chunks", "layer_types", "surface_local_y",
                                 "carve_amp", "magma_local_y"),
            &ChunkGen::set_layout);
    ClassDB::bind_method(D_METHOD("set_fixed_noise", "palette_biome", "topology_biome", "topology_mask", "topology_base"),
            &ChunkGen::set_fixed_noise);
    ClassDB::bind_method(D_METHOD("set_pattern_noise", "index", "encoded", "frequency"), &ChunkGen::set_pattern_noise);
    ClassDB::bind_method(D_METHOD("set_road", "mask_threshold", "depth"), &ChunkGen::set_road);
    ClassDB::bind_method(D_METHOD("add_topology", "layer", "weight", "encoded_detail"), &ChunkGen::add_topology);
    ClassDB::bind_method(D_METHOD("add_ecology", "layer", "band", "weight", "block_bids", "block_depths", "road_bid"),
            &ChunkGen::add_ecology);
    ClassDB::bind_method(D_METHOD("add_ecology_feature", "layer", "band", "eco_index", "pattern", "usage", "block_bid", "threshold"),
            &ChunkGen::add_ecology_feature);
    ClassDB::bind_method(D_METHOD("generate", "cells_addr", "palette_bids_addr", "palette_counts_addr", "noise3_addr", "flatten_addr", "chunk_i", "seed"),
            &ChunkGen::generate);
    ClassDB::bind_method(D_METHOD("surface_height", "world_x", "world_z", "level", "seed", "flatten", "flatten_y"), &ChunkGen::surface_height);
    ClassDB::bind_method(D_METHOD("surface_ecology", "world_x", "world_z", "level", "seed"), &ChunkGen::surface_ecology);
}

} // namespace SSK
