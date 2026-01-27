#include "feature_knowledge.h"

#include <assert.h>
#include <stdio.h>

#include "util/util.h"

const uint32_t tiling_offset[] = {0xaca081b9, 0x666a1c67, 0xc11d6a53, 0x8e5d97c1, 0x0d1cad54, 0x874f71cb, 0x20d2fa13, 0x73f7c4a7,
                                  0x0b701f6c, 0x8388d86d, 0xf72ac9f2, 0xbab16d82, 0x524ac258, 0xb5900302, 0xb48ccc72, 0x632f05bf,
                                  0xe7111073, 0xeb602af4, 0xf3f29ebb, 0x2a6184f2, 0x461da5da, 0x6693471d, 0x62fd0138, 0xc484efb3,
                                  0x81c9eeeb, 0x860f3766, 0x334faf86, 0x5e81e881, 0x14bc2195, 0xf47671a8, 0x75414279, 0x357bc5e0};

const char* MapFeatureTypeString[] = {"PC",
                                      "Offset",
                                      "Delta",
                                      "Address",
                                      "PC_Offset",
                                      "PC_Address",
                                      "PC_Page",
                                      "PC_Path",
                                      "Delta_Path",
                                      "Offset_Path",
                                      "PC_Delta",
                                      "PC_Offset_Delta",
                                      "Page",
                                      "PC_Path_Offset",
                                      "PC_Path_Offset_Path",
                                      "PC_Path_Delta",
                                      "PC_Path_Delta_Path",
                                      "PC_Path_Offset_Path_Delta_Path",
                                      "Offset_Path_PC",
                                      "Delta_Path_PC"};

std::string FeatureKnowledge::getFeatureString(FeatureType feature)
{
  assert(feature < FeatureType::NumFeatureTypes);
  return MapFeatureTypeString[(uint32_t)feature];
}

FeatureKnowledge::FeatureKnowledge(FeatureType feature_type, float alpha, float gamma, uint32_t actions, uint32_t num_tilings, uint32_t num_tiles,
                                   uint32_t hash_type, int32_t enable_tiling_offset)
    : m_feature_type(feature_type), m_alpha(alpha), m_gamma(gamma), m_actions(actions), m_hash_type(hash_type), m_num_tilings(num_tilings),
      m_num_tiles(num_tiles), m_enable_tiling_offset(enable_tiling_offset ? true : false)
{
  assert(m_num_tilings <= FK_MAX_TILINGS);
  assert(m_num_tilings == 1 || m_enable_tiling_offset); /* enforce the use of tiling offsets in case of multiple tilings */

  /* create Q-table */
  m_qtable = (float***)calloc(m_num_tilings, sizeof(float**));
  assert(m_qtable);
  for (uint32_t tiling = 0; tiling < m_num_tilings; ++tiling) {
    m_qtable[tiling] = (float**)calloc(m_num_tiles, sizeof(float*));
    assert(m_qtable[tiling]);
    for (uint32_t tile = 0; tile < m_num_tiles; ++tile) {
      m_qtable[tiling][tile] = (float*)calloc(m_actions, sizeof(float));
      assert(m_qtable[tiling][tile]);
    }
  }

  /* init Q-table */
  m_init_value = (float)1ul / (1 - gamma);

  for (uint32_t tiling = 0; tiling < m_num_tilings; ++tiling) {
    for (uint32_t tile = 0; tile < m_num_tiles; ++tile) {
      for (uint32_t action = 0; action < m_actions; ++action) {
        m_qtable[tiling][tile][action] = m_init_value;
      }
    }
  }
}

FeatureKnowledge::~FeatureKnowledge() {}

float FeatureKnowledge::getQ(uint32_t tiling, uint32_t tile_index, uint32_t action)
{
  assert(tiling < m_num_tilings);
  assert(tile_index < m_num_tiles);
  assert(action < m_actions);
  return m_qtable[tiling][tile_index][action];
}

void FeatureKnowledge::setQ(uint32_t tiling, uint32_t tile_index, uint32_t action, float value)
{
  assert(tiling < m_num_tilings);
  assert(tile_index < m_num_tiles);
  assert(action < m_actions);
  m_qtable[tiling][tile_index][action] = value;
}

float FeatureKnowledge::retrieveQ(State* state, uint32_t action)
{
  uint32_t tile_index = 0;
  float q_value = 0.0;

  for (uint32_t tiling = 0; tiling < m_num_tilings; ++tiling) {
    tile_index = get_tile_index(tiling, state);
    q_value += getQ(tiling, tile_index, action);
  }

  return q_value;
}

void FeatureKnowledge::updateQ(State* state1, uint32_t action1, int32_t reward, State* state2, uint32_t action2)
{
  uint32_t tile_index1 = 0, tile_index2 = 0;
  float Qsa1, Qsa2, Qsa1_old;

  float QSa1_old_overall = retrieveQ(state1, action1);
  float QSa2_old_overall = retrieveQ(state2, action2);

  for (uint32_t tiling = 0; tiling < m_num_tilings; ++tiling) {
    tile_index1 = get_tile_index(tiling, state1);
    tile_index2 = get_tile_index(tiling, state2);
    Qsa1 = getQ(tiling, tile_index1, action1);
    Qsa2 = getQ(tiling, tile_index2, action2);
    Qsa1_old = Qsa1;
    /* SARSA */
    Qsa1 = Qsa1 + m_alpha * ((float)reward + m_gamma * Qsa2 - Qsa1);
    setQ(tiling, tile_index1, action1, Qsa1);
    MYLOG("<tiling %u> Q(%s,%u) = %0.2f, R = %d, Q(%s,%u) = %0.2f, Q(%s,%u) = %0.2f", tiling, state1->to_string().c_str(), action1, Qsa1_old, reward,
          state2->to_string().c_str(), action2, Qsa2, state1->to_string().c_str(), action1, Qsa1);
  }

  float QSa1_new_overall = retrieveQ(state1, action1);
  MYLOG("<feature %s> Q(%s,%u) = %0.2f, R = %d, Q(%s,%u) = %0.2f, Q(%s,%u) = %0.2f", getFeatureString(m_feature_type).c_str(), state1->to_string().c_str(),
        action1, QSa1_old_overall, reward, state2->to_string().c_str(), action2, QSa2_old_overall, state1->to_string().c_str(), action1, QSa1_new_overall);

  Qsa1_old = Qsa1_old + QSa1_old_overall + QSa2_old_overall + QSa1_new_overall; // some fake computation to keep compiler happy
}

uint32_t FeatureKnowledge::get_tile_index(uint32_t tiling, State* state)
{
  uint64_t pc = state->pc;
  uint64_t page = state->page;
  uint64_t address = state->address;
  uint32_t offset = state->offset;
  int32_t delta = state->delta;
  uint32_t delta_path = state->local_delta_sig2;
  uint32_t pc_path = state->local_pc_sig;
  uint32_t offset_path = state->local_offset_sig;

  switch (m_feature_type) {
  case F_PC:
    return process_PC(tiling, pc);
  case F_Offset:
    return process_offset(tiling, offset);
  case F_Delta:
    return process_delta(tiling, delta);
  case F_Address:
    return process_address(tiling, address);
  case F_PC_Offset:
    return process_PC_offset(tiling, pc, offset);
  case F_PC_Address:
    return process_PC_address(tiling, pc, address);
  case F_PC_Page:
    return process_PC_page(tiling, pc, page);
  case F_PC_Path:
    return process_PC_path(tiling, pc_path);
  case F_Delta_Path:
    return process_delta_path(tiling, delta_path);
  case F_Offset_Path:
    return process_offset_path(tiling, offset_path);
  case F_PC_Delta:
    return process_PC_delta(tiling, pc, delta);
  case F_PC_Offset_Delta:
    return process_PC_offset_delta(tiling, pc, offset, delta);
  case F_Page:
    return process_Page(tiling, page);
  case F_PC_Path_Offset:
    return process_PC_Path_Offset(tiling, pc_path, offset);
  case F_PC_Path_Offset_Path:
    return process_PC_Path_Offset_Path(tiling, pc_path, offset_path);
  case F_PC_Path_Delta:
    return process_PC_Path_Delta(tiling, pc_path, delta);
  case F_PC_Path_Delta_Path:
    return process_PC_Path_Delta_Path(tiling, pc_path, delta_path);
  case F_PC_Path_Offset_Path_Delta_Path:
    return process_PC_Path_Offset_Path_Delta_Path(tiling, pc_path, offset_path, delta_path);
  case F_Offset_Path_PC:
    return process_Offset_Path_PC(tiling, offset_path, pc);
  case F_Delta_Path_PC:
    return process_Delta_Path_PC(tiling, delta_path, pc);
  default:
    assert(false);
    return 0;
  }
}

uint32_t FeatureKnowledge::getMaxAction(State* state)
{
  float max_q_value = 0.0, q_value = 0.0;
  uint32_t selected_action = 0, init_index = 0;

  for (uint32_t action = init_index; action < m_actions; ++action) {
    q_value = retrieveQ(state, action);
    if (q_value > max_q_value) {
      max_q_value = q_value;
      selected_action = action;
    }
  }
  return selected_action;
}

std::string FeatureKnowledge::get_feature_string(State* state)
{
  uint64_t pc = state->pc;
  // uint64_t page = state->page;
  // uint64_t address = state->address;
  // uint32_t offset = state->offset;
  int32_t delta = state->delta;
  // uint32_t delta_path = state->local_delta_sig2;
  // uint32_t pc_path = state->local_pc_sig;
  // uint32_t offset_path = state->local_offset_sig;

  std::stringstream ss;
  switch (m_feature_type) {
  case F_PC:
    ss << std::hex << pc << std::dec;
    break;
  case F_PC_Delta:
    ss << std::hex << pc << std::dec << "|" << delta;
    break;
  default:
    /* @RBERA TODO: define the rest */
    assert(false);
  }
  return ss.str();
}

//----------------------------------------------------//
// Feature index generation functions
//----------------------------------------------------//
uint32_t FeatureKnowledge::process_PC(uint32_t tiling, uint64_t pc)
{
  uint32_t raw_index = folded_xor(pc, 2); /* 32-b folded XOR */
  if (m_enable_tiling_offset)
    raw_index = raw_index ^ tiling_offset[tiling];
  uint32_t hashed_index = HashZoo::getHash(m_hash_type, raw_index);
  return (hashed_index % m_num_tiles);
}

uint32_t FeatureKnowledge::process_offset(uint32_t tiling, uint32_t offset)
{
  if (m_enable_tiling_offset)
    offset = offset ^ tiling_offset[tiling];
  uint32_t hashed_index = HashZoo::getHash(m_hash_type, offset);
  return (hashed_index % m_num_tiles);
}

uint32_t FeatureKnowledge::process_delta(uint32_t tiling, int32_t delta)
{
  uint32_t unsigned_delta = (delta < 0) ? (((-1) * delta) + (1 << (DELTA_BITS - 1))) : delta;
  if (m_enable_tiling_offset)
    unsigned_delta = unsigned_delta ^ tiling_offset[tiling];
  uint32_t hashed_index = HashZoo::getHash(m_hash_type, unsigned_delta);
  return (hashed_index % m_num_tiles);
}

uint32_t FeatureKnowledge::process_address(uint32_t tiling, uint64_t address)
{
  uint32_t raw_index = folded_xor(address, 2); /* 32-b folded XOR */
  if (m_enable_tiling_offset)
    raw_index = raw_index ^ tiling_offset[tiling];
  uint32_t hashed_index = HashZoo::getHash(m_hash_type, raw_index);
  return (hashed_index % m_num_tiles);
}

uint32_t FeatureKnowledge::process_PC_offset(uint32_t tiling, uint64_t pc, uint32_t offset)
{
  uint64_t tmp = pc;
  tmp = tmp << 6;
  tmp += offset;
  uint32_t raw_index = folded_xor(tmp, 2);
  if (m_enable_tiling_offset)
    raw_index = raw_index ^ tiling_offset[tiling];
  uint32_t hashed_index = HashZoo::getHash(m_hash_type, raw_index);
  return (hashed_index % m_num_tiles);
}

uint32_t FeatureKnowledge::process_PC_address(uint32_t tiling, uint64_t pc, uint64_t address)
{
  uint64_t tmp = pc;
  tmp = tmp << 16;
  tmp ^= address;
  uint32_t raw_index = folded_xor(tmp, 2);
  if (m_enable_tiling_offset)
    raw_index = raw_index ^ tiling_offset[tiling];
  uint32_t hashed_index = HashZoo::getHash(m_hash_type, raw_index);
  return (hashed_index % m_num_tiles);
}

uint32_t FeatureKnowledge::process_PC_page(uint32_t tiling, uint64_t pc, uint64_t page)
{
  uint64_t tmp = pc;
  tmp = tmp << 16;
  tmp ^= page;
  uint32_t raw_index = folded_xor(tmp, 2);
  if (m_enable_tiling_offset)
    raw_index = raw_index ^ tiling_offset[tiling];
  uint32_t hashed_index = HashZoo::getHash(m_hash_type, raw_index);
  return (hashed_index % m_num_tiles);
}

uint32_t FeatureKnowledge::process_PC_path(uint32_t tiling, uint32_t pc_path)
{
  if (m_enable_tiling_offset)
    pc_path = pc_path ^ tiling_offset[tiling];
  uint32_t hashed_index = HashZoo::getHash(m_hash_type, pc_path);
  return (hashed_index % m_num_tiles);
}

uint32_t FeatureKnowledge::process_delta_path(uint32_t tiling, uint32_t delta_path)
{
  if (m_enable_tiling_offset)
    delta_path = delta_path ^ tiling_offset[tiling];
  uint32_t hashed_index = HashZoo::getHash(m_hash_type, delta_path);
  return (hashed_index % m_num_tiles);
}

uint32_t FeatureKnowledge::process_offset_path(uint32_t tiling, uint32_t offset_path)
{
  if (m_enable_tiling_offset)
    offset_path = offset_path ^ tiling_offset[tiling];
  uint32_t hashed_index = HashZoo::getHash(m_hash_type, offset_path);
  return (hashed_index % m_num_tiles);
}

uint32_t FeatureKnowledge::process_PC_delta(uint32_t tiling, uint64_t pc, int32_t delta)
{
  uint32_t unsigned_delta = (delta < 0) ? (((-1) * delta) + (1 << (DELTA_BITS - 1))) : delta;
  uint64_t tmp = pc;
  tmp = tmp << 7;
  tmp += unsigned_delta;
  uint32_t raw_index = folded_xor(tmp, 2);
  if (m_enable_tiling_offset)
    raw_index = raw_index ^ tiling_offset[tiling];
  uint32_t hashed_index = HashZoo::getHash(m_hash_type, raw_index);
  return (hashed_index % m_num_tiles);
}

uint32_t FeatureKnowledge::process_PC_offset_delta(uint32_t tiling, uint64_t pc, uint32_t offset, int32_t delta)
{
  uint32_t unsigned_delta = (delta < 0) ? (((-1) * delta) + (1 << (DELTA_BITS - 1))) : delta;
  uint64_t tmp = pc;
  tmp = tmp << 6;
  tmp += offset;
  tmp = tmp << 7;
  tmp += unsigned_delta;
  uint32_t raw_index = folded_xor(tmp, 2);
  if (m_enable_tiling_offset)
    raw_index = raw_index ^ tiling_offset[tiling];
  uint32_t hashed_index = HashZoo::getHash(m_hash_type, raw_index);
  return (hashed_index % m_num_tiles);
}

uint32_t FeatureKnowledge::process_Page(uint32_t tiling, uint64_t page)
{
  uint32_t raw_index = folded_xor(page, 2); /* 32-b folded XOR */
  if (m_enable_tiling_offset)
    raw_index = raw_index ^ tiling_offset[tiling];
  uint32_t hashed_index = HashZoo::getHash(m_hash_type, raw_index);
  return (hashed_index % m_num_tiles);
}

uint32_t FeatureKnowledge::process_PC_Path_Offset(uint32_t tiling, uint32_t pc_path, uint32_t offset)
{
  uint64_t tmp = pc_path;
  tmp = tmp << 6;
  tmp += offset;
  uint32_t raw_index = folded_xor(tmp, 2);
  if (m_enable_tiling_offset)
    raw_index = raw_index ^ tiling_offset[tiling];
  uint32_t hashed_index = HashZoo::getHash(m_hash_type, raw_index);
  return (hashed_index % m_num_tiles);
}

uint32_t FeatureKnowledge::process_PC_Path_Offset_Path(uint32_t tiling, uint32_t pc_path, uint32_t offset_path)
{
  uint64_t tmp = pc_path;
  tmp = tmp << 16;
  tmp += offset_path;
  uint32_t raw_index = folded_xor(tmp, 2);
  if (m_enable_tiling_offset)
    raw_index = raw_index ^ tiling_offset[tiling];
  uint32_t hashed_index = HashZoo::getHash(m_hash_type, raw_index);
  return (hashed_index % m_num_tiles);
}

uint32_t FeatureKnowledge::process_PC_Path_Delta(uint32_t tiling, uint32_t pc_path, int32_t delta)
{
  uint32_t unsigned_delta = (delta < 0) ? (((-1) * delta) + (1 << (DELTA_BITS - 1))) : delta;
  uint64_t tmp = pc_path;
  tmp = tmp << 7;
  tmp += unsigned_delta;
  uint32_t raw_index = folded_xor(tmp, 2);
  if (m_enable_tiling_offset)
    raw_index = raw_index ^ tiling_offset[tiling];
  uint32_t hashed_index = HashZoo::getHash(m_hash_type, raw_index);
  return (hashed_index % m_num_tiles);
}

uint32_t FeatureKnowledge::process_PC_Path_Delta_Path(uint32_t tiling, uint32_t pc_path, uint32_t delta_path)
{
  uint64_t tmp = pc_path;
  tmp = tmp << 16;
  tmp += delta_path;
  uint32_t raw_index = folded_xor(tmp, 2);
  if (m_enable_tiling_offset)
    raw_index = raw_index ^ tiling_offset[tiling];
  uint32_t hashed_index = HashZoo::getHash(m_hash_type, raw_index);
  return (hashed_index % m_num_tiles);
}

uint32_t FeatureKnowledge::process_PC_Path_Offset_Path_Delta_Path(uint32_t tiling, uint32_t pc_path, uint32_t offset_path, uint32_t delta_path)
{
  uint64_t tmp = pc_path;
  tmp = tmp << 10;
  tmp ^= offset_path;
  tmp = tmp << 10;
  tmp ^= delta_path;
  uint32_t raw_index = folded_xor(tmp, 2);
  if (m_enable_tiling_offset)
    raw_index = raw_index ^ tiling_offset[tiling];
  uint32_t hashed_index = HashZoo::getHash(m_hash_type, raw_index);
  return (hashed_index % m_num_tiles);
}

uint32_t FeatureKnowledge::process_Offset_Path_PC(uint32_t tiling, uint32_t offset_path, uint64_t pc)
{
  uint64_t tmp = offset_path;
  tmp = tmp << 32;
  tmp += pc;
  uint32_t raw_index = folded_xor(tmp, 2);
  if (m_enable_tiling_offset)
    raw_index = raw_index ^ tiling_offset[tiling];
  uint32_t hashed_index = HashZoo::getHash(m_hash_type, raw_index);
  return (hashed_index % m_num_tiles);
}

uint32_t FeatureKnowledge::process_Delta_Path_PC(uint32_t tiling, uint32_t delta_path, uint64_t pc)
{
  uint64_t tmp = delta_path;
  tmp = tmp << 32;
  tmp += pc;
  uint32_t raw_index = folded_xor(tmp, 2);
  if (m_enable_tiling_offset)
    raw_index = raw_index ^ tiling_offset[tiling];
  uint32_t hashed_index = HashZoo::getHash(m_hash_type, raw_index);
  return (hashed_index % m_num_tiles);
}