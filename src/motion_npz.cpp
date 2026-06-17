#include "motion_npz.h"
#include "g1_body_names.h"

#include <spdlog/spdlog.h>
#include <zlib.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace {

struct ZipCentralEntry {
  std::string name;
  uint32_t local_header_offset = 0;
  uint32_t compr_bytes = 0;
  uint32_t uncompr_bytes = 0;
  uint16_t compr_method = 0;
};

std::vector<ZipCentralEntry> read_zip_central_directory(FILE* fp)
{
  uint16_t nrecs = 0;
  size_t global_header_size = 0;
  size_t global_header_offset = 0;
  cnpy::parse_zip_footer(fp, nrecs, global_header_size, global_header_offset);

  std::vector<ZipCentralEntry> entries;
  entries.reserve(nrecs);
  fseek(fp, static_cast<long>(global_header_offset), SEEK_SET);

  for (uint16_t i = 0; i < nrecs; ++i) {
    std::vector<char> header(46);
    if (fread(&header[0], sizeof(char), 46, fp) != 46) {
      throw std::runtime_error("motion NPZ: central directory fread failed");
    }

    const uint32_t sig = *reinterpret_cast<uint32_t*>(&header[0]);
    if (sig != 0x02014b50U) {
      throw std::runtime_error("motion NPZ: bad central directory signature");
    }

    ZipCentralEntry entry;
    entry.compr_method = *reinterpret_cast<uint16_t*>(&header[10]);
    entry.compr_bytes = *reinterpret_cast<uint32_t*>(&header[20]);
    entry.uncompr_bytes = *reinterpret_cast<uint32_t*>(&header[24]);
    const uint16_t name_len = *reinterpret_cast<uint16_t*>(&header[28]);
    const uint16_t extra_len = *reinterpret_cast<uint16_t*>(&header[30]);
    const uint16_t comment_len = *reinterpret_cast<uint16_t*>(&header[32]);
    entry.local_header_offset = *reinterpret_cast<uint32_t*>(&header[42]);

    entry.name.resize(name_len);
    if (fread(&entry.name[0], sizeof(char), name_len, fp) != name_len) {
      throw std::runtime_error("motion NPZ: central directory name fread failed");
    }
    fseek(fp, extra_len + comment_len, SEEK_CUR);
    entries.push_back(std::move(entry));
  }
  return entries;
}

void seek_to_zip_entry_data(FILE* fp, const ZipCentralEntry& entry)
{
  fseek(fp, static_cast<long>(entry.local_header_offset), SEEK_SET);

  std::vector<char> local_header(30);
  if (fread(&local_header[0], sizeof(char), 30, fp) != 30) {
    throw std::runtime_error("motion NPZ: local header fread failed");
  }

  const uint16_t name_len = *reinterpret_cast<uint16_t*>(&local_header[26]);
  const uint16_t extra_len = *reinterpret_cast<uint16_t*>(&local_header[28]);
  fseek(fp, name_len + extra_len, SEEK_CUR);
}

cnpy::NpyArray load_stored_npy(FILE* fp)
{
  std::vector<size_t> shape;
  size_t word_size = 0;
  bool fortran_order = false;
  cnpy::parse_npy_header(fp, word_size, shape, fortran_order);

  cnpy::NpyArray arr(shape, word_size, fortran_order);
  const size_t nread = fread(arr.data<char>(), 1, arr.num_bytes(), fp);
  if (nread != arr.num_bytes()) {
    throw std::runtime_error("motion NPZ: stored array fread failed");
  }
  return arr;
}

cnpy::NpyArray load_deflated_npy(FILE* fp, uint32_t compr_bytes, uint32_t uncompr_bytes)
{
  std::vector<unsigned char> buffer_compr(compr_bytes);
  std::vector<unsigned char> buffer_uncompr(uncompr_bytes);
  const size_t nread = fread(&buffer_compr[0], 1, compr_bytes, fp);
  if (nread != compr_bytes) {
    throw std::runtime_error("motion NPZ: compressed payload fread failed");
  }

  z_stream d_stream{};
  d_stream.zalloc = Z_NULL;
  d_stream.zfree = Z_NULL;
  d_stream.opaque = Z_NULL;
  d_stream.avail_in = 0;
  d_stream.next_in = Z_NULL;
  inflateInit2(&d_stream, -MAX_WBITS);

  d_stream.avail_in = compr_bytes;
  d_stream.next_in = &buffer_compr[0];
  d_stream.avail_out = uncompr_bytes;
  d_stream.next_out = &buffer_uncompr[0];
  inflate(&d_stream, Z_FINISH);
  inflateEnd(&d_stream);

  std::vector<size_t> shape;
  size_t word_size = 0;
  bool fortran_order = false;
  cnpy::parse_npy_header(&buffer_uncompr[0], word_size, shape, fortran_order);

  cnpy::NpyArray array(shape, word_size, fortran_order);
  const size_t offset = uncompr_bytes - array.num_bytes();
  memcpy(
    array.data<unsigned char>(),
    &buffer_uncompr[0] + offset,
    array.num_bytes());
  return array;
}

cnpy::NpyArray load_npz_array(const std::string& path, const std::string& key)
{
  FILE* fp = fopen(path.c_str(), "rb");
  if (!fp) {
    throw std::runtime_error("motion NPZ: unable to open " + path);
  }

  const auto entries = read_zip_central_directory(fp);
  for (const auto& entry : entries) {
    std::string name = entry.name;
    if (name.size() >= 4 && name.compare(name.size() - 4, 4, ".npy") == 0) {
      name.erase(name.end() - 4, name.end());
    }
    if (name != key) {
      continue;
    }

    seek_to_zip_entry_data(fp, entry);
    cnpy::NpyArray array = (entry.compr_method == 0)
      ? load_stored_npy(fp)
      : load_deflated_npy(fp, entry.compr_bytes, entry.uncompr_bytes);
    fclose(fp);
    return array;
  }

  fclose(fp);
  throw std::runtime_error("motion NPZ: key '" + key + "' not found in " + path);
}

cnpy::NpyArray load_required(const std::string& path, const char* key)
{
  try {
    return load_npz_array(path, key);
  } catch (const std::exception& e) {
    throw std::runtime_error(
      std::string("motion NPZ missing or unreadable key '") + key + "': " + e.what());
  }
}

bool try_load_optional(const std::string& path, const char* key, cnpy::NpyArray& out)
{
  try {
    out = load_npz_array(path, key);
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

}  // namespace

MotionNpzArrays load_motion_npz(const std::string& path)
{
  MotionNpzArrays arrays;
  arrays.joint_pos = load_required(path, "joint_pos");
  arrays.joint_vel = load_required(path, "joint_vel");
  arrays.body_pos_w = load_required(path, "body_pos_w");
  arrays.body_quat_w = load_required(path, "body_quat_w");
  arrays.body_lin_vel_w = load_required(path, "body_lin_vel_w");
  arrays.body_ang_vel_w = load_required(path, "body_ang_vel_w");
  arrays.has_body_names = try_load_optional(path, "body_names", arrays.body_names);
  return arrays;
}

std::vector<float> load_motion_joint_pos_at(const std::string& path, size_t frame_index)
{
  const MotionNpzArrays arrays = load_motion_npz(path);
  if (arrays.joint_pos.shape.size() != 2) {
    throw std::runtime_error("joint_pos must be [T, num_joints] in " + path);
  }
  const size_t num_frames = arrays.joint_pos.shape[0];
  const size_t num_joints = arrays.joint_pos.shape[1];
  if (frame_index >= num_frames) {
    throw std::runtime_error(
      "Frame index " + std::to_string(frame_index) + " out of range in " + path);
  }

  std::vector<float> joint_pos(num_joints);
  const float* base = arrays.joint_pos.data<float>() + frame_index * num_joints;
  for (size_t j = 0; j < num_joints; ++j) {
    joint_pos[j] = base[j];
  }
  return joint_pos;
}

std::filesystem::path resolve_clip_path(
  const std::filesystem::path& clips_dir,
  const std::string& clip_name)
{
  if (clip_name.find('/') != std::string::npos
      || (clip_name.size() >= 4
          && clip_name.compare(clip_name.size() - 4, 4, ".npz") == 0)) {
    auto path = std::filesystem::path(clip_name);
    if (path.is_relative()) {
      path = clips_dir / path;
    }
    return path;
  }
  return clips_dir / (clip_name + ".npz");
}

int resolve_anchor_body_index(
  const cnpy::NpyArray* body_names,
  const std::string& anchor_name)
{
  if (body_names != nullptr) {
    const size_t n = body_names->shape[0];
    for (size_t i = 0; i < n; ++i) {
      std::string name(
        body_names->data<char>() + i * body_names->word_size,
        body_names->word_size);
      while (!name.empty() && name.back() == '\0') {
        name.pop_back();
      }
      if (name == anchor_name) {
        return static_cast<int>(i);
      }
    }
  }

  for (size_t i = 0; i < G1_FULL_BODY_NAMES.size(); ++i) {
    if (G1_FULL_BODY_NAMES[i] == anchor_name) {
      if (body_names == nullptr) {
        spdlog::warn(
          "NPZ missing body_names; using built-in G1 body list for anchor '{}'",
          anchor_name);
      }
      return static_cast<int>(i);
    }
  }

  throw std::runtime_error("Anchor body '" + anchor_name + "' not found in motion clip");
}
