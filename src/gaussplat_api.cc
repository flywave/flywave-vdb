#include "gaussplat_api.h"
#include "float_grid.hh"
#include "grid_api_impl.hh"
#include "trees.hh"

#include <algorithm>
#include <cmath>
#include <vector>

using namespace flywave;

#include <openvdb/openvdb.h>
#include <openvdb/tools/Interpolation.h>

/* ═══════════════════════════════════════════════════════════════════
   Transfer Function
   ═══════════════════════════════════════════════════════════════════ */

struct ControlPoint {
  float scalar;
  float r, g, b, a;
};

struct _vdb_transfer_function_t {
  std::vector<ControlPoint> points;
  float lut[1024 * 4]; // r,g,b,a packed, 1024-entry LUT + linear interp
  bool dirty = true;
};

FLYWAVE_VDB_API vdb_transfer_function_t *vdb_transfer_function_create() {
  auto *tf = new vdb_transfer_function_t();
  tf->dirty = true;
  return tf;
}

FLYWAVE_VDB_API void vdb_transfer_function_free(vdb_transfer_function_t *tf) {
  delete tf;
}

FLYWAVE_VDB_API void vdb_transfer_function_add_point(
    vdb_transfer_function_t *tf,
    float scalar, float r, float g, float b, float a) {
  // Clamp scalar to [0,1]
  if (scalar < 0.0f)
    scalar = 0.0f;
  if (scalar > 1.0f)
    scalar = 1.0f;
  for (auto &pt : tf->points) {
    if (std::abs(pt.scalar - scalar) < 1e-6f) {
      pt.r = r;
      pt.g = g;
      pt.b = b;
      pt.a = a;
      tf->dirty = true;
      return;
    }
  }
  tf->points.push_back({scalar, r, g, b, a});
  std::sort(tf->points.begin(), tf->points.end(),
            [](const ControlPoint &a, const ControlPoint &b) {
              return a.scalar < b.scalar;
            });
  tf->dirty = true;
}

constexpr int TF_LUT_SIZE = 1024;

FLYWAVE_VDB_API void vdb_transfer_function_build(vdb_transfer_function_t *tf) {
  const auto &pts = tf->points;
  if (pts.empty())
    return;

  for (int i = 0; i < TF_LUT_SIZE; i++) {
    float s = float(i) / float(TF_LUT_SIZE - 1);

    if (s <= pts.front().scalar) {
      tf->lut[i * 4 + 0] = pts.front().r;
      tf->lut[i * 4 + 1] = pts.front().g;
      tf->lut[i * 4 + 2] = pts.front().b;
      tf->lut[i * 4 + 3] = pts.front().a;
      continue;
    }
    if (s >= pts.back().scalar) {
      tf->lut[i * 4 + 0] = pts.back().r;
      tf->lut[i * 4 + 1] = pts.back().g;
      tf->lut[i * 4 + 2] = pts.back().b;
      tf->lut[i * 4 + 3] = pts.back().a;
      continue;
    }

    for (size_t j = 0; j < pts.size() - 1; j++) {
      if (s >= pts[j].scalar && s <= pts[j + 1].scalar) {
        float t = (s - pts[j].scalar) / (pts[j + 1].scalar - pts[j].scalar);
        tf->lut[i * 4 + 0] = pts[j].r + t * (pts[j + 1].r - pts[j].r);
        tf->lut[i * 4 + 1] = pts[j].g + t * (pts[j + 1].g - pts[j].g);
        tf->lut[i * 4 + 2] = pts[j].b + t * (pts[j + 1].b - pts[j].b);
        tf->lut[i * 4 + 3] = pts[j].a + t * (pts[j + 1].a - pts[j].a);
        break;
      }
    }
  }
  tf->dirty = false;
}

FLYWAVE_VDB_API void vdb_transfer_function_map(
    vdb_transfer_function_t *tf,
    float scalar, float *r, float *g, float *b, float *a) {
  if (tf->points.empty()) {
    *r = *g = *b = *a = 0.0f;
    return;
  }
  if (tf->dirty) {
    vdb_transfer_function_build(tf);
  }
  if (scalar < 0.0f)
    scalar = 0.0f;
  if (scalar > 1.0f)
    scalar = 1.0f;

  float f_idx = scalar * float(TF_LUT_SIZE - 1);
  int idx = int(f_idx);
  if (idx < 0) idx = 0;
  if (idx >= TF_LUT_SIZE - 1) idx = TF_LUT_SIZE - 2;

  float t = f_idx - float(idx);
  *r = tf->lut[idx * 4 + 0] + t * (tf->lut[(idx + 1) * 4 + 0] - tf->lut[idx * 4 + 0]);
  *g = tf->lut[idx * 4 + 1] + t * (tf->lut[(idx + 1) * 4 + 1] - tf->lut[idx * 4 + 1]);
  *b = tf->lut[idx * 4 + 2] + t * (tf->lut[(idx + 1) * 4 + 2] - tf->lut[idx * 4 + 2]);
  *a = tf->lut[idx * 4 + 3] + t * (tf->lut[(idx + 1) * 4 + 3] - tf->lut[idx * 4 + 3]);
}

/* ═══════════════════════════════════════════════════════════════════
   Volume Ray March
   ═══════════════════════════════════════════════════════════════════ */

FLYWAVE_VDB_API bool vdb_grid_ray_march(
    vdb_float_grid_t *grid,
    double ox, double oy, double oz,
    double dx, double dy, double dz,
    vdb_transfer_function_t *tfn,
    double step_size, double max_dist,
    float *out_r, float *out_g, float *out_b, float *out_a,
    double *out_depth) {

  if (!grid || !grid->ptr || !grid->ptr->has_grid())
    return false;

  auto g = grid->ptr->grid();
  if (!g)
    return false;

  if (tfn->dirty)
    vdb_transfer_function_build(tfn);

  // Normalize direction
  double len = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (len < 1e-12)
    return false;
  dx /= len;
  dy /= len;
  dz /= len;

  openvdb::tools::GridSampler<float_grid, openvdb::tools::BoxSampler> sampler(*g);

  float acc_r = 0.0f, acc_g = 0.0f, acc_b = 0.0f, acc_a = 0.0f;
  double t = 0.0;
  double closest_hit = max_dist;
  bool hit = false;

  // March until opacity saturated or max distance
  while (t < max_dist && acc_a < 0.99f) {
    double px = ox + t * dx;
    double py = oy + t * dy;
    double pz = oz + t * dz;

    // Sample the grid with trilinear interpolation
    float val = sampler.wsSample(openvdb::Vec3R(px, py, pz));

    // Map through transfer function
    float sr, sg, sb, sa;
    vdb_transfer_function_map(tfn, val, &sr, &sg, &sb, &sa);

    // Pre-multiplied alpha compositing
    float factor = (1.0f - acc_a);
    acc_r += factor * sr;
    acc_g += factor * sg;
    acc_b += factor * sb;
    acc_a += factor * sa;

    if (!hit && sa > 0.01f) {
      hit = true;
      closest_hit = t;
    }

    t += step_size;
  }

  if (!hit || acc_a < 0.01f)
    return false;

  *out_r = acc_r;
  *out_g = acc_g;
  *out_b = acc_b;
  *out_a = acc_a;
  *out_depth = closest_hit;

  return true;
}

FLYWAVE_VDB_API bool vdb_grid_ray_march_many(
    vdb_float_grid_t *grid,
    const double *origins,  /* [n][3] */
    const double *dirs,     /* [n][3] */
    vdb_transfer_function_t *tfn,
    double step_size, double max_dist,
    int n,
    float *colors,       /* [n][4] */
    double *depths) {

  if (!grid || !grid->ptr || !grid->ptr->has_grid())
    return false;
  if (tfn->dirty)
    vdb_transfer_function_build(tfn);

  bool any_hit = false;
  for (int i = 0; i < n; i++) {
    float r, g, b, a;
    double d;
    if (vdb_grid_ray_march(grid,
                           origins[i * 3 + 0], origins[i * 3 + 1], origins[i * 3 + 2],
                           dirs[i * 3 + 0], dirs[i * 3 + 1], dirs[i * 3 + 2],
                           tfn, step_size, max_dist,
                           &r, &g, &b, &a, &d)) {
      colors[i * 4 + 0] = r;
      colors[i * 4 + 1] = g;
      colors[i * 4 + 2] = b;
      colors[i * 4 + 3] = a;
      depths[i] = d;
      any_hit = true;
    } else {
      colors[i * 4 + 0] = 0;
      colors[i * 4 + 1] = 0;
      colors[i * 4 + 2] = 0;
      colors[i * 4 + 3] = 0;
      depths[i] = max_dist;
    }
  }
  return any_hit;
}

/* ═══════════════════════════════════════════════════════════════════
   Multi-View Camera
   ═══════════════════════════════════════════════════════════════════ */

FLYWAVE_VDB_API void vdb_camera_look_at(
    vdb_camera_t *cam,
    double eye_x, double eye_y, double eye_z,
    double target_x, double target_y, double target_z,
    double up_x, double up_y, double up_z,
    double fov) {
  cam->eye[0] = eye_x;
  cam->eye[1] = eye_y;
  cam->eye[2] = eye_z;
  cam->target[0] = target_x;
  cam->target[1] = target_y;
  cam->target[2] = target_z;
  cam->up[0] = up_x;
  cam->up[1] = up_y;
  cam->up[2] = up_z;
  cam->fov = fov;
}

FLYWAVE_VDB_API void vdb_camera_orbit(
    vdb_camera_t *cam,
    double center_x, double center_y, double center_z,
    double radius, double azimuth_deg, double elevation_deg,
    double fov) {
  double az_rad = azimuth_deg * M_PI / 180.0;
  double el_rad = elevation_deg * M_PI / 180.0;

  double eye_x = center_x + radius * std::cos(el_rad) * std::sin(az_rad);
  double eye_y = center_y + radius * std::sin(el_rad);
  double eye_z = center_z + radius * std::cos(el_rad) * std::cos(az_rad);

  // up vector: always Y-up (adjust as needed)
  double up_x = 0.0, up_y = 1.0, up_z = 0.0;

  vdb_camera_look_at(cam, eye_x, eye_y, eye_z,
                     center_x, center_y, center_z,
                     up_x, up_y, up_z, fov);
}

FLYWAVE_VDB_API int vdb_camera_orbit_path(
    vdb_camera_t *cameras, int num_cameras,
    double center_x, double center_y, double center_z,
    double radius, double fov) {
  if (!cameras || num_cameras <= 0)
    return 0;

  for (int i = 0; i < num_cameras; i++) {
    double az = 360.0 * double(i) / double(num_cameras);
    double el = 30.0 * std::sin(az * M_PI / 180.0); // slight elevation variation
    vdb_camera_orbit(&cameras[i], center_x, center_y, center_z,
                     radius, az, el, fov);
  }
  return num_cameras;
}
