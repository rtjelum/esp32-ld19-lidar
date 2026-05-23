#pragma once
// Iterative-Closest-Point localization against a RAM-resident, growable map.
//
// Lifecycle:
//   1. loc_map_seed_add(...) — accumulate the first ~20 stationary scans at
//      pose (0,0,0) to bootstrap the reference map.
//   2. loc_run(...) — refine pose against the current map (closed-form 2D
//      rigid fit, no SVD lib needed).
//   3. loc_map_expand(...) — once pose is trusted, fold in scan points that
//      are far from any existing map point to fill in newly-seen geometry.
//
// All math is float, brute-force NN. Designed for 5–10 Hz updates on a
// 240 MHz ESP32-S3.

#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

struct Pose2D { float x; float y; float theta; };
struct LocalizeResult { Pose2D pose; float err; int inliers; int iters; };

// Caps to keep per-frame work bounded.
#define LOC_MAX_ITERS 6
#define LOC_MAX_SRC   220
#define LOC_MAP_CAP   6000   // ≈ 48 KB of float pairs

// Squared-distance gate (m²). First ICP iteration uses LOOSE so a drifted
// pose can re-acquire; subsequent iterations tighten.
static const float LOC_GATE2_LOOSE = 1.0f;
static const float LOC_GATE2_TIGHT = 0.09f;  // 30 cm

// Default dedup spacing when growing the map: don't add points within 5 cm
// of an existing one. Keeps the map from bloating with redundant returns.
static const float LOC_MAP_MIN_SPACING = 0.05f;
static const float LOC_MAP_MIN_SPACING2 = LOC_MAP_MIN_SPACING * LOC_MAP_MIN_SPACING;

// Map storage.
static float locMap[LOC_MAP_CAP][2];
static int   locMapN = 0;

static inline void loc_map_clear() { locMapN = 0; }
static inline int  loc_map_count() { return locMapN; }

// Brute-force nearest reference point. Returns squared distance.
static inline float loc_nearest(float qx, float qy, int& bestIdx) {
  float best = 1e30f;
  int bi = -1;
  for (int i = 0; i < locMapN; i++) {
    float dx = qx - locMap[i][0], dy = qy - locMap[i][1];
    float d2 = dx*dx + dy*dy;
    if (d2 < best) { best = d2; bi = i; }
  }
  bestIdx = bi;
  return best;
}

// Add a point if no existing point is within sqrt(minSpacing2) of it.
// Returns true if added. Silently drops when the map is full.
static inline bool loc_map_add(float x, float y, float minSpacing2) {
  if (locMapN >= LOC_MAP_CAP) return false;
  for (int i = 0; i < locMapN; i++) {
    float dx = x - locMap[i][0], dy = y - locMap[i][1];
    if (dx*dx + dy*dy < minSpacing2) return false;
  }
  locMap[locMapN][0] = x;
  locMap[locMapN][1] = y;
  locMapN++;
  return true;
}

// Seed the map from a sensor-frame scan at the assumed origin pose (0,0,0).
// Use during initial calibration frames while the robot is stationary.
static inline int loc_map_seed_add(const float* sx, const float* sy, int n) {
  int added = 0;
  for (int i = 0; i < n; i++) {
    if (loc_map_add(sx[i], sy[i], LOC_MAP_MIN_SPACING2)) added++;
  }
  return added;
}

// Expand the map: for each scan point transformed to world frame, add it
// only if its nearest existing map neighbor is farther than `minNew` meters.
// Caller should only invoke this when pose is trusted (low ICP err, high
// inliers); otherwise misaligned points pollute the map.
static inline int loc_map_expand(const float* sx, const float* sy, int n,
                                 Pose2D pose, float minNew) {
  if (n > LOC_MAX_SRC) n = LOC_MAX_SRC;
  float c = cosf(pose.theta), s = sinf(pose.theta);
  float minNew2 = minNew * minNew;
  int added = 0;
  for (int i = 0; i < n; i++) {
    float xw = c*sx[i] - s*sy[i] + pose.x;
    float yw = s*sx[i] + c*sy[i] + pose.y;
    int bi;
    float d2 = loc_nearest(xw, yw, bi);
    if (d2 > minNew2) {
      // Use the per-point gate (minNew2) for dedup so we don't reject points
      // that are "new enough" by the expansion criterion.
      if (loc_map_add(xw, yw, minNew2)) added++;
    }
  }
  return added;
}

// One ICP iteration. Returns mean residual (m) over inliers. Updates pose
// in place via a closed-form 2D rigid fit on centered correspondences:
//   theta_delta = atan2( Σ(s_x·d_y − s_y·d_x), Σ(s_x·d_x + s_y·d_y) )
static inline float loc_icp_step(const float* sx, const float* sy, int n,
                                 Pose2D& pose, float gate2, int& inliersOut) {
  float c = cosf(pose.theta), s = sinf(pose.theta);

  static float wx[LOC_MAX_SRC], wy[LOC_MAX_SRC];
  static float dx[LOC_MAX_SRC], dy[LOC_MAX_SRC];
  float src_mx = 0, src_my = 0, dst_mx = 0, dst_my = 0;
  int   nIn = 0;
  float errSum = 0;

  for (int i = 0; i < n; i++) {
    float xw = c*sx[i] - s*sy[i] + pose.x;
    float yw = s*sx[i] + c*sy[i] + pose.y;
    int bi;
    float d2 = loc_nearest(xw, yw, bi);
    if (d2 < gate2) {
      wx[nIn] = xw; wy[nIn] = yw;
      dx[nIn] = locMap[bi][0]; dy[nIn] = locMap[bi][1];
      src_mx += xw; src_my += yw;
      dst_mx += locMap[bi][0]; dst_my += locMap[bi][1];
      errSum += sqrtf(d2);
      nIn++;
    }
  }
  inliersOut = nIn;
  if (nIn < 8) return errSum / (nIn ? nIn : 1);

  src_mx /= nIn; src_my /= nIn;
  dst_mx /= nIn; dst_my /= nIn;

  float num = 0, den = 0;
  for (int i = 0; i < nIn; i++) {
    float sxc = wx[i] - src_mx, syc = wy[i] - src_my;
    float dxc = dx[i] - dst_mx, dyc = dy[i] - dst_my;
    num += sxc * dyc - syc * dxc;
    den += sxc * dxc + syc * dyc;
  }
  float dth = atan2f(num, den);
  float cd = cosf(dth), sd = sinf(dth);
  float tdx = dst_mx - (cd*src_mx - sd*src_my);
  float tdy = dst_my - (sd*src_mx + cd*src_my);

  float nx = cd*pose.x - sd*pose.y + tdx;
  float ny = sd*pose.x + cd*pose.y + tdy;
  pose.x = nx; pose.y = ny;
  pose.theta = atan2f(sinf(pose.theta + dth), cosf(pose.theta + dth));
  return errSum / nIn;
}

static inline LocalizeResult loc_run(const float* sx, const float* sy, int n,
                                     Pose2D init) {
  LocalizeResult R;
  R.pose = init; R.err = 0.0f; R.inliers = 0; R.iters = 0;
  if (n < 10 || locMapN < 50) return R;
  if (n > LOC_MAX_SRC) n = LOC_MAX_SRC;
  Pose2D p = init;
  float err = 0;
  int inliers = 0;
  for (int it = 0; it < LOC_MAX_ITERS; it++) {
    float prevX = p.x, prevY = p.y, prevTh = p.theta;
    float gate2 = (it == 0) ? LOC_GATE2_LOOSE : LOC_GATE2_TIGHT;
    err = loc_icp_step(sx, sy, n, p, gate2, inliers);
    R.iters = it + 1;
    float dt = p.theta - prevTh;
    if (dt > PI)  dt -= 2*PI;
    if (dt < -PI) dt += 2*PI;
    if (fabsf(p.x - prevX) < 1e-3f && fabsf(p.y - prevY) < 1e-3f && fabsf(dt) < 1e-3f) break;
  }
  R.pose = p;
  R.err = err;
  R.inliers = inliers;
  return R;
}
