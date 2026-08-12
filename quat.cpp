#include "quat.h"

Quat quat_mul(const Quat& q, const Quat& p) {
  return {
    q.w*p.w - q.x*p.x - q.y*p.y - q.z*p.z,
    q.w*p.x + q.x*p.w + q.y*p.z - q.z*p.y,
    q.w*p.y - q.x*p.z + q.y*p.w + q.z*p.x,
    q.w*p.z + q.x*p.y - q.y*p.x + q.z*p.w
  };
}

Quat quat_conj(const Quat& q) {
  return { q.w, -q.x, -q.y, -q.z };
}

float quat_norm(const Quat& q) {
  return sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
}

void quat_normalize(Quat& q) {
  float n = quat_norm(q);
  if (n > 1e-8f) {
    q.w /= n; q.x /= n; q.y /= n; q.z /= n;
  }
}

Quat quat_from_axis_angle(float ax, float ay, float az, float rad) {
  float half = rad / 2.0f;
  float s = sinf(half);
  return { cosf(half), ax * s, ay * s, az * s };
}

void quat_to_euler(const Quat& q, float& roll, float& pitch, float& yaw) {
  roll = atan2f(2.0f * (q.w*q.x + q.y*q.z), 1.0f - 2.0f * (q.x*q.x + q.y*q.y));

  float sinp = 2.0f * (q.w*q.y - q.z*q.x);
  // Load-bearing clamp: float error can push |sinp| a hair past 1, and asinf
  // of that is NaN. Costs nothing, saves an afternoon.
  if (sinp >  1.0f) sinp =  1.0f;
  if (sinp < -1.0f) sinp = -1.0f;
  pitch = asinf(sinp);

  yaw = atan2f(2.0f * (q.w*q.z + q.x*q.y), 1.0f - 2.0f * (q.y*q.y + q.z*q.z));
}

// --- M2 self-test ----------------------------------------------------------
// Counts failures rather than returning early, so one run reports every broken
// case. The old version returned true unconditionally and could not fail.

static int g_fails = 0;

static void check(bool cond, const __FlashStringHelper* msg) {
  if (!cond) {
    g_fails++;
    Serial.print(F("FAIL: "));
    Serial.println(msg);
  }
}

bool quat_selftest() {
  g_fails = 0;

  // A: q (x) q* == identity
  Quat q1 = { 0.5f, 0.5f, 0.5f, 0.5f };
  Quat idA = quat_mul(q1, quat_conj(q1));
  check(fabsf(idA.w - 1.0f) < 1e-5f && fabsf(idA.x) < 1e-5f &&
        fabsf(idA.y) < 1e-5f && fabsf(idA.z) < 1e-5f,
        F("A q*qconj != identity"));

  // B: 90 deg about X, then 90 deg about Y -> hand-computed (0.5, 0.5, 0.5, -0.5)
  Quat qx   = quat_from_axis_angle(1, 0, 0, PI / 2);
  Quat qy   = quat_from_axis_angle(0, 1, 0, PI / 2);
  Quat qseq = quat_mul(qy, qx);
  check(fabsf(qseq.w - 0.5f) < 1e-5f && fabsf(qseq.x - 0.5f) < 1e-5f &&
        fabsf(qseq.y - 0.5f) < 1e-5f && fabsf(qseq.z + 0.5f) < 1e-5f,
        F("B 90X then 90Y composition"));

  // C: two 180 deg rotations compose to identity. w = -1 is the same
  // orientation as w = +1, hence the double fabsf.
  Quat q180 = quat_from_axis_angle(0, 1, 0, PI);
  Quat q360 = quat_mul(q180, q180);
  check(fabsf(fabsf(q360.w) - 1.0f) < 1e-5f && fabsf(q360.y) < 1e-5f,
        F("C 180+180 != identity"));

  // D: axis-angle -> euler gives yaw = 90 deg, roll and pitch zero
  Quat qz = quat_from_axis_angle(0, 0, 1, PI / 2);
  float r, p, y;
  quat_to_euler(qz, r, p, y);
  check(fabsf(r) < 1e-5f && fabsf(p) < 1e-5f && fabsf(y - PI / 2) < 1e-5f,
        F("D axis-angle to euler yaw"));

  // E: gimbal-lock guard. Inject float error at pitch = 90 to force the clamp.
  Quat qg = quat_from_axis_angle(0, 1, 0, PI / 2);
  qg.w += 0.0001f;
  quat_to_euler(qg, r, p, y);
  check(!isnan(p), F("E gimbal lock returned NaN, clamp failed"));

  // F: normalize restores unit length
  Quat qn = { 2.0f, 0.0f, 0.0f, 0.0f };
  quat_normalize(qn);
  check(fabsf(quat_norm(qn) - 1.0f) < 1e-5f, F("F normalize != unit"));

  return g_fails == 0;
}
