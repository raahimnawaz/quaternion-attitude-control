#ifndef QUAT_H
#define QUAT_H

#include <Arduino.h>

// Conventions (blueprint §1) - these must match the plant exactly:
//   storage order   q = [w, x, y, z]   scalar first
//   algebra         Hamilton, not JPL
//   meaning of q    rotates a vector from the BODY frame into the WORLD frame
struct Quat { float w, x, y, z; };

Quat  quat_mul(const Quat& q, const Quat& p);
Quat  quat_conj(const Quat& q);
float quat_norm(const Quat& q);
void  quat_normalize(Quat& q);
Quat  quat_from_axis_angle(float ax, float ay, float az, float rad);
void  quat_to_euler(const Quat& q, float& roll, float& pitch, float& yaw);

// M2 gate. Prints one "FAIL: <case>" line per failing case and returns true
// only if every case passed. Caller prints ALL TESTS PASS - CI waits on that.
bool quat_selftest();

#endif
