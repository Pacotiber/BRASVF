// Arm kinematics: direct translation of the MATLAB code (fk3, ik_complet, ik_sur).
// Frame: X = forward, Y = up, Z = depth (to the right when the base angle is 0).
// Origin: on the floor, on the rotation axis of the base. Units: mm and degrees.
#pragma once
#include <math.h>

struct Arm {
  float L1;   // shoulder axis -> elbow axis
  float L2;   // elbow axis -> wrist axis
  float L3;   // wrist axis -> fingertips, along the gripper axis
  float HP;   // fingertips offset perpendicular to the gripper axis
  float R0;   // shoulder axis in front of the base axis
  float H0;   // height of the shoulder axis above the floor
  float C;    // total sideways offset (sum of the offsets)
};

// joint angles in the model (degrees)
struct JointAngles {
  float base;      // base rotation (positive = towards +Z)
  float shoulder;  // arm 1 relative to the horizontal (positive = upwards)
  float elbow;     // arm 2 relative to arm 1
  float wrist;     // gripper axis relative to arm 2
};

const float DEG = 57.2957795f;  // 180 / pi

inline float wrap180(float a) {            // brings an angle between -180 and 180
  while (a > 180.0f) a -= 360.0f;
  while (a <= -180.0f) a += 360.0f;
  return a;
}

// Forward kinematics: angles -> fingertip position.
// minHeight receives the height of the lowest point of the arm (to avoid the floor).
inline void fk(const Arm& b, const JointAngles& q, float& X, float& Y, float& Z, float& minHeight) {
  float qb = q.base / DEG, t1 = q.shoulder / DEG, t2 = q.elbow / DEG, t3 = q.wrist / DEG;
  float phi = t1 + t2 + t3;                              // gripper axis angle
  // 1) arm in its plane: reach A and height H of each point
  float hElbow  = b.H0 + b.L1 * sinf(t1);
  float hWrist  = hElbow + b.L2 * sinf(t1 + t2);
  float hOffset = hWrist + b.HP * cosf(phi);
  float hTip    = hOffset + b.L3 * sinf(phi);
  float A = b.R0 + b.L1 * cosf(t1) + b.L2 * cosf(t1 + t2) - b.HP * sinf(phi) + b.L3 * cosf(phi);
  // 2) base rotation: (reach, side) rotate, the height does not change
  X = A * cosf(qb) - b.C * sinf(qb);
  Y = hTip;
  Z = A * sinf(qb) + b.C * cosf(qb);
  minHeight = fminf(fminf(hElbow, hWrist), fminf(hOffset, hTip));
}

// Planar arm: reach x and height y measured from the shoulder, phi = wanted gripper angle,
// elbow = -1 or +1 (the two solutions).
inline bool planarIk(const Arm& b, float x, float y, float phi, int elbow,
                     float& t1, float& t2, float& t3) {
  float xw = x - b.L3 * cosf(phi) + b.HP * sinf(phi);    // step back along the gripper...
  float yw = y - b.L3 * sinf(phi) - b.HP * cosf(phi);    // ...then along the perpendicular offset
  float D = (b.L1 * b.L1 + b.L2 * b.L2 - xw * xw - yw * yw) / (2.0f * b.L1 * b.L2);
  if (fabsf(D) > 1.0f) return false;                     // out of reach
  float alpha = acosf(D);
  t2 = elbow * ((float)M_PI - alpha);
  t1 = atan2f(yw, xw) - atan2f(b.L2 * sinf(t2), b.L1 + b.L2 * cosf(t2));
  t3 = phi - t1 - t2;
  return true;
}

// Function provided by the main program: margin (deg) between the angles and the nearest limit,
// negative if an angle is outside the servo limits.
typedef float (*LimitMargin)(const JointAngles& q);

// Full inverse kinematics (ik_sur): first tries the arm turned towards the target, then turned the other way
// (target reached over the shoulder); for each, the gripper flat (phi = 0) then more and more tilted
// (-1, +1, -2, +2... deg up to +-phiMax), and both elbow solutions.
// Among the solutions that respect the limits and the floor, the one furthest from the limits is kept
// (for equal margins, the flattest gripper). Keeping the first one found pushed the elbow against its
// limit for almost every target: only the base seemed to move.
// If phiFixed is true, only the angle phiDeg is tried.
inline bool ik(const Arm& b, float X, float Y, float Z, bool phiFixed, float phiDeg,
               float floorHeight, LimitMargin margin, JointAngles& q, float& phiChosen,
               float phiMax = 180.0f) {
  const int nPhi = phiFixed ? 1 : 2 * (int)phiMax + 1;

  float rho = sqrtf(X * X + Z * Z);                      // base -> target distance seen from above
  if (rho < fabsf(b.C)) return false;                    // too close to the base axis
  float yb = Y - b.H0;                                   // height measured from the shoulder

  for (int side = 1; side >= -1; side -= 2) {            // +1: target in front of the arm, -1: behind
    float a  = side * sqrtf(rho * rho - b.C * b.C);      // real reach in the arm plane
    float qb = atan2f(Z, X) - atan2f(b.C, a);            // aim at the target, corrected for the offset
    float xb = a - b.R0;                                 // distance measured from the shoulder
    bool found = false;
    float best = 0;

    for (int i = 0; i < nPhi; i++) {
      float phi = (phiFixed ? phiDeg : (i % 2 ? -(i + 1) / 2 : i / 2)) / DEG;   // 0, -1, +1, -2, +2...
      for (int elbow = -1; elbow <= 1; elbow += 2) {
        float t1, t2, t3;
        if (!planarIk(b, xb, yb, phi, elbow, t1, t2, t3)) continue;
        JointAngles attempt = { wrap180(qb * DEG), wrap180(t1 * DEG),
                                wrap180(t2 * DEG), wrap180(t3 * DEG) };
        float m = margin(attempt);
        if (m < 0) continue;                             // outside the servo limits
        if (found && m <= best) continue;                // not better than the solution already kept
        float Xc, Yc, Zc, hMin;
        fk(b, attempt, Xc, Yc, Zc, hMin);
        if (hMin < floorHeight) continue;                // a point of the arm would go below the floor
        q = attempt;
        phiChosen = wrap180(phi * DEG);
        best = m;
        found = true;
      }
    }
    if (found) return true;                              // over the shoulder only as a last resort
  }
  return false;
}
