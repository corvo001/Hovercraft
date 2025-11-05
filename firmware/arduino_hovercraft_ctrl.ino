
/*
  Hovercraft Dual Thruster Controller
  - Reads desired speed + heading from serial (v_ref [m/s], psi_ref [rad])
  - Reads IMU yaw rate and encoder/estimator speed (mocked placeholders)
  - Runs two PID loops and outputs to ESCs via PWM (1000–2000us)

  NOTE: Replace placeholders with your hardware reads/writes.
*/

#include <Arduino.h>

struct PID {
  float kp, ki, kd, dt, umin, umax;
  float integ, prev_e;
  PID(float kp_, float ki_, float kd_, float dt_, float umin_, float umax_)
    : kp(kp_), ki(ki_), kd(kd_), dt(dt_), umin(umin_), umax(umax_), integ(0), prev_e(0) {}
  float step(float e){
    integ += e * dt;
    float d = (e - prev_e) / dt;
    prev_e = e;
    float u = kp*e + ki*integ + kd*d;
    if(u > umax) u = umax;
    if(u < umin) u = umin;
    return u;
  }
  void reset(){ integ=0; prev_e=0; }
};

// --- Parameters (tune in field) ---
const float DT = 0.02; // 50 Hz
const float FMAX = 60.0; // map to PWM later
PID pid_speed(18, 4, 2.5, DT, -2*FMAX, 2*FMAX);
PID pid_heading(60, 0, 8, DT, -FMAX, FMAX);

float v_ref=0, psi_ref=0;
float v_now=0, psi_now=0; // mock sensors

// Map force [N] to PWM microseconds (linearized placeholder)
int forceToPWM(float F){
  // Assume F in [-FMAX, FMAX] maps to [1100, 1900]
  float alpha = (F + FMAX) / (2*FMAX);
  int pwm = (int)(1100 + alpha*(1900-1100));
  if(pwm<1000) pwm=1000;
  if(pwm>2000) pwm=2000;
  return pwm;
}

void setup(){
  Serial.begin(115200);
  // attach ESC pins, e.g., pin 5 and 6 with Servo library if used.
  // Servo escL, escR; escL.attach(5); escR.attach(6);
  // Send arming sequence if required by ESCs.
}

unsigned long last=0;

void loop(){
  if(Serial.available()){
    // Expected: "v psi\n"
    String line = Serial.readStringUntil('\n');
    float v, psi;
    if(sscanf(line.c_str(), "%f %f", &v, &psi)==2){
      v_ref = v;
      psi_ref = psi;
    }
  }

  unsigned long now = millis();
  if(now - last >= (unsigned long)(DT*1000)){
    last = now;

    // TODO: read sensors here to update v_now and psi_now

    // Controllers
    float e_psi = psi_ref - psi_now;
    while(e_psi > PI) e_psi -= 2*PI;
    while(e_psi < -PI) e_psi += 2*PI;

    float u_speed = pid_speed.step(v_ref - v_now);
    float u_yaw   = pid_heading.step(e_psi);

    float F_total = u_speed;
    float F_diff  = u_yaw;

    float F_L = 0.5*(F_total - F_diff);
    float F_R = 0.5*(F_total + F_diff);

    int pwmL = forceToPWM(F_L);
    int pwmR = forceToPWM(F_R);

    // escL.writeMicroseconds(pwmL);
    // escR.writeMicroseconds(pwmR);

    // Debug
    Serial.print("PWM_L:"); Serial.print(pwmL);
    Serial.print(" PWM_R:"); Serial.print(pwmR);
    Serial.print(" v_now:"); Serial.print(v_now,3);
    Serial.print(" psi_now:"); Serial.println(psi_now,3);
  }
}
