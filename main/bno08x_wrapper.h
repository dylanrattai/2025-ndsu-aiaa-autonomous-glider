#ifndef BNO08X_WRAPPER_H
#define BNO08X_WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

int bno08xInit(void);
double getRoll(void);
double getPitch(void);
double getYaw(void);
double getRollAccel(void);
double getPitchAccel(void);
double getYawAccel(void);
double getXAccel(void);
double getYAccel(void);
double getZAccel(void);
int getMagnometer(float* mx, float* my, float* mz);

#ifdef __cplusplus
}
#endif

#endif
