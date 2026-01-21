/*************************************************************************************************************
 *  Template project for Extended Kalman Filter library
 * 
 * See https://github.com/pronenewbits for more!
 ************************************************************************************************************/
#include <Wire.h>
#include <math.h>
#include "konfig.h"
#include "matrix.h"
#include "ekf.h"
#include "Adafruit9DOF.h"

Adafruit9DOF imu;

/* Global magnetic reference (inertial frame) and init flag */
float g_mref_x = 1.0f, g_mref_y = 0.0f, g_mref_z = 0.0f;
bool g_mref_init = false;

/* ============================================ EKF Variables/function declaration ============================================ */
#define SS_X_LEN 7   // quaternion (4) + gyro bias (3)
#define SS_Z_LEN 6   // accel + mag
#define SS_U_LEN 3   // gyro

/* EKF initialization constant -------------------------------------------------------------------------------------- */
#define P_INIT      (10.)
#define Q_INIT      (1e-6)
#define R_INIT      (0.0015)
/* P, Q, R matrices (will be initialized in setup) --------------------------------------------------------------- */
Matrix EKF_PINIT(SS_X_LEN, SS_X_LEN);
Matrix EKF_QINIT(SS_X_LEN, SS_X_LEN);
Matrix EKF_RINIT(SS_Z_LEN, SS_Z_LEN);
/* Nonlinear & linearization function ------------------------------------------------------------------------------- */
bool Main_bUpdateNonlinearX(Matrix& X_Next, const Matrix& X, const Matrix& U);
bool Main_bUpdateNonlinearY(Matrix& Y, const Matrix& X, const Matrix& U);
bool Main_bCalcJacobianF(Matrix& Jf, const Matrix& X, const Matrix& U);
bool Main_bCalcJacobianH(Matrix& Jh, const Matrix& X, const Matrix& U);
/* EKF variables ---------------------------------------------------------------------------------------------------- */
Matrix X(SS_X_LEN, 1);
Matrix Y(SS_Z_LEN, 1);
Matrix U(SS_U_LEN, 1);
/* EKF system declaration ------------------------------------------------------------------------------------------- */
/* EKF instance will be created in setup after matrices are initialized */
EKF* pEKF_IMU = nullptr;



/* ========================================= Auxiliary variables/function declaration ========================================= */
/* Use millis() for timing instead of elapsedMillis */
unsigned long prevMillisEKF = 0;
uint64_t u64compuTime;
char bufferTxSer[100];



void setup() {
    /* serial to display data */
    Serial.begin(115200);
    while(!Serial) {}
    
    /* Init IMU */
    if (!imu.begin()) {
        Serial.println("IMU init failed");
    } else {
        Serial.println("IMU init ok");
    }

    /* Take a short set of samples to compute sensor averages for initial bias/mag reference */
    const int CAL_SAMPLES = 100;
    float sum_gx=0, sum_gy=0, sum_gz=0;
    float sum_mx=0, sum_my=0, sum_mz=0;
    float sum_ax=0, sum_ay=0, sum_az=0;
    for (int i=0;i<CAL_SAMPLES;i++) {
        float ax,ay,az,gx,gy,gz,mx,my,mz;
        imu.readAll(ax, ay, az, gx, gy, gz, mx, my, mz);
        sum_gx += gx; sum_gy += gy; sum_gz += gz;
        sum_mx += mx; sum_my += my; sum_mz += mz;
        sum_ax += ax; sum_ay += ay; sum_az += az;
        delay(SS_DT_MILIS);
    }
    float avg_gx = sum_gx / CAL_SAMPLES;
    float avg_gy = sum_gy / CAL_SAMPLES;
    float avg_gz = sum_gz / CAL_SAMPLES;
    float avg_mx = sum_mx / CAL_SAMPLES;
    float avg_my = sum_my / CAL_SAMPLES;
    float avg_mz = sum_mz / CAL_SAMPLES;
    float avg_ax = sum_ax / CAL_SAMPLES;
    float avg_ay = sum_ay / CAL_SAMPLES;
    float avg_az = sum_az / CAL_SAMPLES;

    /* Initialize magnetometer reference (inertial) from averaged body measurement
       assuming initial attitude is identity (q = [1,0,0,0]) */
    float mnorm = sqrtf(avg_mx*avg_mx + avg_my*avg_my + avg_mz*avg_mz);
    if (mnorm > 1e-6f) {
        g_mref_x = avg_mx / mnorm;
        g_mref_y = avg_my / mnorm;
        g_mref_z = avg_mz / mnorm;
        g_mref_init = true;
    }

     /* Initial state: identity quaternion, set gyro bias from averaged samples */
     X.vSetToZero();
     X(0,0) = 1.0; /* q0 */
     const float DEG2RAD = 3.14159265358979323846f/180.0f;
     X(4,0) = avg_gx * DEG2RAD;
     X(5,0) = avg_gy * DEG2RAD;
     X(6,0) = avg_gz * DEG2RAD;

     /* Initialize EKF covariance matrices properly (diagonal).
         Use different values for quaternion states vs bias states. */
     EKF_PINIT.vSetToZero();
     for (int i=0;i<4;i++) EKF_PINIT(i,i) = 1e-3f; /* quat uncertainty */
     for (int i=4;i<7;i++) EKF_PINIT(i,i) = 1.0f;   /* bias uncertainty */

     EKF_QINIT.vSetToZero();
     for (int i=0;i<4;i++) EKF_QINIT(i,i) = 1e-6f;  /* quat process noise */
    for (int i=4;i<7;i++) EKF_QINIT(i,i) = 1e-6f;  /* bias random walk (allow bias to adapt faster) */

     EKF_RINIT.vSetToZero();
    for (int i=0;i<SS_Z_LEN;i++) EKF_RINIT(i,i) = 5e-2f; /* accel & mag measurement noise (increase if noisy) */

    /* Create EKF instance now that matrices are initialized */
    if (pEKF_IMU == nullptr) {
        pEKF_IMU = new EKF(X, EKF_PINIT, EKF_QINIT, EKF_RINIT,
                           Main_bUpdateNonlinearX, Main_bUpdateNonlinearY, Main_bCalcJacobianF, Main_bCalcJacobianH);
    }

    /* initialize millis-based timer */
    prevMillisEKF = millis();
}


void loop() {
    if ((unsigned long)(millis() - prevMillisEKF) >= (unsigned long)SS_DT_MILIS) {
        prevMillisEKF = millis();
    
        /* ================== Read the sensor data / simulate the system here ================== */
        float ax=0, ay=0, az=0;
        float gx=0, gy=0, gz=0;
        float mx=0, my=0, mz=0;
        imu.readAll(ax, ay, az, gx, gy, gz, mx, my, mz);

        /* Fill input U (gyro) - convert deg/s to rad/s */
        const float DEG2RAD = 3.14159265358979323846f/180.0f;
        U(0,0) = gx * DEG2RAD;
        U(1,0) = gy * DEG2RAD;
        U(2,0) = gz * DEG2RAD;

        /* Fill measurement Y with normalized accel and mag vectors */
        float a_norm = sqrtf(ax*ax + ay*ay + az*az);
        if (a_norm > 1e-6f) {
            Y(0,0) = ax / a_norm;
            Y(1,0) = ay / a_norm;
            Y(2,0) = az / a_norm;
        }
        float m_norm = sqrtf(mx*mx + my*my + mz*mz);
        if (m_norm > 1e-6f) {
            Y(3,0) = mx / m_norm;
            Y(4,0) = my / m_norm;
            Y(5,0) = mz / m_norm;
        }
        /* ------------------ Read the sensor data / simulate the system here ------------------ */
        
        
        /* ============================= Update the Kalman Filter ============================== */
        u64compuTime = micros();
        if (!pEKF_IMU->bUpdate(Y, U)) {
            X.vSetToZero();
            pEKF_IMU->vReset(X, EKF_PINIT, EKF_QINIT, EKF_RINIT);
            Serial.println("Whoop ");
        }
        u64compuTime = (micros() - u64compuTime);
        /* ----------------------------- Update the Kalman Filter ------------------------------ */
        
        
        /* =========================== Print to serial (for plotting) ========================== */
        #if (1)
            /* Print: Computation time and roll/pitch/yaw (degrees) from EKF quaternion */
            Matrix Xest = pEKF_IMU->GetX();
            float q0 = Xest(0,0);
            float q1 = Xest(1,0);
            float q2 = Xest(2,0);
            float q3 = Xest(3,0);

            /* Euler angles (rad) from quaternion (scalar first) */
            float roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2));
            float pitch = asinf(fmaxf(-1.0f, fminf(1.0f, 2.0f*(q0*q2 - q3*q1))));
            float yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));

            const float RAD2DEG = 180.0f / 3.14159265358979323846f;
            float roll_d = roll * RAD2DEG;
            float pitch_d = pitch * RAD2DEG;
            float yaw_d = yaw * RAD2DEG;

            /* wrap to [-180,180] */
            auto wrap180 = [](float ang){
                while (ang > 180.0f) ang -= 360.0f;
                while (ang <= -180.0f) ang += 360.0f;
                return ang;
            };
            roll_d = wrap180(roll_d);
            pitch_d = wrap180(pitch_d);
            yaw_d = wrap180(yaw_d);

            snprintf(bufferTxSer, sizeof(bufferTxSer)-1, "%.3f RPY: %.2f %.2f %.2f", ((float)u64compuTime)/1000., roll_d, pitch_d, yaw_d);
            Serial.println(bufferTxSer);
        
        #endif
    }
}


bool Main_bUpdateNonlinearX(Matrix& X_Next, const Matrix& X, const Matrix& U)
{
    /* Insert the nonlinear update transformation here
     *          x(k+1) = f[x(k), u(k)]
     */
    /* State: [q0,q1,q2,q3, bx,by,bz] (quaternion scalar-first)
     * U: gyro rates [wx,wy,wz] (rad/s)
     */
    /* use SS_DT (float_prec) for small-step integration */
    const float_prec dt = SS_DT;

    float q0 = X(0,0);
    float q1 = X(1,0);
    float q2 = X(2,0);
    float q3 = X(3,0);

    float bx = X(4,0);
    float by = X(5,0);
    float bz = X(6,0);

    float wx = U(0,0) - bx;
    float wy = U(1,0) - by;
    float wz = U(2,0) - bz;

    /* quaternion derivative: q_dot = 0.5 * q ⊗ [0, w] */
    float q0d = -0.5f*(q1*wx + q2*wy + q3*wz);
    float q1d =  0.5f*( q0*wx + q2*wz - q3*wy);
    float q2d =  0.5f*( q0*wy + q3*wx - q1*wz);
    float q3d =  0.5f*( q0*wz + q1*wy - q2*wx);

    float q0n = q0 + q0d*dt;
    float q1n = q1 + q1d*dt;
    float q2n = q2 + q2d*dt;
    float q3n = q3 + q3d*dt;

    /* normalize quaternion */
    float qnorm = sqrtf(q0n*q0n + q1n*q1n + q2n*q2n + q3n*q3n);
    if (qnorm > 1e-12f) {
        q0n /= qnorm; q1n /= qnorm; q2n /= qnorm; q3n /= qnorm;
    }

    X_Next(0,0) = q0n;
    X_Next(1,0) = q1n;
    X_Next(2,0) = q2n;
    X_Next(3,0) = q3n;

    /* bias modeled as constant */
    X_Next(4,0) = bx;
    X_Next(5,0) = by;
    X_Next(6,0) = bz;

    return true;
}

bool Main_bUpdateNonlinearY(Matrix& Y, const Matrix& X, const Matrix& U)
{
    /* Insert the nonlinear measurement transformation here
     *          y(k)   = h[x(k), u(k)]
     */
    /* We predict normalized accel and mag directions in body frame from quaternion */
    float q0 = X(0,0);
    float q1 = X(1,0);
    float q2 = X(2,0);
    float q3 = X(3,0);

    /* rotation matrix (inertial -> body) from quaternion (q0 scalar) */
    float r11 = 1.0f - 2.0f*(q2*q2 + q3*q3);
    float r12 = 2.0f*(q1*q2 - q0*q3);
    float r13 = 2.0f*(q1*q3 + q0*q2);
    float r21 = 2.0f*(q1*q2 + q0*q3);
    float r22 = 1.0f - 2.0f*(q1*q1 + q3*q3);
    float r23 = 2.0f*(q2*q3 - q0*q1);
    float r31 = 2.0f*(q1*q3 - q0*q2);
    float r32 = 2.0f*(q2*q3 + q0*q1);
    float r33 = 1.0f - 2.0f*(q1*q1 + q2*q2);

    /* gravity unit vector in inertial frame (pointing down) */
    float gI_x = 0.0f, gI_y = 0.0f, gI_z = 1.0f;

    /* predicted accel (body) = R * gI  (use third column effectively) */
    float axp = r11*gI_x + r12*gI_y + r13*gI_z;
    float ayp = r21*gI_x + r22*gI_y + r23*gI_z;
    float azp = r31*gI_x + r32*gI_y + r33*gI_z;

    /* predicted magnetometer: need a reference magnetic field in inertial frame
     * We'll store/expect the reference in a static variable populated externally
     */
    /* Use global magnetometer reference initialized at startup */
    float mxp = r11*g_mref_x + r12*g_mref_y + r13*g_mref_z;
    float myp = r21*g_mref_x + r22*g_mref_y + r23*g_mref_z;
    float mzp = r31*g_mref_x + r32*g_mref_y + r33*g_mref_z;

    /* fill Y predicted (normalized vectors) */
    /* normalize predicted accel */
    float an = sqrtf(axp*axp + ayp*ayp + azp*azp);
    if (an > 1e-9f) { axp/=an; ayp/=an; azp/=an; }
    float mn = sqrtf(mxp*mxp + myp*myp + mzp*mzp);
    if (mn > 1e-9f) { mxp/=mn; myp/=mn; mzp/=mn; }

    Y(0,0) = axp;
    Y(1,0) = ayp;
    Y(2,0) = azp;
    Y(3,0) = mxp;
    Y(4,0) = myp;
    Y(5,0) = mzp;

    return true;
}

bool Main_bCalcJacobianF(Matrix& Jf, const Matrix& X, const Matrix& U)
{
    /* Insert the linearized update transformation here (i.e. Jacobian matrix of f[x(k), u(k)]) */
    /* Numerical differentiation */
    const float eps = 1e-6f;
    Matrix f0(SS_X_LEN,1);
    Matrix f1(SS_X_LEN,1);
    Main_bUpdateNonlinearX(f0, X, U);
    for (int i=0;i<SS_X_LEN;i++) {
        Matrix Xp = X;
        Xp(i,0) += eps;
        Main_bUpdateNonlinearX(f1, Xp, U);
        for (int j=0;j<SS_X_LEN;j++) {
            Jf(j,i) = (f1(j,0) - f0(j,0)) / eps;
        }
    }
    return true;
}

bool Main_bCalcJacobianH(Matrix& Jh, const Matrix& X, const Matrix& U)
{
    /* Insert the linearized measurement transformation here (i.e. Jacobian matrix of h[x(k), u(k)]) */
    const float eps = 1e-6f;
    Matrix y0(SS_Z_LEN,1);
    Matrix y1(SS_Z_LEN,1);
    Main_bUpdateNonlinearY(y0, X, U);
    for (int i=0;i<SS_X_LEN;i++) {
        Matrix Xp = X;
        Xp(i,0) += eps;
        Main_bUpdateNonlinearY(y1, Xp, U);
        for (int j=0;j<SS_Z_LEN;j++) {
            Jh(j,i) = (y1(j,0) - y0(j,0)) / eps;
        }
    }
    return true;
}





void SPEW_THE_ERROR(char const * str)
{
    #if (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_PC)
        cout << (str) << endl;
    #elif (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_EMBEDDED_ARDUINO)
        Serial.println(str);
    #else
        /* Silent function */
    #endif
    while(1);
}