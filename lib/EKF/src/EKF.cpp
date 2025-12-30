#include "EKF.h"
const float PI = 3.14159265f;

static float wrapAngle(float a) {
    while (a > PI)  a -= 2.0f*PI;
    while (a < -PI) a += 2.0f*PI;
    return a;
}

EKF::EKF() {
    x.fill(0.0f);

    for(int i=0;i<STATE_SIZE;i++){
        for(int j=0;j<STATE_SIZE;j++){
            P[i][j] = (i==j) ? 1.0f : 0.0f;
            Q[i][j] = (i==j) ? 0.001f : 0.0f;
        }
    }

    for(int i=0;i<MEAS_SIZE;i++)
        for(int j=0;j<MEAS_SIZE;j++)
            R[i][j] = (i==j) ? 0.05f : 0.0f;

    for(int i=0;i<MEAS_SIZE;i++)
        for(int j=0;j<STATE_SIZE;j++)
            H[i][j] = (i==j) ? 1.0f : 0.0f;
}

void EKF::predict(const StateVector &u, float dt){
    // Integrate angles
    for(int i=0;i<3;i++)
        x[i] += u[i]*dt;
    // Set rates
    for(int i=3;i<6;i++)
        x[i] = u[i-3];
    // Integrate velocities
    for(int i=6;i<9;i++)
        x[i] += u[i-3]*dt;

    P = matAdd(P,Q);
}

void EKF::update(const MeasVector &z){
    MeasVector y{}; //Diff in meas and pridict
    for(int i=0;i<MEAS_SIZE;i++){
        y[i] = z[i];
        for(int j=0;j<STATE_SIZE;j++)
            y[i] -= H[i][j]*x[j];
    }

    MeasCov S{}; //Uncertainty
    for(int i=0;i<MEAS_SIZE;i++)
        for(int j=0;j<MEAS_SIZE;j++){
            S[i][j] = R[i][j]; // noise
            for(int k=0;k<STATE_SIZE;k++)
                S[i][j] += H[i][k]*P[k][k]*H[j][k];
        }

    MeasMatrix K{};
    for(int i=0;i<STATE_SIZE;i++)
        for(int j=0;j<MEAS_SIZE;j++)
            K[j][i] = P[i][i]*H[j][i]/S[j][j]; //Kalman Gain

    for(int i=0;i<STATE_SIZE;i++)
        for(int j=0;j<MEAS_SIZE;j++)
            x[i] += K[j][i]*y[j];
    
    for(int i=0;i<3;i++)
        x[i] = wrapAngle(x[i]);

    for(int i=0;i<STATE_SIZE;i++)
        for(int j=0;j<STATE_SIZE;j++)
            P[i][j] *= (1.0f - K[j][i]*H[j][i]);
}

EKF::StateMatrix EKF::matAdd(const StateMatrix &a, const StateMatrix &b) const{
    StateMatrix res{};
    for(int i=0;i<STATE_SIZE;i++)
        for(int j=0;j<STATE_SIZE;j++)
            res[i][j] = a[i][j] + b[i][j];
    return res;
}

