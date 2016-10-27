#include "model.h"
#include <math.h>

// BP
void model_step(model_t* m) {
    m->X = - sin(m->theta) * MODEL_G;
    m->Y = sin(m->phi) * MODEL_G;
    m->Z = - MODEL_B * (m->ae1sq + m->ae2sq + m->ae3sq + m->ae4sq) + MODEL_G;

    m->ax = m->X / MODEL_M_S;
    m->ay = m->Y / MODEL_M_S;
    m->az = m->Z / MODEL_M_S;

    m->L = MODEL_B * (- m->ae2sq + m->ae4sq);
    m->M = MODEL_B * (m->ae1sq - m->ae3sq);
    m->N = MODEL_D * (-m->ae1sq + m->ae2sq - m->ae3sq + m->ae4sq);

    m->u = (MODEL_T / MODEL_M) * m->X + m->u;
    m->v = (MODEL_T / MODEL_M) * m->Y + m->v;
    m->w = (MODEL_T / MODEL_M) * m->Z + m->w;

    m->p = (MODEL_T / MODEL_I_L) * m->L + m->p;
    m->q = (MODEL_T / MODEL_I_M) * m->M + m->q;
    m->r = (MODEL_T / MODEL_I_N) * m->N + m->r;

    m->x = MODEL_T * m->u + m->x;
    m->y = MODEL_T * m->v + m->y;
    m->z = MODEL_T * m->w + m->z;

    m->phi = MODEL_T * m->p + m->phi;
    m->theta = MODEL_T * m->q + m->theta;
    m->psi = MODEL_T * m->r + m->psi;
}

// BP
void model_init(model_t* m) {
    m->X = 0;
    m->Y = 0;
    m->Z = 0;

    m->ax = 0;
    m->ay = 0;
    m->az = 0;

    m->L = 0;
    m->M = 0;
    m->N = 0;

    m->u = 0;
    m->v = 0;
    m->w = 0;

    m->p = 0;
    m->q = 0;
    m->r = 0;

    m->x = 0;
    m->y = 0;
    m->z = 0;

    m->phi = 0;
    m->theta = 0;
    m->psi = 0;

    m->ae1sq = 0;
    m->ae2sq = 0;
    m->ae3sq = 0;
    m->ae4sq = 0;
}
