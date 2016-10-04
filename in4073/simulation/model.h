#ifndef MODEL_H
#define MODEL_H

typedef struct model {
    double x;
    double y;
    double z;
    double phi;
    double theta;
    double psi;
    double u;
    double v;
    double w;
    double p;
    double q;
    double r;
    double X;
    double Y;
    double Z;
    double L;
    double M;
    double N;
    double ae1sq;
    double ae2sq;
    double ae3sq;
    double ae4sq;
    double ax;
    double ay;
    double az;
} model_t;

#define MODEL_T     0.01
#define MODEL_B     1.0
#define MODEL_D     1.0
#define MODEL_M     1.0
#define MODEL_M_S   1.0
#define MODEL_I_L   1.0
#define MODEL_I_M   1.0
#define MODEL_I_N   1.0
#define MODEL_G     10.0

void model_init(model_t*);
void model_step(model_t*);

#endif
