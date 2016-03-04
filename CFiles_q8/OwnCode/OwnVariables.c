double R_true = 0.1001405119340;
double L_true = 0.5052864456892;
double dt0 = 0.02;
//double K_omega = 2/(R_true*dt0);
double K_omega = 0;
double K_psi = 2*L_true/(R_true*dt0);
K_psi = K_psi/100;
double u_psi = 0;
double u_omega = 0;
double d0 = 0;
//double theta_goal = atan2((yg-y0),(xg-x0));
double theta_goal = 90;

