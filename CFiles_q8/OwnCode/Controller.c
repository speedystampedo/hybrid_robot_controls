u_psi = K_psi*(theta_goal - theta);
d0 = cos(theta)*(y0 - y)+ sin(theta)*(x0-x);
printf("theta goal is: %f ", theta_goal);
printf("theta is: %f ", theta);
printf("K_psi is: %f ", K_psi);
u_omega = K_omega*d0;
right = u_omega + u_psi/2;
left = u_omega - u_psi/2;
