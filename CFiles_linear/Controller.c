// state variables are defined here
// goal is xg and initial is x0
// 

// Vairables that should be moved

// --------------------------------
theta_g = atan2( (yg - y0),(xg - x0) );
// theta_g = degrad*theta_g;
std::cout << "theta: " << theta << std::endl;
std::cout << "goal: " << degrad*theta_g << std::endl;
// LINEAR CONTROLLER BELOW


// Here follows the controller for translational velocity (task 12)
d_g = (xg - x)*cos(theta_g) + (yg - y)*sin(theta_g);
K_omega = 0.003*1/(dt*R);
u_omega = K_omega*d_g;

// Here follows the contorller for rotational velocity (task 14)
p = 0.9; // p > 0
d_p = p*(theta - degrad*theta_g);
K_psi = -0.003*L/(R*dt*p);
std::cout << "K_psi: " << K_psi << std::endl; 
u_psi = K_psi*d_p;

/*
When evaluating the line controller we want
u_psi = 0 for task 13
and
u_omega = 0 for task 16
*/
//u_psi = 0;
//u_omega = 0;

// End of linear controller

left = u_omega - u_psi/2;
right = u_omega + u_psi/2;

std::cout << "left: " << left << std::endl;
std::cout << "right: " << right << std::endl;
if (left > 200){
	left = 200;
}else if (left < -200){
	left = -200;
}

if (right > 200){
	right = 200;
}else if (right < -200){
	right = -200;
}