theta_g = atan2( (yg - y0),(xg - x0) );
theta_g_deg = degrad*theta_g;

if (abs(theta_g_deg - theta) >= 180 && state == TURNING)
	state = TURNING;
else if((abs(xg-x) >= 10 || abs(yg-y) >= 10) && (state == TURNING || state == STRAIGHT)) 
	state = STRAIGHT;
else
	state = IDLE;

switch (state){
	case TURNING:
		K_psi = L/(R*Ts);
		K_omega = 1/(R*Ts);
		u_psi = K_psi*(theta_g_deg - theta);
		d_0 = cos(raddeg*theta)*(x0 - x)+ sin(raddeg*theta)*(y0 - y);
		u_omega = K_omega*d_0;
		right = u_omega + u_psi/2;
		left = u_omega - u_psi/2;
		break;		
	case STRAIGHT:
		std::cout << "Errors: " << abs(xg-x) << " " << abs(yg-y);
		// Here follows the controller for translational velocity (task 12)dt=c
		d_g = (xg - x)*cos(theta_g) + (yg - y)*sin(theta_g);
		K_omega = 1/(Ts*R);
		u_omega = K_omega*d_g;

		// Here follows the contorller for rotational velocity (task 14)
		p = 0.6; // p > 0
		d_p = sin(theta_g)*(x+p*cos(raddeg*theta)-x0)-cos(theta_g)*(y+p*sin(raddeg*theta)-y0);
		//d_p = p*(theta - degrad*theta_g);
		K_psi = L/(R*Ts*p);
		u_psi = K_psi*d_p;
		if (u_omega > 200){
			u_omega = 200;
		}else if (u_omega < -200){
			u_omega = -200;
		}
		// End of linear controller

		left = u_omega - u_psi/2;
		right = u_omega + u_psi/2;
		break;
	default:
		left=0;
		right=0;
}
std::cout << "state: " << (int) state << std::endl;
// Control signal saturation
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