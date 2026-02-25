// Adaptive Control Course 2025-2026

if (typeof mrac_state === 'undefined') {
    var mrac_state = {
        kx: 0.0,      
        kr: 0.0,      
        ku: 0.0,      
        
        sgn_b: -1.0,

        gamma: 0.95,   
        sigma: 0.1, 

        lastT: 0
    };
}

if (typeof model === 'undefined') {
    var model = {
        xm: 0.0, 
        am: -8.66,
        bm: 8.66,
    };
}

function controlFunction(rocket) {
    // 1. Target x, gimbal angle (δ), y_dot
    var x_target = 0.0;

    var error_x = x_target - rocket.x;
    var x_dot_target = 0.15 * error_x;

    var error_dx = x_dot_target - rocket.dx;
    var theta_target = 0.014 * error_dx;
    
    // 2. Reference signal
    var r = theta_target - rocket.theta;

    // 3. State x = [θ_dot]
    var x = rocket.dtheta;

    // 4. Simulation time step
    var dt = rocket.T - mrac_state.lastT;
    if (dt <= 0 || dt > 0.1) dt = 0.016;

    // 5.1. Compute linear feedback/feedforward component (page 5/lecture 6)
    var u_lin = mrac_state.kx * x + mrac_state.kr * r;

    // 5.2. Compute linear control deficiency (page 7/lecture 6)
    var u_max = 0.2;
    var delta = 0.2 * u_max;
    var u_max_delta = u_max - delta;
    var sat_lin = Math.max(-u_max_delta, Math.min(u_max_delta, u_lin));
    var mu = 20;
    var u_c = 1 / (1 + mu) * (u_lin + mu * u_max_delta * sat_lin);
    var du_lin = u_max * (Math.max(-u_max, Math.min(u_max, u_c))) - u_lin;
    rocket.delta_u = du_lin;

    // 6.1. Reference model: xm_dot = Am * xm + Bm * (r + ku * Δu_lin) (page 8/lecture 6)
    var xm_dot = model.am * model.xm + model.bm * (r + mrac_state.ku * du_lin); 
    // 6.2. Euler integration
    model.xm += xm_dot * dt;

    // 7. Tracking error
    var e = x - model.xm;
    
    var kx_dot = -mrac_state.gamma * x * e  * mrac_state.sgn_b;
    kx_dot -=  mrac_state.sigma * (mrac_state.kx - 2.943);

    var kr_dot = -mrac_state.gamma * r * e * mrac_state.sgn_b;
    kr_dot -= mrac_state.sigma * (mrac_state.kr + 2.943);
    
    var ku_dot = mrac_state.gamma * du_lin * e * model.bm;
    ku_dot -= mrac_state.sigma * (mrac_state.ku + 0.34); 

    mrac_state.kx += kx_dot * dt;
    mrac_state.kr += kr_dot * dt;
    mrac_state.ku += ku_dot * dt;

    mrac_state.lastT = rocket.T;
	
  	var h = Math.max(0, rocket.y - 25);
    var y_dot_target = -Math.sqrt(5.5 * h);
    var throttle = 0.6 + 0.5 * (y_dot_target - rocket.dy);
  	if(throttle > 1) throttle = 1;
  	else if(throttle < 0.6) throttle = 0.6;
  
    monitor('Throttle', throttle);
    monitor('Gimbal', u_c);
    
    return {throttle: throttle, gimbalAngle: u_c};
}