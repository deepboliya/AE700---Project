% forces_moments.m
%   Computes the forces and moments acting on the airframe. 
%
%   Output is
%       F     - forces
%       M     - moments
%       Va    - airspeed
%       alpha - angle of attack
%       beta  - sideslip angle
%       wind  - wind vector in the inertial frame
%

function out = forces_moments(x, delta, wind, MAV)

    % relabel the inputs
    pn      = x(1);
    pe      = x(2);
    pd      = x(3);
    u       = x(4);
    v       = x(5);
    w       = x(6);
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);
    p       = x(10);
    q       = x(11);
    r       = x(12);
    delta_e = delta(1);
    delta_a = delta(2);
    delta_r = delta(3);
    delta_t = delta(4);
    w_ns    = wind(1); % steady wind - North
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis

    % compute wind data in NED
    w_n = w_ns + u_wg;
    w_e = w_es + v_wg;
    w_d = w_ds + w_wg;
    
    % compute air data
    V = sqrt(u^2 + v^2 + w^2)
    Va = sqrt((u-w_n)^2 + (v-w_e)^2 + (w-w_d)^2);
    alpha = atan2((w-w_d),(u-w_n));                    %verify this as it was given as alpha_x
    beta = asin((v-w_e)/Va);
  
    % compute external forces and torques on aircraft


    x_aero = 0.5*MAV.rho*(Va^2)*MAV.S_wing*(-(MAV.C_D_0 + MAV.C_D_alpha)*cos(alpha) + (MAV.C_L_0 + MAV.C_L_alpha)*sin(alpha) +((-MAV.C_D_q*cos(alpha) + MAV.C_L_q*sin(alpha))*MAV.c*q)/(2*Va) + (-MAV.C_D_delta_e*cos(alpha)+MAV.C_L_delta_e*sin(alpha))*delta_e);
    y_aero = 0.5*MAV.rho*(Va^2)*MAV.S_wing*(MAV.C_Y_0 + MAV.C_Y_beta*beta + (MAV.C_Y_p*MAV.b*p)/(2*Va) + (MAV.C_Y_r*MAV.b*r)/(2*Va) + MAV.C_Y_delta_a*delta_a + MAV.C_Y_delta_r*delta_r);
    z_aero = 0.5*MAV.rho*(Va^2)*MAV.S_wing*(-(MAV.C_D_0 + MAV.C_D_alpha)*sin(alpha) - (MAV.C_L_0 + MAV.C_L_alpha*alpha)*cos(alpha) +((-MAV.C_D_q*sin(alpha) - MAV.C_L_q*cos(alpha))*MAV.c*q)/(2*Va) + (-MAV.C_D_delta_e*sin(alpha)-MAV.C_L_delta_e*cos(alpha))*delta_e);

    x_prop = 0.5*MAV.rho*MAV.S_prop*((1000*delta_t)^2 - Va^2); %not actually the formula but since kmotor was not available, used this
    y_prop = 0;
    z_prop = 0;
  
    x_grav = -MAV.mass*MAV.gravity*sin(theta);
    y_grav = MAV.mass*MAV.gravity*cos(theta)*sin(phi);
    z_grav = MAV.mass*MAV.gravity*cos(theta)*cos(phi);



    Force(1) =  (x_grav+x_prop+x_aero);
    Force(2) =  (y_grav+y_prop+y_aero);
    Force(3) =  (z_grav+z_prop+z_aero);

    ktp = 0;
    komega = 0;
    
    Torque(1) = MAV.rho*(Va^2)*MAV.S_wing*MAV.b*(MAV.C_ell_0 + MAV.C_ell_beta*beta + (MAV.C_ell_p*MAV.b*p)/(2*Va)+ (MAV.C_ell_r*MAV.b*r)/(2*Va) + MAV.C_ell_delta_r*delta_r + MAV.C_ell_delta_a*delta_a)/2  - ktp*komega*komega*delta_t*delta_t ;

    Torque(2) = MAV.rho*(Va^2)*MAV.S_wing*MAV.c*(MAV.C_m_0 + MAV.C_m_alpha*alpha + (MAV.C_m_q*MAV.c*q)/(2*Va) + MAV.C_m_delta_e*delta_e)/2;   
    Torque(3) = MAV.rho*(Va^2)*MAV.S_wing*MAV.b*(MAV.C_n_0 + MAV.C_n_beta*beta + (MAV.C_n_p*MAV.b*p)/(2*Va)+ (MAV.C_n_r*MAV.b*r)/(2*Va) + MAV.C_n_delta_r*delta_r + MAV.C_n_delta_a*delta_a)/2;
    % 
    % Torque(1) = 0;
    % Torque(2) = 0;
    % Torque(3) = 0;

    out = [Force(1);Force(2);Force(3); Torque(1); Torque(2);Torque(3); Va; alpha; beta; w_n; w_e; w_d];
