clear;clc;close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Hybrid Glider Simulator (6DOF) %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%     20-09-9  Woensug Choi    %%%%%%
%%%%    Mixture of Brian and Isa  %%%%%%
%%%%  Uses Brian's exp input data %%%%%%
%%%%  ! For Gazebo Verification ! %%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%---------------------------%
%----  Input Variables  ----%
%---------------------------%
% User setup variables
t_max = 60;
t_min = 120;
timestep = 0.1;

%-----------------------------------%
%----   Calculation variables   ----%
%-----------------------------------%
% Time array construction
time = 0:timestep:t_max;
% Volume of ballast array construction
% V_B = zeros(length(time),1);
% Variable mass position array construction
% x_s_c = zeros(length(time),1);
% Thruster force array construction
F_T = zeros(length(time),1)+1;

% Initial conditions: 
x_s_o = 0.4;   % Initial Movable Mass position
x_w_o = 0.711; % Initial Ballast Tank position
alpha = 0;
eta = zeros(6,length(time));
eta_dot = zeros(6,length(time));
nu = zeros(6,length(time));
nu_dot = zeros(6,length(time));
tau = zeros(6,length(time));  
epsilon = zeros(6,length(time));  
eta(:,1) = [0; 0; 0; 0; 0; 0;]; 
eta_dot(:,1) = [0; 0; 0; 0; 0; 0;]; 
nu_dot(:,1) = [0; 0; 0; 0; 0; 0;];    
tau(:,1) = [0; 0; 0; 0; 0; 0;];  
epsilon(:,1) = [0; 0; 0; 0; 0; 0;];  

%---------------------------%
%----   Load variables  ----%
%---------------------------%
load ../CBSoct
m_present_time = CBSoct(:,445)-CBSoct(t_min/5,445);
i_tmax = find(m_present_time > t_max,1);
m_present_time = m_present_time(1:i_tmax);
m_wallast_pumped = CBSoct(1:i_tmax,316).*0.000001;
m_wattpos = -CBSoct(1:i_tmax,321).*0.0254;
m_depth = CBSoct(1:i_tmax,337);
m_depth_rate = CBSoct(1:i_tmax,338);
m_pitch = CBSoct(1:i_tmax,442);
t_max = max(m_present_time);
%remove nans
i = ~isnan(m_wallast_pumped)&~isnan(m_wattpos)&~isnan(m_depth)&~isnan(m_pitch)&~isnan(m_depth_rate);
m_wallast_pumped = m_wallast_pumped(i);
m_wattpos = m_wattpos(i);
m_present_time = m_present_time(i);
m_depth = m_depth(i);
m_depth_rate = m_depth_rate(i);
m_pitch = m_pitch(i);

%interpolate control variables to have the same length
m_wattposi = interp1(m_present_time,m_wattpos,time);
m_wallast_pumpedi = interp1(m_present_time,m_wallast_pumped,time);
m_depthi = interp1(m_present_time,m_depth,time);
m_depth_ratei = interp1(m_present_time,m_depth_rate,time);
m_pitchi = interp1(m_present_time,m_pitch,time);

% Data allocations for Variable mass position and ballast pump
x_s_c = m_wattposi;
V_B = m_wallast_pumpedi;

%-----------------------------------%
%----  Environmental variables  ----%
%-----------------------------------%
% Physical constants
rho=1024; % density [kg/m3]
g=9.81;   % gravity

% Vehicle properties
length_h = 2;       % Vehicle length [m]
r_h = 0.105;        % radius of vehicle [m]
A_h = pi*r_h^2;     % Area [m2]
m_h = 48.2;         % vehicle mass
m_s = 7.8;          % mass shifter mass
max_cbattpos = 0.03;
max_delta_battpos = 0.004;
x_cb = 0; y_cb = 0; z_cb = 0; % Equal to Ballast Tank position fixed
x_cg = 0; y_cg = 0; z_cg = 0.0054;
r_w = 0.07;   % Ballast Tank radius

% Control perperties
pitch_control = 0;
pitch_ap_deadband = 1;
theta_delay = 10;
theta_setpoint = 45;     % degrees
theta_d = [zeros(theta_delay/timestep,1); ...
    ones(length(time)-theta_delay/timestep,1)*theta_setpoint*2*pi()/180];
P_theta = +2.86/5/timestep;
D_theta = -0.6/timestep;


%-------------------------------------%
%----  Glider Kinmatics Matrices  ----%
%-------------------------------------%
% Functions
J = @(phi,theta,psi) [cos(psi)*cos(theta), ...
    -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi), ...
    sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta), 0, 0, 0; ...
    sin(psi)*cos(theta), cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi), ...
    -cos(psi)*sin(phi)+sin(psi)*sin(theta)*cos(phi), 0, 0, 0; ...
    -sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi), 0, 0, 0; ...
    0, 0, 0, 1, sin(phi)*tan(theta), cos(phi)*tan(theta); ...
    0, 0, 0, 0, cos(phi), -sin(phi); 0, 0, 0, 0, sin(phi)/cos(theta), cos(phi)/cos(theta);];
skew = @(x1,x2,x3) [0,-x3,x2; x3,0,-x1; -x2,x1,0;];

% Initiate nu
nu(:,1) = J(eta(4,1),eta(5,1),eta(6,1))\eta_dot(:,1);

% Main loop
for i = 2: length(V_B)-1
    
    Period =100;
    
%     V_B_temp = 0.0;
    V_B_temp = V_B(2);
    x_s_c_temp = sin(time(i)/4);
    
    if (x_s_c_temp > 0.03)
        x_s_c_temp = 0.03;
    end
    if (x_s_c_temp < -0.03)
        x_s_c_temp = -0.03;
    end 
    
%     if (x_s_c_temp(i) > max_cbattpos)
%         x_s_c_temp(i) = max_cbattpos;
%     end
%     if (x_s_c_temp(i) < -max_cbattpos)
%         x_s_c_temp(i) = -max_cbattpos;
%     end
    
%     max_cbattpos = 0.03;
%     max_delta_battpos = 0.004;
    
    
    % ---- Mass Components ---- %
    % Ballast masss
%     m_w = V_B(i)*rho;
    m_w = V_B_temp*rho;
    % Total vehicle mass
    m = m_h + m_s + m_w;
    % Moving mass position
%     x_s = x_s_o + x_s_c(i);
    x_s = x_s_o + x_s_c_temp;
    % Hull mass center (uniform hull mass distribution assumed)
    x_h = -m_s/m_h*x_s_o;
    % Ballast tank mass center (pushing from front)
%     x_w = x_w_o+(V_B(i)/(pi()*r_w^2));
    x_w = x_w_o+(V_B_temp/(pi()*r_w^2));
    % Center of gravity
    x_cg = (x_h*m_h + x_s*m_s + x_w*m_w)/(m_h+m_s+m_w);
    
    
    %-----------------------------------%
    %------- Rigid Body Matrices -------%
    %-----------------------------------%
    %-- Mass matrix --%
    a = length_h/2; % half the length
    b = r_h;   % hull radius
    % inertial matrix (Fossen p.42 (2.156))
    I_yy = 4/15*m*(b^2+a^2); I_zz = I_yy;
    I_xx = 4/15*m*(b^2+b^2);
    I_o = [I_xx,0,0;0,I_yy,0;0,0,I_zz];
    % M_RB = Fossen (2.91)
    M_RB = zeros(6,6);
    M_RB(1:3,1:3) = m*diag([1,1,1]);
    M_RB(1:3,4:6) = -m*skew(x_cg,y_cg,z_cg);
    M_RB(4:6,1:3) = m*skew(x_cg,y_cg,z_cg);
%     M_RB(1:3,4:6) = -m*skew(0,0,0);
%     M_RB(4:6,1:3) = m*skew(0,0,0);
    M_RB(4:6,4:6) = I_o;
    
    %-- Centripal and Coriolis matrix --%
    % C_RB = Fossen (2.93)
    C_RB = zeros(6,6);
    C_RB(1:3,1:3) = zeros(3,3);
    temp = M_RB(1:3,1:3)*nu(1:3,i)+M_RB(1:3,4:6)*nu(4:6,i);
    C_RB(1:3,4:6) = -skew(temp(1),temp(2),temp(3));
    temp = M_RB(1:3,1:3)*nu(1:3,i)+M_RB(1:3,4:6)*nu(4:6,i);
    C_RB(4:6,1:3) = -skew(temp(1),temp(2),temp(3));
    temp = M_RB(4:6,1:3)*nu(1:3,i)+M_RB(4:6,4:6)*nu(4:6,i);
    C_RB(4:6,4:6) = -skew(temp(1),temp(2),temp(3));
    
    
    %-----------------------------------%
    %-------  Added Mass Forces  -------%
    %-----------------------------------%
    %-- Mass matrix --%
    % computed according to the procedure in Fossen p. 41
    e = 1-(b/a)^2;
    alpha_o = (2*(1-e^2)/e^3)*(0.5*log((1+e)/(1-e))-e);
    beta_o = 1/e^2-((1-e^2)/(2*e^3))*log((1+e)/(1-e));
    X_udot = -(alpha_o/(2-alpha_o))*m;
    Y_vdot = -(beta_o/(2-beta_o))*m; %-480 is due to wings
    Z_wdot = Y_vdot;
    K_pdot = 0;
    M_qdot = -0.2*m*((b^2-a^2)^2*(alpha_o-beta_o))/...
        (2*(b^2-a^2)+(b^2+a^2)*(beta_o-alpha_o));
    N_rdot = M_qdot;
    % Isa, eq. 21,32
    M_A_cg = -diag([X_udot, Y_vdot, Z_wdot, K_pdot, M_qdot, N_rdot]);
    
    M_A_Left(1:3,1:3) = diag([1,1,1]);
    M_A_Left(1:3,4:6) = skew(x_cg,y_cg,z_cg);
    M_A_Left(4:6,1:3) = zeros(3,3);
    M_A_Left(4:6,4:6) = diag([1,1,1]);
    M_A_Right(1:3,1:3) = diag([1,1,1]);
    M_A_Right(1:3,4:6) = skew(x_cg,y_cg,z_cg)';
    M_A_Right(4:6,1:3) = zeros(3,3);
    M_A_Right(4:6,4:6) = diag([1,1,1]);    
    M_A = M_A_Left*M_A_cg*M_A_Right;
    
    %-- Centripal and Coriolis matrix --%
    % C_RB = Fossen (2.93)
    C_A = zeros(6,6);
    C_A(1:3,1:3) = zeros(3,3);
    temp = M_A(1:3,1:3)*nu(1:3,i)+M_A(1:3,4:6)*nu(4:6,i);
    C_A(1:3,4:6) = -skew(temp(1),temp(2),temp(3));
    temp = M_A(1:3,1:3)*nu(1:3,i)+M_A(1:3,4:6)*nu(4:6,i);
    C_A(4:6,1:3) = -skew(temp(1),temp(2),temp(3));
    temp = M_A(4:6,1:3)*nu(1:3,i)+M_A(4:6,4:6)*nu(4:6,i);
    C_A(4:6,4:6) = -skew(temp(1),temp(2),temp(3));
    
    
    %-----------------------------------%
    %-------  Hydrostatic Forces -------%
    %-----------------------------------%
    % Hydrostatic restoring forces
    % Fossen (2.168)
    W = m*g; % Gravitational force acting on center of gravity
    B = (m_h+m_s)*g; % Buoyancy force acting on center of buoyancy
    g_eta= [(W-B)*sin(eta(5,i)); ...
          -(W-B)*cos(eta(5,i))*sin(eta(4,i)); ...
          -(W-B)*cos(eta(5,i))*cos(eta(4,i)); ...
          -(y_cg*W-y_cb*B)*cos(eta(5,i))*cos(eta(4,i))+(z_cg*W-z_cb*B)*cos(eta(5,i))*sin(eta(4,i)); ...
          (z_cg*W-z_cb*B)*sin(eta(5,i))+(x_cg*W-x_cb*B)*cos(eta(5,i))*cos(eta(4,i)); ...
          -(x_cg*W-x_cb*B)*cos(eta(5,i))*sin(eta(4,i))-(y_cg*W-y_cb*B)*sin(eta(5,i));];
    % Hystostatic force
    tau_R = g_eta;
    
    %-----------------------------------%
    %-------    Damping Forces   -------%
    %-----------------------------------%
    % Damping force for hull
    % Estimated by Brian Claus. Cannot be determined analyitically.
    Xu = 0; Yv = 0; Zw = -14;
    Kp = 0; Mq = -12; Nr = 0;
    D_h = -diag([Xu,Yv,Zw,Kp,Mq,Nr]);    
    
    
    %-----------------------------------%
    %------- Hydrodynamic Forces -------%
    %-----------------------------------%
    % angle of attack of hull
    if nu(1,i) > 0
        alpha_h = atan(nu(3,i)/nu(1,i));
    else
        alpha_h = 0;
    end
    % Graver hydrodynamic forces for hull
    C_D_h = 0.214 + 32.3*alpha_h^2;
    C_L_h = 11.76*alpha_h + 4.6 * alpha_h^2;
    C_M_h = 0.63*alpha_h;
    F_D_h = 0.5*rho*A_h*((nu(1,i)^2+nu(3,i)^2)^0.5)^2*C_D_h;
    F_L_h = 0.5*rho*A_h*((nu(1,i)^2+nu(3,i)^2)^0.5)^2*C_L_h;
    F_M_h = 0.5*rho*A_h*((nu(1,i)^2+nu(3,i)^2)^0.5)^2*C_M_h;
    tau_H_h = [ F_L_h*sin(alpha_h)-F_D_h*cos(alpha_h);0; ...
            -F_L_h*cos(alpha_h)-F_D_h*sin(alpha_h);0;F_M_h;0;];
    
    % Hydrodynamic forces by propeller
    tau_H_p = [F_T(i);0;0;0;0;0];        
        
    %-----------------------------------%
    %-------    Solver (Euler)   -------%
    %-----------------------------------%
    % Total matrices and forces
    M_tot = M_RB+M_A;
    C_tot = C_RB+C_A;
    D_tot = D_h;
    tau_H = tau_H_h + tau_H_p;
%     M_tot = M_RB+0;
%     C_tot = 0+0;
%     D_tot = 0;
%     tau_H = 0 + tau_H_p;
    
    % Solve for nu_dot
    h = time(i)-time(i-1);
    nu_dot(:,i) = M_tot\(tau_H-C_tot*nu(:,i)-D_tot*nu(:,i)-tau_R);
    nu(:,i+1)=nu(:,i) + h.*nu_dot(:,i)-0.5*h*(nu_dot(:,i)-nu_dot(:,i-1));
    eta_dot(:,i+1) = J(eta(4,i),eta(5,i),eta(6,i))*nu(:,i+1);
    eta(:,i+1) = eta(:,i) + (h)*eta_dot(:,i)-0.5*h*(eta_dot(:,i)-eta_dot(:,i-1));

    
    %-----------------------------------%
    %-------      Controller     -------%
    %-----------------------------------%
    % implement controllers
    if(pitch_control == 1)%servo pitch according to webb research controller
        pitch_error(i) = eta(3,i)-theta_d(i);
        if(abs(pitch_error(i)) > pitch_ap_deadband*2*pi()/180)
            delta_battpos = P_theta*pitch_error(i);

            if(delta_battpos > max_delta_battpos)
                delta_battpos = max_delta_battpos;
            end
            if(delta_battpos < -max_delta_battpos)
                delta_battpos = -max_delta_battpos;
            end    
            m_wattpos(i+1)=m_wattpos(i)+delta_battpos;
            if(m_wattpos(i+1) > max_cbattpos)
                m_wattpos(i+1) = max_cbattpos;
            end
            if(m_wattpos(i+1) < -max_cbattpos)
                m_wattpos(i+1) = -max_cbattpos;
            end   
        end
    end
end

opts = delimitedTextImportOptions("NumVariables", 7);
opts.DataLines = [3, Inf];opts.Delimiter = ",";
opts.VariableNames = ["t", "x", "y", "z", "p", "q", "r"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double"];
opts.ExtraColumnsRule = "ignore";opts.EmptyLineRule = "read";
tbl = readtable("C:\Users\woens\Dropbox\EPIC-DAUG\Simulator\Kinematics\02__Gazebo_Results\data.txt", opts);
t = tbl.t;x = tbl.x;y = tbl.y;z = tbl.z;p = tbl.p;q = tbl.q;r = tbl.r;clear opts tbl

figure;
subplot(2,1,1)
plot(t,q.*180./(2*pi()),'k','linewidth',2);hold on; grid on;
plot(time,eta(5,:).*180./(2*pi()),'k:','linewidth',2)
legend('\theta_{GAZEBO}','\theta_{MATLAB}','Location','Best')
xlabel('Time [s]');ylabel(['Degrees [' char(176) ']']);
xlim([0 t_max]);
subplot(2,1,2)
plot(t,x,'r','linewidth',2);hold on; grid on;
plot(t,z+5.0,'k','linewidth',2)
plot(time,eta(1,:),'r:','linewidth',2)
plot(time,-eta(3,:),'k:','linewidth',2)
legend('x_{GAZEBO}','z_{GAZEBO}','x_{MATLAB}','z_{MATLAB}','Location','NorthWest')
xlim([0 t_max]);
xlabel('Time [s]');ylabel('Position [m]');

% sgtitle(['battery pos. = ' num2str(x_s_c_temp) ' / pump vol. = ' num2str(V_B_temp)   ' / Thrust = ' num2str(F_T(i))]);
sgtitle(['battery pos. = sin(t/4) (max.=0.03) / pump vol. = ' num2str(V_B_temp)   ' / Thrust = ' num2str(F_T(i))]);

