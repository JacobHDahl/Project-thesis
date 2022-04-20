% M_rb * nu_dot + C_rb(nu)*nu = tau_rb
%
% nu = [u, v, w, p, q, r]'; in body frame
% tau_rb = [X, Y, Z, K, M, N]'; in body frame
% 
%
% b = [0, 0.1]';
%
% u = sin(t);


% Legg til gravitasjon !!!

% Legg til krefter (i tau) som tar inn geometri fra vingen og produserer
% en kraftfunksjon

% Finn angrepsvinkel for å finne kraften. 

% "Trim" er basisinnstilling når flyr stabilt.
%  Lag en vingemodul

% Finite wing analysis heter greia som hjelper med dette

% Sett på to vinger, 

% ship parameters 
m = 10;          % mass (kg)
Iz = 500;         % yaw moment of inertia (kg m^3)
xg = 0;              % CG x-ccordinate (m)

% Aero parameters
rho = 1.112; %air density at 1000m above sea level https://www.engineeringtoolbox.com/standard-atmosphere-d_604.html

h = 0.01; %timestep

iterations = 2000;

% rigid-body mass matrix
MRB = [ m 0    0 
        0 m    m*xg
        0 m*xg Iz ];
Minv = inv(MRB);

B = eye(3);

nus = zeros(3,iterations);
etas = zeros(3,iterations);

us = zeros(3,iterations);

us(1,:) = 1000*ones(1,iterations);
%us(2,:) = 100*ones(1,iterations);
%us(3,:) = deg2rad(10)*ones(1,iterations);

eta_init = [0, 0, deg2rad(7)]'; % x, z, theta in inertial frame
nu_init = [0, 0, 0]'; % v(velocity_x), w(velocity_z), omega(rot_velocity) in body frame

nus(:,1) = nu_init;
etas(:,1) = eta_init;

%main loop

t = zeros(1,iterations);


% x = [-1  , 1/3, 1/3, 1, 1/3, 1/3,-1  ];
% y = [-1/3,-1/3,-1/2, 0, 1/2, 1/3, 1/3];
% g = hgtransform;
% patch('XData',x,'YData',y,'FaceColor','blue','Parent',g)
% hold off

%h_fig = figure;

anim = animation;

for i = 1:iterations-1
    t(i) = (i-1)*h;
    nu = nus(:,i);
    eta = etas(:,i);
    u = us(:,i);

%     k = waitforbuttonpress;
%     % 28 leftarrow
%     % 29 rightarrow
%     % 30 uparrow
%     % 31 downarrow
%     value = double(get(gcf,'CurrentCharacter'));
%     disp(value)


    % Add Aerodynamic forces
    %rho = Air density
    %V_a = Velocity of vehicle through surrounding air mass
    %S = Planform area of wing (overflateareal)
    %C_L = Coefficient of lift
    %C_D = Coefficient of drag
    %C_M = Coefficient of aero moment drag
    %c = Mean  chord of wing (arm from center of pressure to CG(?))

    %F_lift = 0.5 * rho * V_a*V_a * S * C_L %Lift force in stability frame
    %F_drag = 0.5 * rho * V_a*V_a * S * C_D %Drag force in stability frame
    %M_aero = 0.5 * rho * V_a*V_a * S * c * C_M % Drag moment in stability
    
    %From stability frame to body frame:
    %[F_x;F_z] = [cos(alpha), -sin(alpha); sin(alpha), cos(alpha)];

    % Add gravity:
    G = -m * 9.81; %in inertial frame   



    %transfer to body-frame TODO: use Rzyx
    F_x_g = cos(eta(3))*G; 
    F_z_g = sin(eta(3))*G;

    V_a = nu(2);
    alpha = deg2rad(10);%constant, assuming no wind
    
    C_L = 0.8; %
    C_D = 0.1; %
    S = 1;


    F_lift = 0.5 * rho * V_a*V_a * S * C_L;
    F_drag = 0.5 * rho * V_a*V_a * S * C_D;
    %M_aero = 0.5 * rho * V_a*V_a * S * c * C_M; 
    
    %Rotation from stability frame to body frame
    R_stab_to_body = [cos(alpha),-sin(alpha);
                        sin(alpha), cos(alpha)];

    F_aero = R_stab_to_body * [F_drag;F_lift];

    F_x_aero = F_aero(1);
    F_z_aero = F_aero(2);
    
    u(1) = u(1)+F_z_g- F_z_aero;
    u(2) = u(2)+F_x_g - F_x_aero;


    tau = B * u;

    % state-dependent time-varying matrices
    CRB = m * nu(3) * [ 0 -1 -xg 
                    1  0  0 
                    xg 0  0  ];

    R = Rzyx(0,0,eta(3)); %Rotation matrix from body to inertial


    nu_dot = Minv * (tau - CRB * nu);

    eta_dot = R * nu;

    nu = nu + h * nu_dot;
    eta = eta + h * eta_dot;

    
    anim=anim.update(eta);
    nus(:,i+1) = nu;
    etas(:,i+1) = eta;

    pause(h); %This relates to the perceived time spent in the simulation. Relate to time t. TODO

end


