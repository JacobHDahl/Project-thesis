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

h = 0.01; %timestep

iterations = 200;

% rigid-body mass matrix
MRB = [ m 0    0 
        0 m    m*xg
        0 m*xg Iz ];
Minv = inv(MRB);

B = eye(3);

nus = zeros(3,iterations);
etas = zeros(3,iterations);

us = zeros(3,iterations);

us(1,:) = 100*sin(linspace(0,10,iterations));%0.1 * ones(1,iterations);%  %;
us(2,:) = 10*ones(1,iterations);
us(3,:) = 100*ones(1,iterations);

eta_init = [0, 0, 0]'; % x, z, theta in inertial frame
nu_init = [0, 0, 0]'; % v(velocity_x), w(velocity_z), omega(rot_velocity) in body frame

nus(:,1) = nu_init;
etas(:,1) = eta_init;

%main loop

t = zeros(1,iterations);


x = [-1  , 1/3, 1/3, 1, 1/3, 1/3,-1  ];
y = [-1/3,-1/3,-1/2, 0, 1/2, 1/3, 1/3];
g = hgtransform;
patch('XData',x,'YData',y,'FaceColor','blue','Parent',g)
hold off

%h_fig = figure;


xlim([-5,5]);
ylim([-5,5]);


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

    eta_anim = [eta(1), eta(2), 0];
    eta_anim_prev = [etas(1,i), etas(2,i), 0];
    
    g.Matrix = makehgtform('translate',eta_anim_prev + t(i)*(eta_anim-eta_anim_prev), ...
                         'zrotate',etas(3,i) + t(i)*(eta(3)-etas(3,i)));
    drawnow
    %plot(eta(1),eta(2),'o')
    
    nus(:,i+1) = nu;
    etas(:,i+1) = eta;
    pause(0.01);

end
% limitScaling = 1.1;
% 
% xlim([limitScaling*min(etas(1,:)),limitScaling*max(etas(1,:))])
% ylim([limitScaling*min(etas(2,:)),limitScaling*max(etas(2,:))])

% x = etas(1,:);
% z = etas(2,:);
% 
% figure(1)
% plot(x,z)



