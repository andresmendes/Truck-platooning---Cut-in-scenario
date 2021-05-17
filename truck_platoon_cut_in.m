%% Truck platooning - Cut-in scenario
% Simulation and animation of a cut-in scenario between trucks one and two
% in a four-truck platoon with 1 second time gap.
%
%%

clear ; close all ; clc

%% Scenario
% https://www.ebanataw.com.br/trafegando/gabaritorodoviario.htm

N_trucks = 4;                   % Number of trucks
th      = 1.0;                  % Time gap                  [s]

% Parameters
tf      = 30;                   % Final time                            [s]
fR      = 30;                   % Frame rate                            [fps]
dt      = 1/fR;                 % Time resolution                       [s]
time    = linspace(0,tf,tf*fR); % Time                                  [s]

% Road
distance_analysis   = 160;      % Distance of analysis                  [m]
trackWidth          = 20;       % Track width                           [m]
laneWidth           = 4.3;      % Lane width                            [m]
laneMargin          = 2;        % Margin lane                           [m]

% Truck 1 (Leading truck)
truck_1_length      = 18.6;     % Length of the leading truck           [m]
truck_1_width       = 2.6;      % Width of the leading truck            [m]
truck_1_init_speed  = 72/3.6;   % Speed of the leading truck            [m/s]
% Initial position of the leading truck [m]
truck_1_init_pos    = distance_analysis;      

% Truck 2 (Second truck)
truck_2_length      = 18.6;     % Length of the second truck            [m]
truck_2_width       = 2.6;      % Width of the second truck             [m]
truck_2_init_speed  = 72/3.6;   % Speed of the second truck             [m/s]
% Initial position of the second truck  [m]
truck_2_init_pos    = truck_1_init_pos-truck_1_length-th*truck_2_init_speed;

% Truck 3 (Third truck)
truck_3_length      = 18.6;     % Length of the third truck             [m]
truck_3_width       = 2.6;      % Width of the third truck              [m]
truck_3_init_speed  = 72/3.6;   % Speed of the third truck              [m/s]
% Initial position of the third truck   [m]
truck_3_init_pos    = truck_2_init_pos-truck_2_length-th*truck_3_init_speed;

% Truck 4 (Fourth truck)
truck_4_length      = 18.6;     % Length of the third truck             [m]
truck_4_width       = 2.6;      % Width of the third truck              [m]
truck_4_init_speed  = 72/3.6;   % Speed of the third truck              [m/s]
% Initial position of the fourth truck   [m]
truck_4_init_pos    = truck_3_init_pos-truck_3_length-th*truck_4_init_speed;

% Car (cut-in)
car_length          = 5;        % Length of the car                     [m]
car_width           = 2.6;      % Width of the car                      [m]
car_init_speed      = 72/3.6;   % Speed of the car                      [m/s]
% Initial position of the car           [m]
car_init_lon_pos    = truck_1_init_pos-2*truck_1_length + 10;
% Initial lateral position of the car   [m]
car_init_lat_pos    = 4.3;      

% PARAMETERS 
% Struct to ode45
% Trucks
parameters.th               = th;
% Length
parameters.truck_1_length   = truck_1_length;
parameters.truck_2_length   = truck_2_length;
parameters.truck_3_length   = truck_3_length;
parameters.truck_4_length   = truck_4_length;
% Width
parameters.truck_2_width    = truck_2_width;
% Car
parameters.car_length       = car_length;
parameters.car_width        = car_width;

%% Simulation

% INITIAL CONDITIONS
% Pair position and speed of first truck, pair position and speed of the
% second truck and so on ... pair position and speed of the car
states_initial_condition = [truck_1_init_pos 
                            truck_1_init_speed
                            truck_2_init_pos
                            truck_2_init_speed
                            truck_3_init_pos
                            truck_3_init_speed
                            truck_4_init_pos
                            truck_4_init_speed
                            car_init_lon_pos
                            car_init_speed
                            car_init_lat_pos];

% SIMULATION
options = odeset('RelTol',1e-9,'AbsTol',1e-9);
[TOUT,YOUT] = ode45(@(t,z) simulation(t,z,parameters),time,states_initial_condition,options);

% RETRIEVING STATES
% Truck 1
truck_1_position    = YOUT(:,1);
truck_1_speed       = YOUT(:,2);
% Truck 2
truck_2_position    = YOUT(:,3);
truck_2_speed       = YOUT(:,4);
% Truck 3
truck_3_position    = YOUT(:,5);
truck_3_speed       = YOUT(:,6);
% Truck 4
truck_4_position    = YOUT(:,7);
truck_4_speed       = YOUT(:,8);
% Car
car_pos_lon         = YOUT(:,9);
car_speed           = YOUT(:,10);
car_pos_lat         = YOUT(:,11);

% Acceleration
% Preallocating
truck_1_acc         = zeros(1,length(TOUT));
truck_2_acc         = zeros(1,length(TOUT));
truck_3_acc         = zeros(1,length(TOUT));
truck_4_acc         = zeros(1,length(TOUT));
for i=1:length(TOUT)
    [dz]    = simulation(TOUT(i),YOUT(i,:),parameters);
    truck_1_acc(i)  = dz(2);
    truck_2_acc(i)  = dz(4);
    truck_3_acc(i)  = dz(6);
    truck_4_acc(i)  = dz(8);
end

% Distances
dist_1_2 = truck_1_position - truck_2_position - truck_1_length;
dist_2_3 = truck_2_position - truck_3_position - truck_2_length;
dist_3_4 = truck_3_position - truck_4_position - truck_3_length;

%% Results

c = cool(N_trucks); % Colormap
    
figure
set(gcf,'Position',[50 50 1280 720]) % 720p
% set(gcf,'Position',[50 50 854 480]) % 480p

% Create and open video writer object
v = VideoWriter('truck_platoon_cut_in.avi');
v.Quality = 100;
open(v);

for i=1:length(time)
    subplot(3,2,1)
        hold on ; grid on
        position_max = max(truck_1_position);
        set(gca,'xlim',[0 TOUT(end)],'ylim',[0 1.2*position_max])
        cla 
        plot(TOUT,truck_1_position,'color',c(1,:))
        plot(TOUT,truck_2_position,'color',c(2,:))
        plot(TOUT,truck_3_position,'color',c(3,:))
        plot(TOUT,truck_4_position,'color',c(4,:))
        plot([time(i) time(i)],[0 1.2*position_max],'k--') 
        xlabel('Time [s]')
        ylabel('Position [m]')
        title('Position')
        legend('Truck 1','Truck 2','Truck 3','Truck 4','location','SouthEast')
    subplot(3,2,3)
        hold on ; grid on
        speed_max = max(max([truck_1_speed truck_2_speed truck_3_speed truck_4_speed]));
        set(gca,'xlim',[0 TOUT(end)],'ylim',[0 1.2*speed_max])
        cla 
        plot(TOUT,truck_1_speed,'color',c(1,:))
        plot(TOUT,truck_2_speed,'color',c(2,:))
        plot(TOUT,truck_3_speed,'color',c(3,:))
        plot(TOUT,truck_4_speed,'color',c(4,:))
        plot([time(i) time(i)],[0 1.2*speed_max],'k--') 
        xlabel('Time [s]')
        ylabel('Speed [m/s]')
        title('Speed')
        legend('Truck 1','Truck 2','Truck 3','Truck 4','location','SouthEast')
    subplot(3,2,2)
        hold on ; grid on
        acc_min = min(min([truck_1_acc truck_2_acc truck_3_acc truck_4_acc]));
        acc_max = max(max([truck_1_acc truck_2_acc truck_3_acc truck_4_acc]));
        set(gca,'xlim',[0 TOUT(end)],'ylim',[1.2*acc_min 1.2*acc_max])
        cla 
        plot(TOUT,truck_1_acc,'color',c(1,:))
        plot(TOUT,truck_2_acc,'color',c(2,:))
        plot(TOUT,truck_3_acc,'color',c(3,:))
        plot(TOUT,truck_4_acc,'color',c(4,:))
        plot([time(i) time(i)],[1.2*acc_min 1.2*acc_max],'k--') 
        xlabel('Time [s]')
        ylabel('Acceleration [m/s2]')
        title('Acceleration')
        legend('Truck 1','Truck 2','Truck 3','Truck 4','location','SouthEast')
    subplot(3,2,4)
        hold on ; grid on
        dist_max = max(max([dist_1_2 dist_2_3 dist_3_4]));
        set(gca,'xlim',[0 TOUT(end)],'ylim',[0 1.1*dist_max])
        cla 
        plot(TOUT,dist_1_2,'color',c(2,:))
        plot(TOUT,dist_2_3,'color',c(3,:))
        plot(TOUT,dist_3_4,'color',c(4,:))
        plot([time(i) time(i)],[0 1.2*dist_max],'k--') 
        xlabel('Time [s]')
        ylabel('Distance [m]')
        title('Separation Distance')
        legend('Trucks 1 & 2','Trucks 2 & 3','Trucks 3 & 4','location','SouthEast')
    subplot(3,2,5:6)
        hold on ; axis equal
        cla 
        % POSITION AT INSTANT [m]
        % Trucks
        truck_1_position_inst   = truck_1_position(i);
        truck_2_position_inst   = truck_2_position(i);
        truck_3_position_inst   = truck_3_position(i);
        truck_4_position_inst   = truck_4_position(i);
        % Car 
        car_pos_lon_inst        = car_pos_lon(i);
        car_pos_lat_inst        = car_pos_lat(i);

        % ROAD MARKINGS
        sideMarkingsX = [truck_1_position_inst-distance_analysis truck_1_position_inst];
        set(gca,'xlim',[truck_1_position_inst-distance_analysis truck_1_position_inst],'ylim',[-trackWidth/2-laneMargin +trackWidth/2+laneMargin])
        set(gca,'XTick',0:20:truck_1_position(end))
        % Central lane left marking
        plot(sideMarkingsX,[+laneWidth/2 +laneWidth/2],'k--') 
        % Central lane right marking
        plot(sideMarkingsX,[-laneWidth/2 -laneWidth/2],'k--') 
        % left marking left lane
        plot(sideMarkingsX,[laneWidth+laneWidth/2 laneWidth+laneWidth/2],'k--')
        % right marking right lane
        plot(sideMarkingsX,[-laneWidth-laneWidth/2 -laneWidth-laneWidth/2],'k--')

        % DIMENSIONS
        % Truck 1
        truck_1_dimension_X = [truck_1_position_inst truck_1_position_inst truck_1_position_inst-truck_1_length truck_1_position_inst-truck_1_length];
        truck_1_dimension_Y = [+truck_1_width/2 -truck_1_width/2 -truck_1_width/2 +truck_1_width/2];
        % Truck 2
        truck_2_dimension_X = [truck_2_position_inst truck_2_position_inst truck_2_position_inst-truck_2_length truck_2_position_inst-truck_2_length];
        truck_2_dimension_Y = [+truck_2_width/2 -truck_2_width/2 -truck_2_width/2 +truck_2_width/2];
        % Truck 3
        truck_3_dimension_X = [truck_3_position_inst truck_3_position_inst truck_3_position_inst-truck_3_length truck_3_position_inst-truck_3_length];
        truck_3_dimension_Y = [+truck_3_width/2 -truck_3_width/2 -truck_3_width/2 +truck_3_width/2];
        % Truck 4
        truck_4_dimension_X = [truck_4_position_inst truck_4_position_inst truck_4_position_inst-truck_4_length truck_4_position_inst-truck_4_length];
        truck_4_dimension_Y = [+truck_4_width/2 -truck_4_width/2 -truck_4_width/2 +truck_4_width/2];
        % Car
        car_dimension_X = [car_pos_lon_inst car_pos_lon_inst car_pos_lon_inst-car_length car_pos_lon_inst-car_length];
        car_dimension_Y = [car_pos_lat_inst+car_width/2 car_pos_lat_inst-car_width/2 car_pos_lat_inst-car_width/2 car_pos_lat_inst+car_width/2];

        % Plotting trucks
        fill(truck_1_dimension_X,truck_1_dimension_Y,c(1,:))
        fill(truck_2_dimension_X,truck_2_dimension_Y,c(2,:))
        fill(truck_3_dimension_X,truck_3_dimension_Y,c(3,:))
        fill(truck_4_dimension_X,truck_4_dimension_Y,c(4,:))
        % Plotting car
        car_rear_right_corner_lateral_pos = car_pos_lat_inst - car_width/2;
        if car_rear_right_corner_lateral_pos < truck_2_width/2
            % Car detected distance sensor
            car_color = 'r';
        else
            % Car not detected distance sensor
            car_color = 'g';
        end
        fill(car_dimension_X,car_dimension_Y,car_color)
    
        xlabel('Lon. distance [m]')
        ylabel('Lat. distance [m]')
        
    frame = getframe(gcf);
    writeVideo(v,frame);
    
end

close(v);

%% Auxiliary functions

function dz = simulation(t,z,parameters)
    % PARAMETERS
    % Length
    truck_1_length  = parameters.truck_1_length;
    truck_2_length  = parameters.truck_2_length;
    truck_3_length  = parameters.truck_3_length;
    car_length      = parameters.car_length;
    % width
    truck_2_width   =  parameters.truck_2_width;
        
    % RETRIEVING STATES
    % Truck 1
    truck_1_position    = z(1);
    truck_1_speed       = z(2);
    truck_1_states      = [truck_1_position truck_1_speed];
    % Truck 2
    truck_2_position    = z(3);
    truck_2_speed       = z(4);
    truck_2_states      = [truck_2_position truck_2_speed];
    % Truck 3
    truck_3_position    = z(5);
    truck_3_speed       = z(6);
    truck_3_states      = [truck_3_position truck_3_speed];
    % Truck 4
    truck_4_position    = z(7);
    truck_4_speed       = z(8);
    truck_4_states      = [truck_4_position truck_4_speed];
    % Car
    car_pos_lon         = z(9);
    car_speed           = z(10);
    car_states          = [car_pos_lon car_speed];
    car_pos_lat         = z(11);
    
    % SENSORS
    % Truck 2
    % Cut in detection between trucks one and two.
    car_width = parameters.car_width;
    car_rear_right_corner_lateral_pos = car_pos_lat - car_width/2;
    if car_rear_right_corner_lateral_pos < truck_2_width/2
        % If car in front
        Truck_2_sensors.distance_preceding  = (car_pos_lon-car_length) - truck_2_position;
        Truck_2_sensors.speed_preceding     = car_speed;
    else
        % If truck one in front
        Truck_2_sensors.distance_preceding  = (truck_1_position-truck_1_length) - truck_2_position;
        Truck_2_sensors.speed_preceding     = truck_1_speed;        
    end
    % Truck 3
    Truck_3_sensors.distance_preceding = (truck_2_position-truck_2_length) - truck_3_position;
    Truck_3_sensors.speed_preceding = truck_2_speed; 
    % Truck 4
    Truck_4_sensors.distance_preceding = (truck_3_position-truck_3_length) - truck_4_position;
    Truck_4_sensors.speed_preceding = truck_3_speed; 
    
    % DYNAMIC MODELS
    % Truck 1
    truck_1_derivative_states = truck_model(t,truck_1_states,parameters,1,1);
    truck_2_derivative_states = truck_model(t,truck_2_states,parameters,2,Truck_2_sensors);
    truck_3_derivative_states = truck_model(t,truck_3_states,parameters,3,Truck_3_sensors);
    truck_4_derivative_states = truck_model(t,truck_4_states,parameters,4,Truck_4_sensors);
    car_derivative_states     = car_model(t,car_states);
    
    % OUTPUT STATES
    % Truck 1
    dz(1,1)     = truck_1_derivative_states(1,1);
    dz(2,1)     = truck_1_derivative_states(2,1);
    % Truck 2
    dz(3,1)     = truck_2_derivative_states(1,1);
    dz(4,1)     = truck_2_derivative_states(2,1);
    % Truck 3
    dz(5,1)     = truck_3_derivative_states(1,1);
    dz(6,1)     = truck_3_derivative_states(2,1);
    % Truck 4
    dz(7,1)     = truck_4_derivative_states(1,1);
    dz(8,1)     = truck_4_derivative_states(2,1);
    % Car
    dz(9,1)     = car_derivative_states(1,1);
    dz(10,1)    = car_derivative_states(2,1);
    % Lateral speed car (kinematic model)
    if car_pos_lat <= 0
        dz(11,1) = 0;
    else
        dz(11,1) = -0.5; % lateral speed [m/s]
    end

end

function dstates = truck_model(~,states,parameters,truck_flag,truck_sensors)
    % truck_flag indicates the current truck

    % Parameters
    m   = 40000;                % Mass                      [kg]
    g   = 9.81;                 % Gravity                   [m/s2]
    Cd  = 0.78;                 % Drag coefficient          [-]
    A   = 10;                   % Frontal area              [m2]
    rho = 1;                    % Air density               [kg/m2]

    % States
%     X = states(1);
    V = states(2);

    % Drag resistance
    C  = 0.5*rho*Cd*A;
    Dx = C*V^2;

    % Rolling resistance
    Rx=0;
    
    % Gravity force
    theta   = 0;                % Road slope                [rad]
    Gx      = m*g*sin(theta);   %                           [N]

    if truck_flag == 1
        % CC
        V_r = 20;               % Reference speed           [m/s]
        Kp  = 500;              % Controller gain
        Ft  = Kp*(V_r - V) + Dx; % Longitudinal force       [N]
    else
        sensor_distance_preceding = truck_sensors.distance_preceding;
        sensor_speed_preceding = truck_sensors.speed_preceding;
        % ACC
        th      = parameters.th;
        desired_distance = th*V + 0;
        Kp      = 10000;
        Kd      = 10000;
        Ft = Kp*(sensor_distance_preceding - desired_distance) + Kd*(sensor_speed_preceding - V) + Dx;    
    end

    % Vehicle Dynamics
    dstates(1,1) = V;
    dstates(2,1) = (Ft - Dx - Rx - Gx)/m;
    
end

function dstates = car_model(~,states)

    % Parameters
    m   = 1500;                 % Mass                      [kg]
    g   = 9.81;                 % Gravity                   [m/s2]
    Cd  = 0.4;                  % Drag coefficient          [-]
    A   = 2.5;                  % Frontal area              [m2]
    rho = 1;                    % Air density               [kg/m2]

    % States
%     X = states(1);
    V = states(2);

    % Drag resistance
    C  = 0.5*rho*Cd*A;
    Dx = C*V^2;

    % Rolling resistance
    Rx=0;
    
    % Gravity force
    theta   = 0;                % Road slope                [rad]
    Gx      = m*g*sin(theta);   %                           [N]

    % CC
    V_r = 20;                   % Reference speed           [m/s]
    Kp  = 500;                  % Controller gain
    Ft  = Kp*(V_r - V) + Dx;    % Longitudinal force        [N]

    % Vehicle Dynamics
    dstates(1,1) = V;
    dstates(2,1) = (Ft - Dx - Rx - Gx)/m;
    
end
