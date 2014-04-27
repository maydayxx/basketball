function res = basketballAnimation(initSpeed, omegaInit, XYdistanceToBasket, angleToBasket, shotAngle, shotHeight, shotAngleVariation)
    %This function simulates a basketball shot. initSpeed is the initial ball velocity,
    %omegaInit is the angular speed of the basketball, XYdistanceToBasket is 
    %the initial displacement from the center of the hoop in feet, 
    %angleToBasket is the angle between straight on and where you are in
    %degrees, shotAngle is the angle from horizontal of the ball's initial velocity
    %shotHeight is the initial height of the ball after it has left the
    %hand, and shotAngleVariation is the angle that the shooter is off
    %while aiming at the basket. All inputs are scalars. This function
    %animates the ball's trajectory. This function returns 1 if the shot
    %went in, and 0 if it missed
    % Author: Sawyer Vaughan
    
    xyPosition = [XYdistanceToBasket*cos(angleToBasket/180*pi); 
    XYdistanceToBasket*sin(angleToBasket/180*pi)];
    vxy = initSpeed*cos(shotAngle*pi/180);
    vz = initSpeed*sin(shotAngle*pi/180);
    vx = vxy*sin(pi/2 - angleToBasket*pi/180 - shotAngleVariation*pi/180);
    vy = vxy*cos(pi/2 - angleToBasket*pi/180 - shotAngleVariation*pi/180);
    v=[-vx;-vy;vz];
    position = [xyPosition/3.28; (shotHeight-10)/3.28];
    
    timestep = .01;
    close all
    
    %% This section includes all the parameters of the hoop
    g = [0; 0; -9.8]; % gravity, m/s^2
    s = 1.25/3.28; %distance between center of rim and backboard, m
    rHoop = .75/3.28; %radius of the hoop, m
    rBall = .243/2; %radius of basketball, m
    a = .15; %vertical distance between bottom of backboard and hoop, m
    L = 6/3.28; %width of the backboard, m
    H = 3.5/3.28; %height of the backboard, m
    k = .7; %basketball coefficient of restitution
    Cd = .47; % drag coefficient
    p = 1.225; %air density at sea level, kg/m^3
    mBall = .62; % mass of the ball, kg
    I = 2/3*mBall*rBall^2; %moment of inertia of the ball\
    in = 0;
    

    %% This section defines many of the parameters of a backboard
    topRight = [-s; L/2; H-a];
    topLeft = [-s; -L/2; H-a];
    bottomRight = [-s; L/2; -a];
    bottomLeft = [-s; -L/2; -a];
        
    %% This function computes all the information relating to a collision
    function [value, isterminal, direction] = events(t, Y)
        % This function should return 0 or 1 if there is a collision, 0 or
        % 1 if the program should run again, and the necessary parameters
        % for the program to run again if the program were to run again.
        pos = Y(1:3);
        omega = Y(4);
        initVel = Y(5:7);
        
        %computes distances to the corners of the backboards
        distance(1) = point_to_point(pos, topRight);
        value(1) = distance(1)-rBall;
        isterminal(1) = 1;
        direction(1) = -1;
        
        distance(2) = point_to_point(pos, topLeft);
        value(2) = distance(2)-rBall;
        isterminal(2) = 1;
        direction(2) = -1;
        
        distance(3) = point_to_point(pos, bottomRight);
        value(3) = distance(3)-rBall;
        isterminal(3) = 1;
        direction(3) = -1;
        
        distance(4) = point_to_point(pos, bottomLeft);
        value(4) = distance(4)-rBall;
        isterminal(4) = 1;
        direction(4) = -1;
        
        %computes distances to corners of backboard
        distance(5) = point_to_line(pos, topRight, topLeft);
        value(5) = distance(5)-rBall;
        isterminal(5) = 1;
        direction(5) = -1;
        
        distance(6) = point_to_line(pos, topRight, bottomRight);
        value(6) = distance(6)-rBall;
        isterminal(6) = 1;
        direction(6) = -1;
        
        distance(7) = point_to_line(pos, bottomLeft, topLeft);
        value(7) = distance(7)-rBall;
        isterminal(7) = 1;
        direction(7) = -1;
        
        distance(8) = point_to_line(pos, bottomLeft, bottomRight);
        value(8) = distance(8)-rBall;
        isterminal(8) = 1;
        direction(8) = -1;
        
        
        %computes the distance to the rim
        distance(9) = point_to_circle(pos, [0;0;0], rHoop);
        value(9) = distance(9)-rBall;
        isterminal(9) = 1;
        direction(9) = -1;
        
        
        %computes the distance to the backboard
        distance(10) = point_to_rectangle3(pos, -s);
        value(10) = distance(10)-rBall;
        isterminal(10) = 1;
        direction(10) = -1;
        
        value(11) = pos(3)+3.28;
        isterminal(11) = 1;
        direction(11) = -1;
        
        value(12) = pos(3);
        isterminal(12) = 1;
        direction(12) = 0;
    end



    options = odeset('Events', @events);
    
    %% This section is the function that runs after there is a collision
    
    function res = postCollision(Y)
        % This function should return 0 or 1 if there is a collision, 0 or
        % 1 if the program should run again, and the necessary parameters
        % for the program to run again if the program were to run again.
        pos = Y(1:3)';
        omega = Y(4);
        initVel = Y(5:7)';
        runAgain = 1;
        
        %computes distances to the corners of the backboards
        distance(1) = point_to_point(pos, topRight);
        distance(2) = point_to_point(pos, topLeft);
        distance(3) = point_to_point(pos, bottomRight);
        distance(4) = point_to_point(pos, bottomLeft);
        
        %computes distances to corners of backboard
        distance(5) = point_to_line(pos, topRight, topLeft);
        distance(6) = point_to_line(pos, topRight, bottomRight);
        distance(7) = point_to_line(pos, bottomLeft, topLeft);
        distance(8) = point_to_line(pos, bottomLeft, bottomRight);
        
        %computes the distance to the rim
        distance(9) = point_to_circle(pos, [0;0;0], rHoop);
        
        %computes the distance to the backboard
        distance(10) = point_to_rectangle2(pos, topLeft,  bottomRight, -s);
        
        continueRunning = 1;
        for i = 1:length(distance)
            if distance(i)<rBall
                continueRunning = 0;
                collisionPoint = i;
            end
        end
        
        if exist('collisionPoint', 'var')~=0
            switch collisionPoint
                case 1
                    [vel, om] = reflection(topRight - pos, Y(5:7), omega);
                    vel = k*vel;
                case 2
                    [vel, om] = reflection(topLeft - pos, Y(5:7), omega);
                    vel = k*vel;
                case 3
                    [vel, om] = reflection(bottomRight - pos, Y(5:7), omega);
                    vel = k*vel;
                case 4
                    [vel, om] = reflection(bottomLeft - pos, Y(5:7), omega);
                    vel = k*vel;
                case 5
                    r(1) = -s-pos(1);
                    r(2) = 0;
                    r(3) = H-a-pos(3);
                    [vel, om] = reflection(r', initVel, omega);
                    vel = k*vel;
                case 6
                    r(1) = -s-pos(1);
                    r(2) = L/2-pos(2);
                    r(3) = 0;
                    [vel, om] = reflection(r', initVel, omega);
                    vel = k*vel;
                case 7
                    r(1) = -s-pos(1);
                    r(2) = -L/2-pos(2);
                    r(3) = 0;
                    [vel, om] = reflection(r', initVel, omega);
                    vel = k*vel;
                case 8
                    r(1) = -s-pos(1);
                    r(2) = 0;
                    r(3) = -a-pos(3);
                    [vel, om] = reflection(r', initVel, omega);
                    vel = k*vel;
                case 9
                    hoop(1:2) = pos(1:2);
                    hoop=hoop/norm(hoop)*rHoop;
                    hoop(3)=0;
                    [vel, om] = reflection(hoop' - pos, initVel, omega);
                    vel = k*vel;
                case 10
                    r(1)=-s-pos(1);
                    r(2)=0;
                    r(3)=0;
                    [vel, om] = reflection(r', initVel, omega);
                    vel = k*vel;
            end
        else
            vel = initVel;
            om = omega;
        end
        
        if Y(3) < -10/3.28+rBall
            continueRunning = 0;
            runAgain = 0;
        end
        
        if pos(3) <= 1e-10 && pos(3) >= -1e-10
            if norm(pos(1:2)) < rHoop-rBall
                in = 1;
            end
        end
        
        res = [continueRunning; runAgain; in; pos; om; vel];
    end
        
    
    %% This section runs the ode45
    [T, Y] = ode45(@derivatives, 0:timestep:5, [position; omegaInit; v], options);
    collisionValues = postCollision(Y(end,:));
    while collisionValues(2) == 1
        [Time, Variables] = ode45(@derivatives, [T(end):timestep:T(end)+5], collisionValues(4:end), options);
        [m,n]=size(Time);
        T(end+1:end+m)=Time;
        [m,n]=size(Variables);
        Y(end+1:end+m,:)=Variables;
        collisionValues = postCollision(Y(end,:));
    end
    
    %% This section plots the flight of the ball
    hold on
    plot([-s, -s], [-a, H-a], 'LineWidth', 3)
    plot(Y(:,1),Y(:,3))
    plot(rHoop,0,'.', 'MarkerSize', 10)
    plot(-rHoop,0,'.', 'MarkerSize', 10)
    plot_circle(rBall, [Y(end,1), Y(end,3)])
    axis([-s*2, Y(1,1), min(Y(:,3))-1, max(Y(:,3))+1])
    
    %% 3D animation
    plotTimestep=0.00001;
    jumps = 5;
    figure
    for h=1:length(T)/jumps
        clf
        hold on
        
        [x1,y1,z1] = sphere(50);
        x1 = x1*rBall + Y(h*jumps,1);
        y1 = y1*rBall + Y(h*jumps,2);
        z1 = z1*rBall + Y(h*jumps,3);
        surface(x1,y1,z1,'FaceColor', 'none','EdgeColor', [1 0.5 0.2])
        hold on

        subplot(1,2,1), surface(x1,y1,z1,'FaceColor', 'none','EdgeColor', [1 0.5 0.2])
        hold on
        plot3([-s, -s, -s, -s, -s], [-L/2, -L/2, L/2, L/2, -L/2], [-a, H-a, H-a, -a, -a], 'k')
        plot3([-s, -s, -s, -s, -s], [-.59/2, -.59/2, .59/2, .59/2, -.59/2], [0, .45, .45, 0, 0], 'k')
        plot3circle(rHoop, [0;0;0])
        axis([-2*s max(Y(:,1))+1 -max(Y(:,2))-1 max(Y(:,2))+1 -4 max(Y(:,3))+1])
        view(30, 20)
        
        subplot(1,2,2), surface(x1,y1,z1,'FaceColor', 'none','EdgeColor', [1 0.5 0.2])
        hold on
        plot3circle(rHoop, [0;0;0])
        plot3([-s, -s, -s, -s, -s], [-L/2, -L/2, L/2, L/2, -L/2], [-a, H-a, H-a, -a, -a], 'k')
        plot3([-s, -s, -s, -s, -s], [-.59/2, -.59/2, .59/2, .59/2, -.59/2], [0, .45, .45, 0, 0], 'k')
        plot3circle(rHoop, [0;0;0])
        axis([-2*s max(Y(:,1))+1 -1 1 -4 max(Y(:,3))+1])
        view(0, 90)
        
        pause(plotTimestep)
    end
    
   %% This function is the derivatives function for ode45 to take as an input
    function deriv = derivatives(t, input)
        %unpacks the input
        X = input(1:3);
        om = input(4);
        V = input(5:7);
        
        %calculates the derivatives
        dXdt = V;
        domdt = 0;
        dVdt = g - Cd*pi*rBall^2*norm(V)*V*p/2/mBall;
        
        %repacks
        result = [dXdt; domdt; dVdt];
        
        deriv = result;
    end


    %% This function computes the minimum distance from a point to a line
    %This is useful for my events function
    function distance = point_to_line(point, lineend1, lineend2)
        %function takes a point and the two endpoints of a line and
        %computes the distance to that line. It takes all inputs as a
        %3-vector of x,y and z position
        
        distance = norm(cross(lineend1-point,lineend2-point))/norm(lineend1-lineend2);
    end
        

%% This function takes velocity and collision vector (The vector from
    %%the center of the ball to the point of collision) and outputs the
    %%velocity vector that would result from the collision. This is useful
    %%for running ode45 multiple times. 
    function [reflect, om] = reflection(r, v, omega)
        %Calculates the velocity after a collision. The function takes the
        %velocity just before the collision and the vector from the center
        %of mass of the ball to the point of collision. r and v are both
        %3-vectors
        unit = r/norm(r);
        vParallel = dot(v, unit)*unit;
        vPerp = v - vParallel;
        omega = [0;omega;0];
        vPoint = cross(r, omega);
        diff = vPerp - vPoint;
        vPerp = vPerp - 2/5*diff;
        vPoint = vPoint + 3/5*diff;
        reflect = vPerp - vParallel;
        om = norm(vPoint)/norm(r);
    end


    %% This function calculates the distance between points (useful for events function)
    function distance = point_to_point(point1, point2)
        distance = norm(point1-point2);
    end

    %% This function calculates the distance between a point and a circle
    %circle is in the xy plane
    function distance = point_to_circle(point, center, radius)
        %input point, and center as 3-vectors
        xydistance = norm(point(1:2)-center(1:2))-radius;
        zdistance = point(3)-center(3);
        distance = norm([xydistance, zdistance]);
    end

    %% This function computes the distance between a point and a rectangle
    function res = point_to_rectangle(point, topLeft, topRight, bottomLeft, bottomRight, xDistance)
        % This assumes that the rectangle is in the yz plane. Points should
        % be input as 3-vectors. This function also assumes that the edges
        % of the rectange are only vertical or horizontal
        if point(2) > topLeft(2) && point(2) < topRight(2) %if y position is in the backboard's y range
            if point(3) < topLeft(3) && point(3) > bottomLeft(3) %if z position is in the backboard's z range
                res = abs(point(1) - xDistance);
            elseif point(3)<bottomLeft(3) %if the basketball is below the backboard's z range
                res = point_to_line(point, bottomLeft, bottomRight); 
            elseif point(3)>bottomRight %if the basketball is above the backboard's z range
                res = point_to_line(point, topLeft, topRight);
            end
        elseif point(2) > topRight(2) %if y position is greater than the basketball's y range
            if point(3) < topLeft(3) && point(3) > bottomLeft(3) %if z position is in the backboard's z range
                res = point_to_line(point, topRight, bottomRight);
            elseif point(3)<bottomLeft(3) %if the basketball is below the backboard's z range
                res = point_to_point(point, bottomRight); 
            elseif point(3)>bottomRight %if the basketball is above the backboard's z range
                res = point_to_point(point, topRight);
            end
        elseif point(2) < topLeft(2) %if y position is less than the basketball's y range
            if point(3) < topLeft(3) && point(3) > bottomLeft(3) %if z position is in the backboard's z range
                res = point_to_line(point, topLeft, bottomLeft);
            elseif point(3)<bottomLeft(3) %if the basketball is below the backboard's z range
                res = point_to_point(point, bottomLeft); 
            elseif point(3)>bottomRight %if the basketball is above the backboard's z range
                res = point_to_point(point, topLeft);
            end
        end
    end

%% This function computes the distance between a point and a rectangle
    function res = point_to_rectangle2(point, topLeft, bottomRight, xDistance)
        % This assumes that the rectangle is in the xy plane. Points should
        % be input as 3-vectors. This function also assumes that the edges
        % of the rectange are only vertical or horizontal
        if point(2) > topLeft(2) && point(2) < bottomRight(2)
            if point(3) < topLeft(3) && point(3) > bottomRight(3)
                res = abs(point(1) - xDistance);
            else
                res = 1; %This way, it will not detect a collision because the distance > rBall
            end
        else
            res = 1;
        end
    end

%% This function computes the distance between a point and a rectangle
    function res = point_to_rectangle3(point, xDistance)
        % This assumes that the rectangle is in the xy plane. Points should
        % be input as 3-vectors. This function also assumes that the edges
        % of the rectange are only vertical or horizontal
        res = point(1) - xDistance;
    end

    %% This section is the function that plots a circle
    function plot_circle(r, pos)
        steps = 40;
        x = zeros(steps, 1);
        y = zeros(steps, 1);
        for i = 1:steps
            x(i) = pos(1) + r*cos((i-1)/(steps-1)*2*pi);
            y(i) = pos(2) + r*sin((i-1)/(steps-1)*2*pi);
            hold on 
        end
        plot(x,y)
    end

%% This section is the function that plots a circle in 3D
% circle is in xy plane
    function plot3circle(r, pos)
        steps = 40;
        for i = 1:steps
            x(i) = pos(1) + r*cos((i-1)/(steps-1)*2*pi);
            y(i) = pos(2) + r*sin((i-1)/(steps-1)*2*pi);
            z(i) = 0;
            hold on 
        end
        plot3(x,y,z, 'Color', [1 0.5 0.2], 'LineWidth', 2)
    end

%% Running code
res = in;

end