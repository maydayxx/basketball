function percentMade = basketballSimulation(angleToBasket, shotHeight)
%This is the function that loops over various initial conditions and
%computes how successful the shot conditions are
% Author: Sawyer Vaughan

load('initialBasketballConditions.mat')
steps = 20;
speedVariation = .2;
omegaVariation = .2;
angleVariation = .5;
horizAngleVariation = asin(2/3/15);
simulations = 300;
countIn = zeros(21, 40, steps+1);


for k = 1:length(conditions(:,1,1))
    distance = k;
    for targetomega = 0:6/steps:6
        for j = 1:length(conditions(1,:,1))
            targetspeed = conditions(distance, j, 1);
            targetangle = conditions(distance, j, 2);
            shotAngleVariation = 2*(rand-.5)*horizAngleVariation;
            for i = 1:simulations
                speed = targetspeed + 2*(rand-.5)*speedVariation;
                angle = targetangle + 2*(rand-.5)*angleVariation;
                omega = targetomega + 2*(rand-.5)*omegaVariation;
                countIn(distance,angle,omega)=countIn(distance,angle,omega)+basketball(speed, omega, distance, angleToBasket, angle, shotHeight, shotAngleVariation);
            end
        end
    end
end

percentMade = countIn/simulations;

end