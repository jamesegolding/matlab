% RunBicycleSim
% 
% Example script to run a simulation using simple bicycle model

close all

% plot tyre characteristics as a check
tyre = TyreModel();
vCar = 10;
rSlip = (-10:0.2:10);
aSlip = (-10:0.2:10) * pi / 180;
[FLon, ~] = tyre.GetForces(vCar, 0, (1 + rSlip) * vCar / tyre.R, 3000);
[~, FLat] = tyre.GetForces(vCar, aSlip, vCar / tyre.R, 3000);
% pure long and lat plot
figure
title('Tyre Characteristic');
plot(rSlip, FLon / 3000)
xlabel('Slip Ratio [%]')
ylabel('Normalized Force Long [-]')
addsubplot(gcf);
plot(aSlip, FLat / 3000)
xlabel('Slip Angle [rad]')
ylabel('Normalized Force Lat [-]')

% setup bicycle model and define initial conditions
bike = BicycleModel();
bike.SetInitialConditions(20, 0, 0);
    
% define the simulation inputs
% simType = 'StraightLine';
simType = 'SineSteer';

time = 0:0.001:12;
switch simType
    case 'StraightLine'
        aSteer = zeros(1, numel(time));
        rPedal = zeros(1, numel(time));
        rPedal(time>2) = 0.5;
        rPedal(time>8) = -0.3;
    case 'SineSteer'
        aSteer = zeros(1, numel(time));
        rPedal = 0.1 * ones(1, numel(time));
        aSteer(time>1) = 2 * pi / 180 * sin(0.5 * 2 * pi * (time(time>1) - 1));
    otherwise
        error('Unrecognised sim type')
end

% set up arrays for outputs
bike.SetInputs(aSteer(1), rPedal(1));
s = repmat(bike.GetStates(), 1, numel(time));
Out = bike.GetOutputs();
fields = fieldnames(Out);
for i = 1:numel(fields)
    Out.(fields{i}) = repmat(Out.(fields{i}), 1, numel(time));
end

for i = 2:numel(time)
    % set inputs
    bike.SetInputs(aSteer(i-1), rPedal(i-1));
    % get derivatives
    sDot = bike.GetDerivatives();
    % forward euler step
    s(:,i) = s(:,i-1) + sDot * (time(i) - time(i-1));
    % get outputs and add to the array
    OutTemp = bike.GetOutputs();
    for j = 1:numel(fields)
        Out.(fields{j})(i) = OutTemp.(fields{j});
    end
    % update states of model
    bike.SetStates(s(:,i));
end

% plot some outputs
figure
plot(time, Out.vLon * 3.6);
hold on;
plot(time, Out.nWhlF * bike.tyreF.R * 3.6);
plot(time, Out.nWhlR * bike.tyreR.R * 3.6);
legend('Car', 'Front Rot', 'Rear Rot');
ylabel('Long Vel [kph]')
addsubplot(gcf);
plot(time, Out.vLat * 3.6);
ylabel('Lat Vel [kph]')

figure
plot(time, Out.gLon / 9.81);
ylabel('Long Accel [g]')
addsubplot(gcf);
plot(time, Out.gLat / 9.81);
ylabel('Lat Accel [g]')
addsubplot(gcf);
plot(time, Out.nYaw);
ylabel('Yaw Vel [rad/s]')
