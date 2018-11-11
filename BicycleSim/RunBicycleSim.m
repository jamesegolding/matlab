

tyre = TyreModel();
rSlip = (-10:0.2:10);
FLon = tyre.GetForces(10, 0, (1 + rSlip) * 10 / tyre.R, 3000);
plot(rSlip, FLon)
title('Tyre Londitudinal Characteristic');
xlabel('Slip Ratio [%]')
ylabel('Longitudinal Force [N]')

% WIP
bike = BicycleModel();
bike.SetInitialConditions(20, 0, 0);

time = 0:0.01:20;
s = repmat(bike.GetStates(), 1, numel(time));

u = zeros(2, numel(time));
u(1, time>2) = 1;
u(1, time>15) = -1;

for i = 2:numel(time)
    bike.SetInputs(u(1,i-1), u(2,i-1));
    sDot = bike.GetDerivatives();
    s(:,i) = s(:,i-1) + sDot * (time(i) - time(i-1));
    bike.SetStates(s(:,i));
end
