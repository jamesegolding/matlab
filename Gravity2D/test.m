
space = SolarSystem();

time = 1:1000:1*seconds(years(1));

s = repmat(space.GetStates(), 1, numel(time));
for i = 2:numel(time)
    sDot = space.GetDerivatives();
    s(:,i) = s(:,i-1) + sDot * (time(i) - time(i-1));
    space.SetStates(s(:,i));
end

S = space.PostProcess(s, time);

plotplanets(S)