function plotplanets(S)

h = figure;
hP1 = uipanel(h, 'Units', 'Normalized', 'Position', [0.01, 0.1, 0.98, 0.89]);
hP2 = uipanel(h, 'Units', 'Normalized', 'Position', [0.01, 0.01, 0.98, 0.08]);

planets = fieldnames(S);
TMax = max(S.(planets{1}).x.Time);

hA = axes(hP1, ...
    'Units', 'Normalized', 'Position', [0.01, 0.01, 0.98, 0.98]);
hS = uicontrol(hP2, 'Style', 'slider', ...
    'Units', 'Normalized', 'Position', [0.01, 0.01, 0.98, 0.98], ...
    'Callback', @updatepositions, ...
    'Min', 0.0, 'Max', TMax, ...
    'SliderStep', [0.01, 0.1]);

colors = [
    0.0, 0.0, 1.0
    1.0, 0.0, 0.0
    0.0, 1.0, 0.0
    0.0, 0.5, 0.5
    0.5, 0.5, 0.0
    0.5, 0.0, 0.5
    0.3, 0.3, 0.3
    0.3, 0.3, 0.0
    0.3, 0.0, 0.3
    0.0, 0.3, 0.3
    ];

for i = 1:numel(planets)
    hSc(i) = scatter(NaN, NaN, 20 * (S.(planets{i}).mass / 5e24) ^ 0.25, colors(i, :));
    hold on; grid on;
    hL(i) = plot(NaN, NaN, 'Color', colors(i, :));
end

xlim(hA, [-5e12, 5e12])
ylim(hA, [-5e12, 5e12])

updatepositions([], []);

    function updatepositions(~, ~)
        t = get(hS, 'Value');
        for iP = 1:numel(planets)
            [~, iT] = min(abs(S.(planets{iP}).x.Time - t));
            set(hSc(iP), 'XData', S.(planets{iP}).x.Data(iT(1)));
            set(hSc(iP), 'YData', S.(planets{iP}).y.Data(iT(1)));
            set(hL(iP), 'XData', S.(planets{iP}).x.Data(1:iT(1)));
            set(hL(iP), 'YData', S.(planets{iP}).y.Data(1:iT(1)));
        end
    end
end
