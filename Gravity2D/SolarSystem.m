classdef SolarSystem < Space
    methods
        function this = SolarSystem()
            this = this@Space();
            this.AddBody(Body(1.989e30, 'Sun',     0,             0, 0, 0));
            this.AddBody(Body(5.972e24, 'Earth',   149.6e9,       0, 0, 2 * pi / seconds(years(1))));
            this.AddBody(Body(7.347e22, 'Moon',    149.6e9+3.8e8, 0, 0, 2 * pi / seconds(years(0.97))));
            this.AddBody(Body(3.285e23, 'Mercury', 57.91e9,       0, 0, 2 * pi / seconds(days(88))));
            this.AddBody(Body(4.867e24, 'Venus',   108.2e9,       0, 0, 2 * pi / seconds(days(225))));
            this.AddBody(Body(6.39e23,  'Mars',    227.9e9,       0, 0, 2 * pi / seconds(days(687))));
%             this.AddBody(Body(1.898e27, 'Jupiter', 778.5e9,       0, 0, 2 * pi / seconds(years(12))));
%             this.AddBody(Body(5.683e26, 'Saturn',  1.434e12,      0, 0, 2 * pi / seconds(years(29))));
%             this.AddBody(Body(8.681e25, 'Uranus',  2.871e12,      0, 0, 2 * pi / seconds(years(84))));
%             this.AddBody(Body(1.024e26, 'Neptune', 4.495e12,      0, 0, 2 * pi / seconds(years(165))));
        end
    end
end
