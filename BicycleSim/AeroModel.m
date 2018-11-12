classdef AeroModel < handle
    properties
        Cd = 0.6;               % Drag coeff
        Cl = 2.5;               % Lift coeff
        A = 1.2;                % Reference area
        rho = 1.225;            % Air density
    end
    
    methods
        function this = AeroModel()
            
        end
        
        function [Fx, Fz] = GetForces(this, v)
            Fx = - 0.5 * this.rho * this.A * this.Cd * v .^ 2;
            Fz = - 0.5 * this.rho * this.A * this.Cl * v .^ 2;
        end
        
        function set.Cd(this, c)
            assert(isnumeric(c) && numel(c) == 1, 'Must be a scalar')
            assert(c > 0, 'Must be positive')
            this.Cd = c;
        end
        
        function set.Cl(this, c)
            assert(isnumeric(c) && numel(c) == 1, 'Must be a scalar')
            assert(c > 0, 'Must be positive')
            this.Cl = c;
        end
        
        function set.A(this, a)
            assert(isnumeric(a) && numel(a) == 1, 'Must be a scalar')
            assert(a > 0, 'Must be positive')
            this.A = a;
        end
              

        function set.rho(this, r)
            assert(isnumeric(r) && numel(r) == 1, 'Must be a scalar')
            assert(r > 0, 'Must be positive')
            this.rho = r;
        end
    end
end
        