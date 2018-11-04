classdef Body < handle
    properties
        mass    % mass in kg
        name    % label
        s       % State vector of body
    end
    
    properties (Access = protected)
        G = 6.67e-11; % Gravitational constant
    end
    
    methods
        function this = Body(mass, name, r, theta, rDot, thetaDot)
            % Create an instance of Body
            % Body(mass, name, r, theta, rDot, thetaDot)
            % Provide r, theta, rDot, thetaDot as initial conditions
            this.mass = mass;
            this.name = name;
            this.SetInitialConitions(r, theta, rDot, thetaDot);
        end
        
        function SetInitialConitions(this, r, theta, rDot, thetaDot)
            % convert initial conditions to cartesian and store in state 
            % vector
            [x, y, xDot, yDot] = this.Polar2Cartesian(r, theta, rDot, thetaDot);
            this.s = this.Scale([x; y; xDot; yDot]);
        end
        
        function x = GetPosition(this)
            % Return cartesian position of Body
            sSc = this.Descale(this.s);
            x = sSc(1:2);
        end
        
        function SetStates(this, s)
            % Set s to the state vector
            this.s = s;
        end
        
        function s = GetStates(this)
            % Return state vector
            s = this.s;
        end
        
        function sDot = GetDerivatives(this, bodies)
            % Return the state derivative
            sSc = this.Descale(this.s);
            f = this.GetGravitationalForce(bodies);
            sDot = this.Scale([sSc(3); sSc(4); f / this.mass]);
        end
        
        function f = GetGravitationalForce(this, bodies)
            % Calculate the gravitational force exerted by all other bodies
            % on this Body
            f = [0; 0];
            for i = 1:numel(bodies)
                delta = this.GetVectorToBody(bodies{i});
                rSq = sum(delta .^ 2);
                fMag = this.G * this.mass * bodies{i}.mass / rSq;
                f = f + delta * fMag / sqrt(rSq);
            end
        end
        
        function S = PostProcess(this, s, time)
            % Return the position in cartesian and polar coordinates of the
            % Body
            s = this.Descale(s);
            S.mass = this.mass;
            [r, theta, rDot, thetaDot] = this.Cartesian2Polar(s(1, :), s(2, :), s(3, :), s(4, :));
            S.x = timeseries(s(1, :)', time);
            S.y = timeseries(s(2, :)', time);
            S.xDot = timeseries(s(3, :)', time);
            S.yDot = timeseries(s(4, :)', time);
            S.r = timeseries(r', time);
            S.theta = timeseries(theta', time);
            S.rDot = timeseries(rDot', time);
            S.thetaDot = timeseries(thetaDot', time);
        end
    end
    
    methods (Access = protected)
        function delta = GetVectorToBody(this, body)
            % Calculate the vector between this body and another
            xB = body.GetPosition();
            xT = this.GetPosition();
            delta = xB - xT;
        end
        
        function s = Scale(this, s)
            % Apply state vector scaling
            s = s ./ repmat(this.GetScaleFactors(), 1, size(s, 2));
        end
        
        function s = Descale(this, s)
            % Apply state vector descaling
            s = s .* repmat(this.GetScaleFactors(), 1, size(s, 2));
        end
    end
    
    methods (Static)
        function [x, y, xDot, yDot] = Polar2Cartesian(r, theta, rDot, thetaDot)
            xDot = 0; yDot = 0;
            x = r .* cos(theta);
            y = r .* sin(theta);
            if nargin > 2
                xDot = cos(theta) .* rDot - r .* sin(theta) .* thetaDot;
                yDot = sin(theta) .* rDot + r .* cos(theta) .* thetaDot;
            end
        end
        
        function [r, theta, rDot, thetaDot] = Cartesian2Polar(x, y, xDot, yDot)
            rDot = 0; thetaDot = 0;
            r = sqrt(x .^ 2 + y .^ 2);
            theta = atan(y ./ x);
            if nargin > 2
                rDot = 1 ./ sqrt(x .^ 2 + y .^ 2) .* (x .* xDot + y .* yDot);
                thetaDot = 1 ./ (x .^ 2 + y .^ 2) .* (x .* yDot - y .* xDot);
            end
        end
        
        function r = GetScaleFactors()
            % Store the state scale factors
            r = [1e9; 1e9; 1e3; 1e3];
        end
    end
end
