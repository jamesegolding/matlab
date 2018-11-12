classdef BicycleModel < handle
    properties
        lWhlbase = 2;
        xCoG = 1;
        hCoG = 0.5;
        m = 1400;
        Jz = 3000;
        JWhlF = 1.0;
        JWhlR = 1.0;
        MBrakeMax = 5000;
        MMotorMax = 3500;
        tyreF = TyreModel();
        tyreR = TyreModel();
        aero = AeroModel();
    end
    
    properties (Access=protected)
        u = zeros(2, 1);
        s = zeros(5, 1);
        BOutUpToDate = false;
        Out
    end
    
    methods
        function this = BicycleModel()
            
        end
        
        function SetInitialConditions(this, vLon, vLat, nYaw)
            % set tyre slips to zero
            nWhlF = vLon / this.tyreF.R;
            nWhlR = vLon / this.tyreR.R;
            this.SetStates([vLon, vLat, nYaw, nWhlF, nWhlR]');
            this.BOutUpToDate = false;
        end
        
        function [u, s, itHist] = SolveSteadyState(this, v, gLon, gLat)
            % solve quasi static steady state condition
            % target velocity, longitudinal and lateral acceleration
            % trim states and inputs to achieve target
            x = zeros(6, 1);
            % set state vector and constrain equation scalings
            scX = [1, 1, 0.01, 0.01, 1, 1]';
            scC = [0.1, 0.1, 1, 0.1, 1e-8, 1e-8]';
            ceqfun = @(x)this.SteadyStateCEq(x, v, gLon, gLat, scX, scC);
            % call solver
            [x, itHist] = broyden(ceqfun, x);
            x = x ./ scX;
            u = x(1:2);
            s = [v; x(3:end)];
        end
        
        function c = SteadyStateCEq(this, x, v, gLon, gLat, scX, scC)
            x = x .* scX;
            this.SetInputs(x(1), x(2));
            this.SetStates([v; x(3:end)]);
            O = this.GetOutputs();
            % constraints defined here
            c = [
                O.gLon - gLon
                O.gLat - gLat
                O.nYaw - (gLat / max(eps, v))
                O.dnYaw
                O.dnWhlF
                O.dnWhlR
                ] .* scC;
        end
        
        function SetInputs(this, aSteer, rPedal)
            this.u = [aSteer; rPedal];
            this.BOutUpToDate = false;
        end
        
        function SetStates(this, s)
            this.s = s;
            this.BOutUpToDate = false;
        end
        
        function s = GetStates(this)
            s = this.s;
        end
        
        function dsdt = GetDerivatives(this)
            % get aero forces from aero model
            [FxAero, FzAero] = this.GetAeroForces();
            % get tyre forces
            [FxF, FyF, MyF, FxR, FyR, MyR] = this.GetTyreForces(FzAero);
            % calculate the tyre rotation
            [dnWhlF, dnWhlR] = this.GetWheelRotation(MyF, MyR);
            [Fx, Fy, Mz] = this.ResolveInCarFrame(FxF, FyF, FxR, FyR);
            % store outputs
            this.Out.vLon = this.s(1);
            this.Out.vLat = this.s(2);
            this.Out.nYaw = this.s(3);
            this.Out.nWhlF = this.s(4);
            this.Out.nWhlR = this.s(5);
            this.Out.gLon = (Fx + FxAero) / this.m;
            this.Out.gLat = (Fy) / this.m;
            this.Out.dnYaw = Mz / this.Jz;
            this.Out.dnWhlF = dnWhlF;
            this.Out.dnWhlR = dnWhlR;
            this.BOutUpToDate = true;
            % return state derivatives
            dsdt = [this.Out.gLon; this.Out.gLat; this.Out.dnYaw; this.Out.dnWhlF; this.Out.dnWhlR];
        end
        
        function Out = GetOutputs(this)
            % if we haven't made the calculation, make it
            if ~this.BOutUpToDate
                this.GetDerivatives();
            end
            Out = this.Out;
        end
        
        function set.m(this, m)
            assert(isnumeric(m) && numel(m) == 1, 'Must be a scalar')
            this.m = m;
        end
        
        function set.Jz(this, J)
            assert(isnumeric(J) && numel(J) == 1, 'Must be a scalar')
            this.Jz = J;
        end
        
        function set.lWhlbase(this, l)
            assert(isnumeric(l) && numel(l) == 1, 'Must be a scalar')
            this.lWhlbase = l;
        end
        
        function set.xCoG(this, x)
            assert(isnumeric(x) && numel(x) == 1, 'Must be a scalar')
            this.xCoG = x;
        end
        
        function set.hCoG(this, h)
            assert(isnumeric(h) && numel(h) == 1, 'Must be a scalar')
            this.hCoG = h;
        end
        
        function set.MMotorMax(this, M)
            assert(isnumeric(M) && numel(M) == 1, 'Must be a scalar')
            this.MMotorMax = M;
        end
        
        function set.MBrakeMax(this, M)
            assert(isnumeric(M) && numel(M) == 1, 'Must be a scalar')
            this.MBrakeMax = M;
        end
        
        function set.tyreF(this, tyre)
            assert(isa(tyre, 'TyreModel'), 'Must be a tyre model')
            this.tyreF = tyre;
        end
        
        function set.tyreR(this, tyre)
            assert(isa(tyre, 'TyreModel'), 'Must be a tyre model')
            this.tyreR = tyre;
        end
    end
    
    methods (Access=protected)        
        function [Fx, Fy, Mz] = ResolveInCarFrame(this, FxF, FyF, FxR, FyR)
            aWhlF = -this.u(1);
            aWhlR = 0;
            % Rotate Fx and Fy into car frame
            FxFCar = FxF * cos(aWhlF) - FyF * sin(aWhlF);
            FxRCar = FxR * cos(aWhlR) - FyR * sin(aWhlR);
            FyFCar = FxF * sin(aWhlF) + FyF * cos(aWhlF);
            FyRCar = FxR * sin(aWhlR) + FyR * cos(aWhlR);
            % Calculate total lateral force and moment about CoG
            Fx = FxFCar + FxRCar;
            Fy = FyFCar + FyRCar;
            Mz = FyFCar * this.xCoG - FyRCar * (this.lWhlbase - this.xCoG);
        end
        
        function [dnWhlF, dnWhlR] = GetWheelRotation(this, MyF, MyR)
            % get rotational velocities
            nF = this.s(4);
            nR = this.s(5);
            % calculate braking and motor power
            this.Out.MBrakeF = 2/pi * atan(5 * nF) * min(0, this.u(2)) * this.MBrakeMax;
            this.Out.MBrakeR = 2/pi * atan(5 * nR) * min(0, this.u(2)) * this.MBrakeMax;
            this.Out.MMotor = max(0, this.u(2)) * this.MMotorMax;
            MWhlF = this.Out.MBrakeF;
            MWhlR = this.Out.MBrakeR + this.Out.MMotor;
            % calculate the rotational acceleration of the wheels
            dnWhlF = (MWhlF - MyF) / this.JWhlF;
            dnWhlR = (MWhlR - MyR) / this.JWhlR;
        end
        
        function [Fx, Fz] = GetAeroForces(this)
            vLon = this.s(1);
            [Fx, Fz] = this.aero.GetForces(vLon);
            % store outputs
            this.Out.FAeroDrag = -Fx;
            this.Out.FAeroLift = -Fz;
        end
        
        function [FxF, FyF, MyF, FxR, FyR, MyR] = GetTyreForces(this, FzAero)
            % get initial guess of contact patch forces
            [FzF, FzR] = this.GetTyreVerticalForces(0, FzAero);
            [vF, aF, nF, vR, aR, nR] = this.GetTyreVelocities();
            % get longitudinal accel estimate and update vertical force
            [FxF, FyF] = this.tyreF.GetForces(vF, aF, nF, FzF, 2);
            [FxR, FyR] = this.tyreR.GetForces(vR, aR, nR, FzR, 2);
            Fx = this.ResolveInCarFrame(FxF, FyF, FxR, FyR);
            gLon = Fx / this.m;
            % get update of contact patch forces
            [FzF, FzR] = this.GetTyreVerticalForces(gLon, FzAero);
            [FxF, FyF, MyF] = this.tyreF.GetForces(vF, aF, nF, FzF, 2);
            [FxR, FyR, MyR] = this.tyreR.GetForces(vR, aR, nR, FzR, 2);
        end
        
        function [FzF, FzR] = GetTyreVerticalForces(this, gLon, FzAero)
            a = this.xCoG;
            b = this.lWhlbase - this.xCoG;
            % get front and rear vertical force
            FzF = (-FzAero * a + this.m * 9.81 * b - this.hCoG * this.m * gLon) / this.lWhlbase;
            FzR = (-FzAero * b + this.m * 9.81 * a + this.hCoG * this.m * gLon) / this.lWhlbase;
            this.Out.FzF = FzF;
            this.Out.FzR = FzR;
        end
        
        function [vF, aF, nF, vR, aR, nR] = GetTyreVelocities(this)
            vLon = this.s(1);
            vLat = this.s(2);
            nYaw = this.s(3);
            nF = this.s(4);
            nR = this.s(5);
            % get axle velocities
            vLatAxlF = vLat + nYaw * this.xCoG;
            vLatAxlR = vLat - nYaw * (this.lWhlbase - this.xCoG);
            % get wheel orientation to vehicle
            aSteerF = this.u(1);
            aSteerR = 0;
            % resolve in wheel frame
            vF = sqrt(vLatAxlF ^ 2 + vLon ^ 2);
            vR = sqrt(vLatAxlR ^ 2 + vLon ^ 2);
            aF = atan(vLatAxlF / max(eps, vLon)) - aSteerF;
            aR = atan(vLatAxlR / max(eps, vLon)) - aSteerR;
            % store outputs
            this.Out.aSlipF = aF;
            this.Out.aSlipR = aR;
            this.Out.rSlipF = (nF .* this.tyreF.R - vF .* cos(aF)) ./ max(eps, vF .* cos(aF));
            this.Out.rSlipR = (nR .* this.tyreR.R - vR .* cos(aF)) ./ max(eps, vR .* cos(aR));
        end
    end
end
        