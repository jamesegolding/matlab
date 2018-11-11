classdef BicycleModel < handle
    properties
        lWhlbase = 2;
        xCoG = 1;
        hCoG = 0.5;
        m = 1400;
        Jz = 3000;
        PBrakeMax = 400000;
        PMotorMax = 200000;
        tyreF = TyreModel();
        tyreR = TyreModel();
        JWhlF = 1.2;
        JWhlR = 2.0;
    end
    
    properties (Access=protected)
        u
        s
    end
    
    methods
        function this = BicycleModel()
            
        end
        
        function SetInitialConditions(this, vLon, vLat, nYaw)
            nWhlF = vLon / this.tyreF.R;
            nWhlR = vLon / this.tyreR.R;
            this.SetStates([vLon, vLat, nYaw, nWhlF, nWhlR]');
        end
        
        function SetInputs(this, aSteer, rPedal)
            this.u = [aSteer; rPedal];
        end
        
        function SetStates(this, s)
            this.s = s;
        end
        
        function s = GetStates(this)
            s = this.s;
        end
        
        function dsdt = GetDerivatives(this)
            [FxF, FyF, MyF, FxR, FyR, MyR] = this.GetTyreForces();
            [dnWhlF, dnWhlR] = this.GetWheelRotation(MyF, MyR);
            [Fx, Fy, Mz] = this.ResolveInCarFrame(FxF, FyF, FxR, FyR);
            dsdt = this.CalcDerivatives(Fx, Fy, Mz, dnWhlF, dnWhlR);
        end
        
        function Out = GetOutputs(this)
            this.GetDerivatives();
            Out = struct('a', this.s(1));
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
        
        function set.PMotorMax(this, P)
            assert(isnumeric(P) && numel(P) == 1, 'Must be a scalar')
            this.PMotorMax = P;
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
        function dsdt = CalcDerivatives(this, Fx, Fy, Mz, dnWhlF, dnWhlR)
            dsdt = [
                Fx / this.m
                Fy / this.m
                Mz / this.Jz
                dnWhlF
                dnWhlR
                ];
        end
        
        function [Fx, Fy, Mz] = ResolveInCarFrame(this, FxF, FyF, FxR, FyR)
            aWhlF = this.u(1);
            aWhlR = 0;
            % Rotate Fx and Fy into car frame
            Fx = FxF * cos(-aWhlF) + FyF * sin(-aWhlF) + FxR * cos(-aWhlR) + FyR * sin(-aWhlR);
            FyFCar = FxF * sin(-aWhlF) + FyF * cos(-aWhlF);
            FyRCar = FxR * sin(-aWhlR) + FyR * cos(-aWhlR);
            % Calculate total lateral force and moment about CoG
            Fy = FyFCar + FyRCar;
            Mz = FyFCar * this.xCoG - FyRCar * (this.lWhlbase - this.xCoG);
        end
                
        function [dnWhlF, dnWhlR] = GetWheelRotation(this, MyF, MyR)
            nF = this.s(4);
            nR = this.s(5);
            PBrakeF = 2 / pi * atan(5 * nF) * min(0, this.u(2)) * this.PBrakeMax;
            PBrakeR = 2 / pi * atan(5 * nR) * min(0, this.u(2)) * this.PBrakeMax;
            PWhlF = PBrakeF;
            PWhlR = PBrakeR + min(0, this.u(2)) * this.PBrakeMax;
            dnWhlF = (PWhlF / nF - MyF) / this.JWhlF;
            dnWhlR = (PWhlR / nR - MyR) / this.JWhlR;
        end
        
        function [FxF, FyF, MyF, FxR, FyR, MyR] = GetTyreForces(this)
            % get initial guess of contact patch forces
            [FzF, FzR] = this.GetTyreVerticalForces(0);
            [vF, aF, nF, vR, aR, nR] = this.GetTyreVelocities();
            % get longitudinal accel estimate and update vertical force
            [FxF, FyF] = this.tyreF.GetForces(vF, aF, nF, FzF);
            [FxR, FyR] = this.tyreR.GetForces(vR, aR, nR, FzR);
            Fx = this.ResolveInCarFrame(FxF, FyF, FxR, FyR);
            gLon = Fx / this.m;
            % get update of contact patch forces
            [FzF, FzR] = this.GetTyreVerticalForces(gLon);
            [FxF, FyF, MyF] = this.tyreF.GetForces(vF, aF, nF, FzF);
            [FxR, FyR, MyR] = this.tyreR.GetForces(vR, aR, nR, FzR);
        end
        
        function [FzF, FzR] = GetTyreVerticalForces(this, gLon)
            a = this.xCoG;
            b = this.lWhlbase - this.xCoG;
            FzF = this.m * (9.81 * b - this.hCoG * gLon) / this.lWhlbase;
            FzR = this.m * (9.81 * a + this.hCoG * gLon) / this.lWhlbase;
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
            aF = atan(vLatAxlF / vLon) - aSteerF;
            aR = atan(vLatAxlR / vLon) - aSteerR;
        end
    end
end
        