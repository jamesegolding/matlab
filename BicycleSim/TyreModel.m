classdef TyreModel < handle
    properties
        kLonLin = -0.05;               % Long Lin Coeff
        kLonTan = 0.5;                % Long Tan Coeff
        kLonGain = 1.2;               % Long Peak Coeff
        kLatLin = -0.05;               % Lat Lin Coeff
        kLatTan = 0.5;                % Lat Tan Coeff
        kLatGain = 1.2;               % Lat Peak Coeff
        R = 0.35;                     % TLatre Radius
    end
    
    methods
        function this = TyreModel()
            
        end
        
        function [FLon, FLat, MRot] = GetForces(this, v, aSlip, nWhl, Fz)
            vWhl = v .* cos(aSlip);
            rSlip = (nWhl .* this.R - vWhl) ./ max(eps, vWhl);
            rFLon = this.kLonLin .* rSlip + this.kLonGain * atan(this.kLonTan * rSlip);
            rFLat = this.kLatLin .* aSlip + this.kLatGain * atan(this.kLatTan * aSlip);
            FLon = Fz * rFLon;
            FLat = Fz * rFLat;
            MRot = FLon * this.R;
        end
        
        function set.kLonLin(this, k)
            assert(isnumeric(k) && numel(k) == 1, 'Must be a scalar')
            assert(k < 0, 'Must be negative')
            this.kLonLin = k;
        end
        
        function set.kLatLin(this, k)
            assert(isnumeric(k) && numel(k) == 1, 'Must be a scalar')
            assert(k < 0, 'Must be negative')
            this.kLatLin = k;
        end
        
        function set.kLonTan(this, k)
            assert(isnumeric(k) && numel(k) == 1, 'Must be a scalar')
            assert(k > 0, 'Must be positive')
            this.kLonTan = k;
        end
        
        function set.kLatTan(this, k)
            assert(isnumeric(k) && numel(k) == 1, 'Must be a scalar')
            assert(k > 0, 'Must be positive')
            this.kLatTan = k;
        end
        
        function set.kLonGain(this, k)
            assert(isnumeric(k) && numel(k) == 1, 'Must be a scalar')
            assert(k > 0, 'Must be positive')
            this.kLonGain = k;
        end
        
        function set.kLatGain(this, k)
            assert(isnumeric(k) && numel(k) == 1, 'Must be a scalar')
            assert(k > 0, 'Must be positive')
            this.kLatGain = k;
        end
        
    end
end
        