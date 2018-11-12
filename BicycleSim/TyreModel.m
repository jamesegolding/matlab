classdef TyreModel < handle
    properties
        kLonTan = 0.8;              % Long Tan Coeff
        kLonPos = 1.6;              % Long Peak Coeff
        kLonTanDec = 0.2;           % Long Tan Decay Coeff
        kLonDec = 0.8;              % Long Peak Decay Coeff
        kLatTan = 0.8;              % Lat Tan Coeff
        kLatPos = 1.6;              % Lat Peak Coeff
        kLatTanDec = 0.2;           % Lat Tan Decay Coeff
        kLatDec = 0.8;              % Lat Peak Decay Coeff
        R = 0.35;                   % TLatre Radius
    end
    
    methods
        function this = TyreModel()
            
        end
        
        function [FLon, FLat, MRot] = GetForces(this, v, aSlip, nWhl, Fz, rScale)
            if nargin < 6
                rScale = 1;
            end
            aSlipDeg = aSlip * 180 / pi;
            vWhl = v .* cos(aSlip);
            rSlip = (nWhl .* this.R - vWhl) ./ max(eps, vWhl);
            rFLon = -this.kLonDec * atan(this.kLonTanDec * rSlip) + this.kLonPos * atan(this.kLonTan * rSlip);
            rFLat = -this.kLatDec * atan(this.kLatTanDec * aSlipDeg) + this.kLatPos * atan(this.kLatTan * aSlipDeg);
            FLon = Fz * rFLon * rScale;
            FLat = -Fz * rFLat * rScale;
            MRot = FLon * this.R;
        end
        
        function set.kLonPos(this, k)
            assert(isnumeric(k) && numel(k) == 1, 'Must be a scalar')
            assert(k < 0, 'Must be negative')
            this.kLonPos = k;
        end
        
        function set.kLatPos(this, k)
            assert(isnumeric(k) && numel(k) == 1, 'Must be a scalar')
            assert(k < 0, 'Must be negative')
            this.kLatPos = k;
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
        
        function set.kLonDec(this, k)
            assert(isnumeric(k) && numel(k) == 1, 'Must be a scalar')
            assert(k < 0, 'Must be negative')
            this.kLonDec = k;
        end
        
        function set.kLatDec(this, k)
            assert(isnumeric(k) && numel(k) == 1, 'Must be a scalar')
            assert(k < 0, 'Must be negative')
            this.kLatDec = k;
        end
        
        function set.kLonTanDec(this, k)
            assert(isnumeric(k) && numel(k) == 1, 'Must be a scalar')
            assert(k > 0, 'Must be positive')
            this.kLonTanDec = k;
        end
        
        function set.kLatTanDec(this, k)
            assert(isnumeric(k) && numel(k) == 1, 'Must be a scalar')
            assert(k > 0, 'Must be positive')
            this.kLatTanDec = k;
        end        
    end
end
        