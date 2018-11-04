classdef Space < handle
    properties
        Bodies
    end
    
    properties (Access = protected)
        s
    end
    
    methods
        function this = Space()
            % Space
            % Environment containing classes of type "Body"
        end
        
        function this = AddBody(this, body)
            % Add a body to the Space environment
            this.Bodies(end+1).handle = body;
            this.s = [this.s; body.GetStates()];
        end
        
        function s = GetStates(this)
            % Return the state vector of the whole system
            s = this.s;
        end
        
        function SetStates(this, s)
            % Set the state vector
            for i = 1:numel(this.Bodies)
                iS = this.GetStateIndices(i);
                this.Bodies(i).handle.SetStates(s(iS));
            end 
        end
        
        function sDot = GetDerivatives(this)
            % Get the state vector derivative of all bodies within Space
            bodyHandles = {this.Bodies(:).handle};
            allInds = 1:numel(this.Bodies);
            sDot = NaN(size(this.s));
            for i = allInds
                thisBody = this.Bodies(i).handle;
                iS = this.GetStateIndices(i);
                sDot(iS) = thisBody.GetDerivatives(bodyHandles(setdiff(allInds, i)));
            end
        end
        
        function S = PostProcess(this, s, time)
            % Return named results for the Space environment
            for i = 1:numel(this.Bodies)
                iS = this.GetStateIndices(i);
                S.(this.Bodies(i).handle.name) = this.Bodies(i).handle.PostProcess(s(iS,:), time);
            end
        end
    end
    
    methods (Static, Access = protected)
        function iS = GetStateIndices(iBody)
            % Calculate the state indices of the ith Body in Space
            iS = (iBody - 1) * 4 + 1:iBody * 4;
        end
    end
end
        