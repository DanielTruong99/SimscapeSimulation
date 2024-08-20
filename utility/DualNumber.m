classdef DualNumber   
    properties
        real
        dual
    end
    
    methods
        function obj = DualNumber(varargin)
            switch nargin 
                case 2
                    obj.real = varargin{1};
                    obj.dual = varargin{2};
            end

            
        end

    end
end

