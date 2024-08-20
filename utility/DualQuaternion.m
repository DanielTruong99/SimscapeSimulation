classdef DualQuaternion
    
    properties(Access = public)
        m_Norm         % dual number a = a0 + e * a1 (a0, a1 are scalar)
        m_NormSquare
        m_RealPart   % 1x4
        m_DualPart   % 1x4
    end
    
    methods
        % Constructor (real_part, dual_part) 1x4, 1x4
        % Constructor (theta, n, a, d) 1x1, 1x3, 1x3, 1x1
        % Constructor (q, t) 1x4, 1x3
        function obj = DualQuaternion(varargin)
            switch nargin
                case 2
                    if length(varargin{2}) == 4
                        real_part = varargin{1};
                        dual_part = varargin{2};
                        obj.m_RealPart = real_part;
                        obj.m_DualPart = dual_part;
                    elseif length(varargin{2}) == 3
                        q = varargin{1};
                        t = varargin{2};
                        obj.m_RealPart = q;
                        obj.m_DualPart = 0.5 * quatmultiply([0, t], q);                        
                    end
                     
                case 4
                    RotAngle = varargin{1};
                    RotAxis = varargin{2};
                    ArbitraryPoint = varargin{3};
                    DistanceTrans = varargin{4};
                    RealPart = [cos(RotAngle/2), sin(RotAngle/2) * RotAxis];
                    DualPart = [-DistanceTrans/2 * sin(RotAngle/2), DistanceTrans/2 * cos(RotAngle/2) * RotAxis + sin(RotAngle/2) * cross(ArbitraryPoint, RotAxis)];
                    obj.m_RealPart = RealPart;
                    obj.m_DualPart = DualPart;
            end
            
%             real = norm(obj.m_RealPart);
%             dual = quatmultiply(quatconj(obj.m_RealPart), obj.m_DualPart) + quatmultiply(quatconj(obj.m_DualPart), obj.m_RealPart);
%             dual = dual / (2 * real); dual = dual(1);
%             obj.m_Norm = DualNumber(real, dual);
%             obj.m_NormSquare = real;
        end
        
        function obj_output = inv(obj)
            real_part = quatinv(obj.m_RealPart);
            dual_part = - quatmultiply( quatmultiply(real_part, obj.m_DualPart), real_part );
            obj_output = DualQuaternion(real_part, dual_part);           
        end
        
        % Overload Operator '*' dual quarternion multiplication
        function obj_output = mtimes(obj_1, obj_2)
            real_part = quatmultiply(obj_1.m_RealPart, obj_2.m_RealPart);
            dual_part = quatmultiply(obj_1.m_RealPart, obj_2.m_DualPart) + quatmultiply(obj_1.m_DualPart, obj_2.m_RealPart);
            obj_output = DualQuaternion(real_part, dual_part);
        end
        
        % Overload Operator '.*' scalar multiplication
        function obj_output = times(obj, scalar)
            real_part = scalar * obj.m_RealPart;
            dual_part = scalar * obj.m_DualPart;
            obj_output = DualQuaternion(real_part, dual_part);
        end        
        
        % Overload Operator '+'
        function obj_output = plus(obj_1, obj_2)
            real_part = obj_1.m_RealPart + obj_2.m_RealPart;
            dual_part = obj_1.m_DualPart + obj_2.m_DualPart;
            obj_output = DualQuaternion(real_part, dual_part);
        end
        
        % Overload Operator '-'
        function obj_output = minus(obj_1, obj_2)
            real_part = obj_1.m_RealPart - obj_2.m_RealPart;
            dual_part = obj_1.m_DualPart - obj_2.m_DualPart;
            obj_output = DualQuaternion(real_part, dual_part);
        end
        
        % Overload Operator ' q' ' treat as quaternion conjugate operator
        % q = r + e * d => q' = r' + e * d'
        function obj_output = ctranspose(obj)
            real_part = quatconj(obj.m_RealPart);
            dual_part = quatconj(obj.m_DualPart);
            obj_output = DualQuaternion(real_part, dual_part);
        end     
        
        % Overload Operator ' q.' ' treat as conjugate of dual number
        % q = r + e * d => q.' = r - e * d
        function obj_output = transpose(obj)
            real_part = obj.m_RealPart;
            dual_part = obj.m_DualPart * (-1);
            obj_output = DualQuaternion(real_part, dual_part);
        end      
        
        
    end
end

