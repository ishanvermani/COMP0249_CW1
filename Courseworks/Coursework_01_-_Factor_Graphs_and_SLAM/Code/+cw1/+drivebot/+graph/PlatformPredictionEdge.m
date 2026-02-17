classdef PlatformPredictionEdge < g2o.core.BaseBinaryEdge
    % PlatformPredictionEdge summary of PlatformPredictionEdge
    %
    % This class stores the factor representing the process model which
    % transforms the state from timestep k to k+1
    %
    % The process model is as follows.
    %
    % Define the rotation vector
    %
    %   M = dT * [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0;0 0 1];
    %
    % The new state is predicted from 
    %
    %   x_(k+1) = x_(k) + M * [vx;vy;theta]
    %
    % Note in this case the measurement is actually the mean of the process
    % noise. It has a value of 0. The error vector is given by
    %
    % e(x,z) = inv(M) * (x_(k+1) - x_(k))
    %
    % Note this requires estimates from two vertices - x_(k) and x_(k+1).
    % Therefore, this inherits from a binary edge. We use the convention
    % that vertex slot 1 contains x_(k) and slot 2 contains x_(k+1).
    
    properties(Access = protected)
        % The length of the time step
        dT;
    end
    
    methods(Access = public)
        function obj = PlatformPredictionEdge(dT)
            % PlatformPredictionEdge for PlatformPredictionEdge
            %
            % Syntax:
            %   obj = PlatformPredictionEdge(dT);
            %
            % Description:
            %   Creates an instance of the PlatformPredictionEdge object.
            %   This predicts the state from one timestep to the next. The
            %   length of the prediction interval is dT.
            %
            % Outputs:
            %   obj - (handle)
            %       An instance of a PlatformPredictionEdge

            assert(dT >= 0);
            obj = obj@g2o.core.BaseBinaryEdge(3);            
            obj.dT = dT;
        end
       
        function initialEstimate(obj)
            % INITIALESTIMATE Compute the initial estimate of a platform.
            %
            % Syntax:
            %   obj.initialEstimate();
            %
            % Description:
            %   Compute the initial estimate of the platform x_(k+1) given
            %   an estimate of the platform at time x_(k) and the control
            %   input u_(k+1)

            % First Vertex Estimate
            priorX = obj.edgeVertices{1}.estimate();

            c = cos(priorX(3));
            s = sin(priorX(3));

            %M Matrix
            M = obj.dT * [c -s 0;
            s c 0;
            0 0 1];

            % Second Vertex Propogation
            predictedX = priorX + M * obj.z;

            %Normalize Theta
            predictedX(3) = g2o.stuff.normalize_theta(predictedX(3));

            % Compute the posterior assming no noise
            obj.edgeVertices{2}.setEstimate(predictedX);
        end
        
        function computeError(obj)
            % COMPUTEERROR Compute the error for the edge.
            %
            % Syntax:
            %   obj.computeError();
            %
            % Description:
            %   Compute the value of the error, which is the difference
            %   between the measurement and the parameter state in the
            %   vertex. Note the error enters in a nonlinear manner, so the
            %   equation has to be rearranged to make the error the subject
            %   of the formulat
                       
            %Error is v or noise
            % e = inv(M) ((x k+1 - xk)) - u
            %Inverse of a rotation matrix is just the transpose

            xk = obj.edgeVertices{1}.x;
            xk_1 = obj.edgeVertices{2}.x;

            c = cos(xk(3));
            s = sin(xk(3));

            M_inverse = (1/obj.dT) * [c s 0;
            -s c 0;
            0 0 1];

            % Error is M(difference of x) - u
            obj.errorZ = M_inverse * (xk_1 - xk) - obj.z;

            obj.errorZ(3) = g2o.stuff.normalize_theta(obj.errorZ(3));
        end
        
        % Compute the Jacobians
        function linearizeOplus(obj)
            % LINEARIZEOPLUS Compute the Jacobians for the edge.
            %
            % Syntax:
            %   obj.computeError();
            %
            % Description:
            %   Compute the Jacobians for the edge. Since we have two
            %   vertices which contribute to the edge, the Jacobians with
            %   respect to both of them must be computed.
            %

            xk = obj.edgeVertices{1}.x;
            xk_1 = obj.edgeVertices{2}.x;

            % delta x
            dx = xk_1 - xk;

            ck = cos(xk(3));
            sk = sin(xk(3));
            
            % 
            obj.J{1} = zeros(3);

            obj.J{1}(1, 1) = -ck;
            obj.J{1}(2, 1) = sk;

            obj.J{1}(1, 2) = -sk;
            obj.J{1}(2, 2) = -ck;

            obj.J{1}(1, 3) = (-sk*dx(1) + ck*dx(2));
            obj.J{1}(2, 3) = (-ck*dx(1) -sk*dx(2));

            obj.J{1}(3, 3) = -1;

            obj.J{1} = obj.J{1} / obj.dT;



            M_inverse = (1/obj.dT) * [ck sk 0;
            -sk ck 0;
            0 0 1];

            obj.J{2} = M_inverse;
        end
    end    
end