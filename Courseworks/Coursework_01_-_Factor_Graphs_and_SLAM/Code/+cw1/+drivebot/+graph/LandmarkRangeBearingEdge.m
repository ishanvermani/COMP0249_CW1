classdef LandmarkRangeBearingEdge < g2o.core.BaseBinaryEdge
    

    methods(Access = public)

        function obj = LandmarkRangeBearingEdge()
            obj = obj@g2o.core.BaseBinaryEdge(2); 
        end

        function initialEstimate(obj)
            
            x = obj.edgeVertices{1}.estimate(); 
            px = x(1); py = x(2); theta = x(3);

            r = obj.z(1);
            beta = obj.z(2);

            ang = theta + beta;
            lx = px + r * cos(ang);
            ly = py + r * sin(ang);

            obj.edgeVertices{2}.setEstimate([lx; ly]);
        end

        function computeError(obj)
            x = obj.edgeVertices{1}.estimate();
            px = x(1); py = x(2); theta = x(3);

            l = obj.edgeVertices{2}.estimate();
            lx = l(1); ly = l(2);

            dx = lx - px;
            dy = ly - py;

            r_pred = sqrt(dx*dx + dy*dy);
            beta_pred = atan2(dy, dx) - theta;

            obj.errorZ(1) = obj.z(1) - r_pred;
            obj.errorZ(2) = obj.wrapToPi(obj.z(2) - beta_pred);
        end

        function linearizeOplus(obj)
            x = obj.edgeVertices{1}.estimate();
            px = x(1); py = x(2); theta = x(3); 

            l = obj.edgeVertices{2}.estimate();
            lx = l(1); ly = l(2);

            dx = lx - px;
            dy = ly - py;

            r2 = dx*dx + dy*dy;
            r = sqrt(r2);

            
            if r < 1e-12
                obj.J{1} = zeros(2,3);
                obj.J{2} = zeros(2,2);
                return;
            end

            
            obj.J{1} = [  dx/r,      dy/r,     0;
                         -dy/r2,     dx/r2,    1 ];

            
            obj.J{2} = [ -dx/r,     -dy/r;
                          dy/r2,    -dx/r2 ];
        end
    end

    methods(Static, Access = private)
        function a = wrapToPi(a)
            
            a = atan2(sin(a), cos(a));
        end
    end
end
