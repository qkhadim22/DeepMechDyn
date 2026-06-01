classdef RotationUtils
    methods(Static)

        function R = rotX(theta)
            R = [1      0               0;
                 0      cos(theta)  -sin(theta);
                 0      sin(theta) cos(theta)];
        end

        function R = rotY(theta)
            R = [cos(theta)     0           -sin(theta);
                        0       1               0;
                 sin(theta)     0           cos(theta)];
        end

        function R = rotZ(theta)
            R = [cos(theta)   -sin(theta)        0;
                 sin(theta)   cos(theta)        0;
                        0           0           1];
        end

        function R = rot(theta)

            th  = theta(1);   
            psi = theta(2);   
            phi = theta(3);   

            R   = RotationUtils.rotX(th)*RotationUtils.rotY(psi)*RotationUtils.rotZ(phi);
        end 

    end
end

