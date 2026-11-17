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


            R = R(1:2,1:2);
        end

        function R = rot(theta)

            th  = theta(1);   
            psi = theta(2);   
            phi = theta(3);   

            R   = RotationUtils.rotX(th)*RotationUtils.rotY(psi)*RotationUtils.rotZ(phi);
        end

        function S = skew(v)
            S = [   0     -v(2);
                   v(2)    0    ];

            % S = [   0     -v(3)    v(2);
            %         v(3)    0      -v(1);
            %       -v(2)    v(1)     0 ];
        end

    end
end

