classdef ControlSignals < handle

    methods (Static)
        %% Lift cylinder control signal
        function u = uref_1(t)

            Lifting_Time_Start_1  = 0.5;
            Lifting_Time_End_1    = 2.0;

            Lowering_Time_Start_1 = 2.5;
            Lowering_Time_End_1   = 4.5;

            Lowering_Time_Start_2 = 5.0;
            Lowering_Time_End_2   = 7.0;

            Lowering_Time_Start_3 = 7.5;
            Lowering_Time_End_3   = 9.5;

            if Lifting_Time_Start_1 <= t && t < Lifting_Time_End_1
                u = -10;

            elseif Lowering_Time_Start_1 <= t && t < Lowering_Time_End_1
                u = 10;

            elseif Lowering_Time_Start_2 <= t && t < Lowering_Time_End_2
                u = -10;

            elseif Lowering_Time_Start_3 <= t && t < Lowering_Time_End_3
                u = 10;

            else
                u = 0;
            end

        end

        %% Tilt cylinder control signal

        function u = uref_2(t)

            Lifting_Time_Start_1  = 0.5;
            Lifting_Time_End_1    = 2.5;

            Lowering_Time_Start_1 = 3.0;
            Lowering_Time_End_1   = 5.0;

            Lowering_Time_Start_2 = 5.5;
            Lowering_Time_End_2   = 7.5;

            Lowering_Time_Start_3 = 8.0;
            Lowering_Time_End_3   = 9.5;

            if Lifting_Time_Start_1 <= t && t < Lifting_Time_End_1
                u = -10;

            elseif Lowering_Time_Start_1 <= t && t < Lowering_Time_End_1
                u = 10;

            elseif Lowering_Time_Start_2 <= t && t < Lowering_Time_End_2
                u = -10;

            elseif Lowering_Time_Start_3 <= t && t < Lowering_Time_End_3
                u = 10;

            else
                u = 0;
            end

        end

    end

end