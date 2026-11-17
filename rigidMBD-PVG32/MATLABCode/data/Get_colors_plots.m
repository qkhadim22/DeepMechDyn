function [color,plots] = Get_colors_plots()
color.pink = [1, 0.8, 0.8];%, 0.2
color.grey = [0.65, 0.65, 0.65];%, 0.2
color.yellow = [1, 1 , 0.8,0.2];
color.lightblue = [91/255, 207/255, 244/255];%  0.2
color.green = [0.4660, 0.6740, 0.1880];
color.orange = [0.9100 0.4100 0.1700];
color.purple = [0.6, 0, 0.6];
color.blue = [0, 0, 0.6];
color.brass = [0.8, 0.58, 0.46];
color.darkgrey = [0.52, 0.52, 0.5];
color.Red = [0.89, 0, 0.13];
color.Golden = [1 0.87 0];
color.applegreen = [0.224,0.694,0.522];
color.grassgreen = [0.612, 0.796, 0.525];
color.lightyellow = [0.914, 0.886, 0.612];
color.lightorange = [0.933, 0.706, 0.475];
color.deeporange = [ 0.91, 0.518, 0.443];
color.lightpink = [ 0.812, 0.349, 0.494];



plots.names     = [ "z1" "z2" "dotz1" "dotz2" "p1" "p2" "p3" "p4" ...
                    "s1" "s2" "dots1" "dots2" ...
                    "Fmu1" "Fmu2" "Be1" "Be2" ];

plots.xlabels   = [ "Time (s)" "Time (s)" "Time (s)" "Time (s)" "Time (s)" "Time (s)" "Time (s)" "Time (s)" ...
                      "Time (s)" "Time (s)" "Time (s)" "Time (s)" ...
                       "Velocity (m/s)" "Velocity (m/s)" "Pressure (bar)" "Pressure (bar)" ];


plots.ylabels   = ["$z_1\ \mathrm{[^\circ]}$" "$z_2\ \mathrm{[^\circ]}$" ...
                   "$\dot{z}_1\,\mathrm{[^\circ/s]}$" "$\dot{z}_2\,\mathrm{[^\circ/s]}$"...
                    "$p_1\ \mathrm{[Pa]}$" "$p_2\ \mathrm{[Pa]}$" ...
                    "$p_3\ \mathrm{[Pa]}$" "$p_4\ \mathrm{[Pa]}$" ...
                    "$s_1\ \mathrm{[mm]}$" "$s_2\ \mathrm{[mm]}$" ...
                    "$\dot{s}_1\ \mathrm{[mm/s]}$" "$\dot{s}_1\ \mathrm{[mm/s]}$" ...
                    "$F_{\mathrm{\mu_1}}\ \mathrm{[N]}$" "$F_{\mathrm{\mu_2}}\ \mathrm{[N]}$" ...
                    "$B_{\mathrm{e_1}}\ \mathrm{[bar]}$" "$B_{\mathrm{e_2}}\ \mathrm{[bar]}$" ];

plots.xlim = {[0 35], [0 35], [0 35], [0 35], [0 35], [0 35], ...
              [0 35], [0 35], [0 35], [0 35], [0 35], [0 35], ...
              [-0.06 0.06], [-0.01 0.01], [40 80], [0 4.5]};

plots.ylim = {[-11 61],   [-60 40],  [-15 15],  [-15 15],  [40e5 90e5],  [0e5 10e5],...
              [0e5 20e5],   [20e5 50e5],  [900 1500],  [1200 1350],  [-60 30],  [-10 10],...
              [-1800 1800],   [-400 400],  [1.2e3 1.7e3],  [0 4.8] };






% 
% function states              = system_forestry_crane(x)
%         states              = containers.Map();
%         states("z_1")       = x(1,:);
%         states("z_2")       = x(2,:);
%         states("dz_1")      = x(3,:);
%         states("dz_2")      = x(4,:);
%         states("p_1")       = x(5,:);
%         states("p_2")       = x(6,:);
%         states("p_3")       = x(7,:);
%         states("p_4")       = x(8,:);
%         states("U_1")       = x(9,:);
%         states("U_2")       = x(10,:);
% end
% 
% 
% function Hycyls              = hydraulic_cylinders(Z) 
%         Hycyls              = containers.Map();
%         Hycyls("s_1")       = Z(1,:);
%         Hycyls("s_2")       = Z(2,:);
%         Hycyls("ds_1")      = Z(3,:);
%         Hycyls("ds_2")      = Z(4,:);
%         Hycyls("F_mu_1")    = Z(5,:);
%         Hycyls("F_mu_2")    = Z(6,:);
%         Hycyls("F_s_1")     = Z(7,:);
%         Hycyls("F_s_2")     = Z(8,:);
%         Hycyls("B_e_1")     = Z(9,:);
%         Hycyls("B_e_2")     = Z(10,:);
%         Hycyls("B_e_3")     = Z(11,:);
%         Hycyls("B_e_4")     = Z(12,:);
% end
