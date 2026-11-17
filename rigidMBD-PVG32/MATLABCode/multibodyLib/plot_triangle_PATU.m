function plot_triangle_PATU(A_pt, B_pt, J4_pt, z1, z4, l_cyl)
% Schematic of the tilt-cylinder triangle (your points only):
%   A_pt  : tilt cylinder base (lift boom)
%   B_pt  : tilt cylinder rod pin (bracket 1)
%   J4_pt : bracket-1 pivot
%   z1    : lift-boom angle [rad]          (boom x-axis)
%   z4    : bracket-1 angle, relative [rad] (bracket x-axis = z1 + z4)
%   l_cyl : cylinder barrel length [m] (optional, default 1.050)
if nargin < 6, l_cyl = 1.050; end

cB  = [0.85 0.33 0.10];     % barrel
cR  = [0.55 0.20 0.05];     % rod
cBr = [0.10 0.60 0.40];     % bracket 1
cAn = [0.40 0.30 0.80];     % angle B
O   = [0; 0];
s2  = norm(B_pt - A_pt);
e   = (B_pt - A_pt)/s2;

figure('Color','w'); hold on; axis equal; axis off;

% boom x-axis and boom (O -> J4)
ex  = [cos(z1); sin(z1)];
ey  = [-sin(z1); cos(z1)];
L   = norm(J4_pt - O);
plot([O(1) J4_pt(1)], [O(2) J4_pt(2)], '-', 'Color', [0.85 0.85 0.85], 'LineWidth', 8);
quiver(O(1), O(2), 1.15*L*ex(1), 1.15*L*ex(2), 0, 'k--', 'MaxHeadSize', 0.02, 'LineWidth', 0.5);
quiver(O(1), O(2), 0.35*L*ey(1), 0.35*L*ey(2), 0, 'k--', 'MaxHeadSize', 0.1, 'LineWidth', 0.5);
text(O(1)+1.15*L*ex(1), O(2)+1.15*L*ex(2), '  boom x-axis', 'FontSize', 8);
text(O(1)+0.35*L*ey(1), O(2)+0.35*L*ey(2), '  boom y', 'FontSize', 8);

% A - J4 (dashed)
plot([A_pt(1) J4_pt(1)], [A_pt(2) J4_pt(2)], 'k--', 'LineWidth', 0.8);

% cylinder: barrel (l_cyl) + rod
Bb = A_pt + min(l_cyl, s2)*e;
plot([A_pt(1) Bb(1)],  [A_pt(2) Bb(2)],  '-', 'Color', cB, 'LineWidth', 7);
plot([Bb(1) B_pt(1)],  [Bb(2) B_pt(2)],  '-', 'Color', cR, 'LineWidth', 2.5);

% bracket 1 (J4 -> B) and its x-axis
plot([J4_pt(1) B_pt(1)], [J4_pt(2) B_pt(2)], '-', 'Color', cBr, 'LineWidth', 4);
bx  = [cos(z1+z4); sin(z1+z4)];
lb  = 1.3*norm(B_pt - J4_pt);
quiver(J4_pt(1), J4_pt(2), lb*bx(1), lb*bx(2), 0, '--', 'Color', cBr, 'MaxHeadSize', 0.3, 'LineWidth', 0.5);
text(J4_pt(1)+lb*bx(1), J4_pt(2)+lb*bx(2), ' bracket x-axis', 'Color', cBr, 'FontSize', 8);

% angle B at J4 (between J4->A and J4->B)
r1  = 0.12*norm(A_pt - J4_pt);
a1  = atan2(A_pt(2)-J4_pt(2), A_pt(1)-J4_pt(1));
a2  = atan2(B_pt(2)-J4_pt(2), B_pt(1)-J4_pt(1));
da  = atan2(sin(a1-a2), cos(a1-a2));
t   = linspace(a2, a2+da, 40);
plot(J4_pt(1)+r1*cos(t), J4_pt(2)+r1*sin(t), '-', 'Color', cAn, 'LineWidth', 1);
tm  = a2 + da/2;
text(J4_pt(1)+1.25*r1*cos(tm), J4_pt(2)+1.25*r1*sin(tm), 'B', 'Color', cAn, 'FontSize', 9);

% angle z4 at J4 (boom x-axis -> bracket x-axis)
r2  = 0.6*lb;
t   = linspace(z1, z1+z4, 60);
plot(J4_pt(1)+r2*cos(t), J4_pt(2)+r2*sin(t), '-', 'Color', cBr, 'LineWidth', 1);
tm  = z1 + z4/2;
text(J4_pt(1)+1.1*r2*cos(tm), J4_pt(2)+1.1*r2*sin(tm), sprintf('z_4 = %.1f°', rad2deg(z4)), 'Color', cBr, 'FontSize', 9);

% points
plot(O(1), O(2), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 7);
plot(J4_pt(1), J4_pt(2), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 7);
plot(A_pt(1), A_pt(2), 'o', 'Color', [0.3 0.3 0.3], 'MarkerFaceColor', [0.3 0.3 0.3], 'MarkerSize', 7);
plot(B_pt(1), B_pt(2), 'o', 'Color', cBr, 'MarkerFaceColor', cBr, 'MarkerSize', 7);
text(O(1), O(2), {'', 'O (0,0)'}, 'FontSize', 8, 'FontWeight', 'bold');
text(J4_pt(1), J4_pt(2), {'', 'J4'}, 'FontSize', 8, 'FontWeight', 'bold');
text(A_pt(1), A_pt(2), {'A', ''}, 'FontSize', 8, 'FontWeight', 'bold');
text(B_pt(1), B_pt(2), {'D', ''}, 'FontSize', 8, 'FontWeight', 'bold');

title(sprintf('s_2 = %.3f m  (stroke %.3f m)', s2, s2 - l_cyl), 'FontWeight', 'normal');
end
