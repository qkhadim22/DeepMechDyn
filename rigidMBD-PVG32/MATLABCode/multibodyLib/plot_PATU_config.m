function plot_PATU_config(z, param, param_bod, joints, s2meas, upto)
% Plots the PATU crane body by body (tree order 1-2-3-4).
% upto = number of bodies to draw (default 4).

if nargin < 5, s2meas = []; end
if nargin < 6 || isempty(upto), upto = 4; end

[bodies,~,pos] = Get_bodies_PATU_model_semi("180KG");   % pins do not depend on the load

A   = @(a) [cos(a) -sin(a); sin(a) cos(a)];
z   = z(:);
phi = (param.Tz*z).';
[R,rj] = Recursive_position_vector_PATU(phi, param_bod);

P   = @(k,p) R(:,k) + A(phi(k))*p;          % body-local (COM frame) -> global
loc = @(k,G) A(phi(k)).'*(G - R(:,k));      % global -> body-local

% ---------------- body table (points in each body's COM frame) ----------------
lug2 = [param_bod.u2_bar_j2_j3x; param_bod.u2_bar_j2_j3y];
J2l  = loc(2, rj(:,2));

b(1).name = 'lift boom';  b(1).col = 'k';
b(1).pts  = [loc(1,rj(:,1)) loc(1,rj(:,2))];                       % J1 -> J2
b(1).att  = [pos.bar_hy1_up_1 pos.bar_hy2_up_1];                   % cylinder lugs

b(2).name = 'tilt boom';  b(2).col = [0.90 0.45 0.00];
b(2).pts  = [J2l+[bodies(2).L1+lug2(1);0]  J2l  loc(2,rj(:,3))];   % tip -> J2 -> C
b(2).att  = zeros(2,0);

b(3).name = 'bracket 2';  b(3).col = 'm';
b(3).pts  = [loc(3,rj(:,3)) joints(1).bar_ibody];                  % C -> pin
b(3).att  = zeros(2,0);

b(4).name = 'bracket 1';  b(4).col = 'b';
b(4).pts  = [loc(4,rj(:,4)) joints(1).bar_jbody];                  % J4 -> pin
b(4).att  = pos.bar_hy2_up_4;                                      % rod-end lug

% ---------------- cylinders: {bodyA, ptA, bodyB, ptB}, body 0 = ground ----------------
cyl = {0, pos.bar_hy1_up_0, 1, pos.bar_hy1_up_1;    % lift cylinder
    1, pos.bar_hy2_up_1, 4, pos.bar_hy2_up_4};   % tilt cylinder

% ---------------- figure ----------------
fig = findobj('Type','figure','Tag','PATUconfig');
if isempty(fig), fig = figure('Tag','PATUconfig','Name','PATU configuration'); end
figure(fig); clf(fig);
ax = axes(fig); hold(ax,'on'); axis(ax,'equal'); grid(ax,'on');

% ---------------- bodies ----------------
for k = 1:min(upto, numel(b))
    G = P(k, b(k).pts);
    plot(ax, G(1,:), G(2,:), '-o', 'Color', b(k).col, 'LineWidth', 3, 'MarkerFaceColor', 'w');
    for a = 1:size(b(k).att,2)                                     % thin lugs from body origin joint
        T = P(k, b(k).att(:,a));
        plot(ax, [G(1,1) T(1)], [G(2,1) T(2)], ':', 'Color', b(k).col, 'LineWidth', 1);
    end
    plot(ax, R(1,k), R(2,k), 'g+', 'MarkerSize', 8);               % COM
    text(ax, G(1,end), G(2,end), ['  ' b(k).name], 'Color', b(k).col);
end

% ---------------- cylinders ----------------
s = nan(1,size(cyl,1));
for i = 1:size(cyl,1)
    if max(cyl{i,1}, cyl{i,3}) <= upto
        pa = gpt(cyl{i,1}, cyl{i,2});  pb = gpt(cyl{i,3}, cyl{i,4});
        plot(ax, [pa(1) pb(1)], [pa(2) pb(2)], 'r-o', 'LineWidth', 2, 'MarkerFaceColor', 'r');
        s(i) = norm(pb - pa);
    end
end
s1 = s(1); s2 = s(2);

% ---------------- loop closure ----------------
gap = NaN;
if upto >= 4
    Ci = P(joints(1).ibody, joints(1).bar_ibody);
    Cj = P(joints(1).jbody, joints(1).bar_jbody);
    plot(ax, Ci(1), Ci(2), 'mo', 'MarkerFaceColor', 'm');
    plot(ax, Cj(1), Cj(2), 'bo', 'MarkerSize', 9);
    gap = norm(Ci - Cj);
end

% ---------------- title ----------------
if isempty(s2meas)
    s2txt = sprintf('s_2 = %.4f m', s2);
else
    s2txt = sprintf('s_2 = %.4f m (meas %.4f)', s2, s2meas);
end
title(ax, {sprintf('z = [%.2f %.2f %.2f %.2f] deg', rad2deg(z)), ...
    sprintf('s_1 = %.4f m,  %s,  loop gap = %.2f mm', s1, s2txt, 1e3*gap)});
xlabel(ax,'x [m]'); ylabel(ax,'y [m]');
drawnow;

    function g = gpt(k, p)                  % global point; k = 0 means ground
        if k == 0, g = p; else, g = P(k, p); end
    end
end