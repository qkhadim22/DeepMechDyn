function plot_PATU_config(z, param, param_bod, joints, s2meas)
% plot_PATU_config  Draw the PATU crane pose for relative coordinates z (4x1).
%   plot_PATU_config(z, param, param_bod, joints)
%   plot_PATU_config(z, param, param_bod, joints, s2meas)
% If s2meas is given, a dashed circle of radius s2meas is drawn around the
% tilt-cylinder base: the model rod end should lie on that circle.
% The loop gap |Ci-Cj| is non-zero while Newton has not converged.

persistent bodies pos
if isempty(pos)
    [bodies,~,pos] = Get_bodies_PATU_model_semi("180KG");   % lengths and cylinder pins do not depend on the load
end
if nargin < 5, s2meas = []; end

A   = @(a) [cos(a) -sin(a); sin(a) cos(a)];
z   = z(:);
phi = (param.Tz*z).';
[R,rj] = Recursive_position_vector_PATU(phi, param_bod);

% joints (global frame, origin at lift-boom pivot I)
J1 = rj(:,1); J2 = rj(:,2); J3 = rj(:,3); J4 = rj(:,4);
ib = joints(1).ibody; jb = joints(1).jbody;
Ci = R(:,ib) + A(phi(ib))*joints(1).bar_ibody;     % loop pin, end of bracket 2 (body 3)
Cj = R(:,jb) + A(phi(jb))*joints(1).bar_jbody;     % loop pin, lug on tilt boom (body 4)

% booms
tip1 = J1 + A(phi(1))*[bodies(1).L1; 0];
lug4 = [param_bod.u4_bar_j4_o4x; param_bod.u4_bar_j4_o4y] + joints(1).bar_jbody;   % = [-Lcx; -Lcy] in body-4 frame
tip4 = J4 + A(phi(4))*[bodies(4).L1 + lug4(1); 0];

% cylinders
L0 = pos.bar_hy1_up_0;                           % lift cylinder base (ground)
L1 = R(:,1) + A(phi(1))*pos.bar_hy1_up_1;        % lift cylinder rod end (lift boom)
T1 = R(:,1) + A(phi(1))*pos.bar_hy2_up_1;        % tilt cylinder base (lift boom)
T2 = R(:,2) + A(phi(2))*pos.bar_hy2_up_2;        % tilt cylinder rod end (bracket 1)
s1  = norm(L1-L0);
s2  = norm(T2-T1);
gap = norm(Ci-Cj);

fig = findobj('Type','figure','Tag','PATUconfig');
if isempty(fig), fig = figure('Tag','PATUconfig','Name','PATU configuration'); end
figure(fig); clf(fig);
ax = axes(fig); hold(ax,'on'); axis(ax,'equal'); grid(ax,'on');

plot(ax,[J1(1) tip1(1)],[J1(2) tip1(2)],'k-','LineWidth',4);                  % lift boom
plot(ax,[J1(1) L1(1)],[J1(2) L1(2)],'k-','LineWidth',1);                      % lift-cylinder lug
plot(ax,[J1(1) T1(1)],[J1(2) T1(2)],'k:','LineWidth',1);                      % tilt-cylinder lug
plot(ax,[J4(1) tip4(1)],[J4(2) tip4(2)],'Color',[0.3 0.3 0.3],'LineWidth',4); % tilt boom
plot(ax,[J4(1) Cj(1)],[J4(2) Cj(2)],'Color',[0.3 0.3 0.3],'LineWidth',2);     % tilt-boom lug
plot(ax,[J2(1) J3(1)],[J2(2) J3(2)],'b-','LineWidth',3);                      % bracket 1 (body 2)
plot(ax,[J2(1) T2(1)],[J2(2) T2(2)],'b:','LineWidth',1);                      % rod-end lug on bracket 1
plot(ax,[J3(1) Ci(1)],[J3(2) Ci(2)],'m-','LineWidth',3);                      % bracket 2 (body 3)
plot(ax,[L0(1) L1(1)],[L0(2) L1(2)],'r-','LineWidth',2);                      % lift cylinder
plot(ax,[T1(1) T2(1)],[T1(2) T2(2)],'r-','LineWidth',2);                      % tilt cylinder
if ~isempty(s2meas)
    th = linspace(0,2*pi,400);
    plot(ax,T1(1)+s2meas*cos(th),T1(2)+s2meas*sin(th),'r--');                % measured s2
end
plot(ax,[J1(1) J2(1) J3(1) J4(1)],[J1(2) J2(2) J3(2) J4(2)],'ko','MarkerFaceColor','w');
plot(ax,Ci(1),Ci(2),'mo','MarkerFaceColor','m');
plot(ax,Cj(1),Cj(2),'ks','MarkerSize',9);
plot(ax,R(1,:),R(2,:),'g+','MarkerSize',8);                                   % centres of mass
plot(ax,[L0(1) L1(1) T1(1) T2(1)],[L0(2) L1(2) T1(2) T2(2)],'ro','MarkerFaceColor','r');
text(ax,J2(1),J2(2),'  j2'); text(ax,J3(1),J3(2),'  j3'); text(ax,J4(1),J4(2),'  j4');
text(ax,Cj(1),Cj(2),'  C'); text(ax,T2(1),T2(2),'  rod');

if isempty(s2meas)
    s2txt = sprintf('s_2 = %.4f m', s2);
else
    s2txt = sprintf('s_2 = %.4f m (meas %.4f)', s2, s2meas);
end
title(ax,{sprintf('z = [%.2f %.2f %.2f %.2f] deg', rad2deg(z)), ...
          sprintf('s_1 = %.4f m,  %s,  loop gap = %.2f mm', s1, s2txt, 1e3*gap)});
xlabel(ax,'x [m]'); ylabel(ax,'y [m]');
drawnow;
end