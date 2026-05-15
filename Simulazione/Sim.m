%% =========================================================================
%  DRONE DUCTED FAN — Simulazione MATLAB con visualizzazione 3D animata
%  VERSIONE CORRETTA: fisica PWM→spinta verificata, cilindro sempre visibile
%
%  Flusso: 1) Calcola intera traiettoria  2) Anima frame per frame
%  Nessun Simulink — puro script MATLAB
% =========================================================================
clc; clear; close all;

%% =========================================================================
%  1. PARAMETRI FISICI  (da altimetro.h — Relazione_RBC)
% =========================================================================
MASSA       = 1.269;      % [kg]
G           = 9.81;       % [m/s^2]
RHO         = 1.225;      % [kg/m^3]
D_PROP      = 0.2286;     % [m]   elica APC 9"
FSCIA       = 0.7;        % [-]   fattore scia motore inferiore
CT          = 0.2;        % [-]   coefficiente di spinta
VBATT       = 14.8;       % [V]   LiPo 4S
KV          = 1400;       % [RPM/V]

%% =========================================================================
%  2. PARAMETRI TIMER / PWM  (PSC=99, ARR=14999 -> 50 Hz)
% =========================================================================
ARR          = 14999;
CCR_MIN      = 750;
CCR_MAX      = 1500;
CCR_BASE     = 900;
CCR_OFFSET   = 1300;

CCR_CENTER   = 1125;
CCR_SRV_MAX  = 1545;
CCR_SRV_MIN  = 705;
CCR_PER_DEG  = 10.0;
MAX_FLAP_DEG = 30.0;

%% =========================================================================
%  3. GUADAGNI PID
% =========================================================================
Kp_m = 1000; Ki_m = 500; Kd_m = 800; dt_m = 0.033; lpf_m = 0.3;
Kp_r = 0; Ki_r = 0; Kd_r = 0; dt_s = 0.01;
Kp_p = 0; Ki_p = 0; Kd_p = 0;
Kp_y = 0; Ki_y = 0; Kd_y = 0; dt_y = 0.033; lpf_y = 0.3;

%% =========================================================================
%  4. RIFERIMENTI DI VOLO
% =========================================================================
REF_ALT  = 500;
REF_RP   = 0.0;
REF_YAW  = 0.0;
YAW_TRIM = 0.0;

%% =========================================================================
%  5. SETUP SIMULAZIONE
% =========================================================================
T_SIM = 15.0;  DT = 0.01;  N = round(T_SIM/DT);  t = (0:N-1)*DT;

DIST_T = 4.0;  DIST_ANG = 8.0;  DIST_DUR = 2.0;

%% =========================================================================
%  6. VERIFICA FISICA (stampa a console)
% =========================================================================
fprintf('=== Verifica fisica PWM -> spinta ===\n');
fprintf('Spinta @CCR=%d  : %.3f N\n', CCR_MIN,    thrust_fn(CCR_MIN,   RHO,VBATT,ARR,KV,CT,D_PROP));
fprintf('Spinta @CCR=%d : %.3f N\n', CCR_MAX,    thrust_fn(CCR_MAX,   RHO,VBATT,ARR,KV,CT,D_PROP));
fprintf('Spinta @CCR=%d : %.3f N  (offset PID)\n', CCR_OFFSET, thrust_fn(CCR_OFFSET,RHO,VBATT,ARR,KV,CT,D_PROP));
fprintf('Peso drone     : %.3f N  (serve %.3f N/motore)\n\n', MASSA*G, MASSA*G/2);

%% =========================================================================
%  7. CICLO DI SIMULAZIONE
% =========================================================================
quot=0; vel=0; acc=0;
roll=0; pitch=0; yaw=0;
m_pwm=CCR_BASE; top_f=CCR_BASE; bot_f=CCR_BASE;
flap_r=0; flap_p=0;

pid_m = makePID(Kp_m,Ki_m,Kd_m,dt_m,CCR_MIN,CCR_MAX,CCR_OFFSET,lpf_m);
pid_r = makePID(Kp_r,Ki_r,Kd_r,dt_s,-2*MAX_FLAP_DEG,2*MAX_FLAP_DEG,0,1);
pid_p = makePID(Kp_p,Ki_p,Kd_p,dt_s,-MAX_FLAP_DEG,MAX_FLAP_DEG,0,1);
pid_y = makePID(Kp_y,Ki_y,Kd_y,dt_y,-200,200,0,lpf_y);

L.quot=zeros(1,N); L.vel=zeros(1,N);  L.acc=zeros(1,N);
L.roll=zeros(1,N); L.pitch=zeros(1,N);L.yaw=zeros(1,N);
L.mpwm=zeros(1,N); L.top=zeros(1,N); L.bot=zeros(1,N);
L.su=zeros(1,N);   L.sb=zeros(1,N);
L.fr=zeros(1,N);   L.fp=zeros(1,N);

cnt_m=0; cnt_s=0;
DIV_M=round(dt_m/DT); DIV_S=round(dt_s/DT);

for k=1:N
    r_ref = DIST_ANG * (t(k)>=DIST_T && t(k)<(DIST_T+DIST_DUR));
    roll  = roll + DT*(1/0.3)*(r_ref - roll);

    cnt_s=cnt_s+1;
    if cnt_s>=DIV_S
        cnt_s=0;
        [dr, pid_r] = computePID(pid_r, REF_RP, roll);
        [dp, pid_p] = computePID(pid_p, REF_RP, pitch);
        flap_r = clamp(dr, -MAX_FLAP_DEG, MAX_FLAP_DEG);
        flap_p = clamp(dp, -MAX_FLAP_DEG, MAX_FLAP_DEG);
        [yc,pid_y]  = computePID(pid_y, REF_YAW, yaw);
        top_f = clamp(m_pwm + yc + YAW_TRIM/2, CCR_MIN, CCR_MAX);
        bot_f = clamp(m_pwm - yc - YAW_TRIM/2, CCR_MIN, CCR_MAX);
    end

    cnt_m=cnt_m+1;
    if cnt_m>=DIV_M
        cnt_m=0;
        alt_c = quot * cosd(roll) * cosd(pitch);
        [mf, pid_m] = computePID(pid_m, REF_ALT, alt_c);
        m_pwm = round(mf);
    end

    su = thrust_fn(top_f, RHO,       VBATT, ARR, KV, CT, D_PROP);
    sb = thrust_fn(bot_f, RHO*FSCIA, VBATT, ARR, KV, CT, D_PROP);
    acc = (su+sb)/MASSA - G;
    vel = vel + acc*DT;
    q_m = quot/1000 + vel*DT + 0.5*acc*DT^2;
    if q_m < 0, q_m=0; vel=0; end
    quot = q_m*1000;

    L.quot(k)=quot; L.vel(k)=vel;    L.acc(k)=acc;
    L.roll(k)=roll; L.pitch(k)=pitch;L.yaw(k)=yaw;
    L.mpwm(k)=m_pwm;L.top(k)=top_f; L.bot(k)=bot_f;
    L.su(k)=su;     L.sb(k)=sb;
    L.fr(k)=flap_r; L.fp(k)=flap_p;
end

fprintf('Simulazione OK  |  Quota finale = %.1f mm  |  Vel = %.4f m/s\n\n', ...
        L.quot(end), L.vel(end));

%% =========================================================================
%  8. GEOMETRIA 3D DEL CILINDRO
% =========================================================================
R_EXT = 0.2350/2;   R_INT = 0.2300/2;   H_CYL = 0.2200;
NC    = 64;
th    = linspace(0, 2*pi, NC+1);

% Coordinate locali (Z=0 al centro del cilindro)
xext = R_EXT*cos(th);   yext = R_EXT*sin(th);
xint = R_INT*cos(th);   yint = R_INT*sin(th);

%% =========================================================================
%  9. SETUP FIGURA
% =========================================================================
fig = figure('Name','DPDF — Simulazione 3D','NumberTitle','off', ...
             'Position',[40 40 1380 760],'Color',[0.10 0.10 0.13]);

ax3 = axes('Parent',fig,'Position',[0.01 0.05 0.55 0.92]);
setup_ax3d(ax3);

C=[0.10 0.10 0.13]; Ct=[0.85 0.85 0.85];
ax_q = axes('Parent',fig,'Position',[0.59 0.55 0.39 0.38]);
ax_v = axes('Parent',fig,'Position',[0.59 0.08 0.39 0.38]);
for a=[ax_q ax_v]
    a.Color=C; a.GridColor=[0.35 0.35 0.35]; a.GridAlpha=0.5;
    a.XColor=Ct; a.YColor=Ct; hold(a,'on'); grid(a,'on');
end
ylabel(ax_q,'Quota [mm]','Color',Ct);
title(ax_q,'Quota vs Setpoint','Color','w','FontSize',11);
ylabel(ax_v,'m/s  /  deg','Color',Ct);
title(ax_v,'Velocita  &  Roll','Color','w','FontSize',11);
xlabel(ax_q,'Tempo [s]','Color',Ct);
xlabel(ax_v,'Tempo [s]','Color',Ct);
yline(ax_q, REF_ALT,'--','Color',[1 0.45 0.1],'LineWidth',1.4);
yline(ax_v, 0,'--','Color',[0.5 0.5 0.5],'LineWidth',0.8);
ln_q = plot(ax_q,nan,nan,'Color',[0.25 0.75 1.0],'LineWidth',1.6);
ln_v = plot(ax_v,nan,nan,'Color',[0.25 0.90 0.45],'LineWidth',1.6);
ln_r = plot(ax_v,nan,nan,'Color',[1.00 0.35 0.35],'LineWidth',1.6,'LineStyle','--');
legend(ax_v,{'Velocita','Roll'},'TextColor',Ct,'Color',C,'Location','northwest');

%% =========================================================================
%  10. ANIMAZIONE
% =========================================================================
SKIP = 4;
trail_pts = zeros(3,0);

for k=1:N
    if mod(k,SKIP)~=0, continue; end

    z_pos   = L.quot(k)/1000;
    rr      = deg2rad(L.roll(k));
    pr      = deg2rad(L.pitch(k));
    yr      = deg2rad(L.yaw(k));
    R_mat   = Rzyx(yr,pr,rr);
    pos     = [0; 0; z_pos + H_CYL/2];

    % ---- Genera superficie cilindro ----
    % Mesh 2 righe x (NC+1) colonne: riga 1=basso, riga 2=alto
    Xloc = [xext; xext];
    Yloc = [yext; yext];
    Zloc = [-H_CYL/2*ones(1,NC+1); H_CYL/2*ones(1,NC+1)];
    [Xe,Ye,Ze] = rot_mesh(Xloc, Yloc, Zloc, R_mat, pos);

    Xloc_i = [xint; xint];
    Yloc_i = [yint; yint];
    [Xi,Yi,Zi] = rot_mesh(Xloc_i, Yloc_i, Zloc, R_mat, pos);

    % Dischi (vettori 1D -> fill3)
    xd = [0, xext];  yd = [0, yext];
    N_d = length(xd);
    [Xtop,Ytop,Ztop] = rot_mesh(xd, yd,  H_CYL/2*ones(1,N_d), R_mat, pos);
    [Xbot,Ybot,Zbot] = rot_mesh(xd, yd, -H_CYL/2*ones(1,N_d), R_mat, pos);

    % Flap
    [frx,fry,frz] = make_flap(R_mat,pos,H_CYL,R_EXT*0.8,L.fr(k),[1;0;0]);
    [fpx,fpy,fpz] = make_flap(R_mat,pos,H_CYL,R_EXT*0.8,L.fp(k),[0;1;0]);

    % Colore cilindro
    alpha_c = clamp(z_pos/(REF_ALT/1000*1.4), 0, 1);
    cyl_col = viridis_approx(alpha_c);

    % Freccia spinta
    Fnet    = (L.su(k)+L.sb(k))/MASSA - G;
    arr_len = max(0.02, Fnet*0.03);

    % Trail
    trail_pts(:,end+1) = pos;
    if size(trail_pts,2)>250, trail_pts=trail_pts(:,end-249:end); end

    %% --- DISEGNO ---
    cla(ax3); hold(ax3,'on');

    % Pavimento
    surf(ax3, [-0.45 0.45;-0.45 0.45], ...
              [-0.45 -0.45;0.45 0.45], ...
              zeros(2,2), ...
         'FaceColor',[0.12 0.22 0.12], ...
         'EdgeColor',[0.18 0.28 0.18],'FaceAlpha',0.55);

    % Piano setpoint (contorno quadrato trasparente)
    qh = REF_ALT/1000;
    plot3(ax3, [-0.4 0.4 0.4 -0.4 -0.4], ...
               [-0.4 -0.4 0.4 0.4 -0.4], ...
               [qh qh qh qh qh], ...
          '--','Color',[1 0.45 0.1],'LineWidth',1.2);
    text(ax3, 0.22, 0.22, qh+0.012, sprintf('%.0f mm',REF_ALT), ...
         'Color',[1 0.6 0.2],'FontSize',8);

    % Trail
    Nt = size(trail_pts,2);
    if Nt > 1
        cmap_t = cool(Nt);
        for ti=1:Nt-1
            plot3(ax3, trail_pts(1,ti:ti+1), ...
                       trail_pts(2,ti:ti+1), ...
                       trail_pts(3,ti:ti+1), ...
                  '-','Color',cmap_t(ti,:),'LineWidth',1.1);
        end
    end

    % === CILINDRO ESTERNO (parete) ===
    surf(ax3, Xe, Ye, Ze, ...
         'FaceColor', cyl_col, 'EdgeColor','none', 'FaceAlpha', 0.80);

    % === CILINDRO INTERNO (parete interna, piu scura) ===
    surf(ax3, Xi, Yi, Zi, ...
         'FaceColor', cyl_col*0.40, 'EdgeColor','none', 'FaceAlpha', 0.55);

    % === DISCO TOP e BOT ===
    fill3(ax3, Xtop(:)', Ytop(:)', Ztop(:)', cyl_col*0.65, ...
          'EdgeColor','none','FaceAlpha',0.90);
    fill3(ax3, Xbot(:)', Ybot(:)', Zbot(:)', cyl_col*0.65, ...
          'EdgeColor','none','FaceAlpha',0.90);

    % Bordo anelli (wire)
    plot3(ax3, Xe(1,:), Ye(1,:), Ze(1,:), '-', ...
          'Color', min(cyl_col*1.4+0.1,1), 'LineWidth',0.9);
    plot3(ax3, Xe(2,:), Ye(2,:), Ze(2,:), '-', ...
          'Color', min(cyl_col*1.4+0.1,1), 'LineWidth',0.9);

    % === FLAP ===
    fill3(ax3, frx, fry, frz, [0.88 0.18 0.18], ...
          'EdgeColor','w','LineWidth',0.9,'FaceAlpha',0.95);
    fill3(ax3, fpx, fpy, fpz, [0.95 0.82 0.08], ...
          'EdgeColor','w','LineWidth',0.9,'FaceAlpha',0.95);

    % === 6 PIEDI ===
    for li=0:5
        la = li*pi/3;
        p0 = R_mat*[0;0;-H_CYL/2] + pos;
        p1 = R_mat*[R_EXT*1.15*cos(la); R_EXT*1.15*sin(la); -H_CYL/2-0.04] + pos;
        plot3(ax3,[p0(1) p1(1)],[p0(2) p1(2)],[p0(3) p1(3)], ...
              '-','Color',[0.72 0.72 0.72],'LineWidth',2.2);
    end

    % === FRECCIA SPINTA ===
    quiver3(ax3, pos(1), pos(2), pos(3)+H_CYL*0.55, ...
                 0, 0, arr_len, ...
            0,'Color',[0.25 1.0 0.45],'LineWidth',2.2,'MaxHeadSize',0.9);

    % === ASSE Z BODY ===
    az_tip = R_mat*[0;0;H_CYL*0.7]+pos;
    quiver3(ax3,pos(1),pos(2),pos(3), ...
            az_tip(1)-pos(1),az_tip(2)-pos(2),az_tip(3)-pos(3), ...
            0,'Color',[1 1 1],'LineWidth',1.4,'MaxHeadSize',0.4);

    % === HUD ===
    hud = sprintf(['t = %5.2f s\n' ...
                   'Quota : %7.1f mm\n' ...
                   'Vel   : %+6.3f m/s\n' ...
                   'Roll  : %+5.2f deg\n' ...
                   'Pitch : %+5.2f deg\n' ...
                   'Motor : %4d CCR\n' ...
                   'FlpR  : %+5.1f deg\n' ...
                   'FlpP  : %+5.1f deg'], ...
                  t(k), L.quot(k), L.vel(k), ...
                  L.roll(k), L.pitch(k), L.mpwm(k), ...
                  L.fr(k), L.fp(k));
    text(ax3,-0.42,-0.40,0.68, hud, 'Color','w','FontSize',8.5, ...
         'FontName','Courier','BackgroundColor',[0.04 0.04 0.05], ...
         'VerticalAlignment','top','Interpreter','none');

    setup_ax3d(ax3);

    % Grafici live
    set(ln_q,'XData',t(1:k),'YData',L.quot(1:k));
    set(ln_v,'XData',t(1:k),'YData',L.vel(1:k));
    set(ln_r,'XData',t(1:k),'YData',L.roll(1:k));
    xlim(ax_q,[0 T_SIM]); xlim(ax_v,[0 T_SIM]);
    ylim(ax_q,[0 max(50, max(L.quot(1:k))*1.25)]);
    vr_m = max([abs(L.vel(1:k)), abs(L.roll(1:k)), 0.05]);
    ylim(ax_v,[-vr_m*1.2, vr_m*1.2]);

    drawnow limitrate;
end

%% =========================================================================
%  11. GRAFICI FINALI
% =========================================================================
figure('Name','Risultati Simulazione','NumberTitle','off', ...
       'Position',[80 60 1180 680],'Color',[0.10 0.10 0.13]);
tiledlayout(2,3,'TileSpacing','compact','Padding','compact');
dark_tile(t, L.quot, 'Quota [mm]',  'Quota vs Setpoint', REF_ALT, [0.25 0.75 1.0]);
dark_tile(t, L.vel,  'Vel. [m/s]',  'Velocita',           0,       [0.25 0.90 0.45]);
dark_tile(t, L.acc,  'Accel [m/s2]','Accelerazione',       0,       [0.80 0.50 0.95]);
dark_tile(t, L.su,   'Spinta [N]',  'Spinte Motori',       [],      [0.25 0.75 1.0]);
  hold on; plot(t,L.sb,'--','Color',[1.0 0.40 0.40],'LineWidth',1.5);
  legend({'Superiore','Inferiore'},'TextColor',[0.85 0.85 0.85],'Color',[0.10 0.10 0.13]);
dark_tile(t, L.roll, 'Roll [deg]',  'Roll',                0,       [1.0 0.35 0.35]);
dark_tile(t, L.mpwm, 'CCR',         'Motor PWM (PID out)', [],      [1.0 0.85 0.20]);
sgtitle('Risultati Simulazione — DPDF','Color','w','FontSize',13);

save('sim_drone_results.mat','-struct','L');
fprintf('Dati salvati in sim_drone_results.mat\n');

%% =========================================================================
%  FUNZIONI LOCALI
%% =========================================================================

function T = thrust_fn(ccr, rho, vbatt, arr, kv, ct, d)
    % CCR -> duty_pct -> Ton_us -> throttle [0-1] -> tensione -> RPM -> N
    duty_pct  = (ccr / arr) * 100;
    ton_us    = (duty_pct/100) * 20000;
    throttle  = clamp((ton_us - 1000) / 1000, 0, 1);
    v_motor   = throttle * vbatt;
    n         = (v_motor * kv) / 60;
    T         = max(0, ct * rho * d^4 * n^2);
end

function v = clamp(x, lo, hi)
    v = max(lo, min(hi, x));
end

function pid = makePID(kp,ki,kd,dt,omin,omax,off,lpf)
    pid = struct('kp',kp,'ki',ki,'kd',kd,'dt',dt,...
                 'omin',omin,'omax',omax,'off',off,'lpf',lpf,...
                 'I',0,'prev_meas',0,'prev_D',0);
end

function [out, pid] = computePID(pid, sp, meas)
    e   = sp - meas;
    P   = pid.kp * e;
    raw = (meas - pid.prev_meas) / pid.dt;
    Df  = pid.lpf * raw + (1-pid.lpf) * pid.prev_D;
    D   = -pid.kd * Df;
    pid.I = pid.I + pid.ki * e * pid.dt;
    out   = pid.off + P + D + pid.I;
    if out > pid.omax
        out   = pid.omax;
        pid.I = pid.I - pid.ki * e * pid.dt;
    elseif out < pid.omin
        out   = pid.omin;
        pid.I = pid.I - pid.ki * e * pid.dt;
    end
    pid.prev_meas = meas;
    pid.prev_D    = Df;
end

function R = Rzyx(yaw, pitch, roll)
    Rz=[cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
    Ry=[cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
    Rx=[1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
    R  = Rz*Ry*Rx;
end

function [Xr,Yr,Zr] = rot_mesh(X,Y,Z,R,pos)
    if isvector(X)
        pts = R*[X(:)';Y(:)';Z(:)'] + pos;
        Xr=pts(1,:); Yr=pts(2,:); Zr=pts(3,:);
    else
        sz=size(X);
        pts=R*[X(:)';Y(:)';Z(:)'];
        Xr=reshape(pts(1,:)+pos(1),sz);
        Yr=reshape(pts(2,:)+pos(2),sz);
        Zr=reshape(pts(3,:)+pos(3),sz);
    end
end

function [fx,fy,fz] = make_flap(R_body, pos, H_cyl, span, angle_deg, axis_v)
    w  = 0.022;   L = span;
    ar = deg2rad(angle_deg);
    if axis_v(1)==1
        v=[[-w/2  w/2  w/2 -w/2];
           [-L/2 -L/2  L/2  L/2];
           [-H_cyl/2 -H_cyl/2 -H_cyl/2 -H_cyl/2]];
        Rf=[1 0 0; 0 cos(ar) -sin(ar); 0 sin(ar) cos(ar)];
    else
        v=[[-L/2  L/2  L/2 -L/2];
           [-w/2 -w/2  w/2  w/2];
           [-H_cyl/2 -H_cyl/2 -H_cyl/2 -H_cyl/2]];
        Rf=[cos(ar) 0 sin(ar); 0 1 0; -sin(ar) 0 cos(ar)];
    end
    vw = R_body*(Rf*v) + pos;
    fx=vw(1,:); fy=vw(2,:); fz=vw(3,:);
end

function rgb = viridis_approx(alpha)
    c1=[0.18 0.42 0.78]; c2=[0.15 0.78 0.55]; c3=[0.95 0.75 0.10];
    if alpha < 0.5
        rgb = c1 + 2*alpha*(c2-c1);
    else
        rgb = c2 + 2*(alpha-0.5)*(c3-c2);
    end
    rgb = max(0,min(1,rgb));
end

function setup_ax3d(ax)
    ax.Color      = [0.07 0.07 0.09];
    ax.GridColor  = [0.32 0.32 0.32];
    ax.GridAlpha  = 0.45;
    ax.XColor=[0.72 0.72 0.72];
    ax.YColor=[0.72 0.72 0.72];
    ax.ZColor=[0.72 0.72 0.72];
    xlabel(ax,'X [m]','Color',[0.8 0.8 0.8]);
    ylabel(ax,'Y [m]','Color',[0.8 0.8 0.8]);
    zlabel(ax,'Quota [m]','Color',[0.8 0.8 0.8]);
    title(ax,'DPDF — Visualizzazione 3D','Color','w', ...
          'FontSize',12,'FontWeight','bold');
    xlim(ax,[-0.45 0.45]); ylim(ax,[-0.45 0.45]); zlim(ax,[0 0.75]);
    view(ax,42,22); grid(ax,'on'); axis(ax,'vis3d');
end

function dark_tile(t, data, ylbl, ttl, ref_val, col)
    nexttile;
    ax=gca;
    ax.Color=[0.09 0.09 0.11];
    ax.GridColor=[0.32 0.32 0.32]; ax.GridAlpha=0.45;
    ax.XColor=[0.8 0.8 0.8]; ax.YColor=[0.8 0.8 0.8];
    hold on; grid on;
    plot(t, data,'Color',col,'LineWidth',1.5);
    if ~isempty(ref_val)
        yline(ref_val,'--','Color',[1 0.45 0.1],'LineWidth',1.2);
    end
    xlabel('Tempo [s]','Color',[0.8 0.8 0.8]);
    ylabel(ylbl,'Color',[0.8 0.8 0.8]);
    title(ttl,'Color','w');
end