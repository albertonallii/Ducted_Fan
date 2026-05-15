%% =========================================================================
%  DPDF — Simulazione 3D interattiva (versione gabbia + slider PWM)
%
%  VERSIONE FLUIDA: rendering handle-based.
%  Tutti gli oggetti grafici sono creati UNA VOLTA SOLA all'avvio e poi
%  aggiornati ad ogni frame solo nelle proprietà (XData/YData/ZData/Vertices).
%  Questo è ~5-10x più veloce del pattern delete-and-redraw.
%
%  - Drone DPDF: cilindro Ø235 mm × 220 mm di altezza (dati tesi)
%  - Gabbia cubica di lato pari a 3× l'altezza del drone (= 660 mm)
%  - Drone fluttuante a 300 mm dal pavimento
%  - 4 corde: dai 4 angoli superiori del drone ai 4 vertici superiori della
%    gabbia (orientati a 45/135/225/315°)
%  - Slider PWM per motore superiore e motore inferiore (CCR 750..1500)
%  - Slider PID per Roll & Pitch
%  - START / STOP / RESET
% =========================================================================
clc; clear; close all;

%% ---- Parametri fisici (da tesi) ----------------------------------------
MASSA   = 1.269;        % [kg]
G       = 9.81;         % [m/s^2]
RHO     = 1.225;        % [kg/m^3]
D_PROP  = 0.2286;       % [m]  (eliche GEMFAN 9x6)
FSCIA   = 0.7;          % fattore scia (downwash motore inferiore)
CT      = 0.10;         % coeff. di spinta (ridotto: spinta meno aggressiva per banco)
VBATT   = 14.8;         % [V]
KV      = 1400;         % [rpm/V]
ARR     = 14999;        % ARR del timer
Ixx = 0.0062; Iyy = 0.0062; Izz = 0.0100;   % [kg m^2]
d_cg_x  = 0.0028;       % offset CG asse X [m]
d_cg_y  = 0.0038;       % offset CG asse Y [m]

%% ---- Geometria drone (da tesi) -----------------------------------------
H_CYL = 0.220;          % altezza cilindro 220 mm
R_EXT = 0.235/2;        % raggio esterno 117.5 mm
R_INT = 0.230/2;        % raggio interno 115 mm
NC = 36; th = linspace(0,2*pi,NC+1);   % 36 segmenti = render veloce
xext = R_EXT*cos(th); yext = R_EXT*sin(th);
xint = R_INT*cos(th); yint = R_INT*sin(th);

%% ---- Gabbia ------------------------------------------------------------
CAGE_SIDE   = 3 * H_CYL;          % 0.660 m
CAGE_HALF   = CAGE_SIDE/2;
CAGE_BOTTOM = 0.0;
CAGE_TOP    = CAGE_BOTTOM + CAGE_SIDE;

ALT_EQ = 0.300;                   % bordo inferiore del drone a 0.300 m
Z_CYL_CENTER_EQ = ALT_EQ + H_CYL/2;

%% ---- Corde -------------------------------------------------------------
CAGE_ANCHORS = [ ...
     CAGE_HALF,  CAGE_HALF, CAGE_TOP;
    -CAGE_HALF,  CAGE_HALF, CAGE_TOP;
    -CAGE_HALF, -CAGE_HALF, CAGE_TOP;
     CAGE_HALF, -CAGE_HALF, CAGE_TOP];
ROPE_ANG = [45 135 225 315];

L_REST = zeros(1,4);
for ci=1:4
    a = deg2rad(ROPE_ANG(ci));
    p_drone = [R_EXT*cos(a); R_EXT*sin(a); ALT_EQ + H_CYL];
    p_cage  = CAGE_ANCHORS(ci,:)';
    L_REST(ci) = norm(p_cage - p_drone);
end
% L_REST = distanza geometrica esatta in posizione di equilibrio.
% Niente pretensionamento: le corde sono a riposo all'inizio. Sotto
% gravità il drone scende di qualche mm finché le corde non si tendono
% quanto basta a sostenerlo (equilibrio statico naturale, ~6 mm sotto ALT_EQ).

K_ROPE = 150000;        % [N/m]  corde elastiche, frequenza propria bassa
D_ROPE = 80;         % [N s/m] smorzamento ALTO (vicino al critico)
F_ROPE_MAX = 80;     % [N] saturazione forza per corda (anti-esplosione)
N_ROPE_SEG = 10;     % segmenti per visualizzazione corde

%% ---- Parametri PWM -----------------------------------------------------
CCR_ARM   = 750;
CCR_HOVER = 1050;
CCR_MAX   = 1500;
FLAP_MAX  = 30.0;

%% ---- Timing ------------------------------------------------------------
DT    = 0.025;       % timer 25 ms (40 Hz)
SUBST = 8;           % 8 sub-step di integrazione -> dt fisico = 3.125 ms
DT_S  = DT/SUBST;
T_END = 60.0;
TRAIL_MAX = 60;      % trail più corto = più veloce

%% =========================================================================
%  STATO DINAMICO
% =========================================================================
S = init_state(ALT_EQ, H_CYL, TRAIL_MAX);

%% =========================================================================
%  FIGURA PRINCIPALE
% =========================================================================
fig_main = figure('Name','DPDF — Simulazione 3D','NumberTitle','off', ...
    'Position',[300 60 1280 820],'Color',[0.09 0.09 0.12], ...
    'CloseRequestFcn',@on_close, ...
    'Renderer','opengl');

ax3 = axes('Parent',fig_main,'Position',[0.01 0.05 0.55 0.90]);
setup_ax3d(ax3, CAGE_HALF, CAGE_TOP);

BG = [0.09 0.09 0.12]; TC = [0.85 0.85 0.85];
ax_z = axes('Parent',fig_main,'Position',[0.60 0.71 0.37 0.24]);
ax_a = axes('Parent',fig_main,'Position',[0.60 0.41 0.37 0.24]);
ax_y = axes('Parent',fig_main,'Position',[0.60 0.10 0.37 0.24]);
for ax = [ax_z ax_a ax_y]
    ax.Color = BG; ax.GridColor = [0.33 0.33 0.33]; ax.GridAlpha = 0.5;
    ax.XColor = TC; ax.YColor = TC; hold(ax,'on'); grid(ax,'on');
    xlabel(ax,'Tempo [s]','Color',TC,'FontSize',8);
end
title(ax_z,'Quota bordo inferiore [mm]','Color','w','FontSize',9);
title(ax_a,'Roll & Pitch [deg]','Color','w','FontSize',9);
title(ax_y,'Yaw [deg]','Color','w','FontSize',9);
ylabel(ax_z,'mm','Color',TC,'FontSize',8);
ylabel(ax_a,'deg','Color',TC,'FontSize',8);
ylabel(ax_y,'deg','Color',TC,'FontSize',8);
yline(ax_z, ALT_EQ*1000,'--','Color',[0.7 0.7 0.3],'LineWidth',1);
yline(ax_a, 0,'--','Color',[0.5 0.5 0.5],'LineWidth',0.7);
yline(ax_y, 0,'--','Color',[0.5 0.5 0.5],'LineWidth',0.7);

ln_z = plot(ax_z, nan, nan, 'Color',[0.25 0.78 1.00],'LineWidth',1.6);
ln_r = plot(ax_a, nan, nan, 'Color',[1.00 0.30 0.30],'LineWidth',1.6);
ln_p = plot(ax_a, nan, nan, '--','Color',[0.25 0.90 0.45],'LineWidth',1.6);
ln_y = plot(ax_y, nan, nan, 'Color',[1.00 0.75 0.15],'LineWidth',1.6);
legend(ax_a,{'Roll','Pitch'},'TextColor',TC,'Color',BG,'FontSize',8, ...
       'Location','northeast');
xlim(ax_z,[0 30]); xlim(ax_a,[0 30]); xlim(ax_y,[0 30]);
ylim(ax_z,[0 700]); ylim(ax_a,[-10 10]); ylim(ax_y,[-10 10]);

%% =========================================================================
%  FINESTRA PID + PWM
% =========================================================================
pid0 = struct('Kp_r',0,'Ki_r',0,'Kd_r',0,'Kp_p',0,'Ki_p',0,'Kd_p',0, ...
              'CCR_top',CCR_ARM,'CCR_bot',CCR_ARM);
fig_ctrl = figure('Name','PID + PWM','NumberTitle','off', ...
    'Position',[10 60 340 760],'Color',[0.11 0.11 0.14], ...
    'MenuBar','none','ToolBar','none','Resize','off','UserData',pid0);

uicontrol(fig_ctrl,'Style','text','String','REGOLATORE PID & PWM', ...
    'Position',[0 725 340 28],'FontSize',12,'FontWeight','bold', ...
    'ForegroundColor',[0.9 0.9 0.9],'BackgroundColor',[0.11 0.11 0.14], ...
    'HorizontalAlignment','center');
uicontrol(fig_ctrl,'Style','text','String','— PWM MOTORI (CCR) —', ...
    'Position',[0 700 340 18],'FontSize',9,'FontWeight','bold', ...
    'ForegroundColor',[0.35 0.85 1.00],'BackgroundColor',[0.11 0.11 0.14], ...
    'HorizontalAlignment','center');
uicontrol(fig_ctrl,'Style','text','String','— ROLL —', ...
    'Position',[0 540 340 18],'FontSize',9,'FontWeight','bold', ...
    'ForegroundColor',[1.0 0.35 0.35],'BackgroundColor',[0.11 0.11 0.14], ...
    'HorizontalAlignment','center');
uicontrol(fig_ctrl,'Style','text','String','— PITCH —', ...
    'Position',[0 290 340 18],'FontSize',9,'FontWeight','bold', ...
    'ForegroundColor',[0.30 0.90 0.45],'BackgroundColor',[0.11 0.11 0.14], ...
    'HorizontalAlignment','center');

defs = { ...
    'CCR motore SUP',  'CCR_top', CCR_ARM, CCR_MAX, CCR_ARM, [0.45 0.85 1.0]; ...
    'CCR motore INF',  'CCR_bot', CCR_ARM, CCR_MAX, CCR_ARM, [0.45 0.85 1.0]; ...
    'Kp roll',         'Kp_r',    0,       8,       0,       [1.0  0.50 0.50]; ...
    'Ki roll',         'Ki_r',    0,       3,       0,       [1.0  0.65 0.65]; ...
    'Kd roll',         'Kd_r',    0,       2,       0,       [1.0  0.80 0.80]; ...
    'Kp pitch',        'Kp_p',    0,       8,       0,       [0.40 1.00 0.55]; ...
    'Ki pitch',        'Ki_p',    0,       3,       0,       [0.55 1.00 0.68]; ...
    'Kd pitch',        'Kd_p',    0,       2,       0,       [0.70 1.00 0.80]; ...
};
y_tops = [660 600   500 440 380   250 190 130];
N = size(defs,1);
sl_h  = gobjects(N,1);
lbl_h = gobjects(N,1);

for i = 1:N
    yy = y_tops(i);
    fmt = '%.0f'; if i>2, fmt = '%.3f'; end
    uicontrol(fig_ctrl,'Style','text','String',defs{i,1}, ...
        'Position',[12 yy+22 150 20], ...
        'ForegroundColor',defs{i,6},'BackgroundColor',[0.11 0.11 0.14], ...
        'FontSize',10,'FontWeight','bold','HorizontalAlignment','left');
    lbl_h(i) = uicontrol(fig_ctrl,'Style','text', ...
        'String',sprintf(fmt, defs{i,5}), ...
        'Position',[235 yy+22 90 20], ...
        'ForegroundColor',[0.25 0.95 0.55],'BackgroundColor',[0.08 0.08 0.10], ...
        'FontSize',10,'FontWeight','bold','HorizontalAlignment','center');
    sl_h(i) = uicontrol(fig_ctrl,'Style','slider', ...
        'Min',defs{i,3},'Max',defs{i,4},'Value',defs{i,5}, ...
        'Position',[12 yy 316 20],'BackgroundColor',[0.22 0.22 0.32]);
    set(sl_h(i),'Callback', ...
        @(src,~) slider_cb(src, lbl_h(i), fig_ctrl, defs{i,2}, fmt));
end

uicontrol(fig_ctrl,'Style','pushbutton','String','Reset slider', ...
    'Position',[90 14 160 32],'ForegroundColor',[0.9 0.9 0.9], ...
    'BackgroundColor',[0.28 0.28 0.40],'FontSize',10,'FontWeight','bold', ...
    'Callback',@(~,~) reset_sliders(sl_h,lbl_h,fig_ctrl,defs));

%% =========================================================================
%  BOTTONI START / STOP / RESET
% =========================================================================
uicontrol(fig_main,'Style','pushbutton','String','▶ START', ...
    'Position',[30 14 120 36],'FontSize',12,'FontWeight','bold', ...
    'ForegroundColor',[0.10 0.95 0.30],'BackgroundColor',[0.15 0.28 0.15], ...
    'Callback',@(~,~) btn_start(fig_main));
uicontrol(fig_main,'Style','pushbutton','String','■ STOP', ...
    'Position',[160 14 120 36],'FontSize',12,'FontWeight','bold', ...
    'ForegroundColor',[1.0 0.5 0.1],'BackgroundColor',[0.28 0.18 0.10], ...
    'Callback',@(~,~) btn_stop(fig_main));
uicontrol(fig_main,'Style','pushbutton','String','↺ RESET', ...
    'Position',[290 14 120 36],'FontSize',12,'FontWeight','bold', ...
    'ForegroundColor',[0.6 0.85 1.0],'BackgroundColor',[0.12 0.18 0.28], ...
    'Callback',@(~,~) btn_reset(fig_main));

%% =========================================================================
%  COSTANTI in UserData
% =========================================================================
C = struct();
C.MASSA = MASSA; C.G = G; C.RHO = RHO; C.D_PROP = D_PROP; C.FSCIA = FSCIA;
C.CT = CT; C.VBATT = VBATT; C.KV = KV; C.ARR = ARR;
C.Ixx = Ixx; C.Iyy = Iyy; C.Izz = Izz;
C.d_cg_x = d_cg_x; C.d_cg_y = d_cg_y;
C.H_CYL = H_CYL; C.R_EXT = R_EXT; C.R_INT = R_INT;
C.xext = xext; C.yext = yext; C.xint = xint; C.yint = yint; C.NC = NC;
C.CAGE_SIDE = CAGE_SIDE; C.CAGE_HALF = CAGE_HALF;
C.CAGE_BOTTOM = CAGE_BOTTOM; C.CAGE_TOP = CAGE_TOP;
C.CAGE_ANCHORS = CAGE_ANCHORS; C.ROPE_ANG = ROPE_ANG;
C.L_REST = L_REST; C.K_ROPE = K_ROPE; C.D_ROPE = D_ROPE;
C.F_ROPE_MAX = F_ROPE_MAX;
C.N_ROPE_SEG = N_ROPE_SEG;
C.ALT_EQ = ALT_EQ; C.Z_CYL_CENTER_EQ = Z_CYL_CENTER_EQ;
C.CCR_ARM = CCR_ARM; C.CCR_HOVER = CCR_HOVER; C.CCR_MAX = CCR_MAX;
C.FLAP_MAX = FLAP_MAX;
C.DT = DT; C.DT_S = DT_S; C.SUBST = SUBST; C.T_END = T_END;
C.TRAIL_MAX = TRAIL_MAX;
C.z_top    =  C.H_CYL/2;
C.z_bot    = -C.H_CYL/2;
C.fig_ctrl = fig_ctrl;

fig_main.UserData.C      = C;
fig_main.UserData.S      = S;
fig_main.UserData.S_init = S;
fig_main.UserData.ax3    = ax3;
fig_main.UserData.ax_z   = ax_z;
fig_main.UserData.ax_a   = ax_a;
fig_main.UserData.ax_y   = ax_y;
fig_main.UserData.ln_z   = ln_z;
fig_main.UserData.ln_r   = ln_r;
fig_main.UserData.ln_p   = ln_p;
fig_main.UserData.ln_y   = ln_y;

% Crea TUTTI gli oggetti grafici 3D una volta sola
H = create_graphics(ax3, C);
fig_main.UserData.H = H;

% Primo aggiornamento
update_graphics(fig_main);

%% =========================================================================
%  TIMER
% =========================================================================
tmr = timer('ExecutionMode','fixedSpacing','Period',DT, ...
            'TimerFcn',@(~,~) timer_step(fig_main), ...
            'ErrorFcn',@(~,~) btn_stop(fig_main), ...
            'BusyMode','drop','Name','dpdf_timer');
fig_main.UserData.tmr = tmr;

fprintf('\n=== DPDF Simulator (versione fluida) ===\n');
fprintf('• Render handle-based: ~40 fps su Mac.\n');
fprintf('• Premi START, poi alza gli slider CCR per far volare il drone.\n\n');


%% =========================================================================
%  ===========================  CALLBACKS  ================================
%% =========================================================================

function S = init_state(ALT_EQ, H_CYL, TRAIL_MAX)
    S.t       = 0;
    S.x       = 0;
    S.y       = 0;
    % Posizione di equilibrio statico calcolata con K_ROPE=600 senza
    % pretensionamento: drone a 270.6 mm dal pavimento (corde appena tese
    % che bilanciano esattamente il peso).
    S.z       = 0.3806;    % centro cilindro -> bordo inferiore a 270.6 mm
    S.vx = 0; S.vy = 0; S.vz = 0;
    % Assetto perfettamente orizzontale all'avvio (niente perturbazione)
    S.roll    = 0;
    S.pitch   = 0;
    S.yaw     = 0;
    S.p_ang   = 0;
    S.q_ang   = 0;
    S.r_ang   = 0;
    S.I_roll  = 0;  S.I_pitch = 0;
    S.D_roll  = 0;  S.D_pitch = 0;
    S.prev_r  = S.roll;
    S.prev_p  = S.pitch;
    S.ccr_top = 750;
    S.ccr_bot = 750;
    S.flap_r  = 0;
    S.flap_p  = 0;
    NMAX = 4000;
    S.log_t     = nan(1,NMAX);
    S.log_z     = nan(1,NMAX);
    S.log_roll  = nan(1,NMAX);
    S.log_pitch = nan(1,NMAX);
    S.log_yaw   = nan(1,NMAX);
    S.log_k     = 0;
    S.trail   = nan(3, TRAIL_MAX);
    S.trail_k = 0;
    S.running = false;
end

% -------------------------------------------------------------------------
function btn_start(fig)
    if ~isvalid(fig), return; end
    tmr = fig.UserData.tmr;
    if strcmp(tmr.Running,'off')
        fig.UserData.S.running = true;
        start(tmr);
    end
end

function btn_stop(fig)
    if ~isvalid(fig), return; end
    tmr = fig.UserData.tmr;
    if strcmp(tmr.Running,'on'), stop(tmr); end
    fig.UserData.S.running = false;
end

function btn_reset(fig)
    if ~isvalid(fig), return; end
    btn_stop(fig);
    C = fig.UserData.C;
    fig.UserData.S = init_state(C.ALT_EQ, C.H_CYL, C.TRAIL_MAX);
    ud = fig.UserData;
    set(ud.ln_z,'XData',nan,'YData',nan);
    set(ud.ln_r,'XData',nan,'YData',nan);
    set(ud.ln_p,'XData',nan,'YData',nan);
    set(ud.ln_y,'XData',nan,'YData',nan);
    set(ud.H.trail,'XData',nan,'YData',nan,'ZData',nan);
    update_graphics(fig);
end

function on_close(fig,~)
    try
        tmr = fig.UserData.tmr;
        if strcmp(tmr.Running,'on'), stop(tmr); end
        delete(tmr);
    catch
    end
    delete(fig);
end

function slider_cb(src, lbl, fig, field, fmt)
    v = src.Value;
    ud = fig.UserData; ud.(field) = v; fig.UserData = ud;
    lbl.String = sprintf(fmt, v);
end

function reset_sliders(sl_h, lbl_h, fig, defs)
    ud = fig.UserData;
    for i = 1:length(sl_h)
        v0 = defs{i,5}; sl_h(i).Value = v0;
        fmt = '%.0f'; if i>2, fmt = '%.3f'; end
        lbl_h(i).String = sprintf(fmt, v0);
        ud.(defs{i,2}) = v0;
    end
    fig.UserData = ud;
end

%% =========================================================================
%  STEP DEL TIMER : fisica + render
%% =========================================================================
function timer_step(fig)
    if ~isvalid(fig), return; end
    C = fig.UserData.C;
    S = fig.UserData.S;

    if S.t > C.T_END
        btn_stop(fig); return;
    end

    try
        pid = C.fig_ctrl.UserData;
    catch
        pid = struct('Kp_r',0,'Ki_r',0,'Kd_r',0,'Kp_p',0,'Ki_p',0,'Kd_p',0, ...
                     'CCR_top',C.CCR_ARM,'CCR_bot',C.CCR_ARM);
    end

    S.ccr_top = pid.CCR_top;
    S.ccr_bot = pid.CCR_bot;

    for ss = 1:C.SUBST
        S = physics_step(S, C, pid);
    end

    k = S.log_k + 1; S.log_k = k;
    if k <= length(S.log_t)
        S.log_t(k)     = S.t;
        S.log_z(k)     = (S.z - C.H_CYL/2) * 1000;
        S.log_roll(k)  = rad2deg(S.roll);
        S.log_pitch(k) = rad2deg(S.pitch);
        S.log_yaw(k)   = rad2deg(S.yaw);
    end

    % Trail come buffer circolare
    S.trail_k = mod(S.trail_k, C.TRAIL_MAX) + 1;
    S.trail(:, S.trail_k) = [S.x; S.y; S.z];

    fig.UserData.S = S;
    update_graphics(fig);
end

% -------------------------------------------------------------------------
function S = physics_step(S, C, pid)
    dt = C.DT_S;

    Tu = thrust_fn(S.ccr_top, C.RHO,           C.VBATT, C.ARR, C.KV, C.CT, C.D_PROP);
    Tb = thrust_fn(S.ccr_bot, C.RHO*C.FSCIA,   C.VBATT, C.ARR, C.KV, C.CT, C.D_PROP);
    Ft = Tu + Tb;

    cR = cos(S.roll); cP = cos(S.pitch);
    cos_tilt = cR * cP;
    Fz_motors = Ft * cos_tilt;

    [Fx_r, Fy_r, Fz_r, Mx_r, My_r] = rope_forces(S, C);

    % Drag aerodinamico traslazionale (l'aria nella gabbia frena il drone)
    K_DRAG_XY = 2.0;    % [N s/m]  drag laterale (aumentato per stabilità)
    K_DRAG_Z  = 1.5;    % [N s/m]  drag verticale (aumentato)
    Fx_drag = -K_DRAG_XY * S.vx;
    Fy_drag = -K_DRAG_XY * S.vy;
    Fz_drag = -K_DRAG_Z  * S.vz;

    ax = ( Fx_r + Fx_drag ) / C.MASSA;
    ay = ( Fy_r + Fy_drag ) / C.MASSA;
    az = ( Fz_motors + Fz_r + Fz_drag ) / C.MASSA - C.G;

    S.vx = S.vx + ax*dt;
    S.vy = S.vy + ay*dt;
    S.vz = S.vz + az*dt;

    % Anti-esplosione: saturo le velocità a valori fisici ragionevoli
    V_MAX = 2.0;   % [m/s]
    S.vx = clamp(S.vx, -V_MAX, V_MAX);
    S.vy = clamp(S.vy, -V_MAX, V_MAX);
    S.vz = clamp(S.vz, -V_MAX, V_MAX);

    S.x = S.x + S.vx*dt;
    S.y = S.y + S.vy*dt;
    S.z = S.z + S.vz*dt;

    z_bottom = S.z - C.H_CYL/2;
    if z_bottom < 0.005
        S.z = 0.005 + C.H_CYL/2;
        S.vz = max(0, S.vz);
    end

    Kp_r = pid.Kp_r; Ki_r = pid.Ki_r; Kd_r = pid.Kd_r;
    Kp_p = pid.Kp_p; Ki_p = pid.Ki_p; Kd_p = pid.Kd_p;

    err_r = -S.roll;
    S.I_roll = clamp(S.I_roll + Ki_r*err_r*dt, ...
                     -deg2rad(C.FLAP_MAX), deg2rad(C.FLAP_MAX));
    d_r = (S.roll - S.prev_r)/dt;
    S.D_roll = 0.25*d_r + 0.75*S.D_roll;
    S.prev_r = S.roll;
    fr = clamp( Kp_r*rad2deg(err_r) + rad2deg(S.I_roll) - Kd_r*rad2deg(S.D_roll), ...
                -C.FLAP_MAX, C.FLAP_MAX);

    err_p = -S.pitch;
    S.I_pitch = clamp(S.I_pitch + Ki_p*err_p*dt, ...
                      -deg2rad(C.FLAP_MAX), deg2rad(C.FLAP_MAX));
    d_p = (S.pitch - S.prev_p)/dt;
    S.D_pitch = 0.25*d_p + 0.75*S.D_pitch;
    S.prev_p = S.pitch;
    fp = clamp( Kp_p*rad2deg(err_p) + rad2deg(S.I_pitch) - Kd_p*rad2deg(S.D_pitch), ...
                -C.FLAP_MAX, C.FLAP_MAX);
    S.flap_r = fr; S.flap_p = fp;

    % Momento dei flap proporzionale alla spinta (più dolce di prima):
    % coeff 0.0025 invece di 0.010 -> flap ~4x meno aggressivi
    Mfr = fr * 0.0025 * Ft;
    Mfp = fp * 0.0025 * Ft;

    % Smorzamento angolare aerodinamico (alto: il drone in gabbia è
    % immerso nel proprio downwash, l'aria frena le rotazioni)
    Cd_ang = 1.50;

    % Sbilancio CG: produce momento ribaltante SOLO quando il drone è
    % sospeso nel suo stesso vettore di spinta. A motori spenti, le corde
    % o il vincolo verticale assorbono il momento gravitazionale.
    % Modulazione tramite il rapporto T/W (saturato a 1).
    TW_ratio = clamp(Ft / (C.MASSA*C.G), 0, 1);
    Mgr =  0.5 * C.MASSA * C.G * C.d_cg_x * cos(S.pitch) * TW_ratio;
    Mgp =  0.5 * C.MASSA * C.G * C.d_cg_y * cos(S.roll)  * TW_ratio;
    Myaw = (Tu - Tb)*0.0008 - 0.10*S.r_ang;

    Mx_total = Mgr + Mfr + Mx_r - Cd_ang*S.p_ang*C.Ixx;
    My_total = Mgp + Mfp + My_r - Cd_ang*S.q_ang*C.Iyy;

    S.p_ang = S.p_ang + (Mx_total/C.Ixx) * dt;
    S.q_ang = S.q_ang + (My_total/C.Iyy) * dt;
    S.r_ang = S.r_ang + (Myaw/C.Izz)     * dt;

    % Saturazione velocità angolari (evita instabilità numerica e
    % rotazioni "fulminee" non realistiche)
    W_MAX = deg2rad(60);     % 60 °/s
    S.p_ang = clamp(S.p_ang, -W_MAX, W_MAX);
    S.q_ang = clamp(S.q_ang, -W_MAX, W_MAX);
    S.r_ang = clamp(S.r_ang, -W_MAX, W_MAX);

    S.roll  = S.roll  + S.p_ang*dt;
    S.pitch = S.pitch + S.q_ang*dt;
    S.yaw   = S.yaw   + S.r_ang*dt;

    S.roll  = clamp(S.roll,  -deg2rad(30), deg2rad(30));
    S.pitch = clamp(S.pitch, -deg2rad(30), deg2rad(30));

    S.t = S.t + dt;
end

% -------------------------------------------------------------------------
function [Fx, Fy, Fz, Mx, My] = rope_forces(S, C)
    Fx = 0; Fy = 0; Fz = 0; Mx = 0; My = 0;
    R = Rzyx(S.yaw, S.pitch, S.roll);
    pos = [S.x; S.y; S.z];
    vel = [S.vx; S.vy; S.vz];
    omega = [S.p_ang; S.q_ang; S.r_ang];

    for ci = 1:4
        a = deg2rad(C.ROPE_ANG(ci));
        p_body = [C.R_EXT*cos(a); C.R_EXT*sin(a); C.H_CYL/2];
        Rp = R*p_body;
        p_world = Rp + pos;
        anchor = C.CAGE_ANCHORS(ci,:)';

        d_vec = anchor - p_world;
        L = norm(d_vec);
        if L < 1e-6, continue; end
        u = d_vec / L;

        stretch = L - C.L_REST(ci);
        if stretch > 0
            v_pt  = vel + cross(omega, Rp);
            v_axial = dot(v_pt, -u);
            F_t = C.K_ROPE * stretch + C.D_ROPE * (-v_axial);
            % Anti-esplosione: saturo la forza di ogni singola corda
            F_t = clamp(F_t, 0, C.F_ROPE_MAX);
            F_vec = F_t * u;

            Fx = Fx + F_vec(1);
            Fy = Fy + F_vec(2);
            Fz = Fz + F_vec(3);

            M = cross(Rp, F_vec);
            Mx = Mx + M(1);
            My = My + M(2);
        end
    end
end

%% =========================================================================
%  GRAFICA — CREAZIONE OGGETTI (UNA SOLA VOLTA)
%% =========================================================================
function H = create_graphics(ax3, C)
    hold(ax3,'on');
    H = struct();

    % ----- Pavimento (statico) ----------------------------------------
    half = C.CAGE_HALF + 0.10;
    surf(ax3, [-half half; -half half], [-half -half; half half], zeros(2,2), ...
         'FaceColor',[0.10 0.18 0.10],'EdgeColor',[0.14 0.22 0.14], ...
         'FaceAlpha',0.55);

    % ----- Gabbia (statica) -------------------------------------------
    draw_cage_static(ax3, C);

    % Linea di equilibrio
    plot3(ax3, [-C.CAGE_HALF C.CAGE_HALF], [0 0], [C.ALT_EQ C.ALT_EQ], ...
          ':','Color',[0.65 0.65 0.28],'LineWidth',0.9);

    % ----- Trail ------------------------------------------------------
    H.trail = plot3(ax3, nan, nan, nan, '-', ...
                    'Color',[0.5 0.8 1.0],'LineWidth',1.2);

    % ----- Cilindro ---------------------------------------------------
    NC = C.NC;
    H.cyl_ext = surf(ax3, nan(2,NC+1), nan(2,NC+1), nan(2,NC+1), ...
        'FaceColor',[0.18 0.52 0.90],'EdgeColor','none','FaceAlpha',0.85);
    H.cyl_int = surf(ax3, nan(2,NC+1), nan(2,NC+1), nan(2,NC+1), ...
        'FaceColor',[0.07 0.20 0.34],'EdgeColor','none','FaceAlpha',0.50);

    H.cap_top = patch(ax3, 'XData',nan(1,NC+1),'YData',nan(1,NC+1), ...
        'ZData',nan(1,NC+1),'FaceColor',[0.11 0.32 0.55],'EdgeColor','none', ...
        'FaceAlpha',0.90);
    H.cap_bot = patch(ax3, 'XData',nan(1,NC+1),'YData',nan(1,NC+1), ...
        'ZData',nan(1,NC+1),'FaceColor',[0.11 0.32 0.55],'EdgeColor','none', ...
        'FaceAlpha',0.90);

    H.rim_top = plot3(ax3, nan(1,NC+1), nan(1,NC+1), nan(1,NC+1), ...
        '-','Color',[0.5 0.8 1.0],'LineWidth',1.0);
    H.rim_bot = plot3(ax3, nan(1,NC+1), nan(1,NC+1), nan(1,NC+1), ...
        '-','Color',[0.5 0.8 1.0],'LineWidth',1.0);

    % ----- Flap -------------------------------------------------------
    H.flap_r = patch(ax3, 'XData',nan(1,4),'YData',nan(1,4),'ZData',nan(1,4), ...
        'FaceColor',[0.92 0.18 0.18],'EdgeColor','w','LineWidth',0.9,'FaceAlpha',0.93);
    H.flap_p = patch(ax3, 'XData',nan(1,4),'YData',nan(1,4),'ZData',nan(1,4), ...
        'FaceColor',[0.96 0.82 0.08],'EdgeColor','w','LineWidth',0.9,'FaceAlpha',0.93);

    % ----- 6 piedi ----------------------------------------------------
    H.feet = gobjects(6,1);
    for li = 1:6
        H.feet(li) = plot3(ax3, [nan nan], [nan nan], [nan nan], ...
            '-','Color',[0.68 0.68 0.68],'LineWidth',2.0);
    end

    % ----- 4 corde + marker -------------------------------------------
    H.ropes       = gobjects(4,1);
    H.rope_drone  = gobjects(4,1);
    H.rope_anchor = gobjects(4,1);
    for ci = 1:4
        H.ropes(ci) = plot3(ax3, nan(1,C.N_ROPE_SEG), nan(1,C.N_ROPE_SEG), ...
            nan(1,C.N_ROPE_SEG), '-','Color',[0.78 0.66 0.42],'LineWidth',1.6);
        H.rope_drone(ci) = plot3(ax3, nan, nan, nan, 'o', ...
            'Color',[1 0.9 0.3],'MarkerSize',5,'MarkerFaceColor',[1 0.9 0.3]);
        anc = C.CAGE_ANCHORS(ci,:);
        H.rope_anchor(ci) = plot3(ax3, anc(1), anc(2), anc(3), 's', ...
            'Color',[0.5 0.8 1.0],'MarkerSize',7, ...
            'MarkerFaceColor',[0.2 0.45 0.7]);
    end

    % ----- Asse Z body ------------------------------------------------
    H.body_z = quiver3(ax3, 0, 0, 0, 0, 0, 1, ...
        0,'Color','w','LineWidth',1.3,'MaxHeadSize',0.4);

    % ----- HUD --------------------------------------------------------
    H.hud = text(ax3, -C.CAGE_HALF-0.05, -C.CAGE_HALF-0.05, C.CAGE_TOP+0.05, '', ...
         'Color','w','FontSize',9,'FontName','Courier', ...
         'BackgroundColor',[0.04 0.04 0.06], ...
         'VerticalAlignment','top','Interpreter','none', ...
         'EdgeColor',[0.30 0.30 0.36],'LineWidth',0.5);
end

% -------------------------------------------------------------------------
function draw_cage_static(ax3, C)
    h = C.CAGE_HALF;
    bot = C.CAGE_BOTTOM; top = C.CAGE_TOP;
    col = [0.55 0.75 1.00]; lw = 1.6;
    corners_xy = [ h  h; -h  h; -h -h;  h -h];
    for i = 1:4
        plot3(ax3, [corners_xy(i,1) corners_xy(i,1)], ...
                   [corners_xy(i,2) corners_xy(i,2)], ...
                   [bot top], '-','Color',col,'LineWidth',lw);
    end
    for i = 1:4
        j = mod(i,4)+1;
        plot3(ax3, [corners_xy(i,1) corners_xy(j,1)], ...
                   [corners_xy(i,2) corners_xy(j,2)], ...
                   [top top], '-','Color',col,'LineWidth',lw);
    end
    for i = 1:4
        j = mod(i,4)+1;
        plot3(ax3, [corners_xy(i,1) corners_xy(j,1)], ...
                   [corners_xy(i,2) corners_xy(j,2)], ...
                   [bot bot], '-','Color',col*0.7,'LineWidth',lw*0.8);
    end
end

%% =========================================================================
%  GRAFICA — AGGIORNAMENTO (AD OGNI FRAME)
%% =========================================================================
function update_graphics(fig)
    if ~isvalid(fig), return; end
    ud = fig.UserData;
    C = ud.C; S = ud.S; H = ud.H;

    R = Rzyx(S.yaw, S.pitch, S.roll);
    pos = [S.x; S.y; S.z];

    NC = C.NC;
    % ------ Cilindro --------------------------------------------------
    X2 = [C.xext; C.xext];
    Y2 = [C.yext; C.yext];
    Z2 = [C.z_bot*ones(1,NC+1); C.z_top*ones(1,NC+1)];
    pts = R * [X2(:)'; Y2(:)'; Z2(:)'];
    Xe = reshape(pts(1,:)+pos(1), 2, NC+1);
    Ye = reshape(pts(2,:)+pos(2), 2, NC+1);
    Ze = reshape(pts(3,:)+pos(3), 2, NC+1);
    set(H.cyl_ext,'XData',Xe,'YData',Ye,'ZData',Ze);

    X2i = [C.xint; C.xint];
    Y2i = [C.yint; C.yint];
    pts_i = R * [X2i(:)'; Y2i(:)'; Z2(:)'];
    Xi = reshape(pts_i(1,:)+pos(1), 2, NC+1);
    Yi = reshape(pts_i(2,:)+pos(2), 2, NC+1);
    Zi = reshape(pts_i(3,:)+pos(3), 2, NC+1);
    set(H.cyl_int,'XData',Xi,'YData',Yi,'ZData',Zi);

    set(H.rim_top,'XData',Xe(2,:),'YData',Ye(2,:),'ZData',Ze(2,:));
    set(H.rim_bot,'XData',Xe(1,:),'YData',Ye(1,:),'ZData',Ze(1,:));

    pts_top = R*[C.xext; C.yext; C.z_top*ones(1,NC+1)];
    set(H.cap_top, 'XData', pts_top(1,:)+pos(1), ...
                   'YData', pts_top(2,:)+pos(2), ...
                   'ZData', pts_top(3,:)+pos(3));
    pts_bot = R*[C.xext; C.yext; C.z_bot*ones(1,NC+1)];
    set(H.cap_bot, 'XData', pts_bot(1,:)+pos(1), ...
                   'YData', pts_bot(2,:)+pos(2), ...
                   'ZData', pts_bot(3,:)+pos(3));

    tilt = abs(rad2deg(S.roll)) + abs(rad2deg(S.pitch));
    if tilt > 25
        cyl_col = [0.88 0.10 0.10];
    else
        f = min(1, tilt/15);
        cyl_col = [0.18+0.30*f, 0.58-0.20*f, 0.92-0.30*f];
    end
    set(H.cyl_ext,'FaceColor',cyl_col);
    set(H.cap_top,'FaceColor',cyl_col*0.62);
    set(H.cap_bot,'FaceColor',cyl_col*0.62);

    % ------ Flap -------------------------------------------------------
    [frx,fry,frz] = make_flap(R,pos,C.H_CYL,C.R_EXT*0.85,S.flap_r,[1;0;0]);
    [fpx,fpy,fpz] = make_flap(R,pos,C.H_CYL,C.R_EXT*0.85,S.flap_p,[0;1;0]);
    set(H.flap_r,'XData',frx,'YData',fry,'ZData',frz);
    set(H.flap_p,'XData',fpx,'YData',fpy,'ZData',fpz);

    % ------ 6 piedi ----------------------------------------------------
    for li = 1:6
        la = (li-1)*pi/3;
        p0 = R*[0;0;-C.H_CYL/2] + pos;
        p1 = R*[C.R_EXT*1.15*cos(la); C.R_EXT*1.15*sin(la); -C.H_CYL/2-0.04] + pos;
        set(H.feet(li),'XData',[p0(1) p1(1)], ...
                       'YData',[p0(2) p1(2)], ...
                       'ZData',[p0(3) p1(3)]);
    end

    % ------ Corde ------------------------------------------------------
    ns = C.N_ROPE_SEG;
    sv = linspace(0,1,ns);
    for ci = 1:4
        a = deg2rad(C.ROPE_ANG(ci));
        p_body = [C.R_EXT*cos(a); C.R_EXT*sin(a); C.H_CYL/2];
        p_world = R*p_body + pos;
        anchor = C.CAGE_ANCHORS(ci,:)';

        d_vec = anchor - p_world;
        L = norm(d_vec);
        stretch = L - C.L_REST(ci);
        if stretch > 0.002
            rope_col = [0.95 0.50 0.30];
            lw = 2.2;
            sag = 0;
        else
            rope_col = [0.78 0.66 0.42];
            lw = 1.6;
            sag = -stretch * 0.6;
        end
        rx = p_world(1) + sv*(anchor(1)-p_world(1));
        ry = p_world(2) + sv*(anchor(2)-p_world(2));
        rz = p_world(3) + sv*(anchor(3)-p_world(3)) - sag*sin(pi*sv);
        set(H.ropes(ci),'XData',rx,'YData',ry,'ZData',rz, ...
                        'Color',rope_col,'LineWidth',lw);
        set(H.rope_drone(ci),'XData',p_world(1),'YData',p_world(2),'ZData',p_world(3));
    end

    % ------ Asse Z body ------------------------------------------------
    az_tip = R*[0;0;C.H_CYL*0.75];
    set(H.body_z, 'XData',pos(1),'YData',pos(2),'ZData',pos(3), ...
                  'UData',az_tip(1),'VData',az_tip(2),'WData',az_tip(3));

    % ------ Trail ------------------------------------------------------
    if S.trail_k > 0
        idx = [S.trail_k+1:C.TRAIL_MAX, 1:S.trail_k];
        Tr = S.trail(:,idx);
        mask = ~isnan(Tr(1,:));
        if any(mask)
            set(H.trail,'XData',Tr(1,mask),'YData',Tr(2,mask),'ZData',Tr(3,mask));
        end
    end

    % ------ HUD --------------------------------------------------------
    Tu = thrust_fn(S.ccr_top, C.RHO,         C.VBATT,C.ARR,C.KV,C.CT,C.D_PROP);
    Tb = thrust_fn(S.ccr_bot, C.RHO*C.FSCIA, C.VBATT,C.ARR,C.KV,C.CT,C.D_PROP);
    TW = (Tu+Tb)/(C.MASSA*C.G);
    z_bottom_mm = (S.z - C.H_CYL/2)*1000;

    hud = sprintf([' t        %6.2f s \n' ...
                   ' Z bordo  %6.1f mm\n' ...
                   ' Roll     %+6.2f °\n' ...
                   ' Pitch    %+6.2f °\n' ...
                   ' Yaw      %+6.2f °\n' ...
                   ' CCR sup  %4.0f \n' ...
                   ' CCR inf  %4.0f \n' ...
                   ' T/W      %.2f  \n'], ...
                   S.t, z_bottom_mm, ...
                   rad2deg(S.roll), rad2deg(S.pitch), rad2deg(S.yaw), ...
                   S.ccr_top, S.ccr_bot, TW);
    set(H.hud,'String',hud);

    % ------ Grafici 2D -------------------------------------------------
    k = S.log_k;
    if k > 0
        tv = S.log_t(1:k);
        set(ud.ln_z,'XData',tv,'YData',S.log_z(1:k));
        set(ud.ln_r,'XData',tv,'YData',S.log_roll(1:k));
        set(ud.ln_p,'XData',tv,'YData',S.log_pitch(1:k));
        set(ud.ln_y,'XData',tv,'YData',S.log_yaw(1:k));
        if tv(end) > ud.ax_z.XLim(2) - 2
            new_xmax = tv(end) + 8;
            xlim(ud.ax_z,[0 new_xmax]);
            xlim(ud.ax_a,[0 new_xmax]);
            xlim(ud.ax_y,[0 new_xmax]);
        end
    end

    drawnow limitrate;
end

%% =========================================================================
%  UTILITY
%% =========================================================================
function T = thrust_fn(ccr, rho, vbatt, arr, kv, ct, d)
    % Modello ESC + motore con dead-band realistica:
    %  - CCR <= 950  -> motore in arming, spinta = 0 (ESC non motorizza)
    %  - 950 < CCR <= 1100 -> rampa dolce (regione di stallo aerodinamico)
    %  - CCR > 1100  -> regione operativa, throttle lineare da 0 a 1
    % I numeri sono coerenti con il comportamento osservato al banco
    % (motori "fermi" finché non si supera l'arming, poi spinta progressiva).
    if ccr <= 950
        throttle = 0;
    elseif ccr <= 1100
        % rampa quadratica dolce su [950, 1100]
        t = (ccr - 950) / 150;          % 0..1
        throttle = 0.20 * t.^2;          % al massimo 0.20 a CCR=1100
    else
        % oltre 1100: rampa lineare da 0.20 fino a 1.0 a CCR=1500
        t = (ccr - 1100) / 400;
        throttle = 0.20 + 0.80 * min(1, t);
    end
    n = (throttle * vbatt * kv) / 60;
    T = max(0, ct * rho * d^4 * n^2);
end

function v = clamp(x, lo, hi)
    v = max(lo, min(hi, x));
end

function R = Rzyx(yaw, pitch, roll)
    Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
    Ry = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
    Rx = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
    R  = Rz*Ry*Rx;
end

function [fx,fy,fz] = make_flap(R_body, pos, H_cyl, span, angle_deg, axis_v)
    w = 0.022; L = span; ar = deg2rad(angle_deg);
    if axis_v(1) == 1
        v = [[-w/2 w/2 w/2 -w/2]; [-L/2 -L/2 L/2 L/2]; -H_cyl/2*ones(1,4)];
        Rf = [1 0 0; 0 cos(ar) -sin(ar); 0 sin(ar) cos(ar)];
    else
        v = [[-L/2 L/2 L/2 -L/2]; [-w/2 -w/2 w/2 w/2]; -H_cyl/2*ones(1,4)];
        Rf = [cos(ar) 0 sin(ar); 0 1 0; -sin(ar) 0 cos(ar)];
    end
    vw = R_body*(Rf*v) + pos;
    fx = vw(1,:); fy = vw(2,:); fz = vw(3,:);
end

function setup_ax3d(ax, cage_half, cage_top)
    ax.Color = [0.07 0.07 0.09];
    ax.GridColor = [0.28 0.28 0.28]; ax.GridAlpha = 0.38;
    ax.XColor = [0.70 0.70 0.70];
    ax.YColor = [0.70 0.70 0.70];
    ax.ZColor = [0.70 0.70 0.70];
    xlabel(ax,'X [m]','Color',[0.75 0.75 0.75],'FontSize',9);
    ylabel(ax,'Y [m]','Color',[0.75 0.75 0.75],'FontSize',9);
    zlabel(ax,'Z [m]','Color',[0.75 0.75 0.75],'FontSize',9);
    title(ax,'DPDF — Simulazione 3D (gabbia + corde)', ...
          'Color','w','FontSize',12,'FontWeight','bold');
    m = cage_half + 0.08;
    xlim(ax,[-m m]); ylim(ax,[-m m]); zlim(ax,[0 cage_top+0.12]);
    view(ax, 42, 22); grid(ax,'on'); axis(ax,'vis3d');
end
