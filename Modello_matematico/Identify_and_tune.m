%% IDENTIFY_AND_TUNE_V2.M
%  Identificazione Black-Box e Tuning automatico PID per Roll o Pitch
%
%  CORREZIONI rispetto alla versione precedente:
%   1. Rimossa la doppia sottrazione dell'offset (non si sottrae più
%      PWM_servo_neutro prima e poi la media: si fa UNA sola centratura)
%   2. Usato tfest ordine 2 con 1 zero invece di ssest ordine 4 fisso
%      (più aderente alla fisica: il sistema flap->angolo è II ordine)
%   3. Convertito l'ingresso in GRADI prima di identificare, così il
%      modello ha unità fisiche coerenti (deg/deg) e pidtune lavora
%      su guadagni in gradi, non in CCR raw
%   4. DesignFocus cambiato da 'disturbance-rejection' a
%      'reference-tracking' per guadagni conservativi adatti al primo test
%   5. Aggiunto report del fit % e avviso se il modello è inaffidabile
%   6. Aggiunta stima del guadagno statico per sanity check fisico

close all; clearvars; clc;

%% =========================================================
%  COSTANTI HARDWARE  (da config.h del firmware)
%% =========================================================
CCR_PER_DEGREE   = 10.0;   % CCR per grado di rotazione del servo
CENTER_SERVO     = 1125;   % CCR posizione neutra servo
UPPER_LIMIT_SERVO = 1545;  % CCR limite fisico superiore
LOWER_LIMIT_SERVO = 705;   % CCR limite fisico inferiore
MAX_FLAP_DEG     = 30.0;   % Escursione massima flap [deg]

%% =========================================================
%  1) SELEZIONE ASSE
%% =========================================================
asse_scelto = questdlg('Quale asse stai analizzando?', ...
    'Selezione Asse', 'Rollio (Roll)', 'Beccheggio (Pitch)', 'Rollio (Roll)');
if isempty(asse_scelto), error('Selezione asse annullata'); end

%% =========================================================
%  2) CARICAMENTO E PULIZIA DATI
%% =========================================================
[file, path] = uigetfile('*.csv', 'Seleziona il log di telemetria');
if isequal(file, 0), error('Selezione file annullata'); end
csv_file = fullfile(path, file);

opts = detectImportOptions(csv_file, 'NumHeaderLines', 1);
opts.VariableNames = {'time_ms', 'roll_cdeg', 'pitch_cdeg', 'yaw_cdeg', ...
                      'alt_mm_int', 'top_pwm', 'bottom_pwm', ...
                      'roll_pwm', 'pitch_pwm', 'motors_enabled'};
opts.VariableTypes = repmat({'double'}, 1, 10);

T = readtable(csv_file, opts);

% Filtro: solo campioni con motori accesi
T = T(T.motors_enabled == 1, :);

% Rimuovi timestamp duplicati
[~, unique_idx] = unique(T.time_ms, 'stable');
T = T(unique_idx, :);

if height(T) < 50
    error('Troppo pochi campioni validi (%d). Verifica il file CSV.', height(T));
end

% Conversione angoli in gradi
T.roll_deg  = T.roll_cdeg  / 100;
T.pitch_deg = T.pitch_cdeg / 100;

% Vettore tempo e periodo di campionamento
t  = T.time_ms / 1000;
Ts = median(diff(t));
fprintf('Periodo di campionamento rilevato: %.4f s (%.1f Hz)\n', Ts, 1/Ts);

%% =========================================================
%  3) CONVERSIONE INGRESSO IN GRADI
%     L'ingresso al modello è l'angolo del flap in gradi,
%     NON il valore CCR raw. Questo rende il guadagno del
%     modello fisicamente interpretabile (deg_uscita / deg_flap)
%     e rende i guadagni PID direttamente usabili nel firmware
%     (che lavora in gradi tramite angle_to_pwm).
%% =========================================================
roll_flap_deg  = (T.roll_pwm  - CENTER_SERVO) / CCR_PER_DEGREE;
pitch_flap_deg = (T.pitch_pwm - CENTER_SERVO) / CCR_PER_DEGREE;

%% =========================================================
%  4) CENTRATURA UNICA DELL'OFFSET
%     Una sola sottrazione della media per lavorare attorno
%     al punto di equilibrio. Nessuna doppia sottrazione.
%% =========================================================
if strcmp(asse_scelto, 'Rollio (Roll)')
    U_raw = roll_flap_deg;
    Y_raw = T.roll_deg;
    titolo = 'ROLLIO';
    colore = 'b';
else
    U_raw = pitch_flap_deg;
    Y_raw = T.pitch_deg;
    titolo = 'BECCHEGGIO';
    colore = 'g';
end

U = U_raw - mean(U_raw);
Y = Y_raw - mean(Y_raw);

%% =========================================================
%  5) VISUALIZZAZIONE DATI GREZZI
%% =========================================================
figure('Name', ['Dati grezzi - ', titolo], 'Position', [50 100 1100 450]);
subplot(2,1,1);
plot(t, U, colore, 'LineWidth', 1.2);
grid on; ylabel('Delta flap [deg]'); title(['Ingresso: angolo flap - ', titolo]);
subplot(2,1,2);
plot(t, Y, 'r', 'LineWidth', 1.2);
grid on; xlabel('Tempo [s]'); ylabel('Angolo drone [deg]');
title(['Uscita: angolo ', titolo, ' del drone']);

%% =========================================================
%  6) IDENTIFICAZIONE  — tfest ordine 2 con 1 zero
%     Motivazione fisica: la risposta angolare di un corpo
%     rigido a un'eccitazione di momento è tipicamente un
%     sistema del II ordine. Un zero al numeratore cattura
%     il ritardo iniziale dell'attuatore.
%     Si prova anche senza zero (ordine 2, 0 zeri) e si
%     sceglie il modello con fit migliore.
%% =========================================================
dati_asse = iddata(Y, U, Ts);

fprintf('\n=== Identificazione %s ===\n', titolo);

opt_tf = tfestOptions('EnforceStability', true, 'Display', 'off');

% Modello A: 2 poli, 1 zero
sysA = tfest(dati_asse, 2, 1, opt_tf);
fitA = goodnessOfFit(sim(sysA, dati_asse).y, Y, 'NRMSE') * 100;

% Modello B: 2 poli, 0 zeri
sysB = tfest(dati_asse, 2, 0, opt_tf);
fitB = goodnessOfFit(sim(sysB, dati_asse).y, Y, 'NRMSE') * 100;

% Modello C: 3 poli, 1 zero (fallback se i precedenti sono insufficienti)
sysC = tfest(dati_asse, 3, 1, opt_tf);
fitC = goodnessOfFit(sim(sysC, dati_asse).y, Y, 'NRMSE') * 100;

fprintf('Fit modello A (2p 1z): %.1f %%\n', fitA);
fprintf('Fit modello B (2p 0z): %.1f %%\n', fitB);
fprintf('Fit modello C (3p 1z): %.1f %%\n', fitC);

% Scegli il modello con fit migliore
[best_fit, best_idx] = max([fitA, fitB, fitC]);
switch best_idx
    case 1; sys_asse = sysA; desc_modello = '2 poli, 1 zero';
    case 2; sys_asse = sysB; desc_modello = '2 poli, 0 zeri';
    case 3; sys_asse = sysC; desc_modello = '3 poli, 1 zero';
end
fprintf('\n>>> Modello selezionato: %s  (fit = %.1f %%)\n', desc_modello, best_fit);

% Avviso se il fit è basso
if best_fit < 40
    fprintf('\n⚠️  ATTENZIONE: fit < 40%%. Il modello non è affidabile.\n');
    fprintf('   I guadagni PID calcolati sono solo un punto di partenza indicativo.\n');
    fprintf('   Cause probabili: eccitazione insufficiente dei flap durante il test,\n');
    fprintf('   drone non in hover, o forte accoppiamento con altri assi.\n\n');
elseif best_fit < 60
    fprintf('\n⚠️  Fit tra 40%% e 60%%. Modello accettabile ma con incertezza.\n');
    fprintf('   Usa i guadagni come punto di partenza e affina con test fisici.\n\n');
else
    fprintf('\n✅  Fit > 60%%. Modello sufficientemente affidabile.\n\n');
end

%% =========================================================
%  7) SANITY CHECK: GUADAGNO STATICO
%     Il guadagno DC (uscita/ingresso a regime) deve essere
%     fisicamente plausibile. Con flap da ±30°, se il guadagno
%     è >> 1 il sistema amplifica troppo; se è << 0.01 i flap
%     non hanno quasi effetto. Valori attorno a 0.01..0.5 sono
%     tipici per un ducted fan su banco.
%% =========================================================
dc_gain = dcgain(sys_asse);
fprintf('Guadagno statico del modello (deg_drone / deg_flap): %.4f\n', dc_gain);
if abs(dc_gain) > 10
    fprintf('⚠️  Guadagno statico molto alto (%.2f). Possibile problema di scaling.\n', dc_gain);
end

%% =========================================================
%  8) VISUALIZZAZIONE VALIDAZIONE MODELLO
%% =========================================================
figure('Name', ['Validazione modello - ', titolo], 'Position', [50 100 1000 400]);
compare(dati_asse, sys_asse);
title(['Confronto dati reali vs modello - ', titolo, ...
       sprintf(' (fit = %.1f%%)', best_fit)]);

%% =========================================================
%  9) TUNING PID
%     DesignFocus 'reference-tracking': guadagni conservativi,
%     adatti al primo test fisico. Meno aggressivo di
%     'disturbance-rejection' che produceva guadagni enormi.
%
%     Nota: i guadagni Kp/Ki/Kd sono in GRADI (come il firmware).
%     Il firmware riceve gradi dall'IMU, calcola il PID in gradi,
%     poi converte in CCR tramite angle_to_pwm() con CCR_PER_DEGREE=10.
%% =========================================================
fprintf('\n=== Calcolo guadagni PIDF ===\n');

pid_opts = pidtuneOptions('DesignFocus', 'reference-tracking');
[C_pid, info] = pidtune(sys_asse, 'PIDF', pid_opts);

fprintf('Margine di fase: %.1f deg\n', info.PhaseMargin);
fprintf('Frequenza di crossover: %.3f rad/s\n', info.CrossoverFrequency);

fprintf('\n--- Guadagni PID %s ---\n', titolo);
fprintf('Kp = %.6f\n', C_pid.Kp);
fprintf('Ki = %.6f\n', C_pid.Ki);
fprintf('Kd = %.6f\n', C_pid.Kd);
fprintf('Tf (filtro derivativo) = %.6f\n', C_pid.Tf);

% Stima dell'azione massima del servo (sanity check attuatore)
errore_test = 5.0; % deg — errore tipico di 5 gradi
azione_max_deg = C_pid.Kp * errore_test;
azione_max_ccr = azione_max_deg * CCR_PER_DEGREE;
fprintf('\nPer un errore di %.0f deg: azione P = %.2f deg flap = %.0f CCR\n', ...
    errore_test, azione_max_deg, azione_max_ccr);
if abs(azione_max_ccr) > (UPPER_LIMIT_SERVO - CENTER_SERVO)
    fprintf('⚠️  Azione > escursione fisica del servo (±%d CCR = ±%.0f deg).\n', ...
        UPPER_LIMIT_SERVO - CENTER_SERVO, MAX_FLAP_DEG);
    fprintf('   Considera di scalare Kp di un fattore %.1f prima del test fisico.\n', ...
        abs(azione_max_ccr) / (UPPER_LIMIT_SERVO - CENTER_SERVO));
end

%% =========================================================
%  10) SIMULAZIONE IN ANELLO CHIUSO
%% =========================================================
sys_cl   = feedback(C_pid * sys_asse, 1);
sys_ctrl = feedback(C_pid, sys_asse);

target_angle    = 10;   % gradino di riferimento in gradi
tempo_simulazione = 5;  % secondi

figure('Name', ['Simulazione anello chiuso - ', titolo], 'Position', [100 100 1200 500]);

subplot(1,2,1);
[y_sim, t_sim] = step(target_angle * sys_cl, tempo_simulazione);
plot(t_sim, y_sim, colore, 'LineWidth', 2); hold on;
yline(target_angle, 'r--', sprintf('Target (%d deg)', target_angle), 'LineWidth', 1.5);
grid on;
xlabel('Tempo [s]'); ylabel('Angolo drone [deg]');
title(['Risposta a gradino +', num2str(target_angle), '° — ', titolo]);
ylim_max = max(abs(y_sim)) * 1.3;
if ylim_max > 0; ylim([-ylim_max/4, ylim_max]); end

subplot(1,2,2);
[u_sim, ~] = step(target_angle * sys_ctrl, t_sim);
plot(t_sim, u_sim, 'm', 'LineWidth', 1.5); hold on;
yline(MAX_FLAP_DEG,  'r--', sprintf('+%.0f° limite', MAX_FLAP_DEG), 'LineWidth', 1);
yline(-MAX_FLAP_DEG, 'r--', sprintf('-%.0f° limite', MAX_FLAP_DEG), 'LineWidth', 1);
grid on;
xlabel('Tempo [s]'); ylabel('Angolo flap richiesto [deg]');
title('Azione di controllo (gradi flap)');

%% =========================================================
%  11) RIEPILOGO FINALE PER IL FIRMWARE
%% =========================================================
fprintf('\n========================================\n');
fprintf('  RIEPILOGO — Valori da inserire in config.c\n');
fprintf('  Asse: %s\n', titolo);
fprintf('  Modello: %s  |  Fit: %.1f%%\n', desc_modello, best_fit);
fprintf('========================================\n');
if strcmp(asse_scelto, 'Rollio (Roll)')
    fprintf('.servo_roll = {\n');
else
    fprintf('.servo_pitch = {\n');
end
fprintf('    .kp = %.4ff,\n', C_pid.Kp);
fprintf('    .ki = %.4ff,\n', C_pid.Ki);
fprintf('    .kd = %.4ff,\n', C_pid.Kd);
fprintf('    ... (altri parametri invariati)\n');
fprintf('},\n');
fprintf('========================================\n');

if best_fit < 40
    fprintf('\n⚠️  Ricorda: fit < 40%%. Inizia con Kp/10 per il primo test fisico.\n');
end

%% =========================================================
%  12) TUNING INTERATTIVO CON pidTuner
%     Apre l'interfaccia grafica per affinare manualmente
%     i guadagni partendo dal modello identificato.
%
%     ISTRUZIONI:
%       1. Si apre la finestra pidTuner con il tuo modello
%       2. Clicca "Add Plot" -> "Step" -> "Closed-Loop"
%          per vedere la risposta a gradino in anello chiuso
%       3. Clicca "Add Plot" -> "Step" -> "Open-Loop"
%          per vedere l'azione sul servo (deve stare < ±30°)
%       4. Trascina il cursore "Response Time" verso sinistra
%          (più veloce) finché la risposta è pulita
%          senza overshoot eccessivo
%       5. Se Kd risulta negativo, passa il tipo da PIDF a PID
%          e imposta Kd = 0 (sistema a fase non minima)
%       6. Verifica che il margine di fase sia > 45°
%       7. Quando sei soddisfatto, copia i valori Kp/Ki/Kd
%          mostrati nel pannello sinistro di pidTuner
%% =========================================================

fprintf('\n========================================\n');
fprintf('  TUNING INTERATTIVO\n');
fprintf('  Apertura pidTuner con il modello %s...\n', titolo);
fprintf('  Segui le istruzioni stampate sopra.\n');
fprintf('========================================\n\n');

fprintf('LIMITI FISICI DA RISPETTARE nel grafico Open-Loop:\n');
fprintf('  Azione servo max: ±%.0f gradi  (= ±%d CCR)\n', ...
    MAX_FLAP_DEG, int32(MAX_FLAP_DEG * CCR_PER_DEGREE));
fprintf('  Se la curva supera questi limiti -> Response Time troppo aggressivo.\n\n');

pidTuner(sys_asse, 'PIDF');