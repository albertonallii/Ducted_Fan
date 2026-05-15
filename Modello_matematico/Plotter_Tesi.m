%% PLOTTER_TESI.M
% Script per generare automaticamente i grafici dei log di volo.
% Tutti i log verranno aperti in un'unica finestra divisa a schede (Tabs).
close all; clearvars; clc;

% 1. SELEZIONE MULTIPLA DEI FILE
[files, path] = uigetfile('*.csv', 'Seleziona i log (puoi selezionarne più di uno)', 'MultiSelect', 'on');
if isequal(files, 0)
    disp('Nessun file selezionato.');
    return;
end

% Se selezioni un solo file, MATLAB lo restituisce come stringa. Lo forziamo a cell array.
if ischar(files)
    files = {files}; 
end

% --- MODIFICA CHIAVE: CREAZIONE DI UN'UNICA FINESTRA PRINCIPALE ---
% Creiamo la finestra principale una sola volta, fuori dal ciclo
fig_principale = figure('Name', 'Analisi Log Telemetria', 'Position', [100, 100, 1200, 800]);

% Creiamo il gruppo di schede (Tab Group) che conterrà tutti i test
gruppo_schede = uitabgroup(fig_principale); 

% 2. CICLO SU TUTTI I FILE SELEZIONATI
for i = 1:length(files)
    csv_file = fullfile(path, files{i});
    fprintf('Generazione grafico per: %s\n', files{i});
    
    % Lettura dati
    opts = detectImportOptions(csv_file, 'NumHeaderLines', 1);
    opts.VariableNames = {'time_ms', 'roll_cdeg', 'pitch_cdeg', 'yaw_cdeg', ...
                          'alt_mm_int', 'top_pwm', 'bottom_pwm', ...
                          'roll_pwm', 'pitch_pwm', 'motors_enabled'};
    opts.VariableTypes = repmat({'double'}, 1, 10);
    T = readtable(csv_file, opts);
    
    % Filtro solo motori accesi e rimuovo duplicati
    T = T(T.motors_enabled == 1, :);
    [~, unique_idx] = unique(T.time_ms, 'stable');
    T = T(unique_idx, :);
    
    if height(T) < 10
        fprintf('Salto %s: dati insufficienti.\n', files{i});
        continue;
    end
    
    % Conversione tempo e angoli
    t = (T.time_ms - T.time_ms(1)) / 1000; % Tempo in secondi partendo da 0
    roll_deg = T.roll_cdeg / 100;
    pitch_deg = T.pitch_cdeg / 100;
    yaw_deg = T.yaw_cdeg / 100;
    
    % Rimuovi l'estensione .csv per i titoli
    titolo_pulito = strrep(files{i}, '.csv', '');
    titolo_pulito = strrep(titolo_pulito, '_', ' '); % Rimuove gli underscore
    
    % --- MODIFICA CHIAVE: CREAZIONE DELLA SCHEDA ---
    % Invece di creare una nuova "figure", creiamo una nuova scheda (uitab) 
    % e usiamo un nome abbreviato per l'etichetta (utile se hai 20 test)
    scheda_corrente = uitab(gruppo_schede, 'Title', sprintf('Test %d', i));
    
    % --- SUBPLOT 1: MOTORI (THROTTLE) ---
    % Diciamo al subplot di disegnarsi DENTRO la scheda_corrente usando 'Parent'
    ax1 = subplot(3, 1, 1, 'Parent', scheda_corrente);
    plot(ax1, t, T.top_pwm, 'b', 'LineWidth', 1.5); hold(ax1, 'on');
    plot(ax1, t, T.bottom_pwm, 'c--', 'LineWidth', 1.5);
    grid(ax1, 'on');
    ylabel(ax1, 'PWM Motori');
    title(ax1, ['Test: ', titolo_pulito], 'FontWeight', 'bold', 'FontSize', 12);
    legend(ax1, 'Top PWM', 'Bottom PWM', 'Location', 'best');
    
    % --- SUBPLOT 2: FLAP (COMANDI SERVO) ---
    ax2 = subplot(3, 1, 2, 'Parent', scheda_corrente);
    plot(ax2, t, T.roll_pwm, 'r', 'LineWidth', 1.5); hold(ax2, 'on');
    plot(ax2, t, T.pitch_pwm, 'g--', 'LineWidth', 1.5);
    yline(ax2, 1125, 'k:', 'Neutro (1125)', 'LineWidth', 1);
    grid(ax2, 'on');
    ylabel(ax2, 'PWM Servo (Flap)');
    legend(ax2, 'Roll PWM', 'Pitch PWM', 'Location', 'best');
    
    % --- SUBPLOT 3: ANGOLI DEL DRONE (RISPOSTA) ---
    ax3 = subplot(3, 1, 3, 'Parent', scheda_corrente);
    plot(ax3, t, roll_deg, 'r', 'LineWidth', 2); hold(ax3, 'on');
    plot(ax3, t, pitch_deg, 'g', 'LineWidth', 2);
    grid(ax3, 'on');
    xlabel(ax3, 'Tempo [secondi]');
    ylabel(ax3, 'Inclinazione [Gradi]');
    legend(ax3, 'Rollio (Roll)', 'Beccheggio (Pitch)', 'Location', 'best');
    
    % Sincronizza l'asse X
    linkaxes([ax1, ax2, ax3], 'x');
end

disp('Generazione grafici completata! Usa le schede in alto per navigare tra i log.');