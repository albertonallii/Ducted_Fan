close all;
clearvars;
clear global STOP_FLAG STOP_REASON serialObj_global EMERGENCY_KILL_SENT SHUTDOWN_DONE STOP_IS_KILL
clc;

%% PARAMETRI PROGRAMMA

HOVER        = 1050;
MOTOR_MIN    = 700;
MOTOR_MAX    = 1500;
STEP_SMALL   = 50;
STEP_MEDIUM  = 100;
SIN_AMP      = 100;
CENTER_SERVO = 1125;

DT_TARGET    = 0.02;     % 50 Hz
duration     = 30;       % durata prova [s]

PRINT_DEBUG_LINES = false;

%% =========================
% STATO GLOBALE CALLBACK
%% =========================

global STOP_FLAG STOP_REASON serialObj_global EMERGENCY_KILL_SENT SHUTDOWN_DONE STOP_IS_KILL

STOP_FLAG = false;
STOP_REASON = "unknown";
serialObj_global = [];
EMERGENCY_KILL_SENT = false;
SHUTDOWN_DONE = false;
STOP_IS_KILL = false;

%% =========================
% STATISTICHE
%% =========================

stats.telemetryRows = 0;
stats.ackOn  = 0;
stats.ackOff = 0;
stats.ackKill = 0;
stats.ackErr = 0;
stats.debugLines = 0;
stats.badLines = 0;

%% =========================
% CONFIGURAZIONE SERIALE
%% =========================

delete(serialportfind);

port = "/dev/cu.usbmodem11103";
baudrate = 230400;

serialObj = serialport(port, baudrate);
configureTerminator(serialObj, "LF");
serialObj.Timeout = 0.02;

serialObj_global = serialObj;

pause(2);

% Flush consentito qui: siamo prima della sequenza di start.
flush(serialObj);

%% =========================
% MENU PROVE
%% =========================

labels = {
    "1.1 Gradino motori piccolo (±50)"
    "1.2 Gradino motori medio (±100)"
    "1.3 Sinusoide motori lenta (0.3 Hz)"
    "1.4 Sinusoide motori veloce (1 Hz)"
    "1.5 Gradino motori grande (±150)"
    "2.1 Gradino yaw differenziale"
    "2.2 Sinusoide yaw (0.5 Hz)"
    "3.1 Roll piccolo (±50)"
    "3.2 Roll medio (±100)"
    "3.3 Roll grande (±200)"
    "3.4 Sinusoide roll lenta (0.5 Hz)"
    "3.5 Sinusoide roll veloce (2 Hz)"
    "3.6 Chirp roll (0.1 - 3 Hz)"
    "4.1 Pitch piccolo"
    "4.2 Pitch medio"
    "4.3 Pitch grande"
    "4.4 Sinusoide pitch lenta"
    "4.5 Sinusoide pitch veloce"
    "4.6 Chirp pitch (0.1 - 3 Hz)"
    "5.1 Roll+Pitch alternati"
    "5.2 PRBS roll (0.5s random)"
    "5.3 PRBS pitch (0.5s random)"
    "5.4 PRBS roll e pitch (0.5s random)"
};

choice = listdlg( ...
    'PromptString', 'Seleziona prova OPEN-LOOP 50 Hz:', ...
    'SelectionMode', 'single', ...
    'ListString', labels, ...
    'ListSize', [360, 380]);

if isempty(choice)
    delete(serialObj);
    return;
end

testName = strrep(labels{choice}, " ", "_");

%% =========================
% FILE CSV
%% =========================

timestamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
folder = fullfile("AcquiredData", "Excitation");

if ~exist(folder, "dir")
    mkdir(folder);
end

filename = fullfile(folder, testName + "_" + timestamp + ".csv");

fid = fopen(filename, 'w');

if fid < 0
    delete(serialObj);
    error("Impossibile aprire il file CSV: %s", filename);
end

header = "time_ms,roll_cdeg,pitch_cdeg,yaw_cdeg," + ...
    "alt_mm,top_pwm,bottom_pwm,roll_pwm,pitch_pwm,motors_enabled," + ...
    "uart_rx_errors,uart_rx_overflows,uart_bad_lines," + ...
    "telemetry_overrun,stop_reason";
    fprintf(fid, "%s\n", header);

%% =========================
% ON CLEANUP
%% =========================

cleanupObj = onCleanup(@() cleanupOnExit(serialObj, fid, MOTOR_MIN, CENTER_SERVO));

%% =========================
% GUI + KILL SWITCH
%% =========================

fig = figure( ...
    "Name", "OPEN-LOOP 50 Hz - " + labels{choice}, ...
    "CloseRequestFcn", @closeFigKill, ...
    "KeyPressFcn", @keyKill);

uicontrol( ...
    "Style", "pushbutton", ...
    "String", "STOP / KILL (SPACE)", ...
    "FontSize", 16, ...
    "Units", "normalized", ...
    "Position", [0.2 0.3 0.6 0.4], ...
    "Callback", @guiKill);

statusLabel = uicontrol( ...
    "Style", "text", ...
    "FontSize", 12, ...
    "Units", "normalized", ...
    "Position", [0.1 0.1 0.8 0.15], ...
    "String", "Inizializzazione...");

%% =========================
% ESECUZIONE PRINCIPALE
%% =========================

try
    fprintf("\n--- ACQUISIZIONE OPEN-LOOP 50 Hz ---\n");
    fprintf("Prova: %s\n", labels{choice});
    fprintf("Porta: %s @ %d baud\n", port, baudrate);
    fprintf("File: %s\n", filename);

    %% =========================
    % START SEQUENCE ROBUSTA
    %% =========================

    fprintf("\n[START] Pulizia buffer iniziale...\n");

    % Flush solo qui, prima di ACK importanti.
    flush(serialObj);
    drainSerial(serialObj, 0.5);
    flush(serialObj);
    pause(0.05);
    fprintf("[START] Reset stato firmware con MOTOR_OFF...\n");

[~, stats] = sendCommandWaitAck( ...
    serialObj, ...
    "MOTOR_OFF", ...
    "ACK_OFF", ...
    0.3, ...
    2, ...
    fid, ...
    stats, ...
    PRINT_DEBUG_LINES);

pause(0.05);

    % Comando sicuro prima di abilitare i motori.
    % Se motors_enabled è false, il firmware ignora i motori ma può centrare i servi.
    fprintf("[START] Invio CMD sicuri al minimo...\n");

    for i = 1:5
        writeline(serialObj, sprintf("CMD,%d,%d,%d,%d", ...
            MOTOR_MIN, MOTOR_MIN, CENTER_SERVO, CENTER_SERVO));

        pause(0.02);
        [stats, ~] = readSerialAvailable(serialObj, fid, stats, "", PRINT_DEBUG_LINES);
    end

    % MOTOR_ON + attesa ACK_ON.
    fprintf("[START] Invio MOTOR_ON e attendo ACK_ON con CMD minimi keep-alive...\n");

keepAliveMinCmd = sprintf("CMD,%d,%d,%d,%d", ...
    MOTOR_MIN, MOTOR_MIN, CENTER_SERVO, CENTER_SERVO);

[okOn, stats] = sendCommandWaitAck( ...
    serialObj, ...
    "MOTOR_ON", ...
    "ACK_ON", ...
    1.0, ...
    3, ...
    fid, ...
    stats, ...
    PRINT_DEBUG_LINES, ...
    keepAliveMinCmd, ...
    0.04);

    if ~okOn
        STOP_REASON = "timeout ACK_ON";
        fprintf(2, "[START] ERRORE: ACK_ON non ricevuto. Abort prova.\n");

        stats = safeStopMotors(serialObj, MOTOR_MIN, CENTER_SERVO, fid, stats, PRINT_DEBUG_LINES);
        SHUTDOWN_DONE = true;

        error("Abort: ACK_ON non ricevuto.");
    end

    fprintf("[START] ACK_ON ricevuto. Invio primo CMD minimo.\n");

    % Subito dopo ACK_ON mando un CMD minimo per aggiornare last_cmd_tick_ms.
    writeline(serialObj, sprintf("CMD,%d,%d,%d,%d", ...
        MOTOR_MIN, MOTOR_MIN, CENTER_SERVO, CENTER_SERVO));

    [stats, ~] = readSerialAvailable(serialObj, fid, stats, "", PRINT_DEBUG_LINES);

    %% =========================
    % LOOP PRINCIPALE
    %% =========================

    fprintf("\n[RUN] Acquisizione avviata.\n");
    fprintf("[RUN] Premi SPACE o il pulsante GUI per KILL.\n\n");

    STOP_REASON = "fine durata";

    t0 = tic;
    nextCmdTime = 0;

    while toc(t0) < duration

        drawnow limitrate;

        if STOP_FLAG
            break;
        end

        t = toc(t0);

        % Lettura frequente, prima di eventuale invio.
        [stats, ~] = readSerialAvailable(serialObj, fid, stats, "", PRINT_DEBUG_LINES);

        % Invio CMD a 50 Hz circa.
        if ~STOP_FLAG && t >= nextCmdTime

            [top, bottom, roll, pitch] = generateSignal( ...
                choice, ...
                t, ...
                HOVER, ...
                CENTER_SERVO, ...
                STEP_SMALL, ...
                STEP_MEDIUM, ...
                SIN_AMP, ...
                duration);

            top    = max(MOTOR_MIN, min(MOTOR_MAX, round(top)));
            bottom = max(MOTOR_MIN, min(MOTOR_MAX, round(bottom)));
            roll   = round(roll);
            pitch  = round(pitch);

            cmd = sprintf("CMD,%d,%d,%d,%d", top, bottom, roll, pitch);
            writeline(serialObj, cmd);

            % Scheduling anti-burst: se MATLAB resta indietro, non recupera
            % mandando tanti CMD consecutivi.
            nextCmdTime = nextCmdTime + DT_TARGET;

            if (t - nextCmdTime) > DT_TARGET
                nextCmdTime = t + DT_TARGET;
            end
        end

        % Lettura frequente anche dopo l'invio.
        [stats, ~] = readSerialAvailable(serialObj, fid, stats, "", PRINT_DEBUG_LINES);

        if isvalid(fig)
            statusLabel.String = sprintf( ...
                "Tempo: %.2f / %.1f s | righe: %d | ACK_ERR: %d", ...
                t, duration, stats.telemetryRows, stats.ackErr);
        end

        drawnow limitrate;
    end

    %% =========================
    % SHUTDOWN CONTROLLATO
    %% =========================

global STOP_IS_KILL

if STOP_FLAG && STOP_IS_KILL

    fprintf("\n[STOP] Kill richiesto: %s\n", STOP_REASON);
    fprintf("[STOP] Eseguo emergenza KILL controllata...\n");

    stats = emergencyKillMotors(serialObj, MOTOR_MIN, CENTER_SERVO, fid, stats, PRINT_DEBUG_LINES);

elseif STOP_FLAG

    fprintf("\n[STOP] Stop normale richiesto: %s\n", STOP_REASON);
    fprintf("[STOP] Eseguo MOTOR_OFF controllato...\n");

    stats = safeStopMotors(serialObj, MOTOR_MIN, CENTER_SERVO, fid, stats, PRINT_DEBUG_LINES);

else

    STOP_REASON = "fine durata";
    fprintf("\n[STOP] Fine durata. Eseguo stop normale con MOTOR_OFF...\n");

    stats = safeStopMotors(serialObj, MOTOR_MIN, CENTER_SERVO, fid, stats, PRINT_DEBUG_LINES);

end

    SHUTDOWN_DONE = true;

catch ME

    if STOP_REASON == "timeout ACK_ON"
        fprintf(2, "\n[ERRORE] %s\n", ME.message);
        fprintf(2, "[ERRORE] ACK_ON non ricevuto. MOTOR_OFF già tentato, non mando KILL.\n");
        SHUTDOWN_DONE = true;

    else
        if STOP_REASON == "unknown"
            STOP_REASON = "errore";
        else
            STOP_REASON = "errore: " + string(ME.message);
        end

        fprintf(2, "\n[ERRORE] %s\n", ME.message);
        fprintf(2, "[ERRORE] Provo stop sicuro di emergenza...\n");

        try
            stats = emergencyKillMotors(serialObj, MOTOR_MIN, CENTER_SERVO, fid, stats, PRINT_DEBUG_LINES);
        catch ME2
            fprintf(2, "[ERRORE] Anche emergencyKillMotors ha fallito: %s\n", ME2.message);
        end

        SHUTDOWN_DONE = true;
    end
end

%% =========================
% CHIUSURA FILE / GUI / SERIALE
%% =========================

try
    [stats, ~] = readSerialAvailable(serialObj, fid, stats, "", PRINT_DEBUG_LINES);
catch
end

try
    fclose(fid);
catch
end

try
    if exist("fig", "var") && isvalid(fig)
        fig.CloseRequestFcn = '';
        delete(fig);
    end
catch
end

try
    delete(serialObj);
catch
end

%% =========================
% REPORT FINALE
%% =========================

fprintf("\n========== REPORT FINALE ==========\n");
fprintf("File CSV: %s\n", filename);
fprintf("Motivo stop: %s\n", STOP_REASON);
fprintf("Righe telemetria salvate: %d\n", stats.telemetryRows);
fprintf("ACK_ON ricevuti:   %d\n", stats.ackOn);
fprintf("ACK_OFF ricevuti:  %d\n", stats.ackOff);
fprintf("ACK_KILL ricevuti: %d\n", stats.ackKill);
fprintf("ACK_ERR ricevuti:  %d\n", stats.ackErr);
fprintf("Righe debug/testo ignorate: %d\n", stats.debugLines);
fprintf("Righe corrotte/vuote ignorate: %d\n", stats.badLines);
fprintf("===================================\n");

clear cleanupObj;

%% ========================================================================
% FUNZIONI LOCALI
%% ========================================================================

function [top, bottom, roll, pitch] = generateSignal(choice, time, HOVER, CENTER, SMALL, MEDIUM, AMP, T_total)

    top = HOVER;
    bottom = HOVER;
    roll = CENTER;
    pitch = CENTER;

    switch choice

        case 1
            % 1.1 Gradino motori piccolo
            val = HOVER - SMALL;
            if time >= 5 && time < 10
                val = HOVER;
            elseif time >= 10 && time < 15
                val = HOVER + SMALL;
            end
            top = val;
            bottom = val;

        case 2
            % 1.2 Gradino motori medio
            val = HOVER - MEDIUM;
            if time >= 5 && time < 10
                val = HOVER;
            elseif time >= 10 && time < 15
                val = HOVER + MEDIUM;
            end
            top = val;
            bottom = val;

        case 3
            % 1.3 Sinusoide motori lenta
            val = HOVER + AMP * sin(2*pi*0.3*time);
            top = val;
            bottom = val;

        case 4
            % 1.4 Sinusoide motori veloce
            val = HOVER + AMP * sin(2*pi*1.0*time);
            top = val;
            bottom = val;

        case 5
            % 1.5 Gradino motori grande (±150)
            val = HOVER - 150;
            if time >= 5 && time < 10
                val = HOVER;
            elseif time >= 10 && time < 15
                val = HOVER + 150;
            end
            top = val;
            bottom = val;

        case 6
            % 2.1 Gradino yaw differenziale
            delta = 0;
            if time >= 5 && time < 10
                delta = SMALL;
            elseif time >= 10 && time < 15
                delta = -SMALL;
            end
            top = HOVER + delta;
            bottom = HOVER - delta;

        case 7
            % 2.2 Sinusoide yaw differenziale
            delta = AMP * sin(2*pi*0.5*time);
            top = HOVER + delta;
            bottom = HOVER - delta;

        case 8
            % 3.1 Roll piccolo
            if time >= 5 && time < 10
                roll = CENTER + SMALL;
            elseif time >= 10 && time < 15
                roll = CENTER - SMALL;
            end

        case 9
            % 3.2 Roll medio
            if time >= 5 && time < 10
                roll = CENTER + MEDIUM;
            elseif time >= 10 && time < 15
                roll = CENTER - MEDIUM;
            end

        case 10
            % 3.3 Roll grande
            if time >= 5 && time < 10
                roll = CENTER + 200;
            elseif time >= 10 && time < 15
                roll = CENTER - 200;
            end

        case 11
            % 3.4 Sinusoide roll lenta
            roll = CENTER + AMP * sin(2*pi*0.5*time);

        case 12
            % 3.5 Sinusoide roll veloce
            roll = CENTER + AMP * sin(2*pi*2.0*time);

        case 13
            % 3.6 Chirp roll 0.1 - 3 Hz
            f0 = 0.1;
            f1 = 3.0;
            phase = 2*pi*(f0*time + 0.5*(f1-f0)*time^2/T_total);
            roll = CENTER + AMP * sin(phase);

        case 14
            % 4.1 Pitch piccolo
            if time >= 5 && time < 10
                pitch = CENTER + SMALL;
            elseif time >= 10 && time < 15
                pitch = CENTER - SMALL;
            end

        case 15
            % 4.2 Pitch medio
            if time >= 5 && time < 10
                pitch = CENTER + MEDIUM;
            elseif time >= 10 && time < 15
                pitch = CENTER - MEDIUM;
            end

        case 16
            % 4.3 Pitch grande
            if time >= 5 && time < 10
                pitch = CENTER + 200;
            elseif time >= 10 && time < 15
                pitch = CENTER - 200;
            end

        case 17
            % 4.4 Sinusoide pitch lenta
            pitch = CENTER + AMP * sin(2*pi*0.5*time);

        case 18
            % 4.5 Sinusoide pitch veloce
            pitch = CENTER + AMP * sin(2*pi*2.0*time);

        case 19
            % 4.6 Chirp pitch 0.1 - 3 Hz
            f0 = 0.1;
            f1 = 3.0;
            phase = 2*pi*(f0*time + 0.5*(f1-f0)*time^2/T_total);
            pitch = CENTER + AMP * sin(phase);

        case 20
            % 5.1 Roll+Pitch alternati
            if time >= 5 && time < 10
                roll = CENTER + SMALL;
                pitch = CENTER;
            elseif time >= 10 && time < 15
                roll = CENTER;
                pitch = CENTER + SMALL;
            elseif time >= 15 && time < 20
                roll = CENTER - SMALL;
                pitch = CENTER;
            elseif time >= 20 && time < 25
                roll = CENTER;
                pitch = CENTER - SMALL;
            end

        case 21
            % 5.2 PRBS roll ogni 0.5 s
            block = floor(time / 0.5);
            rng(block);
            levels = [-MEDIUM, -SMALL, 0, SMALL, MEDIUM];
            idx = randi(numel(levels));
            roll = CENTER + levels(idx);

        case 22
            % 5.3 PRBS pitch ogni 0.5 s
            block = floor(time / 0.5);
            rng(block);
            levels = [-MEDIUM, -SMALL, 0, SMALL, MEDIUM];
            idx = randi(numel(levels));
            pitch = CENTER + levels(idx);

        case 23
            % 5.4 PRBS roll e pitch ogni 0.5 s (sequenze indipendenti)
            block = floor(time / 0.5);
            rng(block);
            levels = [-MEDIUM, -SMALL, 0, SMALL, MEDIUM];
            idx_r = randi(numel(levels));
            roll = CENTER + levels(idx_r);
            rng(block + 1000);          % seed diverso per pitch
            idx_p = randi(numel(levels));
            pitch = CENTER + levels(idx_p);

        otherwise
            top = HOVER;
            bottom = HOVER;
            roll = CENTER;
            pitch = CENTER;
    end
end

function [ok, stats] = sendCommandWaitAck(serialObj, command, expectedAck, timeout_s, retries, fid, stats, printDebug, keepAliveCmd, keepAlivePeriod)

    if nargin < 9
        keepAliveCmd = "";
    end

    if nargin < 10
        keepAlivePeriod = 0.05;
    end

    ok = false;

    for attempt = 1:retries

        writeline(serialObj, command);

        tWait = tic;
        tKeepAlive = tic;

        while toc(tWait) < timeout_s

            [stats, ackFound] = readSerialAvailable(serialObj, fid, stats, expectedAck, printDebug);

            if ackFound
                ok = true;
                return;
            end

            % Keep-alive durante l'attesa ACK.
            % Utile soprattutto per ACK_ON, perché dopo MOTOR_ON
            % il watchdog firmware richiede CMD entro 200 ms.
            if strlength(keepAliveCmd) > 0 && toc(tKeepAlive) >= keepAlivePeriod
                writeline(serialObj, keepAliveCmd);
                tKeepAlive = tic;
            end

            pause(0.002);
        end
    end
end

function [stats, ackFound] = readSerialAvailable(serialObj, fid, stats, expectedAck, printDebug)

    ackFound = false;

    while serialObj.NumBytesAvailable > 0

        try
            rawLine = readline(serialObj);
        catch
            stats.badLines = stats.badLines + 1;
            return;
        end

      line = strtrim(string(rawLine));

if printDebug
    fprintf("[RX] %s\n", line);
end

        if strlength(line) == 0
            stats.badLines = stats.badLines + 1;
            continue;
        end

        % ACK firmware
        if line == "ACK_ON"
            stats.ackOn = stats.ackOn + 1;
            if expectedAck == "ACK_ON"
                ackFound = true;
            end
            continue;

        elseif line == "ACK_OFF"
            stats.ackOff = stats.ackOff + 1;
            if expectedAck == "ACK_OFF"
                ackFound = true;
            end
            continue;

        elseif line == "ACK_KILL"
            stats.ackKill = stats.ackKill + 1;
            if expectedAck == "ACK_KILL"
                ackFound = true;
            end
            continue;

        elseif line == "ACK_ERR"
            stats.ackErr = stats.ackErr + 1;
            if expectedAck == "ACK_ERR"
                ackFound = true;
            end
            continue;
        end

        % Telemetria CSV valida: 10 colonne = 9 virgole.
        if count(line, ",") == 14

            % Salvo solo righe CSV. Non salvo ACK/debug.
            try
                fprintf(fid, "%s\n", line);
                stats.telemetryRows = stats.telemetryRows + 1;
            catch
                stats.badLines = stats.badLines + 1;
            end

            continue;
        end

        % Debug/testo firmware: opzionale.
        if printDebug
            fprintf("[FW] %s\n", line);
        end

        stats.debugLines = stats.debugLines + 1;
    end
end

function stats = safeStopMotors(serialObj, MOTOR_MIN, CENTER_SERVO, fid, stats, printDebug)

    fprintf("[SAFE STOP] Stop normale: CMD minimo -> MOTOR_OFF -> ACK_OFF.\n");

    % 1) Smetto con comandi eccitanti e mando subito minimi.
    for i = 1:5
        writeline(serialObj, sprintf("CMD,%d,%d,%d,%d", ...
            MOTOR_MIN, MOTOR_MIN, CENTER_SERVO, CENTER_SERVO));

        pause(0.015);
        [stats, ~] = readSerialAvailable(serialObj, fid, stats, "", printDebug);
    end

    % 2) MOTOR_OFF con attesa ACK_OFF.
    [okOff, stats] = sendCommandWaitAck( ...
        serialObj, ...
        "MOTOR_OFF", ...
        "ACK_OFF", ...
        0.5, ...
        5, ...
        fid, ...
        stats, ...
        printDebug);

    if ~okOff
        fprintf(2, "[SAFE STOP] WARNING: ACK_OFF non ricevuto entro timeout.\n");
    else
        fprintf("[SAFE STOP] ACK_OFF ricevuto.\n");
    end

    % 3) Ancora CMD minimi dopo MOTOR_OFF.
    % Dopo MOTOR_OFF il firmware forza comunque i motori al minimo.
    % Questi CMD servono solo come ridondanza e per tenere i servi centrati.
    for i = 1:8
        writeline(serialObj, sprintf("CMD,%d,%d,%d,%d", ...
            MOTOR_MIN, MOTOR_MIN, CENTER_SERVO, CENTER_SERVO));

        pause(0.015);
        [stats, ~] = readSerialAvailable(serialObj, fid, stats, "", printDebug);
    end
end

function stats = emergencyKillMotors(serialObj, MOTOR_MIN, CENTER_SERVO, fid, stats, printDebug)

    fprintf("[KILL] Emergenza: invio KILL e attendo ACK_KILL.\n");

    [okKill, stats] = sendCommandWaitAck( ...
        serialObj, ...
        "KILL", ...
        "ACK_KILL", ...
        0.35, ...
        6, ...
        fid, ...
        stats, ...
        printDebug);

    if ~okKill
        fprintf(2, "[KILL] WARNING: ACK_KILL non ricevuto entro timeout.\n");
    else
        fprintf("[KILL] ACK_KILL ricevuto.\n");
    end

    pause(0.1);

    fprintf("[KILL] Reset latch con MOTOR_OFF.\n");

    [okOff, stats] = sendCommandWaitAck( ...
        serialObj, ...
        "MOTOR_OFF", ...
        "ACK_OFF", ...
        0.5, ...
        5, ...
        fid, ...
        stats, ...
        printDebug);

    if ~okOff
        fprintf(2, "[KILL] WARNING: ACK_OFF post-KILL non ricevuto.\n");
    else
        fprintf("[KILL] ACK_OFF post-KILL ricevuto. Latch resettato.\n");
    end

    for i = 1:5
        writeline(serialObj, sprintf("CMD,%d,%d,%d,%d", ...
            MOTOR_MIN, MOTOR_MIN, CENTER_SERVO, CENTER_SERVO));

        pause(0.015);
        [stats, ~] = readSerialAvailable(serialObj, fid, stats, "", printDebug);
    end
end

function guiKill(~, ~)

    global STOP_FLAG STOP_REASON serialObj_global EMERGENCY_KILL_SENT STOP_IS_KILL

    STOP_FLAG = true;
    STOP_IS_KILL = true;
    STOP_REASON = "STOP GUI";
    

    % Emergenza vera: mando subito KILL, ma non chiudo file/figura qui.
    if ~EMERGENCY_KILL_SENT
        EMERGENCY_KILL_SENT = true;

        try
            writeline(serialObj_global, "KILL");
        catch
        end
    end
end

function keyKill(~, event)

    global STOP_FLAG STOP_REASON serialObj_global EMERGENCY_KILL_SENT STOP_IS_KILL

    if strcmp(event.Key, 'space')

        STOP_FLAG = true;
        STOP_IS_KILL = true;
        STOP_REASON = "SPACE";
        
        if ~EMERGENCY_KILL_SENT
            EMERGENCY_KILL_SENT = true;

            try
                writeline(serialObj_global, "KILL");
            catch
            end
        end
    end
end

function closeFigKill(~, ~)

    global STOP_FLAG STOP_REASON STOP_IS_KILL
    STOP_FLAG = true;
    STOP_IS_KILL = false;
    STOP_REASON = "chiusura GUI";
    
    % Non cancello la figura qui.
    % La chiude il main loop dopo aver salvato e fermato tutto.
 end



    function cleanupOnExit(serialObj, fid, MOTOR_MIN, CENTER_SERVO)

    global SHUTDOWN_DONE

    if SHUTDOWN_DONE
        return;
    end

    fprintf(2, "\n[CLEANUP] Uscita non controllata: provo MOTOR_OFF sicuro.\n");

    try
        for i = 1:5
            writeline(serialObj, sprintf("CMD,%d,%d,%d,%d", ...
                MOTOR_MIN, MOTOR_MIN, CENTER_SERVO, CENTER_SERVO));
            pause(0.02);
        end
    catch
    end

    try
        for i = 1:5
            writeline(serialObj, "MOTOR_OFF");
            pause(0.03);
        end
    catch
    end

    try
        fclose(fid);
    catch
    end

    try
        delete(serialObj);
    catch
    end
end

function drainSerial(serialObj, drainTime_s)

    t0 = tic;

    while toc(t0) < drainTime_s

        while serialObj.NumBytesAvailable > 0
            try
                readline(serialObj);
            catch
                break;
            end
        end

        pause(0.01);
    end
end