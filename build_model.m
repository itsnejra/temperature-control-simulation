function build_model(mdl)
%BUILD_MODEL  Programski gradi Simulink model feedback regulacije temperature.
%
%   Struktura:  Setpoint -> (Sum:error) -> PID -> Saturacija(grijac)
%               -> Aktuator -> (+ poremecaj) -> Plant(prostorija)
%               -> (+ ambijent) -> izmjerena temperatura -> feedback
%
%   Blok referencira radne varijable: setpoint, T_amb, R, tau, tau2,
%   dist_gain, u_max, Kp, Ki, Kd, load_ts  (postavljaju se u run_project).

    if nargin < 1, mdl = 'temp_control'; end
    if bdIsLoaded(mdl), close_system(mdl, 0); end

    % model dijeli ime sa .slx fajlom u folderu -> bezopasno upozorenje, utisaj
    ws = warning('off', 'Simulink:Engine:MdlFileShadowing');
    cleanup = onCleanup(@() warning(ws));
    new_system(mdl);

    add = @(src,name,pos,varargin) add_block(src, [mdl '/' name], ...
        'Position', pos, varargin{:});

    % ---- Blokovi (x: lijevo->desno tok signala) -------------------------
    add('simulink/Sources/Constant', 'Setpoint', [30 60 80 90], ...
        'Value', 'setpoint');

    add('simulink/Math Operations/Sum', 'Sum_err', [130 58 160 92], ...
        'Inputs', '+-', 'IconShape', 'round');

    add('simulink/Continuous/PID Controller', 'PID', [210 50 280 100], ...
        'P', 'Kp', 'I', 'Ki', 'D', 'Kd');

    add('simulink/Discontinuities/Saturation', 'Heater_sat', [320 55 360 95], ...
        'UpperLimit', 'u_max', 'LowerLimit', '0');

    add('simulink/Continuous/Transfer Fcn', 'Actuator', [400 55 470 95], ...
        'Numerator', '[1]', 'Denominator', '[tau2 1]');

    add('simulink/Math Operations/Sum', 'Sum_heat', [510 58 540 92], ...
        'Inputs', '++', 'IconShape', 'round');

    add('simulink/Continuous/Transfer Fcn', 'Plant', [580 55 660 95], ...
        'Numerator', '[R]', 'Denominator', '[tau 1]');

    add('simulink/Math Operations/Sum', 'Sum_out', [700 58 730 92], ...
        'Inputs', '++', 'IconShape', 'round');

    % poremecaj: realni podaci -> coupling gain
    add('simulink/Sources/From Workspace', 'Load', [380 150 470 190], ...
        'VariableName', 'load_ts', 'Interpolate', 'on', ...
        'OutputAfterFinalValue', 'Holding final value');
    add('simulink/Math Operations/Gain', 'Coupling', [500 153 540 187], ...
        'Gain', 'dist_gain');

    add('simulink/Sources/Constant', 'Ambient', [660 150 710 180], ...
        'Value', 'T_amb');

    % izlazi
    add('simulink/Sinks/Scope', 'Scope', [800 50 840 90]);
    add('simulink/Sinks/To Workspace', 'T_out', [800 120 870 150], ...
        'VariableName', 'T_meas', 'SaveFormat', 'Timeseries');
    add('simulink/Sinks/To Workspace', 'U_out', [400 -10 470 20], ...
        'VariableName', 'U_cmd', 'SaveFormat', 'Timeseries');

    % ---- Veze -----------------------------------------------------------
    L = @(a,b) add_line(mdl, a, b, 'autorouting', 'on');
    L('Setpoint/1', 'Sum_err/1');
    L('Sum_err/1',  'PID/1');
    L('PID/1',      'Heater_sat/1');
    L('Heater_sat/1','Actuator/1');
    L('Heater_sat/1','U_out/1');          % grana: napon grijaca -> log
    L('Actuator/1', 'Sum_heat/1');
    L('Load/1',     'Coupling/1');
    L('Coupling/1', 'Sum_heat/2');
    L('Sum_heat/1', 'Plant/1');
    L('Plant/1',    'Sum_out/1');
    L('Ambient/1',  'Sum_out/2');
    L('Sum_out/1',  'Scope/1');
    L('Sum_out/1',  'T_out/1');           % grana: temperatura -> log
    L('Sum_out/1',  'Sum_err/2');         % feedback

    % ---- Solver ---------------------------------------------------------
    set_param(mdl, 'Solver', 'ode23t', 'StopTime', 't_end', ...
              'SolverType', 'Variable-step');

    Simulink.BlockDiagram.arrangeSystem(mdl);   % uredi izgled
end
