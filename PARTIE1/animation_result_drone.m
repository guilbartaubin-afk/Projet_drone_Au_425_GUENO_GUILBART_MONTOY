function animation_result_drone(sim, params, options, sim_ref)

% Animation comparative de drone avec référence optionnelle
%
% Syntaxe:
%   animation_comp_test(sim, params, options)           % Sans référence
%   animation_comp_test(sim, params, options, sim_ref)  % Avec référence
%
% Arguments:
%   sim      - Structure contenant t, u_m, y
%   params   - Structure contenant les paramètres du système
%   options  - Structure de configuration
%              * options.record_video : création d'un fichier vidéo MPEG-4
%              * options.video_filename : nom du fichier vidéo
%              * options.speed_ratio : vitesse d'annimation
%   sim_ref  - (optionnel) Structure de référence contenant u_m, y
% --------------------------------------------------------------------------------

%% === Initialisation des paramètres ===

% Détection de la présence de référence
has_ref = (nargin == 4);


% Extraction des options
record_video = getopt(options, 'record_video', false);
video_filename = getopt(options, 'video_filename', 'unamed_simulation_drone.mp4');
speed_ratio = getopt(options, 'speed_ratio', 1);


% Extraction des données
t = sim.t.Data;
u = sim.u_m.Data;
y = sim.y.Data;

if has_ref
    u_ref = sim_ref.u_m.Data;
    y_ref = sim_ref.y.Data;
end

RPM = 60/(2*pi);
mot_sat_down = params.Moteur_RPM_min;
mot_sat_up = params.Moteur_RPM_max;

% Calcul des limites des axes 3D
if has_ref
    y_max = max([max(y(:, [1, 3, 5]), [], 1); 
                 max(y_ref(:, [1, 3, 5]), [], 1)], [], 1);

    y_min = min([min(y(:, [1, 3, 5]), [], 1); 
                 min(y_ref(:, [1, 3, 5]), [], 1)], [], 1);
else
    y_max = max(y(:, [1, 3, 5]), [], 1);
    y_min = min(y(:, [1, 3, 5]), [], 1);
end

xlimit = [y_min(1), y_max(1)] + [-5 5];
ylimit = [y_min(2), y_max(2)] + [-5 5];
zlimit = [y_min(3), y_max(3)] + [-1 1];

% Calcul de l'énergie consomée
Energy = calculate_energy(t, u, params.kt);
if has_ref
    Energy_ref = calculate_energy(t, u_ref, params.kt);
end


%% === Initialisation des figures / texte ===

% Couleurs de référence
ref_alpha = 0.25;
trajectory_color = [0.49 0.18 0.56];


% Création de la figure principale
figure_width = 1500;
figure_height = 500;
viewx = -38; 
viewy = 25;
fig = figure('Visible', 'on', 'Position', [50, 50, figure_width, figure_height]);


% Figure : plot 3D
ax_3d = subplot(3, 5, [1, 2, 6, 7, 11, 12]);
set(ax_3d, 'XLim', xlimit, 'YLim', ylimit, 'ZLim', zlimit);
view(viewx, viewy);
set(gca, 'YDir', 'reverse');  % Inverse l'axe Y
hold on; grid on;
title('3D Trajectory and Orientation');
xlabel('X'); ylabel('Y'); zlabel('Z');


% Initialisation des éléments 3D
if has_ref
    curve_ref = animatedline(ax_3d, 'LineWidth', 1.5, 'LineStyle', ':', 'Color', [trajectory_color ref_alpha]);
    body_x_line_ref = plot3(ax_3d, nan, nan, nan, 'b', 'LineWidth', 1);
    body_y_line_ref = plot3(ax_3d, nan, nan, nan, 'g', 'LineWidth', 1);
    body_z_line_ref = plot3(ax_3d, nan, nan, nan, 'r', 'LineWidth', 1);
    frame_lines_ref = gobjects(4, 1);
    for i = 1:4
        frame_lines_ref(i) = plot3(ax_3d, nan, nan, nan, 'black', 'LineWidth', 2.5);
    end
end

curve = animatedline(ax_3d, 'LineWidth', 1.5, 'LineStyle', ':', 'Color', [trajectory_color 1]);
body_x_line = plot3(ax_3d, nan, nan, nan, 'b', 'LineWidth', 1);
body_y_line = plot3(ax_3d, nan, nan, nan, 'g', 'LineWidth', 1);
body_z_line = plot3(ax_3d, nan, nan, nan, 'r', 'LineWidth', 1);
frame_lines = gobjects(4, 1);
for i = 1:4
    frame_lines(i) = plot3(ax_3d, nan, nan, nan, 'black', 'LineWidth', 2.5);
end


% Figure : plot 2D : states
subplot_configs = [
    % [position, ylabel, y_col_index, has_hlines]
    struct('pos', 3,  'label', 'X_{cart} (m)',      'col', 1,  'color', [0 0.4 0.7],   'hlines', false);
    struct('pos', 8,  'label', 'Y_{cart} (m)',      'col', 3,  'color', [0.5 0.7 0.2], 'hlines', false);
    struct('pos', 13, 'label', 'Z_{cart} (m)',      'col', 5,  'color', [1 0 0],       'hlines', false);
    struct('pos', 4,  'label', 'Roll{body} (rad)', 'col', 7,  'color', [0 0 0],     'hlines', true);
    struct('pos', 9,  'label', 'Pitch_{body} (rad)', 'col', 9, 'color', [0 0 0],     'hlines', true);
    struct('pos', 14, 'label', 'Yaw_{body}  (rad)',  'col', 11, 'color', [0 0 0],     'hlines', false);
];

% Création des subplots 2D
anim_lines = cell(length(subplot_configs), 1);
subplot(3,5,4); 
fill([t(1) t(end) t(end) t(1)], [-pi/5 -pi/5 pi/5 pi/5], [1 0.66 0.5], 'EdgeColor', 'none'); hold on;
fill([t(1) t(end) t(end) t(1)], [-params.angle_max -params.angle_max params.angle_max params.angle_max], [0.9 1 0.9], 'EdgeColor', 'none');


subplot(3,5,9); 
fill([t(1) t(end) t(end) t(1)], [-pi/5 -pi/5 pi/5 pi/5], [1 0.66 0.5], 'EdgeColor', 'none'); hold on;
fill([t(1) t(end) t(end) t(1)], [-params.angle_max -params.angle_max params.angle_max params.angle_max], [0.9 1 0.9], 'EdgeColor', 'none');
ylim([-pi/5 pi/5]);

subplot(3,5,5); 
fill([t(1) t(end) t(end) t(1)], [mot_sat_down mot_sat_down mot_sat_up mot_sat_up], [0.9 1 0.9], 'EdgeColor', 'none');

for i = 1:length(subplot_configs)
    cfg = subplot_configs(i);
    ax = subplot(3, 5, cfg.pos);
    hold on; grid on;
    ylabel(cfg.label);
    
    % Calcul des limites
    y_data = y(:, cfg.col);
    if has_ref
        y_ref_data = y_ref(:, cfg.col);
        y_min = min(min(y_data), min(y_ref_data));
        y_max = max(max(y_data), max(y_ref_data));
    else
        y_min = min(y_data);
        y_max = max(y_data);
    end
    axis([0 t(end) y_min y_max]);
    
    % Création des animatedlines
    if has_ref
        anim_lines{i}.ref = animatedline(ax, 'Color', [0.8 0.8 0.8], 'LineWidth', 1.5);
    end
    anim_lines{i}.main = animatedline(ax, 'Color', cfg.color, 'LineWidth', 1.5);
end

subplot(3,5,4); ylim([-pi/5 pi/5]);
subplot(3,5,9); ylim([-pi/5 pi/5]);


% Figure : plot 2D : vitesses moteurs en RPM
ax_trust = subplot(3, 5, 5);
hold on; grid on;
ylabel('motor speed RPM');

if has_ref
    ani_mot_ref = animatedline(ax_trust, 'Color', [0.8 0.8 0.8], 'LineWidth', 1);
end
ani_mot = animatedline(ax_trust, 'Color', 'k', 'LineWidth', 1);


% Figure : plot 2D : Energie consomée
ax_Energy = subplot(3, 5, 10);
hold on; grid on;
ylabel('Energy consomée (Wh)');

if has_ref
    axis([0 t(end) min(min(Energy), min(Energy_ref)) max(max(Energy), max(Energy_ref))]);
    ani_Energy_ref = animatedline(ax_Energy, 'Color', [0.8 0.8 0.8], 'LineWidth', 1.5);
else
    axis([0 t(end) min(Energy) max(Energy)]);
end
ani_Energy = animatedline(ax_Energy, 'Color', 'k', 'LineWidth', 1.5);


% Initialisation textes
text(0, -0.5, 'Time :', 'FontSize', 12, 'HorizontalAlignment', 'left', 'Units', 'normalized');
TME = text(0.8, -0.5, '0.0', 'FontSize', 12, 'HorizontalAlignment', 'left', 'Units', 'normalized');
if has_ref
    text(0, -0.75, 'Energy ref (Wh):', 'FontSize', 12, 'HorizontalAlignment', 'left', 'Units', 'normalized');
    ENG_ref = text(0.8, -0.75, '0.00', 'FontSize', 12, 'HorizontalAlignment', 'left', 'Units', 'normalized');
end
text(0, -1, 'Energy (Wh):', 'FontSize', 12, 'HorizontalAlignment', 'left', 'Units', 'normalized');
ENG = text(0.8, -1, '0.00', 'FontSize', 12, 'HorizontalAlignment', 'left', 'Units', 'normalized');


% initialisation vidéo
if record_video
    writerObj = VideoWriter(video_filename, 'MPEG-4');
    writerObj.FrameRate = 30;
    open(writerObj);
    disp(['Recording video to: ', video_filename]);
end



%% === Boucle d'annimation ===

disp('Starting Animation...');
pause(0.1);
m_draw = eye(3);
arm_length = 0.5;
arm_offsets = [1 1; 1 -1; -1 1; -1 -1] * arm_length;

for k = 1:speed_ratio:length(t)
    
    % === MISE À JOUR 3D ===
    % Drone principal
    XYZ = y(k, [1, 3, 5])';
    Euler = y(k, [7, 9, 11]);
    T_BtoI = matrixB2I(Euler(1), Euler(2), Euler(3));
    
    % Points des axes du corps
    body_axes = XYZ + T_BtoI(:, 1:3);
    
    % Points des bras du drone
    frame_pts = zeros(3, 4);
    for i = 1:4
        frame_pts(:,i) = XYZ + T_BtoI * [arm_offsets(i,:)'; 0];
    end
    
    % Mise à jour des graphiques principaux
    addpoints(curve, XYZ(1), XYZ(2), XYZ(3));
    set(body_x_line, 'XData', [XYZ(1), body_axes(1,1)], 'YData', [XYZ(2), body_axes(2,1)], 'ZData', [XYZ(3), body_axes(3,1)]);
    set(body_y_line, 'XData', [XYZ(1), body_axes(1,2)], 'YData', [XYZ(2), body_axes(2,2)], 'ZData', [XYZ(3), body_axes(3,2)]);
    set(body_z_line, 'XData', [XYZ(1), body_axes(1,3)], 'YData', [XYZ(2), body_axes(2,3)], 'ZData', [XYZ(3), body_axes(3,3)]);
    for i = 1:4
        set(frame_lines(i), 'XData', [XYZ(1), frame_pts(1,i)], 'YData', [XYZ(2), frame_pts(2,i)], 'ZData', [XYZ(3), frame_pts(3,i)]);
    end
    
    % Drone de référence (si présent)
    if has_ref
        XYZ_ref = y_ref(k, [1, 3, 5])';
        Euler_ref = y_ref(k, [7, 9, 11]);
        T_BtoI_ref = matrixB2I(Euler_ref(1), Euler_ref(2), Euler_ref(3));
        
        body_axes_ref = XYZ_ref + T_BtoI_ref(:, 1:3);
        frame_pts_ref = zeros(3, 4);
        for i = 1:4
            frame_pts_ref(:,i) = XYZ_ref + T_BtoI_ref * [arm_offsets(i,:)'; 0];
        end
        
        addpoints(curve_ref, XYZ_ref(1), XYZ_ref(2), XYZ_ref(3));
        set(body_x_line_ref, 'XData', [XYZ_ref(1), body_axes_ref(1,1)], 'YData', [XYZ_ref(2), body_axes_ref(2,1)], 'ZData', [XYZ_ref(3), body_axes_ref(3,1)]);
        set(body_y_line_ref, 'XData', [XYZ_ref(1), body_axes_ref(1,2)], 'YData', [XYZ_ref(2), body_axes_ref(2,2)], 'ZData', [XYZ_ref(3), body_axes_ref(3,2)]);
        set(body_z_line_ref, 'XData', [XYZ_ref(1), body_axes_ref(1,3)], 'YData', [XYZ_ref(2), body_axes_ref(2,3)], 'ZData', [XYZ_ref(3), body_axes_ref(3,3)]);
        for i = 1:4
            set(frame_lines_ref(i), 'XData', [XYZ_ref(1), frame_pts_ref(1,i)], 'YData', [XYZ_ref(2), frame_pts_ref(2,i)], 'ZData', [XYZ_ref(3), frame_pts_ref(3,i)]);
        end
    end
    
    % === MISE À JOUR 2D ===
    for i = 1:length(subplot_configs)
        cfg = subplot_configs(i);
        if has_ref
            addpoints(anim_lines{i}.ref, t(k), y_ref(k, cfg.col));
        end
        addpoints(anim_lines{i}.main, t(k), y(k, cfg.col));
    end
    
    % Vitesses moteurs
    for m = 1:4
        if has_ref
            addpoints(ani_mot_ref, t(k), RPM * (sqrt(abs(u_ref(k, m)) + params.Wmot2_hover ) * sign(u_ref(k, m)+params.Wmot2_hover)  ));
        end
        addpoints(ani_mot, t(k), RPM * (sqrt(abs(u(k, m)) + params.Wmot2_hover) * sign(u(k, m)+params.Wmot2_hover) ) );
    end

    
    % Énergie
    if has_ref
        addpoints(ani_Energy_ref, t(k), Energy_ref(k));
    end
    addpoints(ani_Energy, t(k), Energy(k));
    
    % === MISE À JOUR TEXTES ===
    TME.String = sprintf('%.1f', t(k));
    if has_ref
        ENG_ref.String = sprintf('%.2f', Energy_ref(k));
    end
    ENG.String = sprintf('%.2f', Energy(k));
    
    drawnow;
    
    if record_video
        writeVideo(writerObj, getframe(fig));
    end
end

fprintf('\n');

% === FINALISATION ===
if record_video
    close(writerObj);
    disp('Video recording finished.');
end

if has_ref
    disp(['Final Estimated Energy ref (Wh): ', sprintf('%.2f', Energy_ref(end))]);
end
disp(['Final Estimated Energy (Wh): ', sprintf('%.2f', Energy(end))]);

end


%% ========== FONCTION DE CALCUL D'ÉNERGIE ==========

function Energy = calculate_energy(t, u, kt)
    % K : constante aérodynamique [kg·m²/rad^3]
    % u : vitesses angulaires [rad/s]
    n = length(t);
    Energy = zeros(n,1);
    
    for i = 2:n
        dt = t(i) - t(i-1);
        % Puissance: P = K * sum(omega_i^3)
        P = kt * sum(abs(u(i,:).^(3/2)));  % [W]
        Energy(i) = Energy(i-1) + P * dt * 3.6e-3;  % [Wh]
    end
end

%% ========== MATRICE DE ROTATION ==========
function m = matrixB2I(phi, theta, psi)
    % Matrice de rotation du repère Body vers Inertial
    % Optimisée en calculant directement la transposée
    cp = cos(phi);   sp = sin(phi);
    ct = cos(theta); st = sin(theta);
    cy = cos(psi);   sy = sin(psi);
    
    % Transposée de R_psi * R_theta * R_phi
    m = [ct*cy,               ct*sy,                -st;
         sp*st*cy - cp*sy,    sp*st*sy + cp*cy,     sp*ct;
         cp*st*cy + sp*sy,    cp*st*sy - sp*cy,     cp*ct];
end


function val = getopt(opts, field, default)
if isfield(opts, field)
    val = opts.(field);
else
    val = default;
end
end