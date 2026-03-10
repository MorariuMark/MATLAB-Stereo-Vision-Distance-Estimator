function projectSWEA()
    % Curatenie
    clc; clear; close all;
    delete(imaqfind); 
    
    fprintf('=== SISTEM MASURARE STEREO (v6 - Anti-Crash) ===\n');

    % --- MEMORIE CALIBRARE ---
    persistent D_calib Z_calib
    
    % Valori implicite (Start)
    if isempty(D_calib)
        D_calib = [251.7, 168.1, 127.1]; 
        Z_calib = [ 70.0, 100.0, 124.0];
    end

    %% 1. MENIU PRINCIPAL
    choice = menu('Meniu Principal', ...
                  '1. CALIBRARE NOUA (Live)', ...
                  '2. MASURARE (Live)', ...
                  '3. MASURARE (Poze din Folder)');
    
    if choice == 0, return; end
    
    % REZOLUTIE FIXA (Important pentru consistenta)
    rezolutie_dorita = '1280x720'; 

    %% RAMURA 1: CALIBRARE LIVE
    if choice == 1
        [D_new, Z_new] = RuleazaCalibrareLive(rezolutie_dorita);
        if ~isempty(D_new)
            D_calib = D_new;
            Z_calib = Z_new;
            fprintf('\n>>> CALIBRARE SALVATA! <<<\n');
            projectSWEA(); % Restart cu noile date
            return;
        end

    %% RAMURA 2: MASURARE LIVE
    elseif choice == 2
        [nume_st, nume_dr] = DeschideInterfata(rezolutie_dorita, 'MASURARE');
        if ~isempty(nume_st)
            ProceseazaSiMasoara(imread(nume_st), imread(nume_dr), D_calib, Z_calib);
        end
        
    %% RAMURA 3: POZE FOLDER
    elseif choice == 3
        [f1, p1] = uigetfile('*.jpg;*.png', 'Selecteaza STANGA'); if f1==0, return; end
        img_st = imread(fullfile(p1, f1));
        [f2, p2] = uigetfile('*.jpg;*.png', 'Selecteaza DREAPTA'); if f2==0, return; end
        img_dr = imread(fullfile(p2, f2));
        
        ProceseazaSiMasoara(img_st, img_dr, D_calib, Z_calib);
    end
end

%% --- MOTORUL DE PROCESARE (Cu Protectie Crash) ---
function ProceseazaSiMasoara(img_st, img_dr, D_calib, Z_calib)
    if size(img_st, 3)==3, g1=rgb2gray(img_st); else, g1=img_st; end
    if size(img_dr, 3)==3, g2=rgb2gray(img_dr); else, g2=img_dr; end
    [rows, cols] = size(g1); g2 = imresize(g2, [rows, cols]);

    % 1. Detectie Stanga (Blob)
    zona_st = g1; 
    zona_st(1:round(rows*0.15),:) = 255; 
    zona_st(round(rows*0.85):end,:) = 255;
    
    bin_st = bwareaopen(zona_st < 85, 1);
    props = regionprops(bin_st, 'BoundingBox', 'Centroid', 'Area');
    
    % PROTECTIE 1: Filtram obiectele uriase (umbre, pereti)
    idx_valid = find([props.Area] < 8000 & [props.Area] > 50); 
    if isempty(idx_valid)
        errordlg('Nu vad punctul! (Posibil prea mult zgomot sau obiecte mari).'), return; 
    end
    [~, idx_rel] = max([props(idx_valid).Area]);
    c_st = props(idx_valid(idx_rel)).Centroid; 
    rect = round(props(idx_valid(idx_rel)).BoundingBox);

    % 2. Detectie Dreapta (Template)
    pad=2; 
    x=max(1,rect(1)-pad); y=max(1,rect(2)-pad); 
    w=rect(3)+2*pad; h=rect(4)+2*pad;
    template = imcrop(g1, [x, y, w, h]);
    
    % PROTECTIE 2: Banda Dinamica (cat sa incapa template-ul)
    h_temp = size(template, 1);
    banda_min = ceil(h_temp/2) + 15; % Minim jumatate + marja
    banda = max(60, banda_min);      % Folosim minim 60px
    
    y1=max(1, round(c_st(2)-banda)); 
    y2=min(rows, round(c_st(2)+banda));
    
    % Ajustare daca suntem la margine si nu incape
    if (y2-y1) < h_temp
       y2 = min(rows, y1 + h_temp + 5);
    end

    strip = g2(y1:y2, :); 
    strip(:, round(c_st(1)):end) = 255; % Mask Right side
    
    % PROTECTIE 3: Verificare finala dimensiuni
    if size(strip,1) < size(template,1) || size(strip,2) < size(template,2)
        errordlg('Eroare tehnica: Obiectul detectat e prea mare pentru banda de cautare.'), return;
    end
    
    c_map = normxcorr2(template, strip);
    [~, imax] = max(abs(c_map(:)));
    [ypeak, xpeak] = ind2sub(size(c_map), imax(1));
    
    x_dr = xpeak - size(template, 2)/2;
    y_dr = ypeak - size(template, 1)/2 + y1 - 1;

    deplasament = abs(c_st(1) - x_dr);

    % 3. Calcul
    if isempty(D_calib)
        msg = 'NECALIBRAT'; Z_final=0;
    else
        Z_final = interp1(D_calib, Z_calib, deplasament, 'pchip', 'extrap');
        msg = sprintf('%.1f cm', Z_final);
    end

    % 4. Afisare
    figure('Name', 'REZULTAT', 'NumberTitle', 'off'); 
    imshow(img_dr); hold on;
    title({['Distanta: ' msg]; ['(Deplasament: ' num2str(deplasament, '%.1f') ' px)']}, 'FontSize', 16, 'BackgroundColor', 'w');
    plot(x_dr, y_dr, 'g+', 'MarkerSize', 20, 'LineWidth', 3);
    plot(c_st(1), c_st(2), 'ro', 'MarkerSize', 10);
    line([c_st(1), x_dr], [c_st(2), y_dr], 'Color', 'y', 'LineStyle', '--');
end

%% --- INTERFATA LIVE ---
function [f_st, f_dr] = DeschideInterfata(res, mod_lucru)
    f_st = ''; f_dr = '';
    try
        delete(imaqfind);
        cams = webcamlist;
        % Logica 3 camere
        if length(cams) >= 3, i1=1; i2=3; else, i1=1; i2=2; end
        
        cam1 = webcam(i1); cam1.Resolution = res;
        cam2 = webcam(i2); cam2.Resolution = res;
    catch
        errordlg(['Nu suporta rezolutia ' res '. Incearca 640x480 in cod.']); return;
    end
    
    hFig = figure('Name', ['MOD: ' mod_lucru], 'NumberTitle', 'off', 'MenuBar', 'none', 'Position', [100,100,1000,600]);
    ax1 = subplot(1,2,1); hIm1 = imshow(zeros(720,1280,3,'uint8')); title('STANGA');
    ax2 = subplot(1,2,2); hIm2 = imshow(zeros(720,1280,3,'uint8')); title('DREAPTA');
    
    btn = uicontrol('Style','pushbutton','String','CAPTURE','Position',[400 20 200 50], ...
              'BackgroundColor','g', 'FontWeight','bold', 'FontSize', 14, 'Callback', @actiune);
          
    run = true;
    while run && ishandle(hFig)
        try
            im1 = snapshot(cam1); im2 = snapshot(cam2);
            set(hIm1, 'CData', im1); set(hIm2, 'CData', im2);
            drawnow limitrate;
        catch, run=false; end
    end
    delete(cam1); delete(cam2);
    
    function actiune(~,~)
        folder='Temp_Caps'; if ~exist(folder,'dir'), mkdir(folder); end
        ts=datestr(now,'HH-MM-SS'); 
        f_st=fullfile(folder,['S_' ts '.jpg']); f_dr=fullfile(folder,['D_' ts '.jpg']);
        imwrite(im1, f_st); imwrite(im2, f_dr);
        run=false; delete(hFig);
    end
end

%% --- LOOP CALIBRARE LIVE (Debugged) ---
function [D_list, Z_list] = RuleazaCalibrareLive(res)
    D_list = []; Z_list = [];
    
    while true
        choice = menu('CALIBRARE LIVE', 'Adauga Punct (Fa Poza)', 'Termina si Salveaza', 'Anuleaza');
        if choice == 3, return; end
        if choice == 2
            [Z_list, idx] = sort(Z_list);
            D_list = D_list(idx);
            return; 
        end
        
        [n_st, n_dr] = DeschideInterfata(res, 'CALIBRARE - Adauga Punct');
        if isempty(n_st), continue; end
        
        ans = inputdlg('Distanta Reala (cm):');
        if isempty(ans), continue; end
        dist = str2double(ans{1});
        
        % ANALIZA RAPIDA
        i1=imread(n_st); i2=imread(n_dr);
        if size(i1,3)==3, g1=rgb2gray(i1); else, g1=i1; end
        if size(i2,3)==3, g2=rgb2gray(i2); else, g2=i2; end
        g2 = imresize(g2, size(g1));
        
        % 1. Blob Stanga
        bw = bwareaopen(g1<85, 1);
        props = regionprops(bw, 'Area', 'Centroid', 'BoundingBox');
        
        % Filtru marime (anti-crash)
        idx_valid = find([props.Area] < 8000 & [props.Area] > 50);
        if isempty(idx_valid), msgbox('Eroare detectie: Punct negasit sau prea mare.'); continue; end
        
        [~,mx] = max([props(idx_valid).Area]); 
        c1 = props(idx_valid(mx)).Centroid; 
        rect = round(props(idx_valid(mx)).BoundingBox);
        
        % 2. Template Dreapta (Dinamic)
        templ = imcrop(g1, rect);
        h_temp = size(templ, 1);
        banda = max(60, ceil(h_temp/2) + 15);
        
        y1=max(1, round(c1(2)-banda)); 
        y2=min(size(g1,1), round(c1(2)+banda));
        
        % Check size
        if (y2-y1) < h_temp, y2 = min(size(g1,1), y1+h_temp+5); end

        strip = g2(y1:y2, :); 
        strip(:, round(c1(1)):end)=255;
        
        if size(strip,1) < size(templ,1) || size(strip,2) < size(templ,2)
             msgbox('Eroare: Obiectul nu incape in banda de cautare.'); continue;
        end
        
        c_map = normxcorr2(templ, strip);
        [~, imax] = max(abs(c_map(:)));
        [~, xpeak] = ind2sub(size(c_map), imax);
        x2 = xpeak - size(templ,2)/2;
        
        d = abs(c1(1)-x2);
        
        D_list = [D_list, d];
        Z_list = [Z_list, dist];
        
        msgbox(sprintf('Adaugat: %.1f cm -> %.1f px', dist, d));
    end
end