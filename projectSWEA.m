function projectSWEA()
    % Initialize workspace and reset video devices
    clc; clear; close all;
    delete(imaqfind); 
    
    % Persistent variables to retain calibration data during execution
    persistent D_calib Z_calib
    
    % Initialize default empirical calibration curve
    if isempty(D_calib)
        D_calib = [251.7, 168.1, 127.1]; % Disparity in pixels
        Z_calib = [ 70.0, 100.0, 124.0]; % Physical distance in cm
    end

    %% Main Menu
    choice = menu('Stereo Vision Measurement', ...
                  '1. New Live Calibration', ...
                  '2. Live Measurement', ...
                  '3. Measure from Folder');
    
    if choice == 0, return; end
    
    % Hardware resolution lock for disparity consistency
    target_res = '1280x720'; 

    %% Action Handler
    if choice == 1
        [D_new, Z_new] = RunLiveCalibration(target_res);
        if ~isempty(D_new)
            D_calib = D_new;
            Z_calib = Z_new;
            fprintf('Calibration updated successfully.\n');
            projectSWEA(); 
            return;
        end

    elseif choice == 2
        [file_L, file_R] = OpenLiveInterface(target_res, 'Live Measurement');
        if ~isempty(file_L)
            img_L = imread(file_L);
            img_R = imread(file_R);
            ProcessAndMeasure(img_L, img_R, D_calib, Z_calib);
        end
        
    elseif choice == 3
        [f1, p1] = uigetfile({'*.jpg;*.png'}, 'Select LEFT Image'); 
        if f1==0, return; end
        img_L = imread(fullfile(p1, f1));
        
        [f2, p2] = uigetfile({'*.jpg;*.png'}, 'Select RIGHT Image'); 
        if f2==0, return; end
        img_R = imread(fullfile(p2, f2));
        
        ProcessAndMeasure(img_L, img_R, D_calib, Z_calib);
    end
end

%% ------------------------------------------------------------------------
% Core Stereo Processing Pipeline
% -------------------------------------------------------------------------
function ProcessAndMeasure(img_L, img_R, D_calib, Z_calib)
    % Image pre-processing
    if size(img_L, 3) == 3, g1 = rgb2gray(img_L); else, g1 = img_L; end
    if size(img_R, 3) == 3, g2 = rgb2gray(img_R); else, g2 = img_R; end
    
    [rows, cols] = size(g1); 
    g2 = imresize(g2, [rows, cols]);

    % 1. Reference Extraction (Left Image)
    roi_L = g1; 
    roi_L(1:round(rows*0.15), :) = 255; % Crop top background
    roi_L(round(rows*0.85):end, :) = 255; % Crop bottom background
    
    bin_L = bwareaopen(roi_L < 85, 1);
    props = regionprops(bin_L, 'BoundingBox', 'Centroid', 'Area');
    
    % Valid blob filtering (noise and artifact rejection)
    idx_valid = find([props.Area] < 8000 & [props.Area] > 50); 
    if isempty(idx_valid)
        errordlg('Detection error: Reference object not found in the Left image.'); 
        return; 
    end
    
    [~, idx_rel] = max([props(idx_valid).Area]);
    centroid_L = props(idx_valid(idx_rel)).Centroid;      
    rect_L = round(props(idx_valid(idx_rel)).BoundingBox); 

    % 2. Target Matching (Right Image) via Template Matching
    pad = 2; 
    x = max(1, rect_L(1)-pad); 
    y = max(1, rect_L(2)-pad); 
    w = rect_L(3) + 2*pad; 
    h = rect_L(4) + 2*pad;
    template = imcrop(g1, [x, y, w, h]);
    
    % Dynamic ROI calculation for search optimization
    h_temp = size(template, 1);
    search_band = max(60, ceil(h_temp/2) + 15); 
    
    y1 = max(1, round(centroid_L(2) - search_band)); 
    y2 = min(rows, round(centroid_L(2) + search_band));
    if (y2 - y1) < h_temp, y2 = min(rows, y1 + h_temp + 5); end

    strip = g2(y1:y2, :); 
    strip(:, round(centroid_L(1)):end) = 255; % Epipolar constraint (x_R < x_L)
    
    if size(strip,1) < size(template,1) || size(strip,2) < size(template,2)
        errordlg('ROI limits error: Template exceeds search band boundaries.'); 
        return;
    end
    
    % Normalized cross-correlation
    c_map = normxcorr2(template, strip);
    [~, imax] = max(abs(c_map(:)));
    [ypeak, xpeak] = ind2sub(size(c_map), imax(1));
    
    x_R = xpeak - size(template, 2)/2;
    y_R = ypeak - size(template, 1)/2 + y1 - 1; 

    % 3. Depth Calculation
    disparity = abs(centroid_L(1) - x_R);

    if isempty(D_calib)
        msg = 'UNINITIALIZED'; Z_final = 0;
    else
        Z_final = interp1(D_calib, Z_calib, disparity, 'pchip', 'extrap');
        msg = sprintf('%.1f cm', Z_final);
    end

    % 4. Data Visualization
    figure('Name', 'Stereo Analysis Result', 'NumberTitle', 'off'); 
    imshow(img_R); hold on;
    
    title({sprintf('Calculated Depth: %s', msg); ...
           sprintf('(Disparity: %.1f px)', disparity)}, ...
           'FontSize', 14, 'BackgroundColor', 'w');
    
    plot(x_R, y_R, 'g+', 'MarkerSize', 15, 'LineWidth', 2);
    plot(centroid_L(1), centroid_L(2), 'ro', 'MarkerSize', 8);
    line([centroid_L(1), x_R], [centroid_L(2), y_R], 'Color', 'y', 'LineStyle', '--');
end

%% ------------------------------------------------------------------------
% Live Video Acquisition GUI
% -------------------------------------------------------------------------
function [file_L, file_R] = OpenLiveInterface(res, window_title)
    file_L = ''; file_R = '';
    
    try
        delete(imaqfind); 
        cams = webcamlist;
        
        % Camera indexing logic (prefer external USB cameras over integrated)
        if length(cams) >= 3, i1 = 1; i2 = 3; else, i1 = 1; i2 = 2; end
        
        cam1 = webcam(i1); cam1.Resolution = res;
        cam2 = webcam(i2); cam2.Resolution = res;
    catch
        errordlg(sprintf('Camera initialization failed. Resolution %s unsupported.', res)); 
        return;
    end
    
    hFig = figure('Name', window_title, 'NumberTitle', 'off', ...
                  'MenuBar', 'none', 'Position', [100, 100, 1000, 600]);
              
    ax1 = subplot(1, 2, 1); hIm1 = imshow(zeros(720, 1280, 3, 'uint8')); title('LEFT CAMERA');
    ax2 = subplot(1, 2, 2); hIm2 = imshow(zeros(720, 1280, 3, 'uint8')); title('RIGHT CAMERA');
    
    uicontrol('Style', 'pushbutton', 'String', 'CAPTURE & ANALYZE',...
              'Position', [400 20 200 50], ...
              'BackgroundColor', 'g', 'FontWeight', 'bold', 'FontSize', 12, ...
              'Callback', @capture_callback);
          
    run_video = true;
    while run_video && ishandle(hFig)
        try
            im1 = snapshot(cam1); 
            im2 = snapshot(cam2);
            set(hIm1, 'CData', im1); 
            set(hIm2, 'CData', im2);
            drawnow limitrate;
        catch
            run_video = false; 
        end
    end
    
    delete(cam1); delete(cam2);
    
    function capture_callback(~, ~)
        folder = 'Caps'; 
        if ~exist(folder, 'dir'), mkdir(folder); end
        
        ts = datestr(now, 'HH-MM-SS'); 
        file_L = fullfile(folder, ['L_' ts '.jpg']); 
        file_R = fullfile(folder, ['R_' ts '.jpg']);
        
        imwrite(im1, file_L); 
        imwrite(im2, file_R);
        
        run_video = false; 
        delete(hFig);
    end
end

%% ------------------------------------------------------------------------
% Live Calibration Routine
% -------------------------------------------------------------------------
function [D_list, Z_list] = RunLiveCalibration(res)
    D_list = []; Z_list = [];
    
    while true
        choice = menu('Calibration Mode', '1. Add Calibration Point', '2. Save Data', '3. Cancel');
        
        if choice == 3, return; end 
        if choice == 2
            [Z_list, idx] = sort(Z_list);
            D_list = D_list(idx);
            return; 
        end
        
        [file_L, file_R] = OpenLiveInterface(res, 'Calibration Point Acquisition');
        if isempty(file_L), continue; end
        
        ans_dlg = inputdlg('Enter physical distance (cm):', 'Input', [1 35]);
        if isempty(ans_dlg), continue; end
        dist = str2double(ans_dlg{1});
        
        % Background processing for calibration
        i1 = imread(file_L); i2 = imread(file_R);
        if size(i1, 3) == 3, g1 = rgb2gray(i1); else, g1 = i1; end
        if size(i2, 3) == 3, g2 = rgb2gray(i2); else, g2 = i2; end
        g2 = imresize(g2, size(g1));
        
        bw = bwareaopen(g1 < 85, 1);
        props = regionprops(bw, 'Area', 'Centroid', 'BoundingBox');
        
        idx_valid = find([props.Area] < 8000 & [props.Area] > 50);
        if isempty(idx_valid)
            msgbox('Extraction Error: Invalid object.'); 
            continue; 
        end
        
        [~, mx] = max([props(idx_valid).Area]); 
        centroid_L = props(idx_valid(mx)).Centroid; 
        rect_L = round(props(idx_valid(mx)).BoundingBox);
        
        templ = imcrop(g1, rect_L);
        h_temp = size(templ, 1);
        search_band = max(60, ceil(h_temp/2) + 15);
        
        y1 = max(1, round(centroid_L(2) - search_band)); 
        y2 = min(size(g1, 1), round(centroid_L(2) + search_band));
        if (y2 - y1) < h_temp, y2 = min(size(g1, 1), y1 + h_temp + 5); end
        
        strip = g2(y1:y2, :); 
        strip(:, round(centroid_L(1)):end) = 255;
        
        if size(strip, 1) < size(templ, 1) || size(strip, 2) < size(templ, 2)
             msgbox('ROI limitation error during calibration.'); 
             continue;
        end
        
        c_map = normxcorr2(templ, strip);
        [~, imax] = max(abs(c_map(:)));
        [~, xpeak] = ind2sub(size(c_map), imax);
        x_R = xpeak - size(templ, 2)/2;
        
        disparity = abs(centroid_L(1) - x_R);
        
        D_list(end+1) = disparity;
        Z_list(end+1) = dist;
        
        msgbox(sprintf('Point Recorded:\nZ = %.1f cm\nd = %.1f px', dist, disparity));
    end
end