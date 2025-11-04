function motion_detection()

    close all; clc;

    hFig = figure('Name', 'Enhanced Motion Detection & Tracking System', ...
        'NumberTitle', 'off', 'Position', [100, 100, 1200, 700], ...
        'CloseRequestFcn', @closeGUI);

    uicontrol('Style', 'text', 'String', 'Video Source:', ...
        'Position', [20, 650, 100, 25], 'HorizontalAlignment', 'left');
    uicontrol('Style', 'pushbutton', 'String', 'Select Source', ...
        'Position', [130, 645, 120, 30], 'Callback', @selectSource);
    uicontrol('Style', 'pushbutton', 'String', 'Start Processing', ...
        'Position', [260, 645, 120, 30], 'Callback', @startProcessing, ...
        'Tag', 'startBtn', 'Enable', 'off');
    uicontrol('Style', 'pushbutton', 'String', 'Stop', ...
        'Position', [390, 645, 80, 30], 'Callback', @stopProcessing, ...
        'Tag', 'stopBtn', 'Enable', 'off');

    uicontrol('Style', 'text', 'String', 'Minimum Blob Area:', ...
        'Position', [20, 590, 150, 20], 'HorizontalAlignment', 'left');
    blobAreaSlider = uicontrol('Style', 'slider', 'Position', [20, 570, 200, 20], ...
        'Min', 100, 'Max', 1000, 'Value', 300, 'Callback', @updateBlobArea);
    blobAreaText = uicontrol('Style', 'text', 'String', '300', ...
        'Position', [230, 570, 50, 20]);

    uicontrol('Style', 'text', 'String', 'Detection Sensitivity:', ...
        'Position', [20, 530, 150, 20], 'HorizontalAlignment', 'left');
    sensitivitySlider = uicontrol('Style', 'slider', 'Position', [20, 510, 200, 20], ...
        'Min', 5, 'Max', 50, 'Value', 20, 'Callback', @updateSensitivity);
    sensitivityText = uicontrol('Style', 'text', 'String', '20', ...
        'Position', [230, 510, 50, 20]);

    axes1 = axes('Parent', hFig, 'Position', [0.05, 0.05, 0.42, 0.55]);
    title('Original Video');
    axes2 = axes('Parent', hFig, 'Position', [0.53, 0.05, 0.42, 0.55]);
    title('Processed Video');

    videoFile = '';
    isProcessing = false;
    obj = [];
    useCamera = false;
    minBlobArea = 300;
    sensitivity = 20;
    frameRate = 30;
    detector = [];
    blobAnalyser = [];
    tracker = [];
    prevFrame = [];

    function selectSource(~, ~)
        choice = questdlg('Select input source:', 'Video Source', ...
            'Video File', 'Camera Feed', 'Cancel', 'Video File');
        switch choice
            case 'Video File'
                [file, path] = uigetfile({'*.mp4;*.avi;*.mov'});
                if isequal(file, 0), return; end
                videoFile = fullfile(path, file);
                useCamera = false;
                msgbox(['Selected video: ' videoFile], 'Source Selected');
                set(findobj('Tag', 'startBtn'), 'Enable', 'on');
            case 'Camera Feed'
                if isempty(webcamlist)
                    errordlg('No camera detected.', 'Camera Error'); return;
                end
                useCamera = true;
                msgbox('Live camera feed selected.', 'Source Selected');
                set(findobj('Tag', 'startBtn'), 'Enable', 'on');
        end
    end

    function updateBlobArea(~, ~)
        minBlobArea = round(get(blobAreaSlider, 'Value'));
        set(blobAreaText, 'String', num2str(minBlobArea));
        if ~isempty(blobAnalyser)
            blobAnalyser.MinimumBlobArea = minBlobArea;
        end
    end

    function updateSensitivity(~, ~)
        sensitivity = round(get(sensitivitySlider, 'Value'));
        set(sensitivityText, 'String', num2str(sensitivity));
        
        if ~isempty(detector)
            release(detector);
            detector.MinimumBackgroundRatio = sensitivity / 1000;
        end
    end

    function startProcessing(~, ~)
        try
            if useCamera
                obj.reader = webcam(1);
                frameRate = 30;
            else
                obj.reader = VideoReader(videoFile);
                frameRate = obj.reader.FrameRate;
            end
        catch ME
            errordlg(['Error initializing source: ' ME.message]);
            return;
        end

        detector = vision.ForegroundDetector( ...
            'NumTrainingFrames', 15, ...
            'InitialVariance', 25^2, ...
            'LearningRate', 0.005, ...
            'MinimumBackgroundRatio', sensitivity / 1000, ...
            'NumGaussians', 5, ...
            'AdaptLearningRate', true);

        blobAnalyser = vision.BlobAnalysis( ...
            'MinimumBlobArea', minBlobArea, ...
            'MaximumBlobArea', 50000, ...
            'ExcludeBorderBlobs', true, ...
            'BoundingBoxOutputPort', true, ...
            'CentroidOutputPort', true, ...
            'AreaOutputPort', true);

        tracker.tracks = struct('bbox', {}, 'centroid', {}, 'age', {}, 'totalVisibleCount', {});
        tracker.nextId = 1;
        prevFrame = [];

        isProcessing = true;
        set(findobj('Tag', 'startBtn'), 'Enable', 'off');
        set(findobj('Tag', 'stopBtn'), 'Enable', 'on');
        processVideo();
    end

    function stopProcessing(~, ~)
        isProcessing = false;
        set(findobj('Tag', 'startBtn'), 'Enable', 'on');
        set(findobj('Tag', 'stopBtn'), 'Enable', 'off');
        if useCamera && isa(obj.reader, 'webcam')
            clear obj.reader;
        end
    end

    function closeGUI(~, ~)
        isProcessing = false;
        if useCamera && isa(obj.reader, 'webcam')
            clear obj.reader;
        end
        delete(hFig);
    end

    function processVideo()
        if useCamera
            frame = snapshot(obj.reader);
        else
            frame = readFrame(obj.reader);
        end

        imHandle1 = imshow(frame, 'Parent', axes1);
        imHandle2 = imshow(frame, 'Parent', axes2);

        while isProcessing
            if useCamera
                frame = snapshot(obj.reader);
            else
                if ~hasFrame(obj.reader)
                    stopProcessing(); break;
                end
                frame = readFrame(obj.reader);
            end

            grayFrame = rgb2gray(frame);

            grayFrameSmooth = imgaussfilt(grayFrame, 1.5);
            
            foregroundMask = detector(grayFrameSmooth);

            maskClean = bwareaopen(foregroundMask, 50);
            
            se1 = strel('disk', 2);
            maskClean = imopen(maskClean, se1);
            
            se2 = strel('disk', 8);
            maskClean = imclose(maskClean, se2);
            
            maskClean = imfill(maskClean, 'holes');
           
            se3 = strel('disk', 1);
            maskClean = imerode(maskClean, se3);
            maskClean = imdilate(maskClean, se3);

           
            if ~isempty(prevFrame)
                frameDiff = abs(double(grayFrame) - double(prevFrame));
                motionMask = frameDiff > 15;
                motionMask = imgaussfilt(double(motionMask), 2) > 0.3;
                
                
                maskClean = maskClean & motionMask;
            end
            prevFrame = grayFrame;

           
            [areas, ~, bboxes] = blobAnalyser(maskClean);

           
            activeBboxes = [];
            if ~isempty(bboxes)
              
                validIdx = areas > minBlobArea;
                bboxes = bboxes(validIdx, :);
                
                if ~isempty(bboxes)
                   
                    mergedBboxes = mergeOverlappingBoxes(bboxes, 50); 
                    
                    
                    for i = 1:size(mergedBboxes, 1)
                        bbox = mergedBboxes(i, :);
                 
                        x1 = max(1, round(bbox(1)));
                        y1 = max(1, round(bbox(2)));
                        x2 = min(size(maskClean, 2), round(bbox(1) + bbox(3)));
                        y2 = min(size(maskClean, 1), round(bbox(2) + bbox(4)));
                        
                        region = maskClean(y1:y2, x1:x2);
                        
                        
                        [rows, cols] = find(region);
                        
                        if ~isempty(rows)
                           
                            tightBox = [x1 + min(cols) - 1, ...
                                       y1 + min(rows) - 1, ...
                                       max(cols) - min(cols) + 1, ...
                                       max(rows) - min(rows) + 1];
                            
                            activeBboxes = [activeBboxes; tightBox];
                        end
                    end
                end
            end

        
            if ~isempty(activeBboxes)
                overlay = insertShape(frame, 'Rectangle', activeBboxes, ...
                    'Color', 'green', 'LineWidth', 6);
            else
                overlay = frame;
            end

            set(imHandle1, 'CData', frame);
            set(imHandle2, 'CData', overlay);
            title(axes2, sprintf('Motion Detected: %d objects | Sens=%.1f | Area>%d', ...
                size(activeBboxes, 1), sensitivity, minBlobArea));

            drawnow limitrate;
            pause(1 / frameRate);
        end
    end

   
    function mergedBoxes = mergeOverlappingBoxes(bboxes, proximityThreshold)
      
        
        if isempty(bboxes)
            mergedBoxes = [];
            return;
        end
        
        n = size(bboxes, 1);
        merged = false(n, 1);
        mergedBoxes = [];
        
        for i = 1:n
            if merged(i)
                continue;
            end
            
    
            currentBox = bboxes(i, :);
            merged(i) = true;
            
        
            changed = true;
            while changed
                changed = false;
                for j = 1:n
                    if merged(j)
                        continue;
                    end
                    
              
                    if boxesOverlapOrClose(currentBox, bboxes(j, :), proximityThreshold)
                        
                        x1 = min(currentBox(1), bboxes(j, 1));
                        y1 = min(currentBox(2), bboxes(j, 2));
                        x2 = max(currentBox(1) + currentBox(3), bboxes(j, 1) + bboxes(j, 3));
                        y2 = max(currentBox(2) + currentBox(4), bboxes(j, 2) + bboxes(j, 4));
                        
                        currentBox = [x1, y1, x2 - x1, y2 - y1];
                        merged(j) = true;
                        changed = true;
                    end
                end
            end
            
            mergedBoxes = [mergedBoxes; currentBox];
        end
    end

    function overlap = boxesOverlapOrClose(box1, box2, threshold)
  
        expandedBox1 = [box1(1) - threshold, box1(2) - threshold, ...
                       box1(3) + 2*threshold, box1(4) + 2*threshold];
        expandedBox2 = [box2(1) - threshold, box2(2) - threshold, ...
                       box2(3) + 2*threshold, box2(4) + 2*threshold];
        
    
        x1_min = expandedBox1(1);
        y1_min = expandedBox1(2);
        x1_max = expandedBox1(1) + expandedBox1(3);
        y1_max = expandedBox1(2) + expandedBox1(4);
        
        x2_min = expandedBox2(1);
        y2_min = expandedBox2(2);
        x2_max = expandedBox2(1) + expandedBox2(3);
        y2_max = expandedBox2(2) + expandedBox2(4);
        
        overlap = ~(x1_max < x2_min || x2_max < x1_min || ...
                    y1_max < y2_min || y2_max < y1_min);
    end
end
