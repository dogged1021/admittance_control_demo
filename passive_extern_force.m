% change the extern force actively

function PID_Demo
    %% Parameters
    % Initial parameters of controlled system.
    f = 0;
    m = 1;
    c = 0;
    k = 0;
    maxF = 50;
    maxM = 20; % Maximum gain allowed by slider.
    maxC = 20;
    maxK = 10;

    initialPos = 0;
    initialVel = 0;

    % Initial controller parameters.
    % p = 0; % Proportional gain (spring).
    % i = 0; % Integral gain.
    % d = 0; % Derivative gain (damper).
    % maxP = 200; % Maximum gain allowed by slider.
    % maxI = 200;
    % maxD = 100;

    tend = 10; % Window of simulation time being shown.

    % Target of controller.
    targetPos = 0;
    targetVel = 0; % Not changed (so far anyway).

    %%%% GUI setup:
    fig = figure(1);
    fig.Name = 'Admittance Example';
    fig.MenuBar = 'none';
    fig.Position(3:4) = [1000, 600];

    plotAx = subplot(4, 4, [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]); % For a time history plot of the result.
    plotAx.ButtonDownFcn = @setpointClickFcn;
    plotAx.Box = 'on';
    grid on;
    hold on;
    positionTrace = plot(0, 0, 'LineWidth', 2);
    currentPositionPt = plot(0, 0, '.r', 'MarkerSize', 30);
    % setpointLine = plot([0, tend], [targetPos, targetPos], ':k', 'LineWidth', 2); % Show the target of the controller.
    hold off;
    axis([0, tend, -20, 20]);

    % Bottom 1/4 is left open for slider bars (P, I, & D gains).
    fSlider = uicontrol(fig, 'Style', 'slider');
    fSlider.Value = 0.5;
    fSlider.Units = 'normalized';
    fSlider.Position(3) = 0.8; % Make it nearly the width of the figure.
    fSlider.Position(2) = 0.2; % Vertically space out sliders.
    fSlider.Callback = @fCallback;
    fSliderLabel = uicontrol(fig, 'Style', 'text', 'String', 'Force: 0', 'Units', 'normalized');
    fSliderLabel.Position = [0.85, 0.15, 0.1, 0.08];

    fClearButton = uicontrol(fig, 'Style', 'pushbutton', 'String', 'Clear Force', 'Units', 'normalized');
    fClearButton.Units = 'normalized';
    % uicontrol(fClearButton);
    fClearButton.Position(1) = 0.85;
    fClearButton.Position(2) = 0.25;
    fClearButton.Position
    fClearButton.Callback = @fClearCallback;
    % fClearLabel = uicontrol(fig, 'Style', 'text', 'String', 'Clear Force', 'Units', 'normalized');
    % fClearLabel.Position = [0.85, 0.25, 0.1, 0.02];

    mSlider = uicontrol(fig, 'Style', 'slider');
    mSlider.Units = 'normalized';
    mSlider.Position(3) = 0.8; % Make it nearly the width of the figure.
    mSlider.Position(2) = 0.14; % Vertically space out sliders.
    mSlider.Callback = @mCallback;
    mSliderLabel = uicontrol(fig, 'Style', 'text', 'String', 'Mass: 0', 'Units', 'normalized');
    mSliderLabel.Position = [0.85, 0.085, 0.1, 0.08];

    cSlider = uicontrol(fig, 'Style', 'slider');
    cSlider.Units = 'normalized';
    cSlider.Position(3) = 0.8; % Make it nearly the width of the figure.
    cSlider.Position(2) = 0.08;
    cSlider.Callback = @cCallback;
    cSliderLabel = uicontrol(fig, 'Style', 'text', 'String', 'Damp: 0', 'Units', 'normalized');
    cSliderLabel.Position = [0.85, 0.02, 0.105, 0.08];

    kSlider = uicontrol(fig, 'Style', 'slider');
    kSlider.Units = 'normalized';
    kSlider.Position(3) = 0.8; % Make it nearly the width of the figure.
    kSlider.Position(2) = 0.02;
    kSlider.Callback = @kCallback;
    kSliderLabel = uicontrol(fig, 'Style', 'text', 'String', 'Spring: 0', 'Units', 'normalized');
    kSliderLabel.Position = [0.85, -0.035, 0.1, 0.08];

    %% Select which dynamics should be simulated currently.
    % dynamicsSelector = uicontrol(fig, 'Style', 'popupmenu');
    % dynamicsSelector.Units = 'normalized';
    % dynamicsSelector.Position = [0.72 0.8 0.25 0.1];
    % dynamicsSelector.String = {'Constant force & Fixed point', 'Dynamic force & Fixed point', 'Constant force & Trajectory', 'Dynamic force & Tracjectory'};
    % dynamicsSelector.Callback = @dynamicsSelectorFcn;
    % currentDynamics = 1; % 1,2,3 corresponding to the selector values.
    % dynamicsLabel = uicontrol(fig, 'Style', 'text', 'String', 'Dynamics (w/o controller)', 'Units', 'normalized');
    % dynamicsLabel.Position = [0.72, 0.9, 0.25, 0.03];

    % % Select the sort of setpoint being tracked. Defaults to click the plot to
    % % select.
    % setpointSelector = uicontrol(fig, 'Style', 'popupmenu');
    % setpointSelector.Units = 'normalized';
    % setpointSelector.Position = [0.72 0.65 0.25 0.1];
    % setpointSelector.String = {'Click plot', 'Square wave', 'Triangle wave', 'Sawtooth wave', 'Sine wave'};
    % setpointSelector.Callback = @setpointSelectorCallback;
    % currentSetpointMode = 1; % 1,2,3,4,5 corresponding to the selector values.
    % setpointLabel = uicontrol(fig, 'Style', 'text', 'String', 'Controller setpoint', 'Units', 'normalized');
    % setpointLabel.Position = [0.72, 0.75, 0.25, 0.03];

    %% Realtime integration and plotting loop.
    currentState = [initialPos, initialVel, 0];
    bufferSize = 200;
    timeBuffer = zeros(bufferSize, 1);
    positionBuffer = ones(bufferSize, 1) * initialPos(1);
    tic;
    currTime = toc;
    prevTime = currTime;

    while (ishandle(fig)) % Dies when window is closed.
        currTime = toc;

        % % What signal are we tracking?
        % switch currentSetpointMode
        %     case 1
        %         % No need to change here.
        %     case 2
        %         targetPos = 5 * square(currTime);
        %     case 3
        %         targetPos = 5 * sawtooth(currTime, 0.5);
        %     case 4
        %         targetPos = 5 * sawtooth(currTime);
        %     case 5
        %         targetPos = 5 * sin(currTime);
        % end

        % RK4 integration, as fast as computer is able to run this loop.
        dt = currTime - prevTime;
        k_1 = dt * admittanceControlFcn(0, currentState);
        k_2 = dt * admittanceControlFcn(0, currentState + k_1 / 2);
        k_3 = dt * admittanceControlFcn(0, currentState + k_2 / 2);
        k_4 = dt * admittanceControlFcn(0, currentState + k_3);
        currentState = currentState + (k_1 + 2 * k_2 + 2 * k_3 + k_4) / 6;
        prevTime = currTime;

        % Only update the plot buffer if there's been enough change in time for
        % it to matter.
        if (currTime - timeBuffer(end) > tend / bufferSize)
            % Shift and update buffers.
            timeBuffer = circshift(timeBuffer, -1);
            timeBuffer(end) = currTime;
            positionBuffer = circshift(positionBuffer, -1);
            positionBuffer(end) = currentState(1);
            positionTrace.XData = timeBuffer;
            % Update plot data and re-draw.
            positionTrace.YData = positionBuffer;
            currentPositionPt.XData = timeBuffer(end);
            currentPositionPt.YData = positionBuffer(end);
            plotAx.XLim = [timeBuffer(1), timeBuffer(end)];
            % setpointLine.XData = [timeBuffer(1), timeBuffer(end)];
            % setpointLine.YData = [targetPos, targetPos];
            drawnow;
        end

    end

    %% Callback functions for user interaction.
    function qdot = admittanceControlFcn(~, q)
        position = q(1);
        velocity = q(2);
        acceleration = q(3);

        qdot = zeros(3, 1);
        qdot(1) = velocity;
        qdot(3) = 0;

        qdot(2) = (f - c * (velocity - initialVel) - k * (position - initialPos)) / m;

    end

    function fClearCallback(src, ~)
        f = 0;
        fSlider.Value = 0.5;
        fSliderLabel.String = strcat('Force: ', num2str(round(f, 1)));
    end

    function fCallback(src, ~)
        f = (src.Value - 0.5) * maxF;
        fSliderLabel.String = strcat('Force: ', num2str(round(f, 1)));
    end

    function mCallback(src, ~)
        m = src.Value * maxM + 1;
        mSliderLabel.String = strcat('Mass: ', num2str(round(m, 1))); % Update text label.
    end

    function cCallback(src, ~)
        c = src.Value * maxC;
        cSliderLabel.String = strcat('Damp: ', num2str(round(c, 1)));
    end

    function kCallback(src, ~)
        k = src.Value * maxK;
        kSliderLabel.String = strcat('Spring: ', num2str(round(k, 1)));
    end

end
