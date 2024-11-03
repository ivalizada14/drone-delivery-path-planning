% Number of UAVs
numUAVs = 5;  

timeStep = 0.1;  % Time step for the simulation

% UAV parameters
takeoffHeight = 10;  % Height to reach after takeoff (flight altitude)
landingSpeed = 0.8;  % Speed during landing and takeoff
uavSpeed = 1.5;      % UAV speed in meters per second

figure;
hold on;
axis([-50 50 -50 50 0 50]);  
grid on;
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('Multi-UAV 3D Simulation with Takeoff, Waypoint Navigation, and Landing');

view(3);  
camproj perspective;  

% Initial positions for each UAV [X, Y, Z]
positions = [
    0, 0, 0;    % UAV 1 starts at (0, 0, 0)
    0, 0, 0;    % UAV 2 starts at (-10, 10, 0)
    0, 0, 0;    % UAV 3 starts at (20, -15, 0)
    0, 0, 0;    % UAV 4 starts at (-30, -30, 0)
    0, 0, 0     % UAV 5 starts at (25, 25, 0)
];

waypoints = {
    [30, 30, 0; -10, 20, 0];  % Waypoints for UAV 1
    [-30, 20, 0; 5, 10, 0];    % Waypoints for UAV 2
    [20, -40, 0; 25, 30, 0; 30, 25, 0; 15, 10, 0];  % Waypoints for UAV 3
    [-25, -25, 0; -15, 10, 0];% Waypoints for UAV 4
    [35, -35, 0; 40, 20, 0; 30, 30, 0]   % Waypoints for UAV 5
};

% Append the starting positions as the final waypoint for each UAV
for i = 1:numUAVs
    waypoints{i} = [waypoints{i}; 0,0,10];  % Return to starting point at the end
end

% UAV status: 1 = takeoff, 2 = flying to waypoint, 3 = landing, 4 = landed
status = ones(numUAVs, 1);  

currentWaypoint = ones(numUAVs, 1); 

distanceTraveled = zeros(numUAVs, 1);

uavPlots = gobjects(numUAVs, 1);
for i = 1:numUAVs
    uavPlots(i) = plot3(positions(i, 1), positions(i, 2), positions(i, 3), '.', 'MarkerSize', 10, 'DisplayName', ['UAV ' num2str(i)]);
end

timeElapsed = 0;

while any(status ~= 0) 
    for i = 1:numUAVs
        previousPosition = positions(i, :);  
        
        if status(i) == 1  
            if positions(i, 3) < takeoffHeight
                positions(i, 3) = positions(i, 3) + landingSpeed; 
            else
                status(i) = 2;  
            end
            
        elseif status(i) == 2  
            targetWaypoint = waypoints{i}(currentWaypoint(i), :);
            

            direction = targetWaypoint - positions(i, :);  
            distance = norm(direction);  
            
            if distance > 0.1
                positions(i, :) = positions(i, :) + uavSpeed * direction / distance * timeStep;
            else
                status(i) = 3;  
            end
            
        elseif status(i) == 3  
            if positions(i, 3) > 0
                positions(i, 3) = positions(i, 3) - landingSpeed;  
            else
                positions(i, 3) = 0;  
                status(i) = 4;  
            end
            
        elseif status(i) == 4  
        
            if currentWaypoint(i) < size(waypoints{i}, 1)
                currentWaypoint(i) = currentWaypoint(i) + 1;  
                status(i) = 1;  
            else
                status(i) = 0;
            end
        end
        
        distanceTraveled(i) = distanceTraveled(i) + norm(positions(i, :) - previousPosition);
        
        set(uavPlots(i), 'XData', positions(i, 1), 'YData', positions(i, 2), 'ZData', positions(i, 3));
    end

    pause(0.01);
    
    timeElapsed = timeElapsed + timeStep;
end

% Display the total distance traveled by each UAV
for i = 1:numUAVs
    disp(['UAV ', num2str(i), ' traveled a total distance of: ', num2str(distanceTraveled(i)), ' meters']);
end

disp(['Total mission time: ', num2str(timeElapsed), ' seconds']);
hold off;
