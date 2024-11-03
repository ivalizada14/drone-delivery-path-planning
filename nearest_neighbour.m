
numUAVs = 5; 

uavCapacity = 20;  

timeStep = 0.1;  

% UAV parameters
takeoffHeight = 10;  
landingSpeed = 0.8;  
uavSpeed = 1.5;      

% 3D plot setup
figure;
hold on;
axis([-50 50 -50 50 0 50]);  
grid on;
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('Multi-UAV 3D Simulation with Takeoff, Waypoint Navigation, and Landing (CVRP)');

view(3);  
camproj perspective; 

positions = zeros(numUAVs, 3);  

waypoints = [
    30, 30, 0, 5;   
    -10, 2, 0, 10; 
    -30, 20, 0, 3; 
    5, 10, 0, 7;    
    20, 40, 0, 8;  
    25, 30, 0, 6; 
    30, 4, 0, 4;   
    15, 8, 0, 2;   
    -25, -25, 0, 9; 
    -15, 10, 0, 1;  
    35, -35, 0, 11; 
    40, 20, 0, 12;  
    30, 30, 0, 5    
];

totalWeight = sum(waypoints(:, 4));

totalCapacity = numUAVs * uavCapacity;

if totalWeight > totalCapacity
    disp('Mission impossible: Total weight exceeds total UAV capacity.');
    return;  
end

[~, idx] = sort(waypoints(:,4), 'descend');
sortedWaypoints = waypoints(idx,:);

uavRoutes = cell(numUAVs,1);
uavPositions = positions;  
uavLoads = zeros(numUAVs, 1);  
uavPayloads = cell(numUAVs, 1);  
uavWaypointIndices = cell(numUAVs, 1);  

for i = 1:size(sortedWaypoints,1)
    wp = sortedWaypoints(i,1:3);  
    weight = sortedWaypoints(i, 4);  

    distances = zeros(numUAVs,1);
    for uav = 1:numUAVs
        if isempty(uavRoutes{uav})
            currentPos = uavPositions(uav,:);
        else
            currentPos = uavRoutes{uav}(end,:);
        end
        distances(uav) = norm(currentPos - wp);
    end

    validUAVs = find(uavLoads + weight <= uavCapacity);  
    if ~isempty(validUAVs)
        [~, minUAVIdx] = min(distances(validUAVs));  
        minUAV = validUAVs(minUAVIdx);  

        uavRoutes{minUAV} = [uavRoutes{minUAV}; wp]; 
        uavLoads(minUAV) = uavLoads(minUAV) + weight;  

        uavPayloads{minUAV} = [uavPayloads{minUAV}; weight];  
        uavWaypointIndices{minUAV} = [uavWaypointIndices{minUAV}; idx(i)]; 
    end
end

for i = 1:numUAVs
    uavRoutes{i} = [uavRoutes{i}; 0, 0, 10];  
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
            targetWaypoint = uavRoutes{i}(currentWaypoint(i), :);

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
            if currentWaypoint(i) < size(uavRoutes{i}, 1)
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

% Display the total distance traveled, payloads, and waypoint indices for each UAV
for i = 1:numUAVs
    disp(['UAV ', num2str(i), ' traveled a total distance of: ', num2str(distanceTraveled(i)), ' meters']);
    disp(['Payloads picked up by UAV ', num2str(i), ': ', num2str(uavPayloads{i}')]);
    disp(['Indices of waypoints traveled by UAV ', num2str(i), ': ', num2str(uavWaypointIndices{i}')]);
end

disp(['Total mission time: ', num2str(timeElapsed), ' seconds']);
hold off;
