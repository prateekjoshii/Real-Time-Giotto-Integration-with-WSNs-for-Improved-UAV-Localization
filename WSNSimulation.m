% Localization refers to the process of determining the physical location or position 
% of nodes within the network. The algorithms for localization and tracking of a mobile node (a drone in our case) 
% can be of the types: distance based, angle based, received signal strength (RSS) or hybrid.

% We consider only the distance-based localization of a single target. 


%% Setting Parameters
N = 8; % number of anchors (considering all vertices of the cube)
M = 1; % number of mobile nodes (set to 1 for the drone)

% Distance-dependent error (standard deviation of the noise normalized to distance)
distMeasurementErrRatio = 0.05; % it means that the accuracy of distance measurement is 95 %
% for instance, the inaccuracy of a 1m measured distance is around 0.05 meter.

networkSize = 100; % we consider a cubic region of 100x100x100 that the mobile can wander

% Anchor locations
anchorLoc = [0,             0,             0; 
             networkSize,   0,             0;
             0,             networkSize,   0;
             networkSize,   networkSize,   0;
             0,             0,             networkSize;
             networkSize,   0,             networkSize;
             0,             networkSize,   networkSize;
             networkSize,   networkSize,   networkSize];

% Mobile true location
mobileLoc = networkSize * rand(M,3); % This is where drone's intial position are received through broadcast. 
                                     % For simulation sake, we are generating random (x, y, z) co-ordinates.

% Preallocation of distance matrix
distance = zeros(N,M);

% Computing the Euclidean distances (vectorized)
for m = 1 : M
    distance(:,m) = sqrt(sum((anchorLoc - mobileLoc(m,:)).^2 , 2));
end

% Plot the scenario
f1 = figure(1);
clf
plot3(anchorLoc(:,1), anchorLoc(:,2), anchorLoc(:,3), 'ko', 'MarkerSize', 8, 'lineWidth', 2, 'MarkerFaceColor', 'k');
hold on
plot3(mobileLoc(:,1), mobileLoc(:,2), mobileLoc(:,3), 'b+', 'MarkerSize', 8, 'lineWidth', 2);

% Noisy measurements
distanceNoisy = distance + distance .* distMeasurementErrRatio .* randn(N,M);

% Using Gauss-Newton to solve the problem
numOfIteration = 25;

% Initial guess (random location)
mobileLocEst = networkSize * rand(M,3);

% Iteration
for m = 1 : M
    for i = 1 : numOfIteration
        % Computing the estimated distances
        distanceEst = sqrt(sum((anchorLoc - mobileLocEst(m,:)).^2 , 2));

        
        % Computing the derivatives
        distanceDrv = (mobileLocEst(m,:) - anchorLoc) ./ distanceEst;
        
        % Delta
        delta = - (distanceDrv.' * distanceDrv) \ (distanceDrv.' * (distanceEst - distanceNoisy(:,m)));
        
        % Updating the estimation
        mobileLocEst(m,:) = mobileLocEst(m,:) + delta.'; % This new estimated positon of drone is sent back via broadcast
                                                         % for position refinement.
    end
end

% Plotting estimated mobile locations
plot3(mobileLocEst(:,1), mobileLocEst(:,2), mobileLocEst(:,3), 'ro', 'MarkerSize', 8, 'lineWidth', 2);
legend('Anchor Locations','Drone Initial Location','Estimated Location', 'Location','Best')

% Compute the Root Mean Squred Error
Err = mean(sqrt(sum((mobileLocEst-mobileLoc).^2)));
title(['Mean Estimation error is ', num2str(Err), ' meter'])

% Setting axis limits
axis([-0.1 1.1 -0.1 1.1] * networkSize)
grid on
hold off
