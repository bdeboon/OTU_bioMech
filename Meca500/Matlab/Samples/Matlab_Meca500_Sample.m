t = tcpip('192.168.0.100', 10000);          % Meca Communication
t.Status                                    % Status of communication
fopen(t)                                    % Open the link
t.Status                                    % Status of the link 
fscanf(t)

fprintf(t, '%s\0', 'ActivateRobot')         % Activate the robot
fscanf(t)                                   % Status of activation

fprintf(t, '%s\0', 'Home')                  % Home position

fprintf(t, '%s\0', 'ResetError')            % Reset Error if exist
fscanf(t)

get(t,{'Name','RemoteHost','RemotePort','Type'})
tcpip(t)

fprintf(t, '%s\0', 'SetJointVel(130)')      % default is 45 and the range from 1 to 135 Degree/sec

fprintf(t, '%s\0', 'MoveJoints(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)')        % Move joints to Zero position

fprintf(t, '%s\0', 'MoveJoints(30.0, 30.0, 30.0, 30.0, 30.0, 30.0)')  % Move joints to 30 Degree

fprintf(t, '%s\0', 'MovePose(206.332,38.525,227.788,-160.086,58.051,177.723)') % Move to Position x,y,z,alpha, beta, gamma

fprintf(t, '%s\0', 'MovePose(133.685,97.390,99.335,-165.747,-4.287,-153.335)')


SetJointVel

fprintf(t, '%s\0', 'MoveLin(10.0, 10.0, 10.0, 10.0, 10.0, 10.0)')
fscanf(t)

fprintf(t, '%s\0', 'ResetError')
fscanf(t)

fprintf(t, '%s\0', 'ActivateRobot')
fscanf(t)

fprintf(t, '%s\0', 'ActivateJointsFeed')
fscanf(t)

fprintf(t, '%s\0', 'ActivatePoseFeed')
fscanf(t)

fprintf(t, '%s\0', 'DeactivateSim')
fscanf(t)

fprintf(t, '%s\0', 'DisableBrakes')
fscanf(t)

fprintf(t, '%s\0', 'GetAutoConf')
fscanf(t)

fprintf(t, '%s\0', 'GetSimulationMode')
fscanf(t)

fprintf(t, '%s\0', 'GetJoints')
fscanf(t)

fprintf(t, '%s\0', 'GetPose')
fscanf(t)

fprintf(t, '%s\0', 'GetJointVel')
fscanf(t)

fprintf(t, '%s\0', 'GetMotionStatus')
fscanf(t)

fprintf(t, 'GetSimulationMode\0')
fscanf(t)

fprintf(t, '%s\0', 'Home')
fscanf(t)



