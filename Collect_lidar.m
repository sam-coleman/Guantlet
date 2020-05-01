sub = rossubscriber('/scan');

% place Neato at the origin pointing in the ihat_G direction
placeNeato(0,0,1,0)

% wait a while for the Neato to fall into place
pause(2);

% Collect data at the room origin
scan_message = receive(sub);
r = scan_message.Ranges(1:end-1);
theta = deg2rad([0:359]');

%save data in a seperate .mat file
save('lidar_gauntlet.mat','r','theta')

% For simulated Neatos only:
% Place the Neato in the specified x, y position and specified heading vector.
function placeNeato(posX, posY, headingX, headingY)
    svc = rossvcclient('gazebo/set_model_state');
    msg = rosmessage(svc);

    msg.ModelState.ModelName = 'neato_standalone';
    startYaw = atan2(headingY, headingX);
    quat = eul2quat([startYaw 0 0]);

    msg.ModelState.Pose.Position.X = posX;
    msg.ModelState.Pose.Position.Y = posY;
    msg.ModelState.Pose.Position.Z = 1.0;
    msg.ModelState.Pose.Orientation.W = quat(1);
    msg.ModelState.Pose.Orientation.X = quat(2);
    msg.ModelState.Pose.Orientation.Y = quat(3);
    msg.ModelState.Pose.Orientation.Z = quat(4);

    % put the robot in the appropriate place
    ret = call(svc, msg);
end
