syms x y
f = x*y - x^2 - y^2 - 2*x - 2*y + 4;
[~, ~, ~, r, ~] = gradientAscent(f, 1, -1, -3, 1, -3, 1, .99, .05); %get list of x and y locations to drive to
pub = rospublisher('raw_vel');

%stop the robot if it's going on
stopMsg = rosmessage(pub);
stopMsg.Data = [0, 0];
send(pub, stopMsg);

placeNeato(1, -1, 0, 1)
pause(2) %wait for neato to drop

rotate_speed = [-.05 .05];
w = (rotate_speed(1, 2) - rotate_speed(1, 1))/.235;
forward_speed = [.3, .3];

for n = 2:length(r(:, 1))-1
    n %show the incramenting
    
    %find the difference of heading between where we are facing now
    %and where we need to be facing
    r_norm = (r(n, :)-r(n-1,:)) / norm(r(n,:)-r(n-1,:));
    r_norm_next = (r(n+1, :) - r(n, :)) / norm((r(n+1, :) - r(n,:)));
    theta_delta = acos(dot(r_norm_next, r_norm));
    
    %deal with first iteration
    if n == 2
        theta_delta = theta_delta + acos(dot(r_norm, [0; 1]));
    end
    
    %rotate in place until facing correct direction
    pause_length = abs(theta_delta/w);
    drive = rosmessage(pub);
    drive.Data = rotate_speed;
    send(pub, drive);
    pause(pause_length)
    drive.Data = [0, 0];
    send(pub, drive);
    
    %determine how far we need to drive straight, and drive that distance
    distance = sqrt((r(n, 1) - r(n-1, 1))^2 + (r(n, 2) - r(n-1, 2))^2);
    pause_length = distance / forward_speed(1, 1);
    drive.Data = forward_speed;
    send(pub, drive);
    pause(pause_length)
    drive.Data = [0, 0];
    send(pub, drive);
end

%make sure the robot actually stops
drive.Data = [0,0];
send(pub, drive);

%placeNeato is given to us by teaching team
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