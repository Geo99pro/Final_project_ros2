function status = gpg_read(s)
%GPG_READ    Read remote GoPiGo3 status.
%   STATUS = GPG_READ(S) reads the status of remote
%   GoPiGo3 connected to socket S. Note that a (possible null)
%   write command must be issued for the robot to send the status.
%
%   STATUS is a struct with the following fields:
%      position: position (in rad) of the wheels
%      line    : line sensors as 10-bit values
%      battery : battery voltage in V
%      distance: ultrasonic distance sensor values in m
%      light   : light sensors as 12-bit values
%
%   EXAMPLE:
%       s = gpg_open('192.168.0.205');
%       gpg_write(s);
%       status = gpg_read(s)
%       gpg_close(s);
%
%   AUTHOR:
%       Wouter Caarls <wouter@caarls.org>

    data = uint8(fread(s, 60, 'uint8'));
    
    size = typecast(data(1:4), 'uint32');

    if size ~= 56
        error('Protocol mismatch error');
    end
    
    status.position = double(typecast(data(5:12), 'int32'))'*pi/180;
    status.line = double(typecast(data(13:32), 'int32'))';
    status.battery = double(typecast(data(33:36), 'single'))';
    v = double(typecast(data(37:52), 'int32'))'/1024*5;
    
    % http://jeremyblythe.blogspot.com/2012/09/raspberry-pi-distance-measuring-sensor.html
    status.distance = (16.2537 * v.^4 - 129.893 * v.^3 + 382.268 * v.^2 - 512.611 * v + 306.439) / 100;
    status.light = double(typecast(data(53:60), 'int32'))';

end
