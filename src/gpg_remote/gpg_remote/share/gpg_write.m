function gpg_write(s, vel, servo)
%GPG_WRITE    Write command to remote GoPiGo3.
%   GPG_WRITE(S) writes a null command to remote GoPiGo3 connected to
%   socket S. Used to read status without changing velocity.
%   GPG_WRITE(S, VEL) writes the command velocity (in rad/s) of the wheels
%   of remote GoPiGo3 connected to socket S.
%   GPG_WRITE(S, VEL, SERVO) additionally writes the command position of
%   the servo in rad. The default is 0.
%
%   EXAMPLE:
%       s = gpg_open('192.168.0.205');
%       gpg_write(s, [pi/4, pi/2]); % curve left
%       pause(1);
%       gpg_close(s);
%
%   AUTHOR:
%       Wouter Caarls <wouter@caarls.org>

    size = 12;

    if nargin < 3
        servo = 0;
    end

    if nargin < 2
        fwrite(s, typecast(uint32(0), 'uint8'));
    else
        data = [typecast(uint32(size), 'uint8') ...
                typecast(int32(vel*180/pi), 'uint8') ...
                typecast(uint32(servo*(1850/pi) + 1500), 'uint8')];
        
        fwrite(s, data);
    end

end
