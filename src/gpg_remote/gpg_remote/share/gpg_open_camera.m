function s = gpg_open_camera(ip)
%GPG_OPEN    Open connection to remote GoPiGo3.
%   S = GPG_OPEN_CAMERA(IP) opens a camera connection to remote GoPiGo3 with
%   address IP, returning the socket S.
%
%   EXAMPLE:
%       s = gpg_open_camera('192.168.0.201');
%
%   AUTHOR:
%       Wouter Caarls <wouter@caarls.org>

    s = tcpip(ip, 8000, 'NetworkRole', 'client', 'InputBufferSize', 262144);
    fopen(s);
end
