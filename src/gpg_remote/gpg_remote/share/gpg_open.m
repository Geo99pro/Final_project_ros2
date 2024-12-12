function s = gpg_open(ip)
%GPG_OPEN    Open connection to remote GoPiGo3.
%   S = GPG_OPEN(IP) opens a connection to remote GoPiGo3 with address IP,
%   returning the socket S.
%
%   EXAMPLE:
%       s = gpg_open('192.168.0.205');
%
%   AUTHOR:
%       Wouter Caarls <wouter@caarls.org>

    s = tcpip(ip, 8002, 'NetworkRole', 'client');
    fopen(s);
end
