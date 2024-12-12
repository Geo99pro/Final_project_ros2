function gpg_close(s)
%GPG_CLOSE    Close connection to remote GoPiGo3.
%   S = GPG_CLOSE(S) closes the connection to remote GoPiGo3 connected to
%   socket S.
%
%   EXAMPLE:
%       s = gpg_open('192.168.0.205');
%       gpg_close(s);
%
%   AUTHOR:
%       Wouter Caarls <wouter@caarls.org>

    fclose(s);
end