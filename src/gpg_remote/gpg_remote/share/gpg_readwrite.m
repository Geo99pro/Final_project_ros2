function status = gpg_readwrite(s, varargin)
%GPG_WRITE    Write command to remote GoPiGo3 and read status.
%   STATUS = GPG_READWRITE(...) performs a gpg_write and subsequent
%   gpg_read.
%
%   EXAMPLE:
%       s = gpg_open('192.168.0.205');
%       status = gpg_readwrite(s, [pi/4, pi/2]); % curve left
%       pause(1);
%       gpg_close(s);
%
%   SEE ALSO:
%       gpg_write, gpg_read
%
%   AUTHOR:
%       Wouter Caarls <wouter@caarls.org>

    gpg_write(s, varargin{:});
    status = gpg_read(s);

end
