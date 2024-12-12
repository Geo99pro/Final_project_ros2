function img = gpg_read_camera(s)
%GPG_READ_CAMERA    Read remote GoPiGo3 camera image.
%   IMAGE = GPG_READ_CAMERA(S) returns the next camera image of 
%   GoPiGo3 connected to socket S.
%
%   EXAMPLE:
%       s = gpg_open_camera('192.168.0.201');
%       while true
%           image(gpg_read_camera(s))
%           drawnow
%       end
%       gpg_close(s);
%
%   AUTHOR:
%       Wouter Caarls <wouter@caarls.org>
%       http://stackoverflow.com/questions/18659586/from-raw-bits-to-jpeg-without-writing-into-a-file

    size = double(swapbytes(uint32(fread(s, 1, 'uint32'))));
    data = fread(s, size, 'uint8');
    
    % decode image stream using Java
    jImg = javax.imageio.ImageIO.read(java.io.ByteArrayInputStream(data));
    h = jImg.getHeight;
    w = jImg.getWidth;

    % convert Java Image to MATLAB image
    p = typecast(jImg.getData.getDataStorage, 'uint8');
    img = permute(reshape(p, [3 w h]), [3 2 1]);
    img = img(:,:,[3 2 1]);
end
