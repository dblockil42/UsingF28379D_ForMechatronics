%send the starting command to 
function startCollect(comport)

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

if (testifcom(comport) == 0)
    display('Choose from the following COM ports');
        list_serialports()
    exception = MException('MATLAB:ComportNotFound','COM Port not found.');
    throw(exception);
end


hex_str = 'CC55'; % header
char_str = char(sscanf(hex_str,'%2X').');

% s = serial(comport);
s = serialport(comport,115200);
% set(s,'BaudRate',115200);
s.InputBufferSize = 150000;
fopen(s);
fwrite(s,char_str);

pause(2); 
%fclose(s);
%delete(s)
clear s
