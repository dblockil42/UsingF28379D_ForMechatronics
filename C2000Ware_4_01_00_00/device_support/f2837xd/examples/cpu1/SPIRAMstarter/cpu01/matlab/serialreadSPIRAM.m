function [part] = serialreadSPIRAM(comport)

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
%s = serial(comport);
s = serialport(comport,115200);
%set(s,'BaudRate',115200);
s.InputBufferSize = 150000;
fopen(s);
% s = comport;
hex_str = 'AA55'; % header for reading data back from spiram
char_str = char(sscanf(hex_str,'%2X').');
fwrite(s,char_str);
packet_size = 200;

while 1
    inchar = fread(s,1);
    if inchar == 42 %the first data sending back is '*', which is ascii 42
        packet_number = fread(s,1); %second number getting back is packet number
        var_num = fread(s,1); %third number getting back is variable number 
        
        for i = 1:packet_number*var_num
            if i == 1
                out3 = fread(s,packet_size,'float32');
                hex_str = 'DD'; %end character for reading new a new packet
                char_str = char(sscanf(hex_str,'%2X').');
                fwrite(s,char_str);
            else 
                out3 = [out3;fread(s,packet_size,'float32')];
%                 if i ~= packet_number
                    hex_str = 'DD'; %end character for reading new a new packet
                    char_str = char(sscanf(hex_str,'%2X').');
                    fwrite(s,char_str);
%                 end
            end
        end
%         out3 = fread(s,packet_size,'float32');%read each packet_size of floats a time
        
% parse the data received based on the varible number
        for i=1:var_num
            temp=out3(i:var_num:end);
            if i == 1
                part = temp;
            else
                try %handle the size not consistent error and show size
                    part = [part, temp];
                catch ME
                    if (strcmp(ME.identifier,'MATLAB:catenate:dimensionMismatch'))
                        msg = ['Dimension mismatch occurred: First argument has shape of ', ...
                            num2str(size(part)),' while second has ', ...
                            num2str(size(temp)),'.'];
                        causeException = MException('MATLAB:myCode:dimensions',msg);
                        ME = addCause(ME,causeException);
                    end
                    rethrow(ME)
                end
            end
        end

        break;
    end
end
hex_str = 'BB'; %end character for reading new a new packet
char_str = char(sscanf(hex_str,'%2X').');
fwrite(s,char_str);

%fclose(s)
%delete(s)
clear s
