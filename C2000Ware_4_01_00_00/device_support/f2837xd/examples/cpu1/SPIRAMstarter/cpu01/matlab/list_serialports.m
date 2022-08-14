function ports = list_serialports()
% returns cell array of found serial ports under Win
% uses CLI MODE command internally
    [~,res]=system('mode');
    ports=regexp(res,'COM\d+:','match')';