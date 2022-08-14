function [isHex] = isHex(str)

try
    test = hex2dec(str);
catch Exception
    isHex =  false;
    return;
end
isHex = true;