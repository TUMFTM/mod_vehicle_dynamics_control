function crc = crc32(data)
%crc32novatel Calculate CRC-32 checksum of input data
%   crc = crc32novatel(data) calculates CRC-32 checksum of input data. The
%   elements of DATA are treated as individual bytes (uint8's). Resulting
%   CRC is unsigned 8-bit integer (uint8). Calculation is exectued
%   according to https://docs.novatel.com/OEM7/Content/Messages/32_Bit_CRC.htm
%
crc = uint32(0);
data = typecast(data, 'uint8');

for ulCount = 1:numel(data)
    ulTemp1 = bitand(bitshift(crc, -8), uint32(16777215));
    ulTemp2 = crc32value(bitand(bitxor(crc, uint32(data(ulCount))), uint32(255)));
    crc = bitxor(ulTemp1, ulTemp2);
end

end

function crcvalue = crc32value(i)
%crc32value Calculate CRC-32 value of input data

CRC32_POLYNOMIAL = uint32(3988292384);
crcvalue = i;

for j = 8:-1:1
    if bitand(crcvalue, 1)
        crcvalue = bitxor(bitshift(crcvalue, -1), CRC32_POLYNOMIAL);
    else
        crcvalue = bitshift(crcvalue, -1);
    end
end

end