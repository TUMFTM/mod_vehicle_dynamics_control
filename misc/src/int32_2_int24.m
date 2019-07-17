function y_bytearray = int32_2_int24(u) 
  % returns a three byte array containing uint8 values which represent together a 
  % signed int24 value 
  % reverse engineered from roborace int24 to int32 cast 
  % y_int24 = bitshift(uint32(bitshift(u, 8)), -8);
  y_int24_separate = typecast(u, 'uint8');
  % extract bytes 
  y_bytearray = y_int24_separate(1:3);
end