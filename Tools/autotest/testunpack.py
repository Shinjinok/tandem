import struct

raw_data = b'@\x16\x17\xb7\xd7\xf2e\x95'
print(len(raw_data))
for i in range(len(raw_data)):
    print(raw_data[i])

b = struct.unpack('>d', raw_data)
print(b)
