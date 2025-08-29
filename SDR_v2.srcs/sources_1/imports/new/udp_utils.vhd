library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

package udp_utils is
-- Calculate UDP checksum with pseudo-header
-- Notes:
-- - udp_len must be the UDP header + payload length (usually 8 + data_bytes)
-- - udp_hdr must have its checksum field zeroed before calling
-- - udp_data is LSB-first packed: byte 0 at bits 7:0, byte 1 at 15:8, etc.
-- - data_bytes range: 0..1472
function calc_udp_checksum(
src_ip : std_logic_vector(31 downto 0); -- Source IP
dst_ip : std_logic_vector(31 downto 0); -- Destination IP
udp_len : std_logic_vector(15 downto 0); -- UDP length (header+payload)
udp_hdr : std_logic_vector(63 downto 0); -- UDP header (checksum field zeroed)
udp_data : std_logic_vector(11775 downto 0); -- UDP payload (max 1472 bytes), LSB-first
data_bytes : integer -- Payload length in bytes (0..1472)
) return std_logic_vector; -- 16-bit checksum (std_logic_vector(15 downto 0))
end package udp_utils;

package body udp_utils is

function calc_udp_checksum(
src_ip : std_logic_vector(31 downto 0);
dst_ip : std_logic_vector(31 downto 0);
udp_len : std_logic_vector(15 downto 0);
udp_hdr : std_logic_vector(63 downto 0);
udp_data : std_logic_vector(11775 downto 0);
data_bytes : integer
) return std_logic_vector is
variable sum : unsigned(16 downto 0) := (others => '0'); -- 17-bit accumulator for end-around carry
variable w : unsigned(15 downto 0);
variable result : unsigned(15 downto 0);
variable udp_hdr_temp : std_logic_vector(63 downto 0);
variable total_words : integer;
variable byte_idx : integer;
variable data_len : integer; -- clamped data_bytes
begin
-- Clamp data_bytes to legal range
if data_bytes < 0 then
data_len := 0;
elsif data_bytes > 1472 then
data_len := 1472;
else
data_len := data_bytes;
end if;

-- Zero UDP checksum field (bits 15 downto 0)
udp_hdr_temp := udp_hdr;
udp_hdr_temp(15 downto 0) := (others => '0');

-- Pseudo-header: Source IP
w := unsigned(src_ip(31 downto 16));
sum := sum + ('0' & w);
if sum(16) = '1' then
  sum := resize(unsigned(sum(15 downto 0)) + 1, 17);
end if;

w := unsigned(src_ip(15 downto 0));
sum := sum + ('0' & w);
if sum(16) = '1' then
  sum := resize(unsigned(sum(15 downto 0)) + 1, 17);
end if;

-- Pseudo-header: Destination IP
w := unsigned(dst_ip(31 downto 16));
sum := sum + ('0' & w);
if sum(16) = '1' then
  sum := resize(unsigned(sum(15 downto 0)) + 1, 17);
end if;

w := unsigned(dst_ip(15 downto 0));
sum := sum + ('0' & w);
if sum(16) = '1' then
  sum := resize(unsigned(sum(15 downto 0)) + 1, 17);
end if;

-- Protocol (17 = UDP) and UDP length
w := x"0011";
sum := sum + ('0' & w);
if sum(16) = '1' then
  sum := resize(unsigned(sum(15 downto 0)) + 1, 17);
end if;

w := unsigned(udp_len);
sum := sum + ('0' & w);
if sum(16) = '1' then
  sum := resize(unsigned(sum(15 downto 0)) + 1, 17);
end if;

-- UDP header (4 words, checksum field already zero)
for i in 0 to 3 loop
  w := unsigned(udp_hdr_temp(63 - 16*i downto 48 - 16*i));
  sum := sum + ('0' & w);
  if sum(16) = '1' then
    sum := resize(unsigned(sum(15 downto 0)) + 1, 17);
  end if;
end loop;

-- UDP payload: LSB-first packing, two bytes per 16-bit word
total_words := (data_len + 1) / 2; -- round up for odd byte count
for i in 0 to total_words - 1 loop
  byte_idx := i * 2;
  w := (others => '0');

  if byte_idx < data_len then
    -- First byte -> MSB of word
    w(15 downto 8) := unsigned(udp_data((byte_idx+1)*8 - 1 downto byte_idx*8));

    if (byte_idx + 1) < data_len then
      -- Second byte -> LSB of word
      w(7 downto 0) := unsigned(udp_data((byte_idx+2)*8 - 1 downto (byte_idx+1)*8));
    else
      -- Odd length: pad LSB with zero
      w(7 downto 0) := x"00";
    end if;

    sum := sum + ('0' & w);
    if sum(16) = '1' then
      sum := resize(unsigned(sum(15 downto 0)) + 1, 17);
    end if;
  end if;
end loop;

-- Final end-around carry
if sum(16) = '1' then
  sum := resize(unsigned(sum(15 downto 0)) + 1, 17);
end if;

-- One's complement
result := not sum(15 downto 0);

-- Per UDP spec: checksum value 0x0000 represented as 0xFFFF
if result = x"0000" then
  result := x"FFFF";
end if;

return std_logic_vector(result);

end function;

end package body udp_utils;