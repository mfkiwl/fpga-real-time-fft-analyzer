--------------------------------------------------------------------------------
-- shift2to8.vhd  (only start collecting when we see in_valid='1' & in_2b="01")
--------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- Converts 2-bit nibbles from RMII into 8-bit bytes
entity shift2to8 is
  Port (
    clk       : in  std_logic;                    -- 50 MHz RMII clock
    rst_n     : in  std_logic;                    -- active-low reset
    in_valid  : in  std_logic;                    -- tied to phy_crs_dv
    init      : in  std_logic;
    in_2b     : in  std_logic_vector(1 downto 0); -- tied to phy_rxd[1:0]
    out_valid : out std_logic;                    -- pulses when a full 8-bit is ready
    out_8b    : out std_logic_vector(7 downto 0)  -- assembled 8-bit MSB-first
  );
end shift2to8;

architecture Behavioral of shift2to8 is
  signal shift_reg     : std_logic_vector(7 downto 0) := (others => '0'); -- builds up the byte
  signal nibble_count  : integer range 0 to 3 := 0; -- tracks which nibble we're on (0-3)
  signal byte_count    : integer:= 0; -- 0 = waiting for start pattern
  signal out_valid_reg : std_logic := '0';
begin

  process(clk)
  begin
    if rising_edge(clk) then
      
      -- Reset everything
      if (rst_n = '0') or (init = '1') then
        shift_reg     <= (others => '0');
        nibble_count  <= 0;
        byte_count    <= 0;
        out_valid_reg <= '0';
      else

        -- Start collecting when we see "01" pattern
        if (in_valid = '1') and (in_2b = "01") and (byte_count = 0) then
          shift_reg(5 downto 0) <= (others => '0');
          shift_reg(7 downto 6) <= in_2b; -- first nibble goes to MSB
          nibble_count  <= 1;
          out_valid_reg <= '0';
          byte_count <= 1;

        -- Keep collecting nibbles for current byte
        elsif (in_valid = '1') and (byte_count > 0) then
          shift_reg(5 downto 0) <= shift_reg(7 downto 2); -- shift left 2 bits
          shift_reg(7 downto 6) <= in_2b; -- new nibble at top

          if nibble_count = 3 then
            -- Got all 4 nibbles, byte is complete
            out_valid_reg <= '1';
            nibble_count  <= 0;
            byte_count <= byte_count + 1;
          else
            out_valid_reg <= '0';
            nibble_count  <= nibble_count + 1;
          end if;
          
        -- No valid data - go back to waiting
        elsif (in_valid = '0') then
          out_valid_reg <= '0';
          byte_count <= 0;
          nibble_count <= 0;

        else
          out_valid_reg <= '0';
        end if;
      end if;
    end if;
  end process;

  -- Output assignments
  out_8b    <= shift_reg;
  out_valid <= out_valid_reg;

end Behavioral;
