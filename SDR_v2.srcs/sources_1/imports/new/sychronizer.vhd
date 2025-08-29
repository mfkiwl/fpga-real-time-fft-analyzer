library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- 2-stage synchronizer to safely cross clock domains
entity synchronizer is
  port (
    i_clk   : in  std_logic;  -- Destination domain clock
    i_rst_n : in  std_logic;  -- Active-low synchronous reset
    i_data  : in  std_logic;  -- Asynchronous input signal
    o_data  : out std_logic   -- Synchronized output signal
  );
end entity synchronizer;

architecture rtl of synchronizer is

  signal meta   : std_logic := '0';  -- first flip-flop (may go metastable)
  signal stable : std_logic := '0';  -- second flip-flop (stable output)

begin

  o_data <= stable;  -- output the synchronized signal

  -- Two-stage sync process to avoid metastability
  p_sync : process(i_clk)
  begin
    if rising_edge(i_clk) then
      if i_rst_n = '0' then
        meta   <= '0';  -- reset both stages
        stable <= '0';
      else
        meta   <= i_data;  -- capture async input (might be metastable)
        stable <= meta;    -- second stage settles to stable value
      end if;
    end if;
  end process p_sync;

end architecture rtl;
