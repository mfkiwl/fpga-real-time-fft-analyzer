library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

-- Reset synchronizer: async reset assertion, sync reset release
entity rst_sync is
  port (
    i_clk   : in  std_logic;  -- Target domain clock
    i_rst_n : in  std_logic;  -- Global async reset (active-low)
    o_rst_n : out std_logic   -- Local synchronized reset (active-low)
  );
end entity rst_sync;

architecture rtl of rst_sync is

  -- 2-stage shift register for clean reset release
  signal sync_ff : std_logic_vector(1 downto 0) := (others => '0');

begin

  -- Reset synchronizer: immediate assert, gradual release
  process(i_clk, i_rst_n)
  begin
    if i_rst_n = '0' then
      -- Async reset assertion (immediate)
      sync_ff <= (others => '0');
    elsif rising_edge(i_clk) then
      -- Sync reset release (shift in 1s on each clock)
      sync_ff(0) <= '1';
      sync_ff(1) <= sync_ff(0);
    end if;
  end process;

  -- Output goes high only after 2 clock cycles of reset release
  o_rst_n <= sync_ff(1);

end architecture rtl;
