library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

library lib_cdc;
use lib_cdc.cdc_pkg.all;

-- Pulse synchronizer: safely transfers single-cycle pulses between clock domains
entity pulse_synchronizer is
  port (
    i_clk_a   : in  std_logic;   -- Source clock domain
    i_rst_n_a : in  std_logic;   -- Active-low reset in source domain
    i_clk_b   : in  std_logic;   -- Destination clock domain
    i_rst_n_b : in  std_logic;   -- Active-low reset in destination domain
    i_pulse_a : in  std_logic;   -- Single-cycle pulse in source domain
    o_pulse_b : out std_logic    -- Single-cycle pulse in destination domain
  );
end entity pulse_synchronizer;

architecture rtl of pulse_synchronizer is

  signal toggle           : std_logic := '0';  -- flips state on each input pulse
  signal toggle_sync      : std_logic := '0';  -- toggle signal synced to dest clock
  signal toggle_sync_pipe : std_logic := '0';  -- previous sync value for edge detect
  signal pulse_out        : std_logic := '0';  -- generated output pulse

begin

  -- Toggle flip-flop in source domain: changes state on each pulse
  p_toggle : process(i_clk_a)
  begin
    if rising_edge(i_clk_a) then
      if i_rst_n_a = '0' then
        toggle <= '0';
      else
        if i_pulse_a = '1' then
          toggle <= not toggle;  -- flip state when pulse arrives
        end if;
      end if;
    end if;
  end process p_toggle;

  -- Synchronize the toggle signal to destination clock domain
  u_toggle_sync : synchronizer
    port map (
      i_clk   => i_clk_b,
      i_rst_n => i_rst_n_b,
      i_data  => toggle,
      o_data  => toggle_sync
    );

  -- Edge detector: generates pulse when synchronized toggle changes
  p_pulse_out : process(i_clk_b)
  begin
    if rising_edge(i_clk_b) then
      if i_rst_n_b = '0' then
        toggle_sync_pipe <= '0';
        pulse_out        <= '0';
      else
        toggle_sync_pipe <= toggle_sync;  -- delay by one clock
        -- Generate pulse when toggle state changes (edge detected)
        if toggle_sync /= toggle_sync_pipe then
          pulse_out <= '1';
        else
          pulse_out <= '0';
        end if;
      end if;
    end if;
  end process p_pulse_out;

  o_pulse_b <= pulse_out;

end architecture rtl;
