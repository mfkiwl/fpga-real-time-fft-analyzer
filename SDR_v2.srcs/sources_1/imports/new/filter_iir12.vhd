library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

library lib_filter;
use lib_filter.filter_pkg.all;

-- 12th-order IIR filter made from 6 cascaded 2nd-order stages using fixed coefficients
entity filter_iir12 is
  port (
    clk         : in  std_logic;                    -- System clock
    rst_n       : in  std_logic;                    -- Active-low synchronous reset
    i_valid     : in  std_logic;                    -- Input sample valid pulse
    rx_valid    : in std_logic;                           -- RX data valid (unused)
    rx_data     : in  std_logic_vector(7 downto 0); -- RX data (unused)
    i_last_data : in  std_logic;                           -- Last input sample flag
    fifo_last   : out std_logic;                        -- Last data flag output
    i_data      : in  std_logic_vector(15 downto 0);-- 16-bit input sample
    o_valid     : out std_logic;                    -- Output sample valid pulse
    o_last_data : out std_logic;                    -- High on last valid output
    o_data      : out std_logic_vector(15 downto 0) -- 16-bit filtered output
  );
end entity filter_iir12;

architecture rtl of filter_iir12 is

  -- Data pipeline between filter stages
  type t_array_sig16 is array(4 downto 0) of std_logic_vector(15 downto 0);
  signal data  : t_array_sig16 := (others => (others => '0')); -- data flowing through stages

  -- Valid and last flags between stages
  signal valid : std_logic_vector(4 downto 0) := (others => '0');
  signal last_s : std_logic_vector(4 downto 0) := (others => '0'); -- last flags through pipeline

begin

  -- Stage 1: Alpha-type 2nd-order IIR (uses fixed coefficients from package)
  u_filter_iir2 : filter_iir
    generic map (
      G_FILTER_TYPE => ALPHA
    )
    port map (
      clk         => clk,
      rst_n       => rst_n,
      i_valid     => i_valid,
      i_last_data => i_last_data,
      fifo_last   => last_s(0),
      i_data      => i_data,        -- input from outside
      o_valid     => valid(0),
      o_last_data => open,          -- internal only
      o_data      => data(0)        -- output to next stage
    );

  -- Stage 2: Beta-type 2nd-order IIR
  u_filter_iir4 : filter_iir
    generic map (
      G_FILTER_TYPE => BETA
    )
    port map (
      clk         => clk,
      rst_n       => rst_n,
      i_valid     => valid(0),      -- input from stage 1
      i_last_data => last_s(0),
      fifo_last   => last_s(1),
      i_data      => data(0),       -- data from stage 1
      o_valid     => valid(1),
      o_last_data => open,
      o_data      => data(1)        -- output to next stage
    );

  -- Stage 3: Alpha-type 2nd-order IIR
  u_filter_iir6 : filter_iir
    generic map (
      G_FILTER_TYPE => ALPHA
    )
    port map (
      clk         => clk,
      rst_n       => rst_n,
      i_valid     => valid(1),      -- input from stage 2
      i_last_data => last_s(1),
      fifo_last   => last_s(2),
      i_data      => data(1),
      o_valid     => valid(2),
      o_last_data => open,
      o_data      => data(2)
    );

  -- Stage 4: Beta-type 2nd-order IIR
  u_filter_iir8 : filter_iir
    generic map (
      G_FILTER_TYPE => BETA
    )
    port map (
      clk         => clk,
      rst_n       => rst_n,
      i_valid     => valid(2),      -- input from stage 3
      i_last_data => last_s(2),
      fifo_last   => last_s(3),
      i_data      => data(2),
      o_valid     => valid(3),
      o_last_data => open,
      o_data      => data(3)
    );

  -- Stage 5: Alpha-type 2nd-order IIR
  u_filter_iir10 : filter_iir
    generic map (
      G_FILTER_TYPE => ALPHA
    )
    port map (
      clk         => clk,
      rst_n       => rst_n,
      i_valid     => valid(3),      -- input from stage 4
      i_last_data => last_s(3),
      fifo_last   => last_s(4),
      i_data      => data(3),
      o_valid     => valid(4),
      o_last_data => open,
      o_data      => data(4)
    );

  -- Stage 6: Final Beta-type 2nd-order IIR (outputs to external)
  u_filter_iir12 : filter_iir
    generic map (
      G_FILTER_TYPE => BETA
    )
    port map (
      clk         => clk,
      rst_n       => rst_n,
      i_valid     => valid(4),      -- input from stage 5
      i_last_data => last_s(4),
      fifo_last   => fifo_last,     -- final output last flag
      i_data      => data(4),
      o_valid     => o_valid,       -- final output valid
      o_last_data => o_last_data,   -- final output last
      o_data      => o_data         -- final filtered output
    );

end architecture rtl;
