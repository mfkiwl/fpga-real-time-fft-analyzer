library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

library lib_filter;
use lib_filter.filter_pkg.all;

-- 12th-order IIR filter made from 6 cascaded 2nd-order stages (Alpha/Beta alternating)
entity filter_iir12_cust is
  port (
    clk         : in  std_logic;                    -- System clock
    rst_n       : in  std_logic;                    -- Active-low synchronous reset
    i_valid     : in  std_logic;                    -- Input sample valid pulse
    rx_valid    : in std_logic;                           -- RX data valid (unused)
    rx_data     : in  std_logic_vector(7 downto 0); -- RX data (unused)
    i_last_data : in  std_logic;                           -- Last input sample flag
    coeff_data  : in  signed(7 downto 0);           -- Coefficient data input
    coeff_addr  : out integer;                      -- Current coefficient address
    fifo_last   : out std_logic;                        -- Last data flag output
    i_data      : in  std_logic_vector(15 downto 0);-- 16-bit input sample
    coeff_valid : in std_logic;                    -- Trigger to start loading coeffs
    o_valid     : out std_logic;                    -- Output sample valid pulse
    o_last_data : out std_logic;                    -- High on last valid output
    o_data      : out std_logic_vector(15 downto 0) -- 16-bit filtered output
  );
end entity filter_iir12_cust;

architecture rtl of filter_iir12_cust is

  -- Data pipeline between filter stages
  type t_array_sig16 is array(4 downto 0) of std_logic_vector(15 downto 0);
  signal data  : t_array_sig16 := (others => (others => '0')); -- data flowing through stages

  -- Valid and last flags between stages
  signal valid : std_logic_vector(4 downto 0) := (others => '0');
  signal last_s : std_logic_vector(4 downto 0) := (others => '0');

  signal start : std_logic := '0';  -- coefficient loading active
  signal cnt : integer  := 0;       -- coefficient counter (0-11)

  -- Storage for all 12 filter coefficients
  type t_array_sig8 is array(11 downto 0) of signed(7 downto 0);
  signal COEFF_IIR_CF : t_array_sig8;

begin

-- Coefficient loading state machine: loads 12 coeffs sequentially
process(clk)
begin
  if rising_edge(clk) then
    if rst_n = '0' then
      COEFF_IIR_CF <= (others => x"00");  -- clear all coefficients
    elsif start = '1' then
      COEFF_IIR_CF(cnt) <= coeff_data;    -- store current coefficient
      cnt <= cnt + 1;
      if cnt = 11 then
        start <= '0';                     -- done loading all 12
      end if;
    elsif coeff_valid = '1' then
      start <= '1';                       -- start coefficient loading sequence
    end if;
  end if;
end process;

coeff_addr <= cnt;  -- tell external module which coefficient we want next

  -- Stage 1: Alpha-type 2nd-order IIR
  u_filter_iir_cust2 : filter_iir_cust
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
      o_data      => data(0),       -- output to next stage
      -- All coefficients (both Alpha and Beta sets) connected to all stages
      COEFF_IIR_CF_B0_0 => COEFF_IIR_CF(0),
      COEFF_IIR_CF_B1_0 => COEFF_IIR_CF(1),
      COEFF_IIR_CF_B2_0 => COEFF_IIR_CF(2),
      COEFF_IIR_CF_A0_0 => COEFF_IIR_CF(3),
      COEFF_IIR_CF_A1_0 => COEFF_IIR_CF(4),
      COEFF_IIR_CF_A2_0 => COEFF_IIR_CF(5),
      COEFF_IIR_CF_B0_1 => COEFF_IIR_CF(6),
      COEFF_IIR_CF_B1_1 => COEFF_IIR_CF(7),
      COEFF_IIR_CF_B2_1 => COEFF_IIR_CF(8),
      COEFF_IIR_CF_A0_1 => COEFF_IIR_CF(9),
      COEFF_IIR_CF_A1_1 => COEFF_IIR_CF(10),
      COEFF_IIR_CF_A2_1 => COEFF_IIR_CF(11)
    );

  -- Stage 2: Beta-type 2nd-order IIR
  u_filter_iir_cust4 : filter_iir_cust
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
      o_data      => data(1),       -- output to next stage
      COEFF_IIR_CF_B0_0 => COEFF_IIR_CF(0),
      COEFF_IIR_CF_B1_0 => COEFF_IIR_CF(1),
      COEFF_IIR_CF_B2_0 => COEFF_IIR_CF(2),
      COEFF_IIR_CF_A0_0 => COEFF_IIR_CF(3),
      COEFF_IIR_CF_A1_0 => COEFF_IIR_CF(4),
      COEFF_IIR_CF_A2_0 => COEFF_IIR_CF(5),
      COEFF_IIR_CF_B0_1 => COEFF_IIR_CF(6),
      COEFF_IIR_CF_B1_1 => COEFF_IIR_CF(7),
      COEFF_IIR_CF_B2_1 => COEFF_IIR_CF(8),
      COEFF_IIR_CF_A0_1 => COEFF_IIR_CF(9),
      COEFF_IIR_CF_A1_1 => COEFF_IIR_CF(10),
      COEFF_IIR_CF_A2_1 => COEFF_IIR_CF(11)
    );

  -- Stage 3: Alpha-type 2nd-order IIR
  u_filter_iir_cust6 : filter_iir_cust
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
      o_data      => data(2),
      COEFF_IIR_CF_B0_0 => COEFF_IIR_CF(0),
      COEFF_IIR_CF_B1_0 => COEFF_IIR_CF(1),
      COEFF_IIR_CF_B2_0 => COEFF_IIR_CF(2),
      COEFF_IIR_CF_A0_0 => COEFF_IIR_CF(3),
      COEFF_IIR_CF_A1_0 => COEFF_IIR_CF(4),
      COEFF_IIR_CF_A2_0 => COEFF_IIR_CF(5),
      COEFF_IIR_CF_B0_1 => COEFF_IIR_CF(6),
      COEFF_IIR_CF_B1_1 => COEFF_IIR_CF(7),
      COEFF_IIR_CF_B2_1 => COEFF_IIR_CF(8),
      COEFF_IIR_CF_A0_1 => COEFF_IIR_CF(9),
      COEFF_IIR_CF_A1_1 => COEFF_IIR_CF(10),
      COEFF_IIR_CF_A2_1 => COEFF_IIR_CF(11)
    );

  -- Stage 4: Beta-type 2nd-order IIR
  u_filter_iir_cust8 : filter_iir_cust
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
      o_data      => data(3),
      COEFF_IIR_CF_B0_0 => COEFF_IIR_CF(0),
      COEFF_IIR_CF_B1_0 => COEFF_IIR_CF(1),
      COEFF_IIR_CF_B2_0 => COEFF_IIR_CF(2),
      COEFF_IIR_CF_A0_0 => COEFF_IIR_CF(3),
      COEFF_IIR_CF_A1_0 => COEFF_IIR_CF(4),
      COEFF_IIR_CF_A2_0 => COEFF_IIR_CF(5),
      COEFF_IIR_CF_B0_1 => COEFF_IIR_CF(6),
      COEFF_IIR_CF_B1_1 => COEFF_IIR_CF(7),
      COEFF_IIR_CF_B2_1 => COEFF_IIR_CF(8),
      COEFF_IIR_CF_A0_1 => COEFF_IIR_CF(9),
      COEFF_IIR_CF_A1_1 => COEFF_IIR_CF(10),
      COEFF_IIR_CF_A2_1 => COEFF_IIR_CF(11)
    );

  -- Stage 5: Alpha-type 2nd-order IIR
  u_filter_iir_cust10 : filter_iir_cust
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
      o_data      => data(4),
      COEFF_IIR_CF_B0_0 => COEFF_IIR_CF(0),
      COEFF_IIR_CF_B1_0 => COEFF_IIR_CF(1),
      COEFF_IIR_CF_B2_0 => COEFF_IIR_CF(2),
      COEFF_IIR_CF_A0_0 => COEFF_IIR_CF(3),
      COEFF_IIR_CF_A1_0 => COEFF_IIR_CF(4),
      COEFF_IIR_CF_A2_0 => COEFF_IIR_CF(5),
      COEFF_IIR_CF_B0_1 => COEFF_IIR_CF(6),
      COEFF_IIR_CF_B1_1 => COEFF_IIR_CF(7),
      COEFF_IIR_CF_B2_1 => COEFF_IIR_CF(8),
      COEFF_IIR_CF_A0_1 => COEFF_IIR_CF(9),
      COEFF_IIR_CF_A1_1 => COEFF_IIR_CF(10),
      COEFF_IIR_CF_A2_1 => COEFF_IIR_CF(11)
    );

  -- Stage 6: Final Beta-type 2nd-order IIR (outputs to external)
  u_filter_iir_cust12 : filter_iir_cust
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
      o_data      => o_data,        -- final filtered output
      COEFF_IIR_CF_B0_0 => COEFF_IIR_CF(0),
      COEFF_IIR_CF_B1_0 => COEFF_IIR_CF(1),
      COEFF_IIR_CF_B2_0 => COEFF_IIR_CF(2),
      COEFF_IIR_CF_A0_0 => COEFF_IIR_CF(3),
      COEFF_IIR_CF_A1_0 => COEFF_IIR_CF(4),
      COEFF_IIR_CF_A2_0 => COEFF_IIR_CF(5),
      COEFF_IIR_CF_B0_1 => COEFF_IIR_CF(6),
      COEFF_IIR_CF_B1_1 => COEFF_IIR_CF(7),
      COEFF_IIR_CF_B2_1 => COEFF_IIR_CF(8),
      COEFF_IIR_CF_A0_1 => COEFF_IIR_CF(9),
      COEFF_IIR_CF_A1_1 => COEFF_IIR_CF(10),
      COEFF_IIR_CF_A2_1 => COEFF_IIR_CF(11)
    );

end architecture rtl;
