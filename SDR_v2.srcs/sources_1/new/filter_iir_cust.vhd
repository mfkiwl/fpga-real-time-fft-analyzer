library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

library lib_filter;
use lib_filter.filter_pkg.all;

-- Configurable IIR filter (Alpha or Beta type) with transposed direct-form II structure
entity filter_iir_cust is
  generic (
    G_FILTER_TYPE : t_filter_iir_types := ALPHA  -- Select filter type
  );
  port (
    clk         : in  std_logic;                           -- System clock
    rst_n       : in  std_logic;                           -- Active-low synchronous reset
    i_valid     : in  std_logic;                           -- Input sample valid
    i_last_data : in  std_logic;                           -- Last input sample flag
    fifo_last   : out std_logic;                        -- Last data for FIFO
    i_data      : in  std_logic_vector(15 downto 0);       -- Input data word
    o_valid     : out std_logic;                           -- Output sample valid
    o_last_data : out std_logic;                           -- High on last valid sample
    o_data      : out std_logic_vector(15 downto 0);        -- Output filtered data
    -- IIR coefficients for two biquad sections (B0,B1,B2,A0,A1,A2 for each)
    COEFF_IIR_CF_B0_0 : in signed(7 downto 0);
    COEFF_IIR_CF_B1_0 : in signed(7 downto 0);
    COEFF_IIR_CF_B2_0 : in signed(7 downto 0);
    COEFF_IIR_CF_A0_0 : in signed(7 downto 0);
    COEFF_IIR_CF_A1_0 : in signed(7 downto 0);
    COEFF_IIR_CF_A2_0 : in signed(7 downto 0);
    COEFF_IIR_CF_B0_1 : in signed(7 downto 0);
    COEFF_IIR_CF_B1_1 : in signed(7 downto 0);
    COEFF_IIR_CF_B2_1 : in signed(7 downto 0);
    COEFF_IIR_CF_A0_1 : in signed(7 downto 0);
    COEFF_IIR_CF_A1_1 : in signed(7 downto 0);
    COEFF_IIR_CF_A2_1 : in signed(7 downto 0)
  );
end entity filter_iir_cust;

architecture rtl of filter_iir_cust is

  -- Shift register arrays for filter history (3 taps each)
  type t_array_sig16 is array(2 downto 0) of signed(15 downto 0);

  signal ve       : t_array_sig16 := (others => (others => '0'));  -- input history (x[n], x[n-1], x[n-2])
  signal vs       : t_array_sig16 := (others => (others => '0'));  -- output history (y[n], y[n-1], y[n-2])

  -- Coefficient multiplication results (24-bit to handle overflow)
  signal mult_b0_1  : signed(23 downto 0);
  signal mult_b1_1  : signed(23 downto 0);
  signal mult_b2_1  : signed(23 downto 0);
  signal mult_a0_1  : signed(23 downto 0);
  signal mult_a1_1  : signed(23 downto 0);

  signal valid    : std_logic := '0';                            -- delayed input valid signal

-- Small FIFO to track "last" flags through the pipeline
signal last_fifo : std_logic_vector(1 downto 0) := (others => '0');
signal wptr, rptr : integer range 0 to 1 := 0;  -- read/write pointers
signal fifo_count : integer range 0 to 2 := 0;  -- how many items in FIFO

begin

-- Simple 2-entry FIFO to delay "last" signal to match filter pipeline
process(clk)
begin
  if rising_edge(clk) then
    if rst_n='0' then
      wptr <= 0; rptr <= 0; fifo_count <= 0;
      fifo_last <= '0';
    else
      -- Write when input is valid and FIFO not full
      if i_valid='1' and fifo_count<2 then
        last_fifo(wptr) <= i_last_data;
        wptr <= (wptr+1) mod 2;
        fifo_count <= fifo_count + 1;
      end if;
      -- Read when output is valid and FIFO not empty
      if valid='1' and fifo_count>0 then
        fifo_last <= last_fifo(rptr);
        rptr <= (rptr+1) mod 2;
        fifo_count <= fifo_count - 1;
      else
        fifo_last <= '0';
      end if;
    end if;
  end if;
end process;

  -- Drive outputs
  o_data      <= std_logic_vector(vs(0));  -- current filter output
  o_valid     <= valid;                    -- delayed valid signal
  o_last_data <= not i_valid and valid;   -- last flag when input stops but output still valid

  -- IIR difference equation: y[n] = B0*x[n] + B1*x[n-1] + B2*x[n-2] - A0*y[n-1] - A1*y[n-2]
  -- (Note: truncated from 24-bit multiply results to 16-bit output)
    vs(0) <= mult_b0_1(22 downto 7)
          + mult_b1_1(22 downto 7)
          + mult_b2_1(22 downto 7)
          - mult_a0_1(22 downto 7)
          - mult_a1_1(22 downto 7);

  -- Alpha filter coefficients (first set)
  g_alpha_1: if G_FILTER_TYPE = ALPHA generate
    mult_b0_1 <= ve(2) * COEFF_IIR_CF_B0_0;  -- B0 * x[n-2]
    mult_b1_1 <= ve(1) * COEFF_IIR_CF_B1_0;  -- B1 * x[n-1]
    mult_b2_1 <= ve(0) * COEFF_IIR_CF_B2_0;  -- B2 * x[n]
    mult_a0_1 <= vs(2) * COEFF_IIR_CF_A0_0;  -- A0 * y[n-2]
    mult_a1_1 <= vs(1) * COEFF_IIR_CF_A1_0;  -- A1 * y[n-1]
  end generate g_alpha_1;

  -- Beta filter coefficients (second set)
  g_beta_1: if G_FILTER_TYPE = BETA generate
    mult_b0_1 <= ve(2) * COEFF_IIR_CF_B0_1;  -- B0 * x[n-2]
    mult_b1_1 <= ve(1) * COEFF_IIR_CF_B1_1;  -- B1 * x[n-1]
    mult_b2_1 <= ve(0) * COEFF_IIR_CF_B2_1;  -- B2 * x[n]
    mult_a0_1 <= vs(2) * COEFF_IIR_CF_A0_1;  -- A0 * y[n-2]
    mult_a1_1 <= vs(1) * COEFF_IIR_CF_A1_1;  -- A1 * y[n-1]
  end generate g_beta_1;   

  -- Pipeline the valid signal by one clock
  p_valid : process(clk)
  begin
    if rising_edge(clk) then
      if rst_n = '0' then
        valid <= '0';
      else
        valid <= i_valid;
      end if;
    end if;
  end process p_valid;

  -- Input shift register: maintains history of last 3 input samples
  g_shift_reg_ve : for i in 0 to 2 generate

    -- ve(0): load new input sample when valid
    g0 : if i = 0 generate
      p_reg0 : process(clk)
      begin
        if rising_edge(clk) then
          if rst_n = '0' then
            ve(0) <= (others => '0');
          elsif i_valid = '1' then
            ve(0) <= signed(i_data);  -- current input
          else
            ve(0) <= (others => '0');
          end if;
        end if;
      end process p_reg0;
    end generate g0;

    -- ve(i>0): shift previous values down the pipeline
    g1 : if i > 0 generate
      p_regn : process(clk)
      begin
        if rising_edge(clk) then
          if rst_n = '0' then
            ve(i) <= (others => '0');
          elsif i_valid = '1' then
            ve(i) <= ve(i-1);  -- shift history
          else
            ve(i) <= (others => '0');
          end if;
        end if;
      end process p_regn;
    end generate g1;

  end generate g_shift_reg_ve;

  -- Output shift register: maintains history of last 2 output samples
  p_vs1 : process(clk)
  begin
    if rising_edge(clk) then
      if rst_n = '0' then
        vs(1) <= (others => '0');
      elsif i_valid = '1' then
        vs(1) <= vs(0);  -- y[n-1] = previous y[n]
      else
        vs(1) <= (others => '0');
      end if;
    end if;
  end process p_vs1;

  p_vs2 : process(clk)
  begin
    if rising_edge(clk) then
      if rst_n = '0' then
        vs(2) <= (others => '0');
      elsif i_valid = '1' then
        vs(2) <= vs(1);  -- y[n-2] = previous y[n-1]
      else
        vs(2) <= (others => '0');
      end if;
    end if;
  end process p_vs2;

end architecture rtl;
