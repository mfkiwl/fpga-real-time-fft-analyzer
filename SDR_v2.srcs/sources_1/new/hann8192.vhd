library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

use work.hann.all;

-- Hann window function: multiplies input samples by windowing coefficients for FFT preprocessing
entity hann_window is
    generic (
        N : integer := 8192               -- FFT length (window size)
    );
    port (
        clk        : in  std_logic;       -- sample clock
        rst_n        : in  std_logic;       -- synchronous reset, active-low
        sample_en  : in  std_logic;       -- pulse when sample_in is valid
        sample_in  : in  signed(15 downto 0);   -- raw ADC sample
        valid      : out std_logic;       -- output valid flag
        sample_out : out std_logic_vector(15 downto 0)    -- windowed sample output
    );
end entity;

architecture rtl of hann_window is
    signal addr : unsigned(13 downto 0) := (others => '0'); -- address counter for ROM lookup
    signal coef_s   : signed(15 downto 0);                  -- current window coefficient
    signal product  : signed(31 downto 0);                  -- multiplication result
begin
    -- Main windowing process: multiply sample by Hann coefficient
    process(clk)
    begin
        if rising_edge(clk) then
            if rst_n ='0' then
                sample_out <= (others=>'0');
                valid <= '0';
            elsif sample_en='1' then
                -- Get windowing coefficient from ROM (Hann coefficients 0-65535 scaled)
                coef_s  <= signed(hann_rom(to_integer(addr))); 
                product <= sample_in * coef_s;                 -- multiply sample by window coeff
                -- Extract upper bits with rounding for 16-bit output
                sample_out <= std_logic_vector(resize(product(31 downto 15) + product(14), 16));
                valid <= '1';                                  -- signal valid output
                addr <= addr + 1;                              -- move to next coefficient
            else
                sample_out <= (others=>'0');                  -- output zero when idle
                valid <= '0';
            end if;
        end if;
    end process;
end architecture;
