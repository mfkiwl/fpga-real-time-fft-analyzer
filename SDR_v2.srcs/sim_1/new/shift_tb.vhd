-- tb_shift2to8.vhd
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tb_shift2to8 is
end entity;

architecture sim of tb_shift2to8 is
  component shift2to8
    port (
      clk      : in  std_logic;
      rst_n     : in  std_logic;
      in_valid  : in  std_logic;
      init     : in  std_logic;
      in_2b     : in  std_logic_vector(1 downto 0);
      out_valid : out std_logic;
      out_8b    : out std_logic_vector(7 downto 0)
    );
  end component;

  constant C_CLK : time := 20 ns;
  signal clk     : std_logic := '0';
  signal rst_n    : std_logic := '0';
  signal in_valid : std_logic := '0';
  signal init    : std_logic := '0';
  signal in_2b    : std_logic_vector(1 downto 0) := (others => '0');
  signal out_valid: std_logic;
  signal out_8b   : std_logic_vector(7 downto 0);
begin
  -- 50 MHz clock
  clk <= not clk after C_CLK/2;

  -- DUT
  dut: shift2to8
    port map (
      clk      => clk,
      rst_n     => rst_n,
      in_valid  => in_valid,
      init     => init,
      in_2b     => in_2b,
      out_valid => out_valid,
      out_8b    => out_8b
    );

  -- Reset then feed four 2-bit nibbles "01" -> expect 0x55
  stim: process
  begin
    rst_n <= '0';
    wait for 100 ns;
    rst_n <= '1';
    wait until rising_edge(clk);

    in_valid <= '1';
    for i in 0 to 3 loop
      in_2b <= "01";
      wait until rising_edge(clk);
    end loop;
    in_valid <= '0';

    -- Wait for assembled byte
    wait until out_valid = '1';
    assert out_8b = x"55"
      report "shift2to8: expected 0x55, got "
      severity error;

    report "tb_shift2to8 PASSED" severity note;
    wait;
  end process;
end architecture;
