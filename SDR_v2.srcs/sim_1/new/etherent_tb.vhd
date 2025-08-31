-- tb_ethernet_top.vhd  -------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tb_ethernet_top is
end entity;

architecture sim of tb_ethernet_top is
  ---------------------------------------------------------------------------
  --  Component declaration identical to DUT
  ---------------------------------------------------------------------------
  component ethernet_top
    port (
      phy_rxd        : in  std_logic_vector(1 downto 0);
      phy_crs_dv     : in  std_logic;
      phy_tx_en      : out std_logic;
      phy_txd        : out std_logic_vector(1 downto 0);
      ref_clk        : out std_logic;
      phy_mdio       : inout std_logic;

      mac_byte       : in  std_logic_vector(7 downto 0);
      mac_rd_en      : out std_logic;
      tx_fifo_empty  : in  std_logic;
      tx_valid       : in  std_logic;

      led_s          : out std_logic_vector(1 downto 0);
      led            : out std_logic_vector(12 downto 0);

      reset_n        : in  std_logic;      -- active-high reset
      rst_b          : in  std_logic;
      sys_clk        : in  std_logic;
      clk            : out std_logic
    );
  end component;
  ---------------------------------------------------------------------------

  ---------------------------------------------------------------------------
  --  Local clock & reset
  ---------------------------------------------------------------------------
  constant SYS_PER : time := 10 ns;  -- 100 MHz
  signal sys_clk   : std_logic := '0';
  signal clk_50    : std_logic;      -- driven by DUT
  signal reset_n   : std_logic := '0';
  signal rst_b     : std_logic := '0';   -- active-low branch reset

  ---------------------------------------------------------------------------
  --  RMII receive side (minimal stimulus)
  ---------------------------------------------------------------------------
  signal phy_rxd    : std_logic_vector(1 downto 0) := "00";
  signal phy_crs_dv : std_logic := '0';

  ---------------------------------------------------------------------------
  --  Transmit-side observations
  ---------------------------------------------------------------------------
  signal phy_tx_en  : std_logic;
  signal phy_txd    : std_logic_vector(1 downto 0);

  ---------------------------------------------------------------------------
  --  Payload-side handshake (held idle)
  ---------------------------------------------------------------------------
  signal mac_byte      : std_logic_vector(7 downto 0) := (others=>'0');
  signal mac_rd_en     : std_logic;
  signal tx_fifo_empty : std_logic := '1';
  signal tx_valid      : std_logic := '0';

  -- LEDs unused
  signal led_s  : std_logic_vector(1 downto 0);
  signal led    : std_logic_vector(12 downto 0);
  signal phy_mdio : std_logic := 'Z';

begin
  ---------------------------------------------------------------------------
  --  Free-running 100 MHz system clock
  ---------------------------------------------------------------------------
  sys_clk <= not sys_clk after SYS_PER/2;

  ---------------------------------------------------------------------------
  --  Instantiate DUT
  ---------------------------------------------------------------------------
  dut : ethernet_top
    port map (
      phy_rxd        => phy_rxd,
      phy_crs_dv     => phy_crs_dv,
      phy_tx_en      => phy_tx_en,
      phy_txd        => phy_txd,
      ref_clk        => open,
      phy_mdio       => phy_mdio,

      mac_byte       => mac_byte,
      mac_rd_en      => mac_rd_en,
      tx_fifo_empty  => tx_fifo_empty,
      tx_valid       => tx_valid,

      led_s          => led_s,
      led            => led,

      reset_n        => reset_n,
      rst_b          => rst_b,
      sys_clk        => sys_clk,
      clk            => clk_50
    );

  ---------------------------------------------------------------------------
  --  Reset sequence
  ---------------------------------------------------------------------------
  reset_n <= '0', '1' after 200 ns;
  rst_b   <= '0', '1' after 200 ns;

  ---------------------------------------------------------------------------
  --  RMII stimulus – feed 8 “01” nibbles (=> 0x55 pre-amble byte)
  ---------------------------------------------------------------------------
  stim : process
  begin
    wait until reset_n='1';

    -- Assert carrier and data-valid
    phy_crs_dv <= '1';

    for i in 0 to 7 loop                -- 8 nibbles = 4 RMII cycles
      phy_rxd <= "01";
      wait until rising_edge(sys_clk);
    end loop;

    -- De-assert RMII lines
    phy_crs_dv <= '0';
    phy_rxd    <= "00";

    -------------------------------------------------------------------------
    --  Wait for transmit enable pulse from DUT
    -------------------------------------------------------------------------
    wait for 2 us;                       -- ample time
    assert phy_tx_en = '1';

    wait;
  end process;

end architecture;
