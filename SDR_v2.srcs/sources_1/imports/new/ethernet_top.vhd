library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity ethernet_top is
  port (
    -- RMII PHY I/O
    phy_rxd         : in  std_logic_vector(1 downto 0);
    phy_crs_dv      : in  std_logic;
    phy_tx_en       : out std_logic;
    phy_txd         : out std_logic_vector(1 downto 0);
    ref_clk         : out std_logic;                       -- exported 50MHz RMII ref clock
    phy_mdio        : inout std_logic;

    mac_byte        : in  std_logic_vector(7 downto 0); -- Byte from payload ROM to TX FIFO
    mac_rd_en       : out std_logic;                     -- Read enable for TX FIFO
    tx_fifo_empty   : in  std_logic;                     -- TX FIFO empty flag
    tx_valid        : in std_logic;                     -- Pulse to start TX

    -- Debug LEDs
    led_s           : out std_logic_vector(1 downto 0);
    led             : out std_logic_vector(12 downto 0);

    -- Global reset/clock
    reset_n           : in  std_logic;                       -- active-high async reset input
    rst_b           : in std_logic;
    sys_clk         : in  std_logic;                        -- system clock (to derive ref clock, VIO)
    clk             : out std_logic                        -- derived system clock (50MHz)
  );
end ethernet_top;

architecture Structural of ethernet_top is

  -- RMII PHY interface (TX/RX handling, CRC hookup, nibble serialization)
  component phy_rmii_if is
    port (
      phy_rxd           : in  std_logic_vector(1 downto 0);
      phy_crs_dv        : in  std_logic;
      phy_tx_en         : out std_logic;
      phy_txd           : out std_logic_vector(1 downto 0);
      phy_ref_clk       : in  std_logic;
      phy_mdio          : inout std_logic;

      -- TX payload/header FIFOs
      tx_fifo_empty     : in  std_logic;
      tx_valid          : in  std_logic;                     -- Pulse to start TX
      mac_tx_byte       : in  std_logic_vector(7 downto 0);
      mac_rd_en         : out std_logic;

      -- RX capture FIFO
      fifo_wr_en        : out std_logic;
      fifo_full         : in  std_logic;
      fifo_din          : out std_logic_vector(7 downto 0);

      -- Control
      crc_gen_en        : out std_logic;
      init_tx           : out std_logic;

      -- Status/CRC
      crc_data_valid    : out std_logic;
      frame_ready       : out std_logic;
      crc_ok            : out std_logic;
      init              : out std_logic;
      crc_gen_data      : out std_logic_vector(7 downto 0);
      crc_data_in       : out std_logic_vector(7 downto 0);
      led_out           : out std_logic_vector(12 downto 0);
      led_s             : out std_logic_vector(1 downto 0);
      crc_tx            : in  std_logic_vector(31 downto 0);
      computed_crc32    : in  std_logic_vector(31 downto 0);

      -- Resets/clocks
      reset_b           : in  std_logic;
      reset_n           : in  std_logic;
      sys_clk           : in  std_logic
    );
  end component;

  -- Simple dual-clock FIFO (used single-clock here)
  component fifo is
    generic (
      G_ADDR_WIDTH : positive := 9;
      G_DATA_WIDTH : positive := 32
    );
    port (
      i_write_clk  : in  std_logic;
      i_write_rstn : in  std_logic;
      i_write_en   : in  std_logic;
      i_write_data : in  std_logic_vector(G_DATA_WIDTH-1 downto 0);
      o_full       : out std_logic;

      i_read_clk   : in  std_logic;
      i_read_rstn  : in  std_logic;
      i_read_en    : in  std_logic;
      o_read_data  : out std_logic_vector(G_DATA_WIDTH-1 downto 0);
      o_empty      : out std_logic
    );
  end component;

  -- RX-side CRC32 checker (expects bit-reversed byte mapping)
  component crc is
    port (
      data_rev : in  std_logic_vector(7 downto 0);
      crc_en   : in  std_logic;
      rst      : in  std_logic;
      clk      : in  std_logic;
      init     : in  std_logic;
      crc_out  : out std_logic_vector(31 downto 0)
    );
  end component;

  -- TX-side CRC32 generator
  component crc_gen is
    port (
      datain  : in  std_logic_vector(7 downto 0);
      crc_en  : in  std_logic;
      rst     : in  std_logic;
      clk     : in  std_logic;
      init    : in  std_logic;
      crc_out : out std_logic_vector(31 downto 0)
    );
  end component;

  -- Clocking wizard: derive 50MHz RMII reference from sys_clk
  component clk_wiz_1 is
    port (
      clk_out1 : out std_logic;
      clk_out2 : out std_logic;
      resetn   : in  std_logic;
      locked   : out std_logic;
      clk_in1  : in  std_logic
    );
  end component;

  -- Resets/clocks
  --signal rst_b           : std_logic := '1';               -- active-low reset (RMII domain)
  signal phy_ref_clk     : std_logic := '0';               -- 50MHz ref clock to PHY
  signal ref_clk_1       : std_logic := '0';
  signal locked          : std_logic := '0';

  -- TX payload/header FIFOs and control
  signal mac_rx_rd_en    : std_logic := '0';               -- RX FIFO read (unused here)
  signal tx_fifo_full    : std_logic := '0';
  --signal mac_rd_en       : std_logic := '0';               -- TX FIFO read enable (asserted by RMII IF)
  --signal tx_fifo_empty   : std_logic := '0';

  -- RX capture FIFO
  signal phy_wr_en       : std_logic := '0';
  signal rx_fifo_full    : std_logic := '0';
  signal rx_fifo_empty   : std_logic := '0';

  -- CRC/status
  signal crc_data_valid  : std_logic := '0';
  signal frame_ready     : std_logic := '0';
  signal crc_ok          : std_logic := '0';
  signal init            : std_logic := '0';
  signal crc_gen_en      : std_logic := '0';
  signal init_tx         : std_logic := '0';

  -- PHY I/O staging
  signal phy_crs_dv_sig  : std_logic := '0';
  signal phy_tx_en_sig   : std_logic := '0';
  signal phy_rxd_sig     : std_logic_vector(1 downto 0) := (others => '0');
  signal phy_txd_sig     : std_logic_vector(1 downto 0) := (others => '0');

  -- Data paths
  signal crc_data_in     : std_logic_vector(7 downto 0) := (others => '0'); -- RX byte to CRC checker
  --signal mac_byte        : std_logic_vector(7 downto 0) := (others => '0'); -- TX FIFO -> RMII IF
  signal mac_rx_byte     : std_logic_vector(7 downto 0) := (others => '0'); -- RMII IF -> RX FIFO
  signal rx_byte         : std_logic_vector(7 downto 0) := (others => '0'); -- RX FIFO -> (unused)
  signal crc_gen_data    : std_logic_vector(7 downto 0) := (others => '0'); -- byte to TX CRC generator
  signal crc_tx          : std_logic_vector(31 downto 0) := (others => '0');-- generated TX CRC
  signal computed_crc32  : std_logic_vector(31 downto 0) := (others => '0');-- RX computed CRC

  -- LEDs/debug
  signal led_debug       : std_logic_vector(1 downto 0) := (others => '0');

  -- ATTRIBUTE MARK_DEBUG : STRING;
  -- ATTRIBUTE MARK_DEBUG OF phy_crs_dv_sig : SIGNAL IS "true";
  -- ATTRIBUTE MARK_DEBUG OF phy_rxd_sig    : SIGNAL IS "true";
  -- ATTRIBUTE MARK_DEBUG OF mac_rx_byte    : SIGNAL IS "true";
  -- ATTRIBUTE MARK_DEBUG OF phy_wr_en      : SIGNAL IS "true";
  -- ATTRIBUTE MARK_DEBUG OF start_frame    : SIGNAL IS "true";
  -- --ATTRIBUTE MARK_DEBUG OF tx_fifo_empty  : SIGNAL IS "true";
  -- ATTRIBUTE MARK_DEBUG OF head_fifo_empty: SIGNAL IS "true";
  -- ATTRIBUTE MARK_DEBUG OF head_rd_en     : SIGNAL IS "true";
  -- --ATTRIBUTE MARK_DEBUG OF mac_byte       : SIGNAL IS "true";
  -- --ATTRIBUTE MARK_DEBUG OF mac_rd_en      : SIGNAL IS "true";
  -- ATTRIBUTE MARK_DEBUG OF mac_tx         : SIGNAL IS "true";
  -- ATTRIBUTE MARK_DEBUG OF mac_wr_en      : SIGNAL IS "true";
  -- ATTRIBUTE MARK_DEBUG OF crc_gen_data   : SIGNAL IS "true";
  -- ATTRIBUTE MARK_DEBUG OF crc_gen_en     : SIGNAL IS "true";
  -- ATTRIBUTE MARK_DEBUG OF crc_tx         : SIGNAL IS "true";
  -- ATTRIBUTE MARK_DEBUG OF phy_tx_en_sig  : SIGNAL IS "true";
  -- ATTRIBUTE MARK_DEBUG OF phy_txd_sig    : SIGNAL IS "true";

begin
  -- MDIO tri-state (not driven here)
  phy_mdio <= 'Z';

  -- Clock distribution
  ref_clk     <= ref_clk_1;               -- export ref clock
  phy_ref_clk <= ref_clk_1;               -- feed RMII core

  -- LED status: frame_ready and CRC OK
  led_s(0) <= crc_ok and frame_ready;
  led_s(1) <= frame_ready;

  -- Stage PHY signals
  phy_crs_dv_sig <= phy_crs_dv;
  phy_rxd_sig    <= phy_rxd;
  phy_txd        <= phy_txd_sig;
  phy_tx_en      <= phy_tx_en_sig;

  -- RMII interface: handles TX preamble/SFD, nibble serialization, RX assembly,
  -- CRC generation/checking, and FIFO handshakes.
  eth_phy: phy_rmii_if
    port map(
      phy_rxd           => phy_rxd_sig,
      phy_crs_dv        => phy_crs_dv_sig,
      phy_tx_en         => phy_tx_en_sig,
      phy_txd           => phy_txd_sig,
      phy_ref_clk       => phy_ref_clk,
      phy_mdio          => phy_mdio,

      tx_fifo_empty     => tx_fifo_empty,
      tx_valid          => tx_valid,                     -- Pulse to start TX
      mac_tx_byte       => mac_byte,
      mac_rd_en         => mac_rd_en,

      fifo_wr_en        => phy_wr_en,
      fifo_full         => rx_fifo_full,
      fifo_din          => mac_rx_byte,

      crc_gen_en        => crc_gen_en,
      init_tx           => init_tx,

      crc_data_valid    => crc_data_valid,
      frame_ready       => frame_ready,
      crc_ok            => crc_ok,
      init              => init,
      crc_gen_data      => crc_gen_data,
      crc_data_in       => crc_data_in,
      led_out           => led,
      led_s             => led_debug,
      crc_tx            => crc_tx,
      computed_crc32    => computed_crc32,

      reset_b           => rst_b,
      reset_n           => reset_n,
      sys_clk           => sys_clk
    );

  -- RX capture FIFO: RMII IF -> FIFO
  cdc_rx: fifo
    generic map(
      G_ADDR_WIDTH => 9,
      G_DATA_WIDTH => 8
    )
    port map(
      i_write_clk  => phy_ref_clk,
      i_write_rstn => rst_b,
      i_write_en   => phy_wr_en,
      i_write_data => mac_rx_byte,
      o_full       => rx_fifo_full,

      i_read_clk   => phy_ref_clk,
      i_read_rstn  => rst_b,
      i_read_en    => mac_rx_rd_en,         -- left '0' by default (no consumer here)
      o_read_data  => rx_byte,
      o_empty      => rx_fifo_empty
    );

  -- RX CRC checker (byte-wise enable)
  crc_check: crc
    port map(
      clk      => phy_ref_clk,
      rst      => rst_b,
      data_rev => crc_data_in,          -- RMII IF provides byte in required bit order
      init     => init,                 -- pulse at frame end to reload preset
      crc_en   => crc_data_valid,       -- advance per valid RX byte
      crc_out  => computed_crc32
    );

  -- TX CRC generator (bytes from headers+payload)
  crc_ge: crc_gen
    port map(
      clk     => phy_ref_clk,
      rst     => rst_b,
      datain  => crc_gen_data,          -- byte stream to include in CRC
      init    => init_tx,               -- init at start-of-frame in TX path
      crc_en  => crc_gen_en,            -- qualify bytes
      crc_out => crc_tx
    );

  -- Clock wizard to generate 50MHz reference for RMII
  clk_50m: clk_wiz_1
    port map(
      clk_out1 => ref_clk_1,
      clk_out2 => clk,
      resetn   => reset_n,
      locked   => locked,
      clk_in1  => sys_clk
    );

end Structural;
