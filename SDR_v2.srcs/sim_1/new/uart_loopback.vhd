library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tb_uart_loopback is end;
architecture sim of tb_uart_loopback is
  -----------------------------------------------------------------------------
  -- Component declarations identical to repository source
  -----------------------------------------------------------------------------
  component uart_tx
    generic ( G_OPERATING_FREQUENCY_MHZ : natural := 50_000_000;
              G_BAUDRATE   : natural := 230_400 );
    port ( i_clk      : in  std_logic;
           i_rst_n    : in  std_logic;
           i_tx_start : in  std_logic;
           i_data_byte  : in  std_logic_vector(7 downto 0);
           o_tx_busy  : out std_logic;
           o_tx_done  : out std_logic;
           o_serial       : out std_logic );
  end component;

  component uart_rx
    generic ( G_OPERATING_FREQUENCY_MHZ : natural := 50_000_000;
              G_BAUDRATE   : natural := 230_400 );
    port ( i_clk      : in  std_logic;
           i_rst_n    : in  std_logic;
           i_serial       : in  std_logic;
           o_data_byte  : out std_logic_vector(7 downto 0);
           o_rx_done : out std_logic );
  end component;
  ---------------------------------------------------------------------------

  constant i_clk_PER : time := 20 ns;      -- 50 MHz
  signal i_clk   : std_logic := '0';
  signal i_rst_n : std_logic := '0';

  -- TX side
  signal i_tx_start, o_tx_busy, o_tx_done : std_logic;
  signal i_data_byte                    : std_logic_vector(7 downto 0);
  signal tx_line                    : std_logic;

  -- RX side
  signal o_data_byte  : std_logic_vector(7 downto 0);
  signal o_rx_done : std_logic;

  -- stimulus vector
  type t_vec is array (natural range <>) of std_logic_vector(7 downto 0);
  constant PAT : t_vec := (x"55",x"A5",x"00",x"FF",x"5A");
begin
  i_clk <= not i_clk after i_clk_PER/2;
  i_rst_n <= '0', '1' after 200 ns;

  U_TX : uart_tx port map (i_clk,i_rst_n,i_tx_start,i_data_byte,o_tx_busy,o_tx_done,tx_line);
  U_RX : uart_rx port map (i_clk,i_rst_n,tx_line,o_data_byte,o_rx_done);

  stim : process
  begin
    wait until i_rst_n='1';
    for i in PAT'range loop
      -- wait until TX is free
      wait until o_tx_busy='0' and rising_edge(i_clk);
      i_data_byte  <= PAT(i);
      i_tx_start <= '1';
      wait until rising_edge(i_clk);
      i_tx_start <= '0';
      -- wait for character to finish and be received
      wait until o_rx_done='1';
      assert o_data_byte = PAT(i)
        report "UART mismatch at index "&integer'image(i)
        severity error;
    end loop;
    report "tb_uart_loopback PASSED" severity note;
    wait;
  end process;
end;
