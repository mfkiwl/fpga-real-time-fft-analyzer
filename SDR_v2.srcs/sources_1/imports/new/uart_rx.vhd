library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- UART receiver (8N1 format) with start bit detection and byte assembly
entity uart_rx is
  generic (
    G_BAUDRATE                : positive := 230400;       -- Baud rate (bits/sec)
    G_OPERATING_FREQUENCY_MHZ : positive := 8             -- Clock frequency (MHz)
  );
  port (
    i_clk       : in  std_logic;                         -- System clock
    i_rst_n     : in  std_logic;                         -- Active-low synchronous reset
    i_serial    : in  std_logic;                         -- Serial RX input
    o_data_byte : out std_logic_vector(7 downto 0);      -- Received byte (LSB first)
    o_rx_done   : out std_logic                          -- High for one clk when byte ready
  );
end entity uart_rx;

architecture rtl of uart_rx is

  -- Calculate clock cycles per bit period
  constant NB_CLKS_PER_BIT : integer := (G_OPERATING_FREQUENCY_MHZ * 1_000_000) / G_BAUDRATE;

  -- 4-state FSM for UART reception sequence
  type state_t is (IDLE, START, DATA, STOP);

  signal state       : state_t := IDLE;                      -- current FSM state
  signal clk_count   : integer range 0 to NB_CLKS_PER_BIT-1 := 0;  -- bit timing counter
  signal bit_count   : integer range 0 to 7 := 0;            -- which data bit we're receiving
  signal shift_reg   : std_logic_vector(7 downto 0) := (others => '0'); -- assembles received byte
  signal rx_done_int : std_logic := '0';                     -- internal done pulse

begin

  -- Connect internal signals to outputs
  o_data_byte <= shift_reg;
  o_rx_done   <= rx_done_int;

  -- Main UART RX state machine
  process(i_clk)
  begin
    if rising_edge(i_clk) then
      if i_rst_n = '0' then
        -- Reset to idle state
        state       <= IDLE;
        clk_count   <= 0;
        bit_count   <= 0;
        shift_reg   <= (others => '0');
        rx_done_int <= '0';
      else
        rx_done_int <= '0';  -- clear done pulse each cycle

        case state is

          when IDLE =>
            -- Wait for start bit (line goes low)
            if i_serial = '0' then
              state     <= START;
              clk_count <= 0;
            end if;

          when START =>
            -- Sample halfway through start bit to verify it's real
            if clk_count = (NB_CLKS_PER_BIT/2 - 1) then
              if i_serial = '0' then
                -- Good start bit, proceed to data
                state     <= DATA;
                clk_count <= 0;
                bit_count <= 0;
              else
                -- False start (noise), go back to idle
                state <= IDLE;
              end if;
            else
              clk_count <= clk_count + 1;
            end if;

          when DATA =>
            -- Sample each data bit at the end of its bit period
            if clk_count = NB_CLKS_PER_BIT-1 then
              clk_count                    <= 0;
              shift_reg(bit_count)         <= i_serial;  -- capture bit LSB first
              if bit_count = 7 then
                state    <= STOP;  -- got all 8 bits
              else
                bit_count <= bit_count + 1;  -- next bit
              end if;
            else
              clk_count <= clk_count + 1;
            end if;

          when STOP =>
            -- Wait through stop bit period, then signal byte complete
            if clk_count = NB_CLKS_PER_BIT-1 then
              state       <= IDLE;
              rx_done_int <= '1';  -- pulse to signal byte received
            else
              clk_count <= clk_count + 1;
            end if;

          when others =>
            state <= IDLE;  -- safety fallback

        end case;
      end if;
    end if;
  end process;

end architecture rtl;
