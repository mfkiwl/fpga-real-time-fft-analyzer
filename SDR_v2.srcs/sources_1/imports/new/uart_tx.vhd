library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- UART transmitter (8N1 format) with start/stop bits and handshaking
entity uart_tx is
  generic (
    G_BAUDRATE                : positive := 230400;  -- Baud rate (bits/sec)
    G_OPERATING_FREQUENCY_MHZ : positive := 8        -- Clock frequency (MHz)
  );
  port (
    i_clk       : in  std_logic;                         -- System clock
    i_rst_n     : in  std_logic;                         -- Active-low sync reset
    i_tx_start  : in  std_logic;                         -- Pulse to start transmission
    i_data_byte : in  std_logic_vector(7 downto 0);      -- Byte to transmit (LSB first)
    o_serial    : out std_logic;                         -- Serial TX output
    o_tx_busy   : out std_logic;                         -- High while transmitting
    o_tx_done   : out std_logic                          -- One-cycle pulse when done
  );
end entity uart_tx;

architecture rtl of uart_tx is

  -- Calculate how many clock cycles per bit period
  constant NB_CLKS_PER_BIT : integer := (G_OPERATING_FREQUENCY_MHZ * 1_000_000) / G_BAUDRATE;

  -- 4-state FSM for UART transmission sequence
  type state_t is (IDLE, START, DATA, STOP);

  signal state       : state_t := IDLE;                       -- current FSM state
  signal clk_count   : integer range 0 to NB_CLKS_PER_BIT-1 := 0; -- bit timing counter
  signal bit_count   : integer range 0 to 7 := 0;             -- which data bit we're on
  signal shift_reg   : std_logic_vector(7 downto 0) := (others => '0'); -- holds byte to send
  signal serial_out  : std_logic := '1';                      -- actual TX line state
  signal busy_int    : std_logic := '0';                      -- internal busy flag
  signal done_int    : std_logic := '0';                      -- internal done pulse

begin

  -- Connect internal signals to outputs
  o_serial  <= serial_out;
  o_tx_busy <= busy_int;
  o_tx_done <= done_int;

  -- Main UART TX state machine
  process(i_clk)
  begin
    if rising_edge(i_clk) then
      if i_rst_n = '0' then
        -- Reset to idle state
        state      <= IDLE;
        clk_count  <= 0;
        bit_count  <= 0;
        shift_reg  <= (others => '0');
        serial_out <= '1';    -- UART idle state is high
        busy_int   <= '0';
        done_int   <= '0';
      else
        done_int <= '0';  -- clear done pulse each cycle

        case state is
          when IDLE =>
            -- Wait for start trigger, keep line high
            serial_out <= '1';
            busy_int   <= '0';
            if i_tx_start = '1' then
              shift_reg <= i_data_byte;  -- latch the byte to send
              state     <= START;
              clk_count <= 0;
              busy_int  <= '1';
            end if;

          when START =>
            -- Send start bit (low) for one bit period
            serial_out <= '0';
            if clk_count = NB_CLKS_PER_BIT-1 then
              clk_count <= 0;
              state     <= DATA;
              bit_count <= 0;
            else
              clk_count <= clk_count + 1;
            end if;

          when DATA =>
            -- Send 8 data bits, LSB first
            serial_out <= shift_reg(bit_count);
            if clk_count = NB_CLKS_PER_BIT-1 then
              clk_count <= 0;
              if bit_count = 7 then
                state <= STOP;  -- sent all 8 bits
              else
                bit_count <= bit_count + 1;  -- next bit
              end if;
            else
              clk_count <= clk_count + 1;
            end if;

          when STOP =>
            -- Send stop bit (high) for one bit period, then done
            serial_out <= '1';
            if clk_count = NB_CLKS_PER_BIT-1 then
              state    <= IDLE;
              done_int <= '1';    -- pulse to signal completion
            else
              clk_count <= clk_count + 1;
            end if;

        end case;
      end if;
    end if;
  end process;

end architecture rtl;
