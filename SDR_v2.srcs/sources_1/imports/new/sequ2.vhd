library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- Dual-mode sequencer: handles FIFO reading and streaming over Ethernet or UART
entity sequ_2 is
  port (
    clk_b         : in  std_logic;                     -- Control/UART clock
    rst_b         : in  std_logic;                     -- Active-low reset
    tx_active     : in  std_logic;                     -- UART TX busy flag
    start_fill    : in  std_logic;                     -- trigger to start filling FIFO
    uart_rx_data  : in  std_logic_vector(7 downto 0);  -- Received UART byte
    uart_rx_valid : in  std_logic;                     -- High one cycle when RX byte ready
    uart_tx_done  : in  std_logic;                     -- High one cycle when TX finishes
    tx_data       : out std_logic_vector(7 downto 0);  -- Byte to transmit (Ethernet)
    uart_tx_data  : out std_logic_vector(7 downto 0);  -- Byte to transmit (UART)
    ether_req     : in  std_logic;                     -- Request to send next byte (Ethernet)
    tx_valid      : out std_logic;                     -- Pulse to start TX (Ethernet)
    uart_tx_valid : out std_logic;                     -- Pulse to start TX (UART)
    fifo_rd_en    : out std_logic;                     -- FIFO read enable
    fill_afifo    : out std_logic;                     -- Pulse to begin FIFO fill
    fifo_data_out : in  std_logic_vector(31 downto 0); -- Word read from FIFO
    fifo_empty    : in  std_logic;                       -- High when FIFO empty
    fifo_full     : in  std_logic;                     -- High when FIFO full
    ready_data    : out std_logic            -- High when ready to send next byte
  );
end entity sequ_2;

architecture Behavioral of sequ_2 is

  -- Ethernet mode state machine
  type t_state is (S_IDLE1, S_FILL, S_READ, S_SEND);
  signal state     : t_state := S_IDLE1;

  -- UART mode state machine  
  type u_state is (U_IDLE1, U_IDLE2, U_FILL, U_READ, U_SEND, U_WAIT);
  signal p_state     : u_state := U_IDLE1;

  -- Command detection signals
  signal prev_rx_v : std_logic := '0';            -- previous uart_rx_valid for edge detect
  signal cmd_valid : std_logic := '0';            -- pulse when new command received
  signal cmd_byte  : std_logic_vector(7 downto 0) := (others => '0'); -- latched command byte

  -- Data handling for both modes
  signal fifo_word : std_logic_vector(31 downto 0) := (others => '0'); -- current FIFO word (Ethernet)
  signal fifo_word_u : std_logic_vector(31 downto 0) := (others => '0'); -- current FIFO word (UART)
  signal byte_idx  : integer range 0 to 3 := 0;    -- byte index within word (Ethernet)
  signal byte_idx_u  : integer range 0 to 3 := 0;    -- byte index within word (UART)

  -- Output registers for both modes
  signal rd_en_r    : std_logic := '0';           -- FIFO read enable (Ethernet)
  signal rd_en_u    : std_logic := '0';           -- FIFO read enable (UART)
  signal fill_r     : std_logic := '0';           -- fill pulse (Ethernet)
  signal fill_u     : std_logic := '0';           -- fill pulse (UART)
  signal tx_data_r  : std_logic_vector(7 downto 0) := (others => '0'); -- TX data (Ethernet)
  signal tx_data_u  : std_logic_vector(7 downto 0) := (others => '0'); -- TX data (UART)
  signal tx_start_r : std_logic := '0';           -- TX start pulse (Ethernet)
  signal tx_start_u : std_logic := '0';           -- TX start pulse (UART)
  signal ready      : std_logic := '0';           -- ready flag
  signal ether_en   : std_logic := '1';           -- mode select: 1=Ethernet, 0=UART

begin

  -- Detect new command bytes (rising edge of uart_rx_valid)
  process(clk_b, rst_b)
  begin
    if rst_b = '0' then
      prev_rx_v <= '0';
      cmd_valid <= '0';
      cmd_byte  <= (others => '0');
    elsif rising_edge(clk_b) then
      prev_rx_v <= uart_rx_valid;
      if uart_rx_valid = '1' and prev_rx_v = '0' then
        cmd_valid <= '1';
        cmd_byte  <= uart_rx_data;
      else
        cmd_valid <= '0';
      end if;
    end if;
  end process;

  -- Mode selection based on commands (0xEF=Ethernet, 0xFE=UART)
  process(clk_b, rst_b)
  begin
    if rst_b = '0' then
      ether_en <= '1';  -- default to Ethernet mode
    elsif rising_edge(clk_b) then
      if cmd_valid = '1' and cmd_byte = x"EF" then
        ether_en <= '1';  -- switch to Ethernet mode
      elsif cmd_valid = '1' and cmd_byte = x"FE" then
        ether_en <= '0';  -- switch to UART mode
      else
        ether_en <= ether_en;  -- keep current mode
      end if;
    end if;
  end process;

  -- Ethernet mode FSM: auto-fill and stream data
  process(clk_b, rst_b)
  begin
    if rst_b = '0' then
      state      <= S_IDLE1;
      rd_en_r    <= '0';
      tx_data_r  <= (others => '0');
      tx_start_r <= '0';
      byte_idx   <= 0;
      fifo_word  <= (others => '0');
    elsif rising_edge(clk_b) then
      -- Default outputs
      rd_en_r    <= '0';
      tx_start_r <= '0';

    if ether_en = '1' then
      -- Auto-fill when FIFO is empty
      if fifo_empty = '1' then
        fill_r <= '1';
      else
        fill_r <= '0';
      end if;

      case state is
        when S_IDLE1 =>
          -- Wait for start trigger
          ready <= '0';
          if start_fill = '1' then
            state  <= S_FILL;
          end if;

        when S_FILL =>
          -- Move to read state after fill
          ready <= '0';
          state <= S_READ;

        when S_READ =>
          -- Read word from FIFO when it has data
          ready <= '0';
          tx_start_r <= '0';
          if fifo_full = '1' then
            rd_en_r   <= '1';
            fifo_word <= fifo_data_out;
            byte_idx  <= 0;
            state     <= S_SEND;
          else
            state <= S_FILL;  -- go back to fill if no data
          end if;

        when S_SEND =>
          -- Send bytes from current word when requested
          rd_en_r <= '0';
          tx_start_r <= '0';
          ready <= '1';  -- signal ready to send
          if ether_req = '1' then
            tx_data_r  <= fifo_word((byte_idx*8+7) downto (byte_idx*8)); -- extract byte
            tx_start_r <= '1';
            byte_idx <= byte_idx + 1;
            if byte_idx = 3 then  -- sent all 4 bytes
              if fifo_empty = '1' then
                ready <= '0';
                state    <= S_READ;
              else 
                rd_en_r   <= '1';  -- get next word
                fifo_word <= fifo_data_out;
                byte_idx  <= 0;
              end if;
            end if;
          else
            tx_start_r <= '0';
          end if;

        when others =>
          state <= S_IDLE1;
      end case;
    else
      state <= S_IDLE1;  -- idle when not in Ethernet mode
    end if;
    end if;
  end process;

  -- UART mode FSM: command-driven operation
  process(clk_b, rst_b)
  begin
    if rst_b = '0' then
      p_state      <= U_IDLE1;
      rd_en_u    <= '0';
      fill_u     <= '0';
      tx_data_u  <= (others => '0');
      tx_start_u <= '0';
      byte_idx_u   <= 0;
      fifo_word_u  <= (others => '0');
    elsif rising_edge(clk_b) then
      -- Default outputs
      rd_en_u    <= '0';
      fill_u     <= '0';
      tx_start_u <= '0';

    if ether_en = '0' then
      -- Auto-fill when FIFO is empty
      if fifo_empty = '1' then
        fill_u <= '1';
      else
        fill_u <= '0';
      end if;

      case p_state is
        when U_IDLE1 =>
          -- Wait for start trigger
          if start_fill = '1' then
            p_state  <= U_IDLE2;
          end if;

        when U_FILL =>
          p_state <= U_READ;

        when U_IDLE2 =>
          -- Wait for read command (0xA5)
          if cmd_valid = '1' and cmd_byte = x"A5" then
            p_state <= U_READ;
          end if;

        when U_READ =>
          -- Read word from FIFO if available
          if fifo_empty = '0' then
            rd_en_u   <= '1';
            fifo_word_u <= fifo_data_out;
            byte_idx_u  <= 0;
            p_state     <= U_SEND;
          else
            p_state <= U_FILL;  -- fill if no data
          end if;

        when U_SEND =>
          -- Send byte when UART is free
          if tx_active = '0' then
            tx_data_u  <= fifo_word_u((byte_idx_u*8+7) downto (byte_idx_u*8));
            tx_start_u <= '1';
            p_state      <= U_WAIT;
          end if;

        when U_WAIT =>
          -- Wait for TX completion, then send next byte or word
          if uart_tx_done = '1' then
            if byte_idx_u < 3 then
              byte_idx_u <= byte_idx_u + 1;
              p_state    <= U_SEND;
            else
              p_state    <= U_READ;  -- get next word
            end if;
          end if;

        when others =>
          p_state <= U_IDLE1;
      end case;
    else
      p_state <= U_IDLE1;  -- idle when in Ethernet mode
    end if;
    end if;
  end process;

  -- Output multiplexing based on current mode
  fifo_rd_en    <= rd_en_r when ether_en = '1' else rd_en_u;
  fill_afifo    <= fill_r when ether_en = '1' else fill_u;
  tx_data       <= tx_data_r;     -- Ethernet data output
  uart_tx_data  <= tx_data_u;     -- UART data output
  tx_valid      <= tx_start_r;    -- Ethernet TX trigger
  uart_tx_valid <= tx_start_u;    -- UART TX trigger
  ready_data    <= ready and ether_en;  -- ready signal only in Ethernet mode

end architecture Behavioral;
