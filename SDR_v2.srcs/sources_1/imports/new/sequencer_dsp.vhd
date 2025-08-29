library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- Sequencer controls data acquisition, FFT processing, and FIFO writing
entity sequencer is
  Port (
    clk               : in  std_logic;                    -- System clock
    rst_n             : in  std_logic;                    -- Active-low synchronous reset
    fft_data          : in  std_logic_vector(31 downto 0);-- FFT output data
    fft_last          : in  std_logic;                    -- Last FFT data flag
    fill_afifo        : in  std_logic;                    -- Pulse to begin FIFO fill
    start             : in  std_logic;                    -- External start trigger
    start_acquisition : out std_logic;                    -- Assert to start modulator
    request_fft       : out std_logic;                    -- Pulse to read next FFT word
    fifo_wr_en        : out std_logic;                    -- Enable write into FIFO
    fifo_i_data_reg   : out std_logic_vector(31 downto 0);-- Data to FIFO input register
    fifo_full         : in  std_logic;                    -- FIFO full flag
    fft_valid         : in  std_logic                     -- FFT output valid
  );
end entity sequencer;

architecture Behavioral of sequencer is

  -- 3-state FSM: wait for start, acquire data, fill FIFO
  type state_type is (IDLE, ACQUIRE, FILL_FIFO);
  signal p_state : state_type := IDLE;

  signal fifo_data_reg    : std_logic_vector(31 downto 0) := (others => '0'); -- unused internal register

begin

  -- Main state machine and output control
  block_g : process(clk, rst_n, p_state)
  begin
    if rst_n = '0' then
      -- Reset everything to safe defaults
      p_state <= IDLE;
      start_acquisition <= '0';
      request_fft <= '0';
      fifo_wr_en <= '0';
      fifo_i_data_reg <= (others => '0');
    elsif rising_edge(clk) then
      -- Default all outputs to inactive
      start_acquisition <= '0';
      request_fft <= '0';
      fifo_wr_en <= '0';
      fifo_i_data_reg <= (others => '0');

      case p_state is
        when IDLE =>
          -- Wait for external start signal
          if start = '1' then
            p_state <= ACQUIRE;
          end if;
          fifo_wr_en        <= '0';

        when ACQUIRE =>
          -- Start data acquisition and request FFT processing
          -- Wait for FFT to complete and FIFO fill trigger
          if fft_last = '1' and fill_afifo = '1' then
            p_state <= FILL_FIFO;
          end if;
          start_acquisition <= '1';  -- keep acquisition running
          request_fft       <= '1';  -- keep requesting FFT data
          fifo_wr_en        <= '0';

        when FILL_FIFO =>
          -- Write FFT results to FIFO until done or FIFO full
          if fft_last = '1' or fifo_full = '1' then
            p_state <= ACQUIRE;  -- go back to acquiring more data
          end if;
          start_acquisition <= '1';       -- continue acquisition
          request_fft       <= '1';       -- keep requesting FFT data
          fifo_i_data_reg   <= fft_data;  -- stage FFT data for FIFO
          if fft_valid = '1' then
            fifo_wr_en <= '1';  -- write to FIFO when data is valid
          end if;

        when others =>
          p_state <= IDLE;  -- safety fallback
      end case;
    end if;
  end process block_g;

end architecture Behavioral;
