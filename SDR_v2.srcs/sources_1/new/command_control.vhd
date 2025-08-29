library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

-- Command control module: processes UART commands to control filter modes and data routing
entity command_control is
  Port (
    clk                 : in std_logic;                    -- main processing clock
    clk_b               : in std_logic;                    -- UART/control clock
    rst_n               : in std_logic;                    -- main reset
    rst_b               : in std_logic;                    -- UART reset
    uart_rx_valid       : in std_logic;                    -- UART received byte valid
    window_valid        : in std_logic;                    -- windowed data valid
    busy                : in std_logic;                    -- system busy flag
    filter_o_valid      : in std_logic;                    -- filter 0 output valid
    filter_o_valid_1    : in std_logic;                    -- filter 1 output valid
    filtered_signal     : in std_logic_vector(15 downto 0); -- filter 0 output data
    filtered_signal_1   : in std_logic_vector(15 downto 0); -- filter 1 output data
    uart_rx_data        : in std_logic_vector(7 downto 0);  -- UART command byte
    window_data         : in std_logic_vector(15 downto 0); -- windowed input data
    filter_valid        : out std_logic;                   -- enable filter 0
    filter_valid_1      : out std_logic;                   -- enable filter 1
    fft_in_valid        : out std_logic;                   -- FFT input valid
    reset_n_pulse       : out std_logic;                   -- reset pulse output
    start_aq            : out std_logic;                   -- start acquisition pulse
    fft_input           : out std_logic_vector(31 downto 0) -- FFT input data (32-bit)
  );
end command_control;

architecture Behavioral of command_control is

signal filter_mode : std_logic_vector(7 downto 0):= x"B1"; -- current filter mode (default bypass)
signal fft_in16 : std_logic_vector(15 downto 0):= (others => '0'); -- 16-bit data to FFT
signal fft_valid : std_logic:= '0';                         -- internal FFT valid signal
signal filter_in_valid  : std_logic:= '0';                 -- filter 0 enable signal
signal filter_in_valid_1  : std_logic:= '0';               -- filter 1 enable signal

-- Start and reset control signals
signal start     : std_logic:= '0';                         -- start command received
signal start_sig : std_logic:= '0';                         -- previous start state
signal reset_n   : std_logic:= '1';                         -- reset command state
signal reset_sig : std_logic_vector(1 downto 0):= (others => '1'); -- reset pulse generator

begin

-- Command decoder: processes UART commands in control clock domain
process(clk_b)
begin
    if rising_edge(clk_b) then
        if rst_b = '0' then
            filter_mode <= x"B1";  -- default to bypass mode
        elsif (uart_rx_valid and (not busy)) = '1' then   
            -- Process commands only when not busy
            if uart_rx_data = x"00" then
                filter_mode <= uart_rx_data;  -- mode 0: use filter 0 output
            elsif uart_rx_data = x"A1" then
                filter_mode <= uart_rx_data;  -- mode A1: use filter 1 output  
            elsif uart_rx_data = x"B1" then
                filter_mode <= uart_rx_data;  -- mode B1: bypass filters
            elsif uart_rx_data = x"FF" then
                reset_n <= '0';               -- reset command
            elsif uart_rx_data = x"55" then
                start <= '1';                 -- start acquisition command
            end if;
        else
            reset_n <= '1';    -- release reset when not commanded
            start   <= '0';    -- clear start when not commanded
            filter_mode <= filter_mode; -- maintain current mode
        end if;
        -- Edge detection registers for pulse generation
        start_sig <= start;
        reset_sig(0) <= reset_n;
        reset_sig(1) <= reset_sig(0);
    end if;
    end process;

-- Generate single-cycle pulses from level signals
start_aq <= (not start_sig) and start;     -- rising edge of start
reset_n_pulse <= reset_sig(1);             -- delayed reset signal

-- Data routing: selects which data goes to FFT based on filter mode
process(clk)
begin
    if rising_edge(clk) then
        if rst_n = '0' then
            fft_valid <= '0';
            fft_in16     <= (others => '0');
            filter_in_valid <= '0';
            filter_in_valid_1 <= '0';
        else
            if filter_mode = x"00" then
                -- Mode 00: route filter 0 output to FFT, enable filter 0
                fft_in16 <= filtered_signal;
                fft_valid <= filter_o_valid;
                filter_in_valid <= window_valid;
                filter_in_valid_1 <= '0';

            elsif filter_mode = x"A1" then
                -- Mode A1: route filter 1 output to FFT, enable filter 1
                fft_in16 <= filtered_signal_1;
                fft_valid <= filter_o_valid_1;
                filter_in_valid <= '0';
                filter_in_valid_1 <= window_valid;

            elsif filter_mode = x"B1" then
                -- Mode B1: bypass filters, route windowed data directly to FFT
                fft_in16 <= window_data;
                fft_valid <= window_valid;
                filter_in_valid <= '0';
                filter_in_valid_1 <= '0';

            else
                -- Default: same as bypass mode
                fft_in16 <= window_data;
                fft_valid <= window_valid;
                filter_in_valid <= '0';
                filter_in_valid_1 <= '0';
            end if;
        end if;
    end if;
        end process;

-- Output assignments: extend 16-bit data to 32-bit for FFT
fft_input <= x"0000" & fft_in16;    -- zero-pad to 32 bits
fft_in_valid <= fft_valid;

-- Connect filter enable signals
filter_valid <= filter_in_valid;
filter_valid_1 <= filter_in_valid_1;

end Behavioral;
