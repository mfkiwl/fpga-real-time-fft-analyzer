library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- Top-level DSP system: ADC → windowing → filters → FFT → FIFO → UART/Ethernet output
entity dsp_system_top is
    Port (
        clk           : in  std_logic;                 -- System clock
        vauxp3        : in  std_logic;                 -- ADC input positive
        vauxn3        : in  std_logic;                 -- ADC input negative
        rst           : in  std_logic;                 -- Asynchronous reset (active high)
        phy_rxd       : in  std_logic_vector(1 downto 0); -- Ethernet RMII RX data
        phy_crs_dv    : in  std_logic;                 -- Ethernet RMII carrier sense
        phy_tx_en     : out std_logic;                 -- Ethernet RMII TX enable
        ref_clk       : out std_logic;                 -- Reference clock for PHY
        phy_txd       : out std_logic_vector(1 downto 0); -- Ethernet RMII TX data
        phy_mdio      : inout std_logic;                 -- Ethernet MDIO management
        led_s         : out std_logic_vector(1 downto 0); -- Status LEDs
        led           : out std_logic_vector(12 downto 0); -- Debug LEDs
        uart_rx_i     : in  std_logic;                 -- UART receive line
        uart_tx_o     : out std_logic                  -- UART transmit line
    );
end entity dsp_system_top;

architecture Behavioral of dsp_system_top is

    -- Reset and clock management
    signal rst_b               : std_logic := '0';     -- synchronized reset for clk_b domain
    signal locked              : std_logic := '0';     -- PLL lock indicator (unused)
    signal clk_b               : std_logic := '0';     -- buffered/derived clock from Ethernet
    signal busy                : std_logic := '0';     -- coefficient loading busy flag

    signal global_rst_n        : std_logic := '0';     -- global reset (inverted input)
    signal rst_n               : std_logic := '1';     -- active-low system reset
    signal rst_u               : std_logic := '1';     -- reset command from UART
    signal rst_u_sync          : std_logic := '1';     -- synchronized reset pulse
    signal rst_u_sig           : std_logic := '1';     -- delayed reset for edge detection

    -- Data path signals: ADC → window → filter → FFT
    signal filtered_signal     : std_logic_vector(15 downto 0) := (others => '0'); -- filter 0 output
    signal filtered_signal_1     : std_logic_vector(15 downto 0) := (others => '0'); -- filter 1 output
    signal adc_out             : std_logic_vector(15 downto 0) := (others => '0'); -- raw ADC data
    signal filter_input         : std_logic_vector(15 downto 0) := (others => '0'); -- input to filters

    -- FFT interface (32-bit complex samples)
    signal fft_input           : std_logic_vector(31 downto 0) := (others => '0'); -- to FFT core
    signal fft_output          : std_logic_vector(31 downto 0) := (others => '0'); -- from FFT core

    -- Control and valid signals throughout the pipeline
    signal filter_o_valid      : std_logic := '0';     -- filter 0 output valid
    signal filter_o_valid_1    : std_logic := '0';     -- filter 1 output valid
    signal filter_last         : std_logic := '0';     -- filter 0 end-of-frame
    signal filter_last_1       : std_logic := '0';     -- filter 1 end-of-frame
    signal filter_valid_in     : std_logic := '0';     -- filter input valid (unused)
    signal fft_ready_s         : std_logic := '0';     -- FFT ready to accept data
    signal fft_valid_s         : std_logic := '0';     -- FFT output valid
    signal fft_last            : std_logic := '0';     -- FFT end-of-frame
    signal fft_in_valid       : std_logic := '0';     -- FFT input valid

    -- Filter enable signals (controlled by command decoder)
    signal filter_valid        : std_logic := '0';     -- enable filter 0
    signal filter_valid_1      : std_logic := '0';     -- enable filter 1
    signal mod_last_s          : std_logic := '0';     -- modulator end-of-frame (unused)

    -- FIFO for clock domain crossing (clk → clk_b)
    signal fifo_i_data         : std_logic_vector(31 downto 0) := (others => '0'); -- FIFO input (unused)
    signal fifo_data_out       : std_logic_vector(31 downto 0) := (others => '0'); -- FIFO output
    signal fifo_wr_en          : std_logic := '0';     -- FIFO write enable
    signal fifo_rd_en          : std_logic := '0';     -- FIFO read enable
    signal fifo_full           : std_logic := '0';     -- FIFO full flag
    signal fifo_empty          : std_logic := '0';     -- FIFO empty flag
    signal fill_afifo_sync     : std_logic_vector(1 downto 0) := (others => '0'); -- sync registers
    signal fill_afifo_sig      : std_logic := '0';     -- FIFO fill trigger

    -- UART interface signals
    signal uart_rx_data        : std_logic_vector(7 downto 0) := (others => '0'); -- received byte
    signal uart_rx_valid       : std_logic := '0';     -- UART RX byte complete
    signal uart_tx_data        : std_logic_vector(7 downto 0) := (others => '0'); -- byte to send
    signal tx_data             : std_logic_vector(7 downto 0) := (others => '0'); -- Ethernet TX data
    signal uart_tx_valid       : std_logic := '0';     -- UART TX data valid (unused)
    signal uart_tx_busy        : std_logic := '0';     -- UART TX busy flag
    signal tx_start            : std_logic := '0';     -- UART TX start pulse
    signal tx_valid            : std_logic := '0';     -- Ethernet TX start pulse
    signal tx_done             : std_logic := '0';     -- UART TX done pulse
    signal ether_req           : std_logic := '0';     -- Ethernet byte request
    signal ready_data          : std_logic := '0';     -- ready to send next byte

    -- System control signals
    signal start_acquisition   : std_logic := '0';     -- start ADC sampling
    signal request_fft         : std_logic := '0';     -- request to read FFT output
    signal start_aq            : std_logic := '0';     -- synchronized start signal
    signal start_aq_sync       : std_logic := '0';     -- start signal from command decoder

    signal coeff_rx_valid     : std_logic := '0';     -- coefficient receive valid
    signal coeff_tx_valid     : std_logic := '0';     -- coefficient transmit valid
    signal adc_ready          : std_logic := '0';     -- ADC data ready signal

    -- Windowing signals
    signal windowed_sample     : std_logic_vector(15 downto 0) := (others => '0'); -- windowed ADC data
    signal adc_q16             : signed(15 downto 0) := (others => '0'); -- ADC in signed format
    signal window_valid        : std_logic := '0';     -- windowed sample valid flag

    signal fifo_sig            : std_logic_vector(1 downto 0) := (others => '0'); -- FIFO full sync
    signal s_axis_tlast        : std_logic := '0';     -- FFT last signal (unused)

    -- Default filter coefficients for both filter sets
    signal COEFF_IIR_CF_B0_0_sig : signed(7 downto 0) := to_signed(73, 8);
    signal COEFF_IIR_CF_B1_0_sig : signed(7 downto 0) := to_signed(-114, 8);
    signal COEFF_IIR_CF_B2_0_sig : signed(7 downto 0) := to_signed(127, 8);
    signal COEFF_IIR_CF_A0_0_sig : signed(7 downto 0) := to_signed(28, 8);
    signal COEFF_IIR_CF_A1_0_sig : signed(7 downto 0) := to_signed(-56, 8);
    signal COEFF_IIR_CF_A2_0_sig : signed(7 downto 0) := to_signed(28, 8);
    signal COEFF_IIR_CF_B0_1_sig : signed(7 downto 0) := to_signed(73, 8);
    signal COEFF_IIR_CF_B1_1_sig : signed(7 downto 0) := to_signed(114, 8);
    signal COEFF_IIR_CF_B2_1_sig : signed(7 downto 0) := to_signed(127, 8);
    signal COEFF_IIR_CF_A0_1_sig : signed(7 downto 0) := to_signed(127, 8);
    signal COEFF_IIR_CF_A1_1_sig : signed(7 downto 0) := to_signed(127, 8);
    signal COEFF_IIR_CF_A2_1_sig : signed(7 downto 0) := to_signed(127, 8);

    -- Coefficient management for customizable filter
    signal coeff_addr    : integer := 0; -- coefficient address for loading
    signal current_coeff : signed(7 downto 0) := (others => '0'); -- current coefficient value
    signal coeff_data    : signed(7 downto 0) := (others => '0'); -- coefficient data

    -- Component declarations (IP cores and custom modules)
    component clk_wiz_0
        port (
            clk_in1  : in  std_logic;
            resetn    : in  std_logic;
            locked   : out std_logic;
            clk_out1 : out std_logic
        );
    end component;

    component xadc_wiz_0 is
        port
        (
            daddr_in        : in  STD_LOGIC_VECTOR (6 downto 0);
            den_in          : in  STD_LOGIC;
            di_in           : in  STD_LOGIC_VECTOR (15 downto 0);
            dwe_in          : in  STD_LOGIC;
            do_out          : out  STD_LOGIC_VECTOR (15 downto 0);
            drdy_out        : out  STD_LOGIC;
            dclk_in         : in  STD_LOGIC;
            reset_in        : in  STD_LOGIC;
            vauxp3          : in  STD_LOGIC;
            vauxn3          : in  STD_LOGIC;
            busy_out        : out  STD_LOGIC;
            channel_out     : out  STD_LOGIC_VECTOR (4 downto 0);
            eoc_out         : out  STD_LOGIC;
            eos_out         : out  STD_LOGIC;
            alarm_out       : out STD_LOGIC;
            vp_in           : in  STD_LOGIC;
            vn_in           : in  STD_LOGIC
        );
    end component;

    component hann_window is
    generic (
        N : integer := 8192
    );
    port (
        clk        : in  std_logic;
        rst_n        : in  std_logic;
        sample_en  : in  std_logic;
        sample_in  : in  signed(15 downto 0);
        valid      : out std_logic;
        sample_out : out std_logic_vector(15 downto 0)
    );
    end component;

    component command_control is
    Port (
        clk                 : in std_logic;
        clk_b               : in std_logic;
        rst_n               : in std_logic;
        rst_b               : in std_logic;
        uart_rx_valid       : in std_logic;
        window_valid        : in std_logic;
        busy                : in std_logic;
        filter_o_valid      : in std_logic;
        filter_o_valid_1    : in std_logic;
        filtered_signal     : in std_logic_vector(15 downto 0);
        filtered_signal_1   : in std_logic_vector(15 downto 0);
        uart_rx_data        : in std_logic_vector(7 downto 0);
        window_data         : in std_logic_vector(15 downto 0);
        filter_valid        : out std_logic;
        filter_valid_1      : out std_logic;
        fft_in_valid        : out std_logic;
        reset_n_pulse       : out std_logic;
        start_aq            : out std_logic;
        fft_input           : out std_logic_vector(31 downto 0)
    );
    end component;

    component filter_iir12 is
        port (
            clk         : in  std_logic;
            rst_n       : in  std_logic;
            i_valid     : in  std_logic;
            i_last_data : in  std_logic;
            fifo_last   : out std_logic;
            i_data      : in  std_logic_vector(15 downto 0);
            o_valid     : out std_logic;
            o_last_data : out std_logic;
            o_data      : out std_logic_vector(15 downto 0)
        );
    end component;

    component filter_iir12_cust is
        port (
            clk         : in  std_logic;
            rst_n       : in  std_logic;
            i_valid     : in  std_logic;
            i_last_data : in  std_logic;
            fifo_last   : out std_logic;
            i_data      : in  std_logic_vector(15 downto 0);
            coeff_data  : in  signed(7 downto 0);
            coeff_addr  : out integer;
            coeff_valid : in  std_logic;
            o_valid     : out std_logic;
            o_last_data : out std_logic;
            o_data      : out std_logic_vector(15 downto 0)
        );
    end component;

    component xfft_0 is
        port (
            aclk                : in  std_logic;
            aresetn             : in  std_logic;
            s_axis_config_tdata : in  std_logic_vector(15 downto 0);
            s_axis_config_tvalid: in  std_logic;
            s_axis_config_tready: out std_logic;
            s_axis_data_tdata   : in  std_logic_vector(31 downto 0);
            s_axis_data_tvalid  : in  std_logic;
            s_axis_data_tready  : out std_logic;
            s_axis_data_tlast   : in  std_logic;
            m_axis_data_tdata   : out std_logic_vector(31 downto 0);
            m_axis_data_tvalid  : out std_logic;
            m_axis_data_tready  : in  std_logic;
            m_axis_data_tlast   : out std_logic
        );
    end component;

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

    component uart_rx is
        generic (
            G_BAUDRATE                : positive := 230400;
            G_OPERATING_FREQUENCY_MHZ : positive := 8
        );
        port (
            i_clk      : in  std_logic;
            i_rst_n    : in  std_logic;
            i_serial   : in  std_logic;
            o_data_byte: out std_logic_vector(7 downto 0);
            o_rx_done  : out std_logic
        );
    end component;

    component uart_tx is
        generic (
            G_BAUDRATE                : positive := 230400;
            G_OPERATING_FREQUENCY_MHZ : positive := 8
        );
        port (
            i_clk      : in  std_logic;
            i_rst_n    : in  std_logic;
            i_tx_start : in  std_logic;
            i_data_byte: in  std_logic_vector(7 downto 0);
            o_serial   : out std_logic;
            o_tx_busy  : out std_logic;
            o_tx_done  : out std_logic
        );
    end component;

    component rx_filter_coeff is
     Port ( 
        clk         : in  std_logic;
        rst       : in  std_logic;
        rx_valid     : in  std_logic;
        rx_data : in  std_logic_vector(7 downto 0);
        busy    : out std_logic;
        coeff_data : out signed(7 downto 0);
        coeff_rx_valid : out std_logic
      );
    end component;

    component sequencer is
        port (
            clk               : in  std_logic;
            rst_n             : in  std_logic;
            fft_data          : in  std_logic_vector(31 downto 0);
            fft_last          : in  std_logic;               
            fill_afifo        : in  std_logic;
            start             : in  std_logic;
            start_acquisition : out std_logic;
            request_fft       : out std_logic;
            fifo_wr_en        : out std_logic;
            fifo_i_data_reg   : out std_logic_vector(31 downto 0);
            fifo_full         : in  std_logic;
            fft_valid         : in  std_logic
        );
    end component;

    component sequ_2 is
        port (
            clk_b         : in  std_logic;
            rst_b         : in  std_logic;
            tx_active     : in  std_logic;
            start_fill    : in  std_logic;
            uart_rx_data  : in  std_logic_vector(7 downto 0);
            uart_rx_valid : in  std_logic;
            uart_tx_done  : in  std_logic;
            tx_data       : out std_logic_vector(7 downto 0);
            uart_tx_data  : out std_logic_vector(7 downto 0);
            ether_req     : in  std_logic;
            tx_valid      : out std_logic;
            uart_tx_valid : out std_logic;
            fifo_rd_en    : out std_logic;
            fill_afifo    : out std_logic;
            fifo_data_out : in  std_logic_vector(31 downto 0);
            fifo_empty    : in  std_logic;
            fifo_full     : in  std_logic;
            ready_data    : out std_logic
        );
    end component;

    component pulse_synchronizer is
    port (
        i_clk_a   : in  std_logic;
        i_rst_n_a : in  std_logic;
        i_clk_b   : in  std_logic;
        i_rst_n_b : in  std_logic;
        i_pulse_a : in  std_logic;
        o_pulse_b : out std_logic
    );
    end component;

    component ethernet_top is
    port (
        phy_rxd         : in  std_logic_vector(1 downto 0);
        phy_crs_dv      : in  std_logic;
        phy_tx_en       : out std_logic;
        phy_txd         : out std_logic_vector(1 downto 0);
        ref_clk         : out std_logic;
        phy_mdio        : inout std_logic;

        mac_byte        : in  std_logic_vector(7 downto 0);
        mac_rd_en       : out std_logic;
        tx_fifo_empty   : in  std_logic;
        tx_valid        : in std_logic;
        
        led_s           : out std_logic_vector(1 downto 0);
        led             : out std_logic_vector(12 downto 0);

        reset_n         : in  std_logic;
        rst_b           : in std_logic;
        sys_clk         : in  std_logic;
        clk             : out std_logic
    );
    end component;

    component rst_sync is
        port (
            i_clk   : in  std_logic;
            i_rst_n : in  std_logic;
            o_rst_n : out std_logic
        );
    end component;

    component coeff_cdc is
        Port (
            src_clk         : in  std_logic;
            src_rst         : in  std_logic;
            src_coeff       : in  signed(7 downto 0);
            src_valid       : in  std_logic;

            coeff_addr      : in  integer;
            current_coeff   : out signed(7 downto 0);
            dst_valid       : out std_logic
        );
    end component;

begin

    -- Global reset management
    global_rst_n <= not rst; 
    rst_n        <= global_rst_n and rst_u_sync;  

    ref_clk <= clk_b; -- provide reference clock to PHY

    -- XADC: samples analog input and converts to digital
    input_adc: xadc_wiz_0
        port map
        (
            daddr_in        => "0010011",  -- channel 13h (aux channel 3)
            den_in          => start_acquisition, -- enable when acquisition starts
            di_in           => (others => '0'),
            dwe_in          => '0',
            do_out          => adc_out, -- 16-bit ADC result
            drdy_out        => open,
            dclk_in         => clk,
            reset_in        => not rst_n,
            vauxp3          => vauxp3,
            vauxn3          => vauxn3,
            busy_out        => open,
            channel_out     => open,
            eoc_out         => adc_ready, -- pulse when conversion complete
            eos_out         => open,
            alarm_out       => open,
            vp_in           => '0',
            vn_in           => '0'
        );

    -- Convert ADC output to signed format (sign extend upper bits)
    adc_q16 <= (15 downto 12 => adc_out(15)) & signed(adc_out(15 downto 4));

    -- Apply Hann window function to ADC samples for FFT preprocessing
    han_512: hann_window
    generic map(
        N => 16384 -- window size
    )
    port map(
        clk        => clk,
        rst_n       => rst_n,
        sample_en  => adc_ready, -- window each ADC sample
        sample_in  => adc_q16,
        valid      => window_valid,
        sample_out => windowed_sample
    );

    -- Command decoder: processes UART commands to control system mode
    uart_control: command_control
    Port map(
        clk                 => clk,
        clk_b               => clk_b,
        rst_n               => rst_n,
        rst_b               => rst_b,
        uart_rx_valid       => uart_rx_valid,
        window_valid        => window_valid,
        busy                => busy,
        filter_o_valid      => filter_o_valid,
        filter_o_valid_1    => filter_o_valid_1,
        filtered_signal     => filtered_signal,
        filtered_signal_1   => filtered_signal_1,
        uart_rx_data        => uart_rx_data,
        window_data         => windowed_sample,
        filter_valid        => filter_valid, -- enable signals for filters
        filter_valid_1      => filter_valid_1,
        fft_in_valid        => fft_in_valid,
        reset_n_pulse       => rst_u,
        start_aq            => start_aq_sync,
        fft_input           => fft_input
    );

    -- Edge detection for reset pulse
    process(clk)
    begin
       if rising_edge(clk) then
        rst_u_sig <= rst_u;
       end if;
    end process;

    rst_u_sync <= (not rst_u_sig) or rst_u;

    -- Synchronize start signal from clk_b to clk domain
    st_sync: pulse_synchronizer
    port map(
        i_clk_a   => clk_b,
        i_rst_n_a => rst_b,
        i_clk_b   => clk,
        i_rst_n_b => rst_n,
        i_pulse_a => start_aq_sync,
        o_pulse_b => start_aq
    );

    -- Filter 0: fixed coefficient 12th-order IIR filter
    U_IIRFilter : filter_iir12
        port map (
            clk         => clk,
            rst_n       => rst_n,
            i_valid     => filter_valid,
            i_last_data => mod_last_s,
            fifo_last   => filter_last,
            i_data      => filter_input,
            o_valid     => filter_o_valid,
            o_last_data => open,
            o_data      => filtered_signal
        );

    -- Filter 1: customizable coefficient 12th-order IIR filter
    U_IIRFilter_2 : filter_iir12_cust
        port map (
            clk         => clk,
            rst_n       => rst_n,
            i_valid     => filter_valid_1,
            i_last_data => mod_last_s,
            fifo_last   => filter_last_1,
            i_data      => filter_input,
            coeff_data  => coeff_data, -- coefficients from UART
            coeff_addr  => coeff_addr,
            coeff_valid => coeff_tx_valid,
            o_valid     => filter_o_valid_1,
            o_last_data => open,
            o_data      => filtered_signal_1
        );

    filter_input <= std_logic_vector(windowed_sample);

    -- FFT core: processes windowed/filtered samples
    U_XFFT : xfft_0
        port map (
            aclk                 => clk,
            aresetn              => rst_n,
            s_axis_config_tdata  => (others => '0'), -- default config
            s_axis_config_tvalid => '0',
            s_axis_config_tready => open,        
            s_axis_data_tdata    => fft_input,
            s_axis_data_tvalid   => fft_in_valid,
            s_axis_data_tready   => fft_ready_s,
            s_axis_data_tlast    => s_axis_tlast,
            m_axis_data_tdata    => fft_output, -- FFT results
            m_axis_data_tvalid   => fft_valid_s,
            m_axis_data_tready   => request_fft,
            m_axis_data_tlast    => fft_last
        );

    -- Async FIFO: crosses clock domains (clk → clk_b) for output streaming
    U_FIFO : fifo
        generic map (
            G_ADDR_WIDTH => 14, -- 16k entries
            G_DATA_WIDTH => 32
        )
        port map (
            -- Write side (processing clock domain)
            i_write_clk  => clk,
            i_write_rstn => rst_n,
            i_write_en   => fifo_wr_en,
            i_write_data => fft_output,
            o_full       => fifo_full,

            -- Read side (communication clock domain)
            i_read_clk   => clk_b,
            i_read_rstn  => rst_b,
            i_read_en    => fifo_rd_en,
            o_read_data  => fifo_data_out,
            o_empty      => fifo_empty
        );

    -- Reset synchronizer for clk_b domain
    sync6 : rst_sync
        port map (
            i_clk   => clk_b,
            i_rst_n => rst_n,
            o_rst_n => rst_b
        );

    -- UART receiver: processes incoming commands
    uart_rx_inst : uart_rx
        generic map (
            G_BAUDRATE                => 230400,
            G_OPERATING_FREQUENCY_MHZ => 50
        )
        port map (
            i_clk       => clk_b,
            i_rst_n     => rst_b,
            i_serial    => uart_rx_i,
            o_data_byte => uart_rx_data,
            o_rx_done   => uart_rx_valid
        );

    -- UART transmitter: sends data over serial
    uart_tx_inst : uart_tx
        generic map (
            G_BAUDRATE                => 230400,
            G_OPERATING_FREQUENCY_MHZ => 50
        )
        port map (
            i_clk       => clk_b,
            i_rst_n     => rst_b,
            i_tx_start  => tx_start,
            i_data_byte => uart_tx_data,
            o_serial    => uart_tx_o,
            o_tx_busy   => uart_tx_busy,
            o_tx_done   => tx_done
        );

    -- Coefficient receiver: decodes filter coefficients from UART
    rx_filter_coeff_inst : rx_filter_coeff
        Port map( 
            clk       => clk_b,
            rst       => rst_b,
            rx_valid  => uart_rx_valid,
            rx_data   => uart_rx_data,
            busy      => busy,
            coeff_data => current_coeff,
            coeff_rx_valid => coeff_rx_valid 
        );

    -- Sequencer 1: controls FFT processing and FIFO writing (clk domain)
    U_Sequencer : sequencer
        port map (
            clk               => clk,
            rst_n             => rst_n,
            fft_data          => fft_output,
            fft_last          => fft_last,
            fill_afifo        => fill_afifo_sync(1),
            start             => start_aq,
            fft_valid         => fft_valid_s,
            start_acquisition => start_acquisition,
            request_fft       => request_fft,
            fifo_wr_en        => fifo_wr_en,
            fifo_i_data_reg   => fifo_i_data,
            fifo_full         => fifo_full
        );

    -- Sequencer 2: controls FIFO reading and UART/Ethernet output (clk_b domain)
    P_Sequencer : sequ_2
        port map (
            clk_b           => clk_b,
            rst_b           => rst_b,
            tx_active       => uart_tx_busy,
            start_fill      => start_aq_sync,
            uart_rx_data    => uart_rx_data,
            uart_rx_valid   => (uart_rx_valid and (not busy)),
            uart_tx_done    => tx_done,
            tx_data         => tx_data,
            uart_tx_data    => uart_tx_data,
            tx_valid        => tx_valid,
            uart_tx_valid   => tx_start,
            ether_req       => ether_req,
            fill_afifo      => fill_afifo_sig,
            fifo_data_out   => fifo_data_out,
            fifo_empty      => fifo_empty,
            fifo_full       => fifo_sig(1),
            fifo_rd_en      => fifo_rd_en,
            ready_data      => ready_data
        );

    -- Synchronize FIFO full flag across clock domains
    process(clk)
    begin 
        if rising_edge(clk) then
            fifo_sig(0) <= fifo_full;
            fifo_sig(1) <= fifo_sig(0);
        end if;
    end process;

    -- Synchronize FIFO fill trigger across clock domains
    process(clk_b)
    begin
        if rising_edge(clk_b) then
            fill_afifo_sync(0) <= fill_afifo_sig;
            fill_afifo_sync(1) <= fill_afifo_sync(0);
        end if;
    end process;

    -- Ethernet interface: provides dual-mode output (UART + Ethernet)
    ether_core: ethernet_top
    port map(
        phy_rxd         => phy_rxd,
        phy_crs_dv      => phy_crs_dv,
        phy_tx_en       => phy_tx_en,
        phy_txd         => phy_txd,
        ref_clk         => clk_b,
        phy_mdio        => phy_mdio,

        mac_byte        => tx_data,
        mac_rd_en       => ether_req,
        tx_fifo_empty   => ready_data,
        tx_valid        => tx_valid,               

        led_s           => led_s,
        led             => led,

        reset_n         => rst_n,
        rst_b           => rst_b,  
        sys_clk         => clk
    );

    -- Coefficient CDC: manages filter coefficient updates across clock domains
    fil_coeff: coeff_cdc
        Port map(
            src_clk         => clk_b,
            src_rst         => rst_b,
            src_coeff       => current_coeff,
            src_valid       => coeff_rx_valid, 
            
            coeff_addr      => coeff_addr,
            current_coeff   => coeff_data,
            dst_valid       => coeff_tx_valid
        );

end architecture Behavioral;
