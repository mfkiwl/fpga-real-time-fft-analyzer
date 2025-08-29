library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

library STD;
use STD.TEXTIO.ALL;

-- RMII PHY interface with TX/RX state machines for Ethernet frames
entity phy_rmii_if is
  generic (
    data_width : integer := 8;                  -- ROM word width (bits)
    depth      : integer := 42;                 -- number of words in ROM
    data_file  : string  := "head_data.mif"       -- text file containing byte values
  );
  port (
    -- RMII PHY interface
    phy_rxd         : in  std_logic_vector(1 downto 0); -- 2-bit RX data from PHY (RMII)
    phy_crs_dv      : in  std_logic;                    -- Carrier Sense/Data Valid
    phy_tx_en       : out std_logic;                    -- TX enable to PHY
    phy_txd         : out std_logic_vector(1 downto 0); -- 2-bit TX data to PHY (RMII)
    phy_ref_clk     : in  std_logic;                    -- 50MHz RMII reference clock

    -- MDIO management interface
    phy_mdio        : inout std_logic;                  -- MDIO data (not driven in this block)
    phy_mdc         : out std_logic;                    -- MDC clock

    -- TX side (headers/payload feeding)
    tx_fifo_empty   : in  std_logic;                    -- payload FIFO empty indicator
    tx_valid        : in std_logic;                     -- Pulse to start TX
    mac_tx_byte     : in  std_logic_vector(7 downto 0); -- payload byte from MAC FIFO
    mac_rd_en       : out std_logic;                    -- read enable for payload FIFO
    fifo_wr_en      : out std_logic;                    -- write enable to capture RX payload
    fifo_full       : in  std_logic;                    -- capture FIFO full (not used here)
    fifo_din        : out std_logic_vector(7 downto 0); -- captured RX byte to FIFO
    crc_gen_en      : out std_logic;                    -- CRC generator enable (byte qualifies)
    init_tx         : out std_logic;                    -- pulse at end of TX (frame done)

    -- CRC/helper/status
    crc_data_valid  : out std_logic;                    -- indicates RX byte valid for CRC
    frame_ready     : out std_logic;                    -- RX frame complete
    crc_ok          : out std_logic;                    -- RX CRC matches expected (simple check)
    init            : out std_logic;                    -- pulse to reinit downstream (derived)
    crc_gen_data    : out std_logic_vector(7 downto 0); -- TX byte presented to CRC generator
    crc_data_in     : out std_logic_vector(7 downto 0); -- RX byte presented to CRC checker
    led_out         : out std_logic_vector(12 downto 0);-- debug LEDs
    led_s           : out std_logic_vector(1 downto 0); -- spare LEDs (unused, left un-driven)
    crc_tx          : in  std_logic_vector(31 downto 0);-- TX CRC (LSB-first nibbles)
    computed_crc32  : in  std_logic_vector(31 downto 0);-- RX computed CRC (external checker)

    -- Resets and system clock
    reset_b         : in  std_logic;                    -- active-low reset (RMII domain)
    reset_n         : in  std_logic;                    -- active-low reset (sys clock domain)
    sys_clk         : in  std_logic                     -- system clock for MDC generation
  );
end entity;

architecture Behavioral of phy_rmii_if is

  -- Converts 2-bit RMII stream to byte with valid strobe on each assembled byte
  component shift2to8 is
    Port (
      clk        : in  std_logic;
      rst_n      : in  std_logic;
      in_valid   : in  std_logic;
      init       : in  std_logic;
      in_2b      : in  std_logic_vector(1 downto 0);
      out_valid  : out std_logic;
      out_8b     : out std_logic_vector(7 downto 0)
    );
  end component;

  -- Single-bit synchronizer
  component synchronizer is
    port (
      i_clk   : in  std_logic;
      i_rst_n : in  std_logic;
      i_data  : in  std_logic;
      o_data  : out std_logic
    );
  end component;

  component ip_checksum_pipe is
    port (
        clk       : in  std_logic;
        rstn      : in  std_logic;          -- synchronous, active-low
        hdr_in    : in  std_logic_vector(159 downto 0);
        valid_in  : in  std_logic;          -- pulses once per header
        chk_out   : out std_logic_vector(15 downto 0);
        valid_out : out std_logic           -- aligned with chk_out
    );
  end component;

  -- Memory array for ROM contents
  type mem_array is array (0 to depth-1) of std_logic_vector(data_width-1 downto 0);

  -- Load ROM from text file at elaboration time
  impure function load_rom(constant filename : in string) return mem_array is
    file rom_file : text;
    variable rom          : mem_array;
    variable open_status  : file_open_status := NAME_ERROR;
    variable L            : line;
    variable data         : std_logic_vector(7 downto 0);
    variable read_ok      : boolean := true;
    variable next_address : integer := 0;
  begin
    -- default initialize ROM to zeros
    for i in 0 to depth-1 loop
      rom(i) := (others => '0');
    end loop;

    -- When filename is non-empty, try to open and read sequential bytes
    if filename'length > 0 then
      file_open(f => rom_file, external_name => filename, open_kind => READ_MODE, status => open_status);
      assert open_status = open_ok report "Cannot open ROM file: " & filename severity failure;

      -- Read one byte per line until EOF or depth reached
      while not endfile(rom_file) and read_ok and next_address < depth loop
        readline(rom_file, L);
        read(L, data, read_ok);  -- expects std_logic_vector literal per line
        assert read_ok report "Failed to parse data at line " & integer'image(next_address) severity error;

        if read_ok then
          rom(next_address) := data;
          next_address := next_address + 1;
        end if;
      end loop;
      file_close(rom_file);
    end if;

    return rom;
  end function load_rom;

  -- Header data ROM initialized from file
  constant ROM : mem_array := load_rom(data_file);

  signal data_out : std_logic_vector(data_width-1 downto 0) := (others => '0');

  -- RX byte assembly signals
  signal byte_ready       : std_logic := '0';                 -- pulse when assembled_byte is valid
  signal assembled_byte   : std_logic_vector(7 downto 0) := (others => '0'); -- current RX byte

  type t_last4_array is array(3 downto 0) of std_logic_vector(7 downto 0);
  signal last4_bytes    : t_last4_array := (others => (others => '0')); -- sliding window for CRC

  -- Edge detection helpers
  signal dv_d             : std_logic := '0';                 -- delayed phy_crs_dv
  signal dv_dd            : std_logic := '0';                 -- twice delayed phy_crs_dv
  signal saw_55_pulse     : std_logic := '0';                 -- debug: pulse when seeing 0x55
  signal init_sig         : std_logic := '0';                 -- internal init pulse

  -- TX data buffers
  signal head_buffer      : std_logic_vector(7 downto 0) := (others => '0'); -- current header byte for TX
  signal byte_buffer      : std_logic_vector(7 downto 0) := (others => '0'); -- current payload byte for TX
  signal ip_len           : std_logic_vector(15 downto 0) := (others => '0'); -- IP packet length
  signal udp_len          : std_logic_vector(15 downto 0) := (others => '0'); -- UDP payload length
  signal ip_checksum      : std_logic_vector(15 downto 0) := (others => '0'); -- calculated IP checksum

  signal ip_checksum_sig      : std_logic_vector(15 downto 0) := (others => '0');

  -- Pre-built IP header template (gets modified with dynamic fields)
  signal full_head   : std_logic_vector(159 downto 0):= x"45" & x"00" & x"05" & x"DC" & x"00" & x"00" & x"00" & x"00" & x"40" & x"11" & x"00" & x"00" & 
  x"A9" & x"FE" & x"FC" & x"FF" & x"FF" & x"FF" & x"FF" & x"FF";

  -- Various counters
  signal count            : integer range 0 to 40  := 0;     -- MDC clock divider
  signal nibble_cnt       : integer range 0 to 15  := 0;     -- nibble position within byte
  signal preamble_cnt     : integer range 0 to 7   := 0;     -- RX preamble byte counter
  signal preamble_cnt_1   : integer range 0 to 31  := 0;     -- TX preamble nibble counter
  signal ifr_count        : integer range 1 to 50  := 1;     -- inter-frame gap counter
  signal byte_count       : integer range 0 to 2047 := 0;    -- RX byte counter within frame
  signal data_count       : integer := 0;                     -- TX payload byte counter
  signal head_count       : integer := 0;                     -- TX header byte counter
  signal pay_len          : integer := 1025;                -- payload length (constant)
  signal frame_count       : unsigned(15 downto 0) := (others => '0'); -- frame sequence number
  signal mark_cnt         : unsigned(5 downto 0) := (others => '0'); -- mark pattern counter

  -- Status flags
  signal complete         : std_logic := '0';                 -- TX complete flag
  signal head_valid       : std_logic := '0';                 -- header ready for checksum calc
  signal check_valid      : std_logic := '0';                 -- checksum result valid

  signal led              : std_logic_vector(12 downto 0) := (others => '0'); -- debug LEDs

  -- TX state machine
  type state is (IDLE, PREAMBLE_SFD, HEADER_S, DATA, CRC, IFR);
  signal p_state_1 : state := IDLE;

  -- RX state machine
  type state_type is (IDLE, PREAMBLE, WAIT_SFD, RECEIVE, CAPTURE_CRC);
  signal p_state   : state_type := IDLE;

begin
  led_out <= led;
  init    <= init_sig;

  -- Convert RMII 2-bit stream to bytes
  shift2b: shift2to8
    Port map(
      clk       => phy_ref_clk,
      rst_n     => reset_b,
      in_valid  => phy_crs_dv,
      init      => init_sig,
      in_2b     => phy_rxd,
      out_valid => byte_ready,
      out_8b    => assembled_byte
    );

  -- Calculate IP header checksum
  checkip: ip_checksum_pipe
    port map(
        clk       => phy_ref_clk,
        rstn      => reset_b,
        hdr_in    => full_head,
        valid_in  => head_valid,
        chk_out   => ip_checksum_sig,
        valid_out => check_valid
    );

  -- Generate MDC clock for MDIO (simple divider)
  process(sys_clk)
  begin
    if rising_edge(sys_clk) then
      if (reset_n = '0') then
        count  <= 0;
        phy_mdc <= '0';
      else
        count <= count + 1;
        if (count = 40) then
          phy_mdc <= '1';  -- pulse high every 41 cycles
          count   <= 0;
        else
          phy_mdc <= '0';
        end if;
      end if;
    end if;
  end process;

  -- TX state machine: sends preamble, header, payload, and CRC
  process(phy_ref_clk)
  begin
    if rising_edge(phy_ref_clk) then
      if reset_b = '0' then
        p_state_1      <= IDLE;
        preamble_cnt_1 <= 0;
        phy_tx_en      <= '0';
        phy_txd        <= "00";
        mac_rd_en      <= '0';
        nibble_cnt     <= 0;
        crc_gen_en     <= '0';
        init_tx        <= '0';
        ifr_count      <= 1;
        head_valid <= '0';
      else
        case p_state_1 is
          when IDLE =>
            -- Wait for trigger to start TX
            phy_tx_en      <= '0';
            preamble_cnt_1 <= 0;
            head_valid <= '0';
            phy_txd        <= "00";
            frame_count <= (others => '0');
            mac_rd_en      <= '0';
            data_count     <= 0;
            nibble_cnt     <= 0;
            mark_cnt       <= (others => '0');
            ifr_count      <= 1;
            crc_gen_en     <= '0';
            init_tx        <= '0';
            if tx_fifo_empty = '1' then
              p_state_1 <= PREAMBLE_SFD;
            end if;

          when PREAMBLE_SFD =>
            -- Send 7 bytes of 0x55 preamble + SFD (0xD5)
            phy_tx_en    <= '1';
            mac_rd_en    <= '0';
            crc_gen_en   <= '0';
            head_valid <= '0';
            ifr_count    <= 1;
            -- Calculate dynamic header fields
            ip_len       <= std_logic_vector(to_unsigned(pay_len + 8 + 20, 16));
            udp_len      <= std_logic_vector(to_unsigned(pay_len + 8, 16));
            -- Build header with dynamic fields
            full_head    <= ROM(14) & ROM(15) & ip_len(15 downto 8) & ip_len(7 downto 0) &
                          std_logic_vector(frame_count(15 downto 8)) & std_logic_vector(frame_count(7 downto 0)) & ROM(20) & ROM(21) &
                          ROM(22) & ROM(23) & ROM(24) & ROM(25) & ROM(26) & ROM(27) &
                          ROM(28) & ROM(29) & ROM(30) & ROM(31) &
                          ROM(32) & ROM(33);
            if preamble_cnt_1 < 32 then
              phy_txd        <= "01";  -- preamble pattern
              preamble_cnt_1 <= preamble_cnt_1 + 1;
              if preamble_cnt_1 = 30 then
                head_buffer  <= ROM(head_count);  -- prep first header byte
                crc_gen_en   <= '1';
                crc_gen_data <= ROM(head_count);
                head_count <= head_count + 1;
              elsif preamble_cnt_1 = 31 then
                phy_txd   <= "11";  -- SFD ending
                head_valid <= '1';
                p_state_1 <= HEADER_S;
              end if;
            end if;

          when HEADER_S =>
            -- Send header bytes nibble by nibble
            crc_gen_en   <= '0';
            mac_rd_en    <= '0';
            phy_tx_en    <= '1';
            head_valid <= '0';
            -- Latch calculated checksum when ready
            if check_valid = '1' then
              ip_checksum <= ip_checksum_sig;
            else
              ip_checksum <= ip_checksum;
            end if;

            if head_count = 42 then
              -- Header done, prep for payload
              phy_txd    <= head_buffer((2*nibble_cnt)+1 downto 2*nibble_cnt);
              nibble_cnt <= nibble_cnt + 1;
              -- Use mark pattern for payload
              byte_buffer <= std_logic_vector(resize(mark_cnt, 8));
              crc_gen_data <= std_logic_vector(resize(mark_cnt, 8));
              if nibble_cnt = 0 then
                mac_rd_en <= '0';
              elsif nibble_cnt = 2 then
                data_count   <= data_count + 1;
              elsif nibble_cnt = 3 then
                crc_gen_en <= '1';
                p_state_1  <= DATA;
                head_count <= 0;
                nibble_cnt <= 0;
              end if;
            else
              -- Continue with header bytes
              phy_txd    <= head_buffer((2*nibble_cnt)+1 downto 2*nibble_cnt);
              nibble_cnt <= nibble_cnt + 1;
              if nibble_cnt = 3 then
                -- Handle special header fields (lengths, checksum, etc.)
                if head_count = 16 then
                  head_buffer  <= ip_len(15 downto 8);
                  crc_gen_data <= ip_len(15 downto 8);
                elsif head_count = 17 then
                  head_buffer  <= ip_len(7 downto 0);
                  crc_gen_data <= ip_len(7 downto 0);
                elsif head_count = 18 then
                  head_buffer  <= std_logic_vector(frame_count(15 downto 8));
                  crc_gen_data <= std_logic_vector(frame_count(15 downto 8));
                elsif head_count = 19 then
                  head_buffer  <= std_logic_vector(frame_count(7 downto 0));
                  crc_gen_data <= std_logic_vector(frame_count(7 downto 0));
                elsif head_count = 24 then
                  head_buffer <= ip_checksum(15 downto 8);
                  crc_gen_data <= ip_checksum(15 downto 8);
                elsif head_count = 25 then
                  head_buffer <= ip_checksum(7 downto 0);
                  crc_gen_data <= ip_checksum(7 downto 0);
                elsif head_count = 38 then
                  head_buffer  <= udp_len(15 downto 8);
                  crc_gen_data <= udp_len(15 downto 8);
                elsif head_count = 39 then
                  head_buffer  <= udp_len(7 downto 0);
                  crc_gen_data <= udp_len(7 downto 0);
                else
                  head_buffer  <= ROM(head_count);
                  crc_gen_data <= ROM(head_count);
                end if;
                crc_gen_en   <= '1';
                head_count <= head_count + 1;
                nibble_cnt   <= 0;
              end if;
            end if;

          when DATA =>
            -- Send payload bytes nibble by nibble
            crc_gen_en   <= '0';
            mac_rd_en    <= '0';
            phy_tx_en    <= '1';
            if data_count = pay_len then
              -- Payload done, finish last byte
              phy_txd    <= byte_buffer((2*nibble_cnt)+1 downto 2*nibble_cnt);
              nibble_cnt <= nibble_cnt + 1;
              if nibble_cnt = 3 then
                p_state_1  <= CRC;
                data_count <= 0;
                crc_gen_en <= '0';
                nibble_cnt <= 0;
              end if;
            else
              -- Continue with payload
              phy_txd    <= byte_buffer((2*nibble_cnt)+1 downto 2*nibble_cnt);
              nibble_cnt <= nibble_cnt + 1;
              if nibble_cnt = 1 then
                mac_rd_en <= '1';
              elsif nibble_cnt = 3 then
                data_count   <= data_count + 1;
                nibble_cnt   <= 0;
              end if;
              -- Use actual payload or mark pattern
              if tx_valid = '1' then
                byte_buffer  <= mac_tx_byte;
                crc_gen_en   <= '1';
                crc_gen_data <= mac_tx_byte;
              end if;
            end if;

          when CRC =>
            -- Send 32-bit CRC nibble by nibble
            phy_tx_en <= '1';
            if nibble_cnt < 16 then
              phy_txd    <= crc_tx((2*nibble_cnt)+1 downto 2*nibble_cnt);
              nibble_cnt <= nibble_cnt + 1;
              if nibble_cnt = 15 then
                nibble_cnt <= 0;
                init_tx    <= '1';  -- signal frame complete
                complete   <= '1';
                p_state_1  <= IFR;
              end if;
            end if;

          when IFR =>
            -- Inter-frame gap (idle time between frames)
            preamble_cnt_1 <= 0;
            mac_rd_en      <= '0';
            data_count     <= 0;
            nibble_cnt     <= 0;
            crc_gen_en     <= '0';
            phy_tx_en  <= '0';
            init_tx    <= '0';
            complete   <= '0';
            phy_txd    <= "00";  -- idle
            ifr_count  <= ifr_count + 1;
            if ifr_count > 49 and tx_fifo_empty = '1' then
              frame_count <= frame_count + 1;
              mark_cnt <= mark_cnt + 1;
              p_state_1 <= PREAMBLE_SFD;  -- start next frame
            end if;

          when others =>
            p_state_1 <= IDLE;
        end case;
      end if;
    end if;
  end process;

  -- RX state machine: detect frames and capture data
  process(phy_ref_clk)
  begin
    if rising_edge(phy_ref_clk) then
      if reset_b = '0' then
        p_state       <= IDLE;
        preamble_cnt  <= 0;
        last4_bytes   <= (others => (others => '0'));
        frame_ready   <= '0';
        crc_ok        <= '0';
        byte_count    <= 0;
        init_sig      <= '0';
        led           <= (others => '0');
      else
        -- Debug: detect 0x55 bytes
        if byte_ready = '1' and assembled_byte = x"55" then
          saw_55_pulse <= '1';
        else
          saw_55_pulse <= '0';
        end if;

        led(0) <= saw_55_pulse;

        case p_state is
          when IDLE =>
            -- Wait for start of preamble
            frame_ready <= '0';
            if byte_ready = '1' and assembled_byte = x"55" then
              preamble_cnt <= 1;
              p_state      <= PREAMBLE;
            else
              preamble_cnt <= 0;
              p_state      <= IDLE;
            end if;
            init_sig    <= '0';
            led(4)      <= '1';
            byte_count  <= 0;

          when PREAMBLE =>
            -- Count preamble bytes, look for SFD
            if byte_ready = '1' then
              if assembled_byte = x"55" then
                led(12 downto 5) <= assembled_byte;
                preamble_cnt     <= preamble_cnt + 1;
                p_state          <= PREAMBLE;
              elsif (preamble_cnt >= 7) and (assembled_byte = x"55") then
                preamble_cnt <= preamble_cnt + 1;
                p_state      <= PREAMBLE;
              elsif (preamble_cnt >= 7) and (assembled_byte = x"D5") then
                -- Found SFD, start receiving frame data
                p_state      <= RECEIVE;
                preamble_cnt <= 0;
                led(12 downto 5) <= assembled_byte;
              else
                -- Bad preamble, go back to idle
                preamble_cnt <= 0;
                p_state      <= IDLE;
              end if;
            end if;
            led(3) <= '1';

          when RECEIVE =>
            -- Capture frame bytes and maintain sliding window
            if byte_ready = '1' then
              -- Shift last 4 bytes window (for CRC extraction)
              last4_bytes(3) <= last4_bytes(2);
              last4_bytes(2) <= last4_bytes(1);
              last4_bytes(1) <= last4_bytes(0);
              last4_bytes(0) <= assembled_byte;

              byte_count <= byte_count + 1;

              -- Check for end of frame (CRS_DV falling edge)
              if phy_crs_dv = '0' and dv_d = '1' then
                p_state <= CAPTURE_CRC;
              else
                p_state <= RECEIVE;
              end if;
            end if;
            led(2) <= '1';

          when CAPTURE_CRC =>
            -- Simple CRC check (magic number comparison)
            if computed_crc32 = x"C704DD7B" then
              crc_ok <= '1';
            else
              crc_ok <= '0';
            end if;
            frame_ready <= '1';
            p_state     <= IDLE;
            init_sig    <= '1';  -- pulse to reinit downstream
            led(1)      <= '1';

          when others =>
            p_state <= IDLE;
        end case;
      end if;
    end if;
  end process;

  -- Delay registers for edge detection
  process(phy_ref_clk)
  begin
    if rising_edge(phy_ref_clk) then
      if reset_b = '0' then
        dv_d  <= '0';
        dv_dd <= '0';
      else
        dv_d  <= phy_crs_dv;
        dv_dd <= dv_d;
      end if;
    end if;
  end process;

  -- Connect RX capture outputs
  fifo_din        <= last4_bytes(3);  -- oldest byte (excluding CRC)
  crc_data_in     <= assembled_byte;  -- current byte to CRC checker
  fifo_wr_en      <= '1' when p_state = RECEIVE and byte_ready = '1' and byte_count >= 4 else '0';
  crc_data_valid  <= '1' when p_state = RECEIVE and byte_ready = '1' else '0';

  led_s <= (others => '0');  -- unused outputs

end Behavioral;
