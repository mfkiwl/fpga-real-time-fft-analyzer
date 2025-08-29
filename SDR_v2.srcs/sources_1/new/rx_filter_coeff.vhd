library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- Filter coefficient receiver - waits for trigger byte then streams out coefficients
entity rx_filter_coeff is
 Port ( 
    clk         : in  std_logic;                     -- System clock
    rst       : in  std_logic;                     -- Active-low synchronous reset
    rx_valid     : in  std_logic;                     -- Input sample valid
    rx_data : in  std_logic_vector(7 downto 0);                     -- Input data byte
    busy    : out std_logic;                     -- Busy flag (acquiring coeffs)
    coeff_data : out signed(7 downto 0);             -- Current coefficient output
    coeff_rx_valid : out std_logic                     -- Pulse when new coefficients available
  );
end rx_filter_coeff;

architecture Behavioral of rx_filter_coeff is

  type t_state is (IDLE, ACQUIRE, S_WAIT);
  signal p_state     : t_state := IDLE;          -- State machine

  signal byte_count : integer := 0; -- Count of received coefficient bytes

  -- Commented out: individual coefficient signals for IIR filter
  -- Would normally output 12 separate coefficient values for two biquad sections

begin

  -- Main state machine for coefficient reception
  process(clk, rst)
  begin
      if rising_edge(clk) then
          if rst = '0' then
              byte_count <= 0;
              coeff_rx_valid <= '0';
              busy <= '0';
              p_state <= IDLE;
          else
              case p_state is
                  when IDLE =>
                      -- Wait for trigger byte 0xF1 to start coefficient acquisition
                      coeff_rx_valid <= '0';
                      busy <= '0';
                      if rx_valid = '1' and rx_data = x"F1" then
                          p_state <= ACQUIRE;
                      end if;
          
                  when ACQUIRE =>
                      -- Receive 12 coefficient bytes and stream them out
                      busy <= '1';
                      coeff_rx_valid <= '0';
                      if rx_valid = '1' and byte_count < 12 then
                          byte_count <= byte_count + 1;
                          coeff_rx_valid <= '1';  -- pulse to indicate new coeff ready
                          coeff_data <= signed(rx_data);  -- output current coefficient
                          -- Commented out: individual coefficient assignment logic
                          -- Would normally assign to specific B0, B1, B2, A0, A1, A2 for each biquad
                      elsif byte_count > 11 then
                          -- All 12 coefficients received, go back to idle
                          coeff_rx_valid <= '0';
                          byte_count <= 0;
                          p_state <= IDLE;
                      else
                          p_state <= ACQUIRE;  -- stay in acquire mode
                      end if;
          
                  when others =>
                      -- Safety catch-all
                      p_state <= IDLE;
          
              end case;
          end if;
      end if;
  end process;

end Behavioral;
