library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- IP header checksum calculator using pipelined approach
entity ip_checksum_pipe is
    port ( clk       : in  std_logic;
           rstn      : in  std_logic;                 -- synchronous, active-low
           hdr_in    : in  std_logic_vector(159 downto 0); -- 20-byte IP header
           valid_in  : in  std_logic;                 -- 1-cycle pulse per header
           chk_out   : out std_logic_vector(15 downto 0); -- calculated checksum
           valid_out : out std_logic );
end entity;

architecture rtl of ip_checksum_pipe is
    type sum_array is array (0 to 10) of unsigned(16 downto 0); -- 17-bit to handle carry
    signal s : sum_array; -- pipeline stages for partial sums
    signal v : std_logic_vector(10 downto 0); -- valid signals through pipeline

    constant ZERO15 : unsigned(15 downto 1) := (others => '0'); -- for carry addition

    -- Extract 16-bit word from header, zero out checksum field
    function hdr_word(h : std_logic_vector; idx : integer) return unsigned is
    begin
        if idx = 5 then
            return x"0000";                   -- zero checksum field
        else
            return unsigned(h(159-16*idx downto 144-16*idx)); -- extract word
        end if;
    end function;
begin
    process(clk)
        variable tmp : unsigned(16 downto 0); -- temp for addition with carry
        variable i   : integer;
    begin
        if rising_edge(clk) then
            -- Reset all pipeline stages
            if rstn = '0' then
                for i in 0 to 10 loop
                    s(i) <= (others => '0');
                end loop;
                v <= (others => '0');

            else
                -- Stage 0: load first word or zero
                if valid_in = '1' then
                    s(0) <= ('0' & hdr_word(hdr_in,0));
                else
                    s(0) <= (others => '0');
                end if;
                v(0) <= valid_in;

                -- Stages 1-9: add next word plus carry from previous stage
                for i in 1 to 9 loop
                    tmp := ('0' & s(i-1)(15 downto 0))     -- previous sum (low 16 bits)
                           + ('0' & hdr_word(hdr_in,i))    -- next header word
                           + (ZERO15 & s(i-1)(16));        -- carry from previous stage
                    s(i) <= tmp(16) & tmp(15 downto 0);    -- store result with new carry
                    v(i) <= v(i-1); -- pass valid signal down pipeline
                end loop;

                -- Stage 10: final carry addition (no new data, just propagate carry)
                tmp := ('0' & s(9)(15 downto 0))
                       + (ZERO15 & s(9)(16));
                s(10) <= tmp;
                v(10) <= v(9);
            end if;
        end if;
    end process;

    -- Output one's complement of final sum
    chk_out   <= std_logic_vector(not s(10)(15 downto 0));
    valid_out <= v(10);
end architecture;
