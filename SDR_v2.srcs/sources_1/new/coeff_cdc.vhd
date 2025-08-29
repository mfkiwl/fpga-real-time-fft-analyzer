library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- Coefficient storage and access module (simplified clock domain crossing)
entity coeff_cdc is
    Port (
        -- Source side (where coefficients come from)
        src_clk         : in  std_logic;
        src_rst         : in  std_logic;
        src_coeff       : in  signed(7 downto 0);      -- incoming coefficient data
        src_valid       : in  std_logic;               -- pulse when new coeff available
        
        -- Access interface (where filter reads coefficients)
        coeff_addr      : in  integer;                 -- which coefficient to read
        current_coeff   : out signed(7 downto 0);      -- coefficient at requested address
        dst_valid       : out std_logic                -- high when all coeffs loaded
    );
end coeff_cdc;

architecture Behavioral of coeff_cdc is

    signal coeff_sig : signed(7 downto 0);    -- unused signal
    signal coeff_sig1 : signed(7 downto 0);   -- unused signal
    signal addr       : integer range 0 to 11; -- current write address

    -- RAM to store 12 coefficients
    type t_coeff_ram is array(0 to 11) of signed(7 downto 0);
    signal coeff_ram : t_coeff_ram;

begin

    -- Store incoming coefficients sequentially in RAM
    process(src_clk)
    begin
        if rising_edge(src_clk) then
            if src_rst = '0' then
                coeff_sig <= (others => '0');
                coeff_ram <= (others => x"00");  -- clear all coefficients
                addr <= 0;                       -- start at address 0
            elsif src_valid = '1' then
                coeff_ram(addr) <= src_coeff;    -- store new coefficient
                addr <= addr + 1;               -- move to next address
            end if;
        end if;
    end process;

    -- Signal that all coefficients are loaded when we've filled all 12 slots
    dst_valid <= '1' when addr = 11 else '0';

    -- Provide direct access to any coefficient by address
    current_coeff <= coeff_ram(coeff_addr);

    -- Commented out: destination clock domain process
    -- Would normally re-register the output in the destination clock domain

end Behavioral;
