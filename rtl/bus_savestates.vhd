-- Savestate bus package.

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

package pBus_savestates is

   constant BUS_buswidth : integer := 64;
   constant BUS_busadr   : integer := 10;

end package;

-- Register interface for Verilog callers.

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

library work;
use work.pBus_savestates.all;

entity eReg_SavestateV is
   generic
   (
      Adr       : integer range 0 to (2**BUS_busadr)-1;
      def       : std_logic_vector(BUS_buswidth-1 downto 0)
   );
   port
   (
      clk       : in    std_logic;
      BUS_Din   : in    std_logic_vector(BUS_buswidth-1 downto 0);
      BUS_Adr   : in    std_logic_vector(BUS_busadr-1 downto 0);
      BUS_wren  : in    std_logic;
      BUS_rst   : in    std_logic;
      BUS_Dout  : out   std_logic_vector(BUS_buswidth-1 downto 0) := (others => '0');
      Din       : in    std_logic_vector(BUS_buswidth-1 downto 0);
      Dout      : out   std_logic_vector(BUS_buswidth-1 downto 0)
   );
end entity;

architecture arch of eReg_SavestateV is

   signal Dout_buffer : std_logic_vector(BUS_buswidth-1 downto 0) := def;

   signal AdrI : std_logic_vector(BUS_Adr'left downto 0);

begin

   AdrI <= std_logic_vector(to_unsigned(Adr, BUS_Adr'length));

   process (clk)
   begin
      if rising_edge(clk) then

         if (BUS_rst = '1') then

            Dout_buffer <= def;

         else

            if (BUS_Adr = AdrI and BUS_wren = '1') then
               Dout_buffer <= BUS_Din;
            end if;

         end if;

      end if;
   end process;

   Dout <= Dout_buffer;

   BUS_Dout <= Din when BUS_Adr = AdrI else (others => '0');

end architecture;
