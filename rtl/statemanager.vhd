library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity statemanager is
   generic
   (
      Softmap_SaveState_ADDR   : integer -- DWORD base address; 3 MiB per slot
   );
   port
   (
      clk                 : in    std_logic;
      reset               : in    std_logic;

      savestate_number    : in    integer;
      save                : in    std_logic;
      load                : in    std_logic;

      request_savestate   : out   std_logic := '0';
      request_loadstate   : out   std_logic := '0';
      -- Address and slot remain stable from dispatch until the next request.
      request_address     : out   integer;
      request_slot        : out   integer range 0 to 3 := 0;
      request_busy        : in    std_logic
   );
end entity;

architecture arch of statemanager is

   -- 3 MiB slots, matching CONF_STR token SS3E000000:300000.
   constant SAVESTATESIZE : integer := 16#C0000#; -- 786432 DWORDs = 3 MiB

   signal save_1         : std_logic := '0';
   signal load_1         : std_logic := '0';
   signal save_buffer    : std_logic := '0';
   signal load_buffer    : std_logic := '0';
   signal save_slot      : integer range 0 to 3 := 0;
   signal load_slot      : integer range 0 to 3 := 0;
   signal awaiting_busy  : std_logic := '0';


begin

   process (clk)
      variable save_dispatched : boolean;
      variable load_dispatched : boolean;
   begin
      if rising_edge(clk) then
         save_dispatched := false;
         load_dispatched := false;

         request_savestate <= '0';
         request_loadstate <= '0';

         save_1 <= save;
         load_1 <= load;

         -- The engine sees a registered request one clock after dispatch.
         -- Wait for its busy acknowledgement before considering another entry.
         if (request_busy = '1') then
            awaiting_busy <= '0';
         end if;

         if (reset = '0' and request_busy = '0' and awaiting_busy = '0') then

            if (save_buffer = '1') then
               request_address   <= Softmap_SaveState_ADDR + (save_slot * SAVESTATESIZE);
               request_slot      <= save_slot;
               awaiting_busy     <= '1';
               save_dispatched := true;
               request_savestate <= '1';
               save_buffer       <= '0';
            elsif (load_buffer = '1') then
               request_address   <= Softmap_SaveState_ADDR + (load_slot * SAVESTATESIZE);
               request_slot      <= load_slot;
               awaiting_busy     <= '1';
               load_dispatched := true;
               request_loadstate <= '1';
               load_buffer       <= '0';
            end if;

         end if;

         -- One pending request per kind; keep its first slot until dispatch.
         -- A new edge on the dispatch cycle refills the just-vacated entry.
         if (reset = '1') then
            save_buffer   <= '0';
            load_buffer   <= '0';
            awaiting_busy <= '0';
         else
            if (save = '1' and save_1 = '0' and
                (save_buffer = '0' or save_dispatched)) then
               save_buffer <= '1';
               save_slot   <= savestate_number;
            end if;
            if (load = '1' and load_1 = '0' and
                (load_buffer = '0' or load_dispatched)) then
               load_buffer <= '1';
               load_slot   <= savestate_number;
            end if;
         end if;

      end if;
   end process;


end architecture;
