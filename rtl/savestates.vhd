-- Based on the NES_MiSTer savestate framework by Robert Peip.
-- PCE memory regions use a 4-bit selector. Arcade RAM is included per
-- transaction; internal registers always include the Arcade Card state.
-- Region map (Save_RAMType, OFFSET always 0 = region-local byte address):
--   0 WRAM 32KB | 1 VRAM0 64KB | 2 SAT0 512B | 3 PALETTE 1KB (9->16 pad)
--   4 VRAM1 64KB | 5 SAT1 512B (SGX) | 6 PRAM 32KB (Populous) | 7 BRM 2KB
--   8 PSG WF_DATA 192B (6ch x 32 x 5b, 1 byte per entry)
--   9 ADPCM DRAM 64KB (128K nibbles packed 2/byte)
--   10 CD/SCD work RAM 256KB | 11 Arcade Card RAM 2MB
library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

use work.pBus_savestates.all;

entity savestates is
   generic
   (
      -- sim override: shrink the CD-RAM region so full save/load cycles run
      -- in sim-minutes (STATESIZE is derived, everything adapts)
      CDRAM_BYTES : integer := 262144
   );
   port
   (
      clk                     : in     std_logic;
      reset_in                : in     std_logic;
      reset_ss                : out    std_logic := '0';
      reset_delay             : out    std_logic := '0';

      load_done               : out    std_logic := '0';
      arcade_used             : in     std_logic := '0';
      arcade_loaded           : out    std_logic := '0';

      increaseSSHeaderCount   : in     std_logic;
      save                    : in     std_logic;
      save_abort              : in     std_logic := '0';	-- suppress control word; cleared header stays invalid
      inhibit                 : in     std_logic := '0'; -- reject only before save/load commits
      load                    : in     std_logic;
      savestate_address       : in     integer;
      savestate_busy          : out    std_logic;
      -- Suppress UI completion for a failed save or a load cancelled before
      -- restoring anything. Held until the next accepted request.
      save_failed             : out    std_logic := '0';

      paused                  : in     std_logic;

      BUS_Din                 : out    std_logic_vector(BUS_buswidth-1 downto 0) := (others => '0');
      BUS_Adr                 : buffer std_logic_vector(BUS_busadr-1 downto 0) := (others => '0');
      BUS_wren                : out    std_logic := '0';
      BUS_rst                 : out    std_logic := '0';
      BUS_Dout                : in     std_logic_vector(BUS_buswidth-1 downto 0) := (others => '0');

      loading_savestate       : out    std_logic := '0';
      saving_savestate        : out    std_logic := '0';
      sleep_savestate         : out    std_logic := '0';

      Save_RAMAddr            : buffer std_logic_vector(24 downto 0) := (others => '0');
      Save_RAMRdEn            : out    std_logic := '0';
      Save_RAMWrEn            : out    std_logic := '0';
      Save_RAMWriteData       : out    std_logic_vector(7 downto 0) := (others => '0');
      Save_RAMReadData        : in     std_logic_vector(7 downto 0);
      Save_RAMRdy             : in     std_logic := '1';  -- slow regions (SDRAM) pace the walk
      Save_RAMType            : out    unsigned(3 downto 0);  -- PCE: 12 regions

      bus_out_Din             : out    std_logic_vector(63 downto 0) := (others => '0');
      bus_out_Dout            : in     std_logic_vector(63 downto 0);
      bus_out_Adr             : buffer std_logic_vector(25 downto 0) := (others => '0');
      bus_out_rnw             : out    std_logic := '0';
      bus_out_ena             : out    std_logic := '0';
      bus_out_be              : out    std_logic_vector(7 downto 0) := (others => '0');
      bus_out_done            : in     std_logic
   );
end entity;

architecture arch of savestates is

   constant MAXSIZEMEM     : integer := 2097152;

   constant SETTLECOUNT    : integer := 100;
   constant HEADERCOUNT    : integer := 2;
   constant INTERNALSCOUNT : integer := 128;

   constant SAVETYPESCOUNT : integer := 12;
   constant ARCADE_SAVETYPE : integer := SAVETYPESCOUNT - 1;
   signal savetype_counter : integer range 0 to SAVETYPESCOUNT;

   type save_type is record
      OFFSET     : integer range 0 to 16#FFFFFF#;
      SIZE       : integer range 0 to MAXSIZEMEM;
   end record;

   type t_savetypes is array(0 to SAVETYPESCOUNT - 1) of save_type;
   constant savetypes : t_savetypes :=
   (
      (16#000000#,   32768),  -- 0 WRAM    (8KB PCE zero-padded / 32KB SGX) -> dpram port B
      (16#000000#,   65536),  -- 1 VRAM0                                    -> dpram port B
      (16#000000#,     512),  -- 2 SAT0 (inside huc6270)
      (16#000000#,    1024),  -- 3 PALETTE (512x9 padded to 16 bits, port A)
      (16#000000#,   65536),  -- 4 VRAM1   (SGX; garbage on PCE, harmless)  -> dpram port B
      (16#000000#,     512),  -- 5 SAT1 (SGX)
      (16#000000#,   32768),  -- 6 PRAM    (Populous)                       -> dpram port B
      (16#000000#,    2048),  -- 7 BRM (backup RAM)
      (16#000000#,     192),  -- 8 PSG WF_DATA (6x32 entries, one byte each)
      (16#000000#,   65536),  -- 9 ADPCM DRAM (128K nibbles packed 2/byte)
      (16#000000#,  CDRAM_BYTES), -- 10 CD/SCD work RAM (paced SDRAM window)
      (16#000000#,  2097152)     -- 11 Arcade Card RAM (full physical 2MB)
   );

   -- Both formats derive from the same walk: full 16#A0532#, short 16#20532#
   -- DWORDs with the default CD-RAM size. Only region 11 is omitted.
   function regions_dwords return integer is
      variable sum : integer := 0;
   begin
      for i in savetypes'range loop
         sum := sum + savetypes(i).SIZE;
      end loop;
      return sum / 4;
   end function;
   constant STATESIZE_FULL  : integer := HEADERCOUNT + INTERNALSCOUNT * 2 + regions_dwords;
   constant STATESIZE_SHORT : integer := STATESIZE_FULL - savetypes(ARCADE_SAVETYPE).SIZE / 4;
   signal transaction_arcade : std_logic := '0';

   type tstate is
   (
      IDLE,
      SAVE_WAITSETTLE,
      SAVECLEARHDR,
      SAVECLEARHDR_WRITE,
      SAVEINTERNALS_WAIT,
      SAVEINTERNALS_WRITE,
      SAVEMEMORY_NEXT,
      SAVEMEMORY_READ,
      SAVEMEMORY_WRITE,
      SAVESIZEAMOUNT,
      LOAD_WAITSETTLE,
      LOAD_HEADERAMOUNTCHECK,
      LOADINTERNALS_READ,
      LOADINTERNALS_WRITE,
      LOADMEMORY_RESET,
      LOADMEMORY_NEXT,
      LOADMEMORY_READ,
      LOADMEMORY_WRITE
   );
   signal state : tstate := IDLE;

   signal count               : integer range 0 to MAXSIZEMEM := 0;
   signal maxcount            : integer range 0 to MAXSIZEMEM;

   signal settle              : integer range 0 to SETTLECOUNT := 0;

   signal bytecounter         : integer range 0 to 7 := 0;
   signal RAMAddrNext         : std_logic_vector(24 downto 0) := (others => '0');
   signal memory_slow         : unsigned(2 downto 0) := (others => '0');

   signal header_amount       : unsigned(31 downto 0) := to_unsigned(1, 32);
   -- Header byte enables. With increaseSSHeaderCount = '0' header writes skip
   -- the change counter (low word): Main copies a slot to SD only when that
   -- counter changes, so the slot stays in memory.
   signal hdr_be              : std_logic_vector(7 downto 0);

begin

   savestate_busy <= '0' when state = IDLE else '1';
   arcade_loaded <= transaction_arcade;
   hdr_be <= x"FF" when increaseSSHeaderCount = '1' else x"F0";

   process (clk)
   begin
      if rising_edge(clk) then

         bus_out_ena   <= '0';
         BUS_wren      <= '0';
         BUS_rst       <= '0';
         reset_ss      <= '0';
         reset_delay   <= '0';
         load_done     <= '0';

         bus_out_be    <= x"FF";

         memory_slow <= memory_slow + 1;
         if (memory_slow = 6) then
            Save_RAMRdEn  <= '0';
            Save_RAMWrEn  <= '0';
         end if;

         case state is

            when IDLE =>
               savetype_counter <= 0;
               -- No RAM region owns the port during settle/header transfers.
               Save_RAMType <= (others => '1');
               if (reset_in = '1') then
                  reset_delay <= '1';
                  reset_ss    <= '1';
                  BUS_rst     <= '1';
               elsif (save = '1' and inhibit = '0') then
                  state                <= SAVE_WAITSETTLE;
                  settle               <= 0;
                  sleep_savestate      <= '1';
                  header_amount        <= header_amount + 1;
                  save_failed          <= '0';
               elsif (load = '1' and inhibit = '0') then
                  state                <= LOAD_WAITSETTLE;
                  settle               <= 0;
                  sleep_savestate      <= '1';
                  -- Each accepted operation replaces the previous result.
                  save_failed          <= '0';
               end if;

            -- #################
            -- SAVE
            -- #################

            when SAVE_WAITSETTLE =>
               if (inhibit = '1') then
                  state           <= IDLE;
                  sleep_savestate <= '0';
                  save_failed     <= '1';
               elsif (paused = '0') then
                  settle <= 0;
               elsif (settle < SETTLECOUNT) then
                  settle <= settle + 1;
               elsif (save_abort = '1') then
                  -- A forced boundary cannot produce a restorable save.
                  -- Reject before invalidating the existing slot.
                  state           <= IDLE;
                  sleep_savestate <= '0';
                  save_failed     <= '1';
               else
                  -- Sample only once, after the running core has frozen and settled.
                  transaction_arcade <= arcade_used;
                  -- invalidate the slot header FIRST: an aborted or torn save
                  -- must leave a rejectable slot, never a stale valid header
                  -- in front of an inconsistent body
                  state            <= SAVECLEARHDR;
                  bus_out_Adr      <= std_logic_vector(to_unsigned(savestate_address, 26));
                  bus_out_rnw      <= '0';
                  BUS_adr          <= (others => '0');
                  count            <= 1;
                  saving_savestate <= '1';
               end if;

            when SAVECLEARHDR =>
               -- Last veto before the first destructive write; then finish atomically.
               if (inhibit = '1') then
                  state            <= IDLE;
                  sleep_savestate  <= '0';
                  saving_savestate <= '0';
                  save_failed      <= '1';
               else
                  -- The load gate reads only the size word, so clearing it
                  -- invalidates the slot even when the counter is kept.
                  bus_out_Din    <= (others => '0');
                  bus_out_be     <= hdr_be;
                  bus_out_ena    <= '1';
                  state          <= SAVECLEARHDR_WRITE;
               end if;

            when SAVECLEARHDR_WRITE =>
               -- ddram latches data and byte enables at the grant, which can
               -- come after the request: hold them until it completes.
               bus_out_be <= hdr_be;
               if (bus_out_done = '1') then
                  state       <= SAVEINTERNALS_WAIT;
                  bus_out_Adr <= std_logic_vector(to_unsigned(savestate_address + HEADERCOUNT, 26));
               end if;

            when SAVEINTERNALS_WAIT =>
               bus_out_Din    <= BUS_Dout;
               bus_out_ena    <= '1';
               state          <= SAVEINTERNALS_WRITE;

            when SAVEINTERNALS_WRITE =>
               if (bus_out_done = '1') then
                  bus_out_Adr <= std_logic_vector(unsigned(bus_out_Adr) + 2);
                  if (count < INTERNALSCOUNT) then
                     state       <= SAVEINTERNALS_WAIT;
                     count       <= count + 1;
                     BUS_adr     <= std_logic_vector(unsigned(BUS_adr) + 1);
                  else
                     state       <= SAVEMEMORY_NEXT;
                     count       <= 8;
                  end if;
               end if;

            when SAVEMEMORY_NEXT =>
               if (savetype_counter < ARCADE_SAVETYPE or
                   (savetype_counter = ARCADE_SAVETYPE and transaction_arcade = '1')) then
                  state          <= SAVEMEMORY_READ;
                  bytecounter    <= 0;
                  count          <= 8;
                  maxcount       <= savetypes(savetype_counter).SIZE;
                  Save_RAMAddr   <= std_logic_vector(to_unsigned(savetypes(savetype_counter).OFFSET, Save_RAMAddr'length));
                  Save_RAMRdEn   <= '1';
                  memory_slow    <= (others => '0');
                  Save_RAMType   <= to_unsigned(savetype_counter, 4);
               else
                  state          <= SAVESIZEAMOUNT;
                  bus_out_Adr    <= std_logic_vector(to_unsigned(savestate_address, 26));
                  if transaction_arcade = '1' then
                     bus_out_Din <= std_logic_vector(to_unsigned(STATESIZE_FULL, 32)) & std_logic_vector(header_amount);
                  else
                     bus_out_Din <= std_logic_vector(to_unsigned(STATESIZE_SHORT, 32)) & std_logic_vector(header_amount);
                  end if;
                  -- Publish the valid header only if the save completed.
                  if (save_abort = '0') then
                     bus_out_ena <= '1';
                  else
                     -- A late abort leaves the cleared header invalid.
                     save_failed <= '1';
                  end if;
                  bus_out_be <= hdr_be;
               end if;

            when SAVEMEMORY_READ =>
               if (memory_slow = 7 and Save_RAMRdy = '1') then
                  bus_out_Din(bytecounter * 8 + 7 downto bytecounter * 8) <= Save_RAMReadData;
                  if (bytecounter < 7) then
                     bytecounter    <= bytecounter + 1;
                     Save_RAMAddr   <= std_logic_vector(unsigned(Save_RAMAddr) + 1);
                     Save_RAMRdEn   <= '1';
                  else
                     state          <= SAVEMEMORY_WRITE;
                     bus_out_ena    <= '1';
                  end if;
               end if;

            when SAVEMEMORY_WRITE =>
               if (bus_out_done = '1') then
                  bus_out_Adr <= std_logic_vector(unsigned(bus_out_Adr) + 2);
                  if (count < maxcount) then
                     state        <= SAVEMEMORY_READ;
                     bytecounter  <= 0;
                     count        <= count + 8;
                     Save_RAMAddr <= std_logic_vector(unsigned(Save_RAMAddr) + 1);
                     Save_RAMRdEn <= '1';
                     memory_slow  <= (others => '0');
                  else
                     savetype_counter <= savetype_counter + 1;
                     state            <= SAVEMEMORY_NEXT;
                  end if;
               end if;

            when SAVESIZEAMOUNT =>
               bus_out_be <= hdr_be;
               if (bus_out_done = '1' or save_abort = '1') then
                  state            <= IDLE;
                  saving_savestate <= '0';
                  sleep_savestate  <= '0';
               end if;


            -- #################
            -- LOAD
            -- #################

            when LOAD_WAITSETTLE =>
               if (inhibit = '1') then
                  state           <= IDLE;
                  sleep_savestate <= '0';
                  save_failed     <= '1';
               elsif (paused = '0') then
                  settle <= 0;
               elsif (settle < SETTLECOUNT) then
                  settle <= settle + 1;
               else
                  state                <= LOAD_HEADERAMOUNTCHECK;
                  bus_out_Adr          <= std_logic_vector(to_unsigned(savestate_address, 26));
                  bus_out_rnw          <= '1';
                  bus_out_ena          <= '1';
               end if;

            when LOAD_HEADERAMOUNTCHECK =>
               if (bus_out_done = '1') then
                  -- Header acceptance commits the restore; never interrupt it afterward.
                  if (inhibit = '1') then
                     state           <= IDLE;
                     sleep_savestate <= '0';
                     save_failed     <= '1';
                  elsif (bus_out_Dout(63 downto 32) = std_logic_vector(to_unsigned(STATESIZE_FULL, 32)) or
                      bus_out_Dout(63 downto 32) = std_logic_vector(to_unsigned(STATESIZE_SHORT, 32))) then
                     -- The slot, never the current session, selects the load walk.
                     if bus_out_Dout(63 downto 32) = std_logic_vector(to_unsigned(STATESIZE_FULL, 32)) then
                        transaction_arcade <= '1';
                     else
                        transaction_arcade <= '0';
                     end if;
                     header_amount        <= unsigned(bus_out_Dout(31 downto 0));
                     state                <= LOADINTERNALS_READ;
                     bus_out_Adr          <= std_logic_vector(to_unsigned(savestate_address + HEADERCOUNT, 26));
                     bus_out_ena          <= '1';
                     BUS_adr              <= (others => '0');
                     count                <= 1;
                     loading_savestate    <= '1';
                  else
                     state                <= IDLE;
                     sleep_savestate      <= '0';
                     save_failed          <= '1';
                  end if;
               end if;

            when LOADINTERNALS_READ =>
               if (bus_out_done = '1') then
                  state           <= LOADINTERNALS_WRITE;
                  BUS_Din         <= bus_out_Dout;
                  BUS_wren        <= '1';
               end if;

            when LOADINTERNALS_WRITE =>
               bus_out_Adr <= std_logic_vector(unsigned(bus_out_Adr) + 2);
               if (count < INTERNALSCOUNT) then
                  state          <= LOADINTERNALS_READ;
                  count          <= count + 1;
                  bus_out_ena    <= '1';
                  BUS_adr        <= std_logic_vector(unsigned(BUS_adr) + 1);
               else
                  state          <= LOADMEMORY_RESET;
               end if;

            when LOADMEMORY_RESET =>
               reset_ss  <= '1';
               state     <= LOADMEMORY_NEXT;
               count     <= 8;

            when LOADMEMORY_NEXT =>
               -- Drain the final paced write before switching region or waking.
               if (memory_slow = 7 and Save_RAMRdy = '1') then
                  if (savetype_counter < ARCADE_SAVETYPE or
                      (savetype_counter = ARCADE_SAVETYPE and transaction_arcade = '1')) then
                     state          <= LOADMEMORY_READ;
                     count          <= 8;
                     maxcount       <= savetypes(savetype_counter).SIZE;
                     RAMAddrNext    <= std_logic_vector(to_unsigned(savetypes(savetype_counter).OFFSET, Save_RAMAddr'length));
                     bytecounter    <= 0;
                     bus_out_ena    <= '1';
                     Save_RAMType   <= to_unsigned(savetype_counter, 4);
                  else
                     state             <= IDLE;
                     loading_savestate <= '0';
                     sleep_savestate   <= '0';
                     load_done         <= '1';
                  end if;
               end if;

            when LOADMEMORY_READ =>
               if (bus_out_done = '1') then
                  state             <= LOADMEMORY_WRITE;
               end if;

            when LOADMEMORY_WRITE =>
               if (memory_slow = 7 and Save_RAMRdy = '1') then
                  RAMAddrNext                    <= std_logic_vector(unsigned(RAMAddrNext) + 1);
                  Save_RAMAddr                   <= RAMAddrNext;
                  Save_RAMWrEn                   <= '1';
                  Save_RAMWriteData              <= bus_out_Dout(bytecounter * 8 + 7 downto bytecounter * 8);
                  memory_slow                    <= (others => '0');
                  if (bytecounter < 7) then
                     bytecounter       <= bytecounter + 1;
                  else
                     bus_out_Adr  <= std_logic_vector(unsigned(bus_out_Adr) + 2);
                     if (count < maxcount) then
                        state          <= LOADMEMORY_READ;
                        count          <= count + 8;
                        bytecounter    <= 0;
                        bus_out_ena    <= '1';
                     else
                        savetype_counter <= savetype_counter + 1;
                        state            <= LOADMEMORY_NEXT;
                     end if;
                  end if;
               end if;

         end case;

      end if;
   end process;


end architecture;
