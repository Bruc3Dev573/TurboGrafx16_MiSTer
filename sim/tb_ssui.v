// Savestate controls and operation completion.
`timescale 1ns/1ps
module tb_ssui;

reg         clk = 0;
reg  [10:0] ps2_key = 0;
reg         allow_ss = 1;
reg         joySS = 0, joyRight = 0, joyLeft = 0, joyDown = 0, joyUp = 0;
reg         ss_busy = 0, ss_failed = 0, ss_active_load = 0;
reg   [1:0] ss_active_slot = 0;
reg   [1:0] status_slot = 0;
reg   [1:0] OSD_saveload = 0;
wire        ss_save, ss_load, ss_info_req, statusUpdate;
wire  [7:0] ss_info;
wire  [1:0] selected_slot;

savestate_ui #(.INFO_TIMEOUT_BITS(8)) uut (
  .clk(clk), .ps2_key(ps2_key), .allow_ss(allow_ss),
  .joySS(joySS), .joyRight(joyRight), .joyLeft(joyLeft),
  .joyDown(joyDown), .joyUp(joyUp),
  .ss_busy(ss_busy), .ss_failed(ss_failed),
  .ss_active_slot(ss_active_slot), .ss_active_load(ss_active_load),
  .status_slot(status_slot), .OSD_saveload(OSD_saveload),
  .ss_save(ss_save), .ss_load(ss_load),
  .ss_info_req(ss_info_req), .ss_info(ss_info),
  .statusUpdate(statusUpdate), .selected_slot(selected_slot)
);

always #10 clk = ~clk;

integer errors = 0;

integer saves = 0, loads = 0, infos = 0, before_infos;
reg [7:0] last_info;
always @(posedge clk) begin
  if (ss_save) saves = saves + 1;
  if (ss_load) loads = loads + 1;
  if (ss_info_req) begin
    infos = infos + 1;
    last_info = ss_info;
  end
end

task key(input [7:0] code, input press);  // one PS2 event (toggle bit 10)
  begin
    @(posedge clk);
    ps2_key <= {~ps2_key[10], press, 1'b0, code};
    repeat (4) @(posedge clk);
  end
endtask

task expect_key(input [7:0] code, input press, input save_expected, input load_expected);
  integer before_saves, before_loads;
  begin
    before_saves = saves;
    before_loads = loads;
    key(code, press);
    if (saves != before_saves + save_expected || loads != before_loads + load_expected)
      $fatal(1, "key %h: saves=%0d loads=%0d", code, saves-before_saves, loads-before_loads);
  end
endtask

task expect_slot(input [1:0] e, input [127:0] what);
  begin
    if (selected_slot !== e) begin
      errors = errors + 1;
      $display("FAIL %0s: selected_slot=%0d expected=%0d", what, selected_slot, e);
    end else
      $display("ok   %0s: slot=%0d", what, selected_slot);
  end
endtask

initial begin
  // --- BOOT: HPS restores persisted status BEFORE a cart is mounted ------
  allow_ss = 0;
  repeat (8) @(posedge clk);
  status_slot <= 2'd2;  repeat (6) @(posedge clk);   // persisted "Slot 3"
  expect_slot(2, "BOOT persisted slot adopted while allow_ss=0");
  allow_ss = 1;
  status_slot <= 2'd0;  repeat (6) @(posedge clk);   // back to slot 1
  expect_slot(0, "BOOT2 back to slot 1");
  repeat (8) @(posedge clk);

  // --- A: OSD slot change then OSD save ---------------------------------
  status_slot <= 2'd1;  repeat (6) @(posedge clk);
  expect_slot(1, "A1 osd slot=2");
  OSD_saveload <= 2'b01; repeat (6) @(posedge clk);   // save toggle
  OSD_saveload <= 2'b00; repeat (6) @(posedge clk);
  expect_slot(1, "A2 after osd save");

  // --- B: F1 load, then OSD re-select same value (no change event) ------
  key(8'h05, 1); key(8'h05, 0);            // F1 press+release (load, base->0)
  repeat (4) @(posedge clk);
  expect_slot(0, "B1 after F1");
  // statusUpdate loopback: the HPS adopts base=0 into its status copy...
  status_slot <= 2'd0; repeat (6) @(posedge clk);
  // ...so the user re-picking "2" in the OSD is a real 0->1 transition
  status_slot <= 2'd1; repeat (6) @(posedge clk);
  expect_slot(1, "B2 osd re-pick after F1 works");

  // --- C: Alt+F3 save --------------------------------------------------
  key(8'h11, 1);                            // Alt make
  key(8'h04, 1);                            // F3 make
  expect_slot(2, "C1 alt+f3 press");
  key(8'h04, 0); key(8'h11, 0);             // releases
  repeat (4) @(posedge clk);
  expect_slot(2, "C2 after release");

  // --- D: Alt typematic repeats between make and F-key ------------------
  key(8'h11, 1); key(8'h11, 1); key(8'h11, 1);  // typematic repeat = repeated makes
  key(8'h0C, 1);                                 // F4
  expect_slot(3, "D1 alt-typematic + F4");
  key(8'h0C, 0); key(8'h11, 0);

  // --- E: pad slot switch ----------------------------------------------
  joySS <= 1; repeat (4) @(posedge clk);
  joyRight <= 1; repeat (4) @(posedge clk); joyRight <= 0; repeat (4) @(posedge clk);
  expect_slot(3, "E1 padR at max stays 3");  // base was 3
  joyLeft <= 1; repeat (4) @(posedge clk); joyLeft <= 0; repeat (4) @(posedge clk);
  expect_slot(2, "E2 padL 3->2");
  joySS <= 0; repeat (4) @(posedge clk);

  // --- F: OSD save right after keyboard changed base --------------------
  status_slot <= 2'd3; repeat (6) @(posedge clk);
  expect_slot(3, "F1 osd slot=4");
  key(8'h05, 1); key(8'h05, 0);             // F1 load -> base 0
  OSD_saveload <= 2'b01; repeat (6) @(posedge clk);
  OSD_saveload <= 2'b00; repeat (6) @(posedge clk);
  expect_slot(0, "F2 osd save goes to slot 1 after F1 (mismatch vs OSD display=4)");

  // Modifier transitions remain visible while commands are inhibited.
  expect_key(8'h11, 1, 0, 0);
  allow_ss = 0;
  expect_key(8'h11, 0, 0, 0);
  expect_key(8'h05, 1, 0, 0);
  expect_key(8'h05, 0, 0, 0);
  allow_ss = 1;
  expect_key(8'h05, 1, 0, 1);
  expect_key(8'h05, 0, 0, 0);
  allow_ss = 0;
  expect_key(8'h11, 1, 0, 0);
  allow_ss = 1;
  expect_key(8'h05, 1, 1, 0);
  expect_key(8'h05, 0, 0, 0);
  expect_key(8'h11, 0, 0, 0);

  // Completion belongs to the active operation, not later input or selection.
  @(negedge clk);
  ss_active_slot = 1; ss_active_load = 1; ss_busy = 1;
  repeat (4) @(negedge clk);
  expect_key(8'h11, 1, 0, 0);
  expect_key(8'h05, 1, 1, 0);
  expect_key(8'h05, 0, 0, 0);
  expect_key(8'h11, 0, 0, 0);
  @(negedge clk);
  ss_active_slot = 2; ss_active_load = 0;
  status_slot = 3; allow_ss = 0;
  repeat (4) @(negedge clk);
  before_infos = infos;
  ss_busy = 0;
  repeat (4) @(negedge clk);
  if (infos != before_infos + 1 || last_info != 13)
    $fatal(1, "completion reported the wrong operation or slot");

  ss_busy = 1; ss_failed = 0;
  repeat (4) @(negedge clk);
  ss_failed = 1; ss_busy = 0;
  before_infos = infos;
  repeat (4) @(negedge clk);
  if (infos != before_infos) $fatal(1, "failed operation announced success");

  if (errors == 0) $display("TB_SSUI PASS");
  else $fatal(1, "TB_SSUI FAIL count=%0d", errors);
  $finish;
end

endmodule
