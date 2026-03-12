// =============================================================================
// Module  : tb_top
// Project : AHB-APB Bridge - Directed Testbench (Fixed)
//
// Fixes:
//   1. hrdata captured inside ahb_read task before bridge returns to IDLE
//   2. Back-to-back write timing fixed - addr1 latched correctly in WWAIT
// =============================================================================

`timescale 1ns/1ps

module tb_top;

  // ===========================================================================
  // Clock and Reset
  // ===========================================================================
  logic        hclk;
  logic        hresetn;

  // AHB Master drives these:
  logic        hselapb;
  logic [31:0] haddr;
  logic        hwrite;
  logic [1:0]  htrans;
  logic [31:0] hwdata;

  // APB Peripheral drives this:
  logic [31:0] prdata;

  // Bridge drives these (we monitor):
  logic        hready;
  logic        hresp;
  logic [31:0] hrdata;
  logic [31:0] paddr;
  logic [31:0] pwdata;
  logic        psel;
  logic        penable;
  logic        pwrite;

  // Captured read data - grabbed inside task before hrdata goes to 0
  logic [31:0] captured_hrdata;

  // ===========================================================================
  // DUT Instantiation
  // ===========================================================================
  ahb_apb_bridge dut (
    .hclk    (hclk),
    .hresetn (hresetn),
    .hselapb (hselapb),
    .haddr   (haddr),
    .hwrite  (hwrite),
    .htrans  (htrans),
    .hwdata  (hwdata),
    .prdata  (prdata),
    .hready  (hready),
    .hresp   (hresp),
    .hrdata  (hrdata),
    .paddr   (paddr),
    .pwdata  (pwdata),
    .psel    (psel),
    .penable (penable),
    .pwrite  (pwrite)
  );

  // ===========================================================================
  // Clock Generation - 10ns period = 100MHz
  // ===========================================================================
  initial hclk = 0;
  always #5 hclk = ~hclk;


  // ===========================================================================
  // TASK: Reset
  // ===========================================================================
  task do_reset();
    $display("\n[%0t] ===== APPLYING RESET =====", $time);
    hresetn = 0;
    hselapb = 0;
    haddr   = 32'h0;
    hwrite  = 0;
    htrans  = 2'b00;
    hwdata  = 32'h0;
    prdata  = 32'h0;
    repeat(2) @(posedge hclk);
    #1;
    hresetn = 1;
    $display("[%0t] ===== RESET RELEASED =====\n", $time);
  endtask

  // ===========================================================================
  // TASK: AHB Idle
  // ===========================================================================
  task ahb_idle();
    hselapb = 0;
    htrans  = 2'b00;
    hwrite  = 0;
    haddr   = 32'h0;
    hwdata  = 32'h0;
  endtask

  // ===========================================================================
  // TASK: Single AHB Write
  //
  // AHB Write timing:
  //   negedge N+0: drive address phase (haddr, hwrite=1, htrans=NONSEQ)
  //   negedge N+1: drive data phase (hwdata), deassert hselapb/htrans
  //   posedge: wait until hready=1 (sample AFTER bridge drives it)
  // ===========================================================================
  task ahb_write(input logic [31:0] addr, input logic [31:0] data);
    $display("[%0t] AHB WRITE: addr=0x%08h data=0x%08h", $time, addr, data);

    // Address phase
    @(negedge hclk);
    hselapb = 1;
    haddr   = addr;
    hwrite  = 1;
    htrans  = 2'b10;      // NONSEQ

    // Data phase - one cycle after address phase
    @(negedge hclk);
    hwdata  = data;
    hselapb = 0;
    htrans  = 2'b00;      // IDLE

    // Wait on posedge - sample hready after bridge drives it
    @(posedge hclk);
    while (!hready) @(posedge hclk);

    @(negedge hclk);
    ahb_idle();

    $display("[%0t] AHB WRITE DONE", $time);
  endtask

  // ===========================================================================
  // TASK: Single AHB Read
  //
  // FIX: Capture hrdata at the posedge where hready=1.
  // This is the exact cycle bridge is in RENABLE state.
  // After this posedge bridge moves to IDLE and hrdata goes to 0.
  // So we must grab captured_hrdata HERE, not after the task returns.
  // ===========================================================================
  task ahb_read(input logic [31:0] addr, input logic [31:0] peri_data);
    $display("[%0t] AHB READ: addr=0x%08h (peripheral returns 0x%08h)",
             $time, addr, peri_data);

    // Address phase
    @(negedge hclk);
    hselapb = 1;
    haddr   = addr;
    hwrite  = 0;
    htrans  = 2'b10;      // NONSEQ
    prdata  = peri_data;  // peripheral data available

    // Deassert after address phase
    @(negedge hclk);
    hselapb = 0;
    htrans  = 2'b00;

    // Wait for hready=1 on posedge - hrdata is valid RIGHT NOW
    @(posedge hclk);
    while (!hready) @(posedge hclk);

    // Capture here - this is RENABLE state, one cycle before hrdata goes to 0
    captured_hrdata = hrdata;
    $display("[%0t] AHB READ DONE: hrdata=0x%08h (expected=0x%08h) %s",
             $time, captured_hrdata, peri_data,
             (captured_hrdata == peri_data) ? "PASS" : "FAIL");

    @(negedge hclk);
    ahb_idle();
  endtask

  // ===========================================================================
  // TASK: Back-to-Back Writes
  //
  // FIX: Timing of addr2 relative to WWAIT latch.
  //
  // RTL latch fires in always_ff when present_state==WWAIT.
  // Timeline:
  //   Cycle 1 posedge: FSM IDLE→WWAIT (addr1 is on haddr)
  //   Cycle 2 posedge: FSM WWAIT→WRITE_P, latch fires: captures haddr=addr1
  //                    BUT haddr must still be addr1 at this posedge!
  //                    So addr2 must go on haddr at negedge of cycle 2,
  //                    AFTER the cycle 2 posedge has already latched addr1.
  //
  //   negedge 1: addr1, hwrite=1, NONSEQ, hselapb=1
  //   posedge 2: FSM enters WWAIT
  //   negedge 2: data1=hwdata (latch will grab this at posedge 3)
  //              addr1 still on haddr (latch grabs addr1 at posedge 3 too) ← KEY
  //   posedge 3: latch fires in WWAIT: haddr_temp=addr1, hwdata_temp=data1 ✓
  //              FSM: valid=1 → WWAIT→WRITE_P
  //   negedge 3: NOW put addr2 on haddr (too late for write1 latch, perfect)
  //              data2 will be driven next cycle
  // ===========================================================================
task ahb_b2b_write(
  input logic [31:0] addr1, input logic [31:0] data1,
  input logic [31:0] addr2, input logic [31:0] data2
);
  $display("[%0t] AHB BACK-TO-BACK WRITE:", $time);
  $display("       Write1: addr=0x%08h data=0x%08h", addr1, data1);
  $display("       Write2: addr=0x%08h data=0x%08h", addr2, data2);

  // Cycle 1: Address phase of Write1
  @(negedge hclk);
  hselapb = 1;
  haddr   = addr1;
  hwrite  = 1;
  htrans  = 2'b10;    // NONSEQ → FSM IDLE→WWAIT at next posedge

  // Cycle 2: Keep addr1 stable, drive data1
  // WWAIT latch captures addr1+data1 at posedge of cycle 3
  @(negedge hclk);
  hwdata  = data1;
  // haddr stays addr1, hselapb=1, htrans=NONSEQ still
  // valid=1 → FSM will go WWAIT→WRITE_P

  // Cycle 3: Write1 APB SETUP (WRITE_P state)
  // NOW drive addr2 - this is Write2's address phase
  // hselapb=1 keeps valid=1 so FSM knows another transfer is pending
  @(negedge hclk);
  haddr   = addr2;
  hwdata  = data2;    // data2 also ready
  htrans  = 2'b10;    // NONSEQ for write2
  hselapb = 1;

  // Cycle 4: Write1 APB ENABLE (WENABLE_P)
  // FSM releases hready=1, then goes to WWAIT for write2
  // Keep addr2/data2 stable so WWAIT can latch them
  @(negedge hclk);
  // hold addr2, data2, hselapb=1

  // Cycle 5: WWAIT for Write2 - latch fires: haddr_temp=addr2, hwdata_temp=data2
  // Now deselect - no more transfers after write2
  @(negedge hclk);
  hselapb = 0;
  htrans  = 2'b00;

  // Wait for Write1 hready
  @(posedge hclk);
  while (!hready) @(posedge hclk);
  $display("[%0t] Write1 APB complete", $time);

  // Wait for Write2 hready
  @(posedge hclk);
  while (!hready) @(posedge hclk);
  $display("[%0t] Write2 APB complete", $time);

  $display("[%0t] BACK-TO-BACK WRITE DONE", $time);
  // Cycle 6: NOW safe to zero hwdata - latch already fired
  @(negedge hclk);
  hwdata  = 32'h0;

  @(negedge hclk);
  ahb_idle();
endtask

  // ===========================================================================
  // SCOREBOARD
  // ===========================================================================
  int pass_count = 0;
  int fail_count = 0;

  task check(
    input string       test_name,
    input logic [31:0] actual,
    input logic [31:0] expected
  );
    if (actual === expected) begin
      $display("[PASS] %s: got 0x%08h", test_name, actual);
      pass_count++;
    end else begin
      $display("[FAIL] %s: got 0x%08h, expected 0x%08h",
               test_name, actual, expected);
      fail_count++;
    end
  endtask

  // ===========================================================================
  // MAIN TEST SEQUENCE
  // ===========================================================================
  initial begin
    $display("============================================");
    $display("  AHB-APB Bridge Directed Testbench");
    $display("============================================");

    do_reset();
    repeat(2) @(posedge hclk);

    // -------------------------------------------------------
    // TEST 1: Single Write
    // -------------------------------------------------------
    $display("\n---------- TEST 1: Single Write ----------");
    ahb_write(32'h0000_0020, 32'hDEAD_BEEF);
    repeat(2) @(posedge hclk);

    // -------------------------------------------------------
    // TEST 2: Single Read
    // -------------------------------------------------------
    $display("\n---------- TEST 2: Single Read ----------");
    ahb_read(32'h0000_0030, 32'h1234_5678);
    check("Single Read", captured_hrdata, 32'h1234_5678);
    repeat(2) @(posedge hclk);

    // -------------------------------------------------------
    // TEST 3: Back-to-Back Writes
    // -------------------------------------------------------
    $display("\n---------- TEST 3: Back-to-Back Writes ----------");
    ahb_b2b_write(
      32'h0000_0040, 32'hAAAA_1111,
      32'h0000_0050, 32'hBBBB_2222
    );
    repeat(2) @(posedge hclk);

    // -------------------------------------------------------
    // TEST 4: Write then Read
    // -------------------------------------------------------
    $display("\n---------- TEST 4: Write then Read ----------");
    ahb_write(32'h0000_0060, 32'hCAFE_BABE);
    repeat(1) @(posedge hclk);
    ahb_read(32'h0000_0070, 32'hFEED_F00D);
    check("Write-then-Read", captured_hrdata, 32'hFEED_F00D);
    repeat(2) @(posedge hclk);

    // -------------------------------------------------------
    // TEST 5: Reset During Transfer
    // -------------------------------------------------------
    $display("\n---------- TEST 5: Reset During Transfer ----------");
    @(negedge hclk);
    hselapb = 1;
    haddr   = 32'h0000_0080;
    hwrite  = 1;
    htrans  = 2'b10;

    @(negedge hclk);
    $display("[%0t] Asserting reset mid-transfer", $time);
    hresetn = 0;

    repeat(2) @(posedge hclk);
    #1;
    hresetn = 1;
    ahb_idle();
    $display("[%0t] Reset released - bridge should be in IDLE", $time);
    repeat(2) @(posedge hclk);

    ahb_write(32'h0000_0090, 32'h5A5A_5A5A);
    repeat(2) @(posedge hclk);

    // -------------------------------------------------------
    // RESULTS
    // -------------------------------------------------------
    $display("\n============================================");
    $display("  TEST RESULTS");
    $display("  PASS: %0d", pass_count);
    $display("  FAIL: %0d", fail_count);
    if (fail_count == 0)
      $display("  STATUS: ALL TESTS PASSED ✓");
    else
      $display("  STATUS: SOME TESTS FAILED - check waveform");
    $display("============================================\n");

    $finish;
  end

  // ===========================================================================
  // TIMEOUT WATCHDOG
  // ===========================================================================
  initial begin
    #10000;
    $display("[TIMEOUT] Simulation exceeded 10000ns - possible deadlock!");
    $finish;
  end

  // ===========================================================================
  // APB MONITOR
  // ===========================================================================
  always @(posedge hclk) begin
    if (psel && penable) begin
      if (pwrite)
        $display("[APB MON] WRITE: paddr=0x%08h pwdata=0x%08h", paddr, pwdata);
      else
        $display("[APB MON] READ:  paddr=0x%08h prdata=0x%08h hrdata=0x%08h",
                 paddr, prdata, hrdata);
    end
  end

endmodule