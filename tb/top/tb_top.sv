// =============================================================================
// Module  : tb_top
// Project : AHB-APB Bridge - Directed Testbench
// 
// Description:
//   Directed testbench to verify basic bridge operation before UVM.
//   Tests:
//     1. Single Write
//     2. Single Read
//     3. Back-to-back Writes  (exercises WRITE_P, WENABLE_P)
//     4. Write then Read      (exercises WENABLE → READ path)
//     5. Reset during transfer
//
//   Waveform: run "make wave TEST=directed" to view in GTKWave
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
  // Clock Generation
  // 10ns period = 100MHz
  // ===========================================================================
  initial hclk = 0;
  always #5 hclk = ~hclk;

  // ===========================================================================
  // VCD Dump for GTKWave
  // ===========================================================================
  initial begin
    $dumpfile("sim.vcd");
    $dumpvars(0, tb_top);  // dump ALL signals in tb_top and below
  end

  // ===========================================================================
  // TASK: Reset
  // Drives hresetn low for 2 cycles then releases
  // All AHB signals driven to safe idle state during reset
  // ===========================================================================
  task do_reset();
    $display("\n[%0t] ===== APPLYING RESET =====", $time);
    hresetn = 0;
    hselapb = 0;
    haddr   = 32'h0;
    hwrite  = 0;
    htrans  = 2'b00;   // IDLE
    hwdata  = 32'h0;
    prdata  = 32'h0;
    repeat(2) @(posedge hclk);
    #1;                // small delay after posedge to avoid race
    hresetn = 1;
    $display("[%0t] ===== RESET RELEASED =====\n", $time);
  endtask

  // ===========================================================================
  // TASK: AHB Idle
  // Drives bus to idle state - used between transfers
  // ===========================================================================
  task ahb_idle();
    hselapb = 0;
    htrans  = 2'b00;  // IDLE
    hwrite  = 0;
    haddr   = 32'h0;
    hwdata  = 32'h0;
  endtask

  // ===========================================================================
  // TASK: Single AHB Write
  //
  // AHB Write is TWO phases:
  //   Phase 1 (Address phase): drive haddr, hwrite=1, htrans=NONSEQ, hselapb=1
  //   Phase 2 (Data phase):    drive hwdata (address phase of next transfer OR idle)
  //
  // We wait for hready=1 before releasing - confirms bridge is done
  //
  // Arguments:
  //   addr  - target address
  //   data  - write data
  // ===========================================================================
  task ahb_write(input logic [31:0] addr, input logic [31:0] data);
    $display("[%0t] AHB WRITE: addr=0x%08h data=0x%08h", $time, addr, data);

    // -- Address Phase --
    @(negedge hclk);      // drive on negedge to be sampled on next posedge
    hselapb = 1;
    haddr   = addr;
    hwrite  = 1;
    htrans  = 2'b10;      // NONSEQ - new transfer

    // -- Data Phase --
    // hwdata is valid one cycle after address phase
    @(negedge hclk);
    hwdata  = data;
    hselapb = 0;          // deselect after address phase
    htrans  = 2'b00;      // IDLE - no more transfers

    // -- Wait for bridge to complete (hready=1) --
    // Bridge will insert wait states (hready=0) while APB completes
    @(negedge hclk);
    while (!hready) @(negedge hclk);

    // Small idle gap between transfers
    ahb_idle();
    @(negedge hclk);

    $display("[%0t] AHB WRITE DONE: paddr=0x%08h pwdata=0x%08h", 
             $time, paddr, pwdata);
  endtask

  // ===========================================================================
  // TASK: Single AHB Read
  //
  // AHB Read is simpler than write - no data phase from master side:
  //   Phase 1: drive haddr, hwrite=0, htrans=NONSEQ, hselapb=1
  //   Phase 2: bridge drives hrdata when hready=1
  //
  // We supply prdata to simulate APB peripheral response
  //
  // Arguments:
  //   addr      - target address
  //   peri_data - data the APB peripheral will return on prdata
  // ===========================================================================
  task ahb_read(input logic [31:0] addr, input logic [31:0] peri_data);
    $display("[%0t] AHB READ: addr=0x%08h (peripheral will return 0x%08h)", 
             $time, addr, peri_data);

    // -- Address Phase --
    @(negedge hclk);
    hselapb = 1;
    haddr   = addr;
    hwrite  = 0;          // READ
    htrans  = 2'b10;      // NONSEQ
    prdata  = peri_data;  // peripheral data ready (simulate APB peripheral)

    // -- Deselect after address phase --
    @(negedge hclk);
    hselapb = 0;
    htrans  = 2'b00;

    // -- Wait for bridge to complete --
    @(negedge hclk);
    while (!hready) @(negedge hclk);

    $display("[%0t] AHB READ DONE: hrdata=0x%08h (expected=0x%08h) %s",
             $time, hrdata, peri_data,
             (hrdata == peri_data) ? "PASS" : "FAIL");

    ahb_idle();
    @(negedge hclk);
  endtask

  // ===========================================================================
  // TASK: Back-to-Back Writes
  //
  // This is the KEY pipelining test.
  // Second write's address phase overlaps with first write's data phase.
  // This exercises WRITE_P and WENABLE_P states.
  //
  // Timeline:
  //   Cycle 1: addr1, htrans=NONSEQ  ← address phase of write 1
  //   Cycle 2: addr2, htrans=NONSEQ  ← address phase of write 2
  //            data1                  ← data phase of write 1 (overlap!)
  //   Cycle 3: idle
  //            data2                  ← data phase of write 2
  // ===========================================================================
  task ahb_b2b_write(
    input logic [31:0] addr1, input logic [31:0] data1,
    input logic [31:0] addr2, input logic [31:0] data2
  );
    $display("[%0t] AHB BACK-TO-BACK WRITE:", $time);
    $display("       Write1: addr=0x%08h data=0x%08h", addr1, data1);
    $display("       Write2: addr=0x%08h data=0x%08h", addr2, data2);

    // -- Address phase of Write 1 --
    @(negedge hclk);
    hselapb = 1;
    haddr   = addr1;
    hwrite  = 1;
    htrans  = 2'b10;    // NONSEQ

    // -- Address phase of Write 2 overlaps data phase of Write 1 --
    @(negedge hclk);
    hwdata  = data1;    // data for write 1
    haddr   = addr2;    // address for write 2 simultaneously!
    htrans  = 2'b10;    // NONSEQ (new transfer, not SEQ, since addr is not sequential)

    // -- Data phase of Write 2, deselect --
    @(negedge hclk);
    hwdata  = data2;
    hselapb = 0;
    htrans  = 2'b00;

    // -- Wait for both transfers to complete --
    @(negedge hclk);
    while (!hready) @(negedge hclk);
    @(negedge hclk);
    while (!hready) @(negedge hclk);

    $display("[%0t] BACK-TO-BACK WRITE DONE", $time);

    ahb_idle();
    @(negedge hclk);
  endtask

  // ===========================================================================
  // SCOREBOARD LOGIC
  // Simple checker - compare expected vs actual
  // In UVM we'll replace this with a proper scoreboard class
  // ===========================================================================
  int pass_count = 0;
  int fail_count = 0;

  task check(
    input string   test_name,
    input logic [31:0] actual,
    input logic [31:0] expected
  );
    if (actual === expected) begin
      $display("[PASS] %s: got 0x%08h", test_name, actual);
      pass_count++;
    end else begin
      $display("[FAIL] %s: got 0x%08h, expected 0x%08h", test_name, actual, expected);
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

    // -------------------------------------------------------
    // INIT + RESET
    // -------------------------------------------------------
    do_reset();
    repeat(2) @(posedge hclk);

    // -------------------------------------------------------
    // TEST 1: Single Write
    // Write 0xDEAD_BEEF to address 0x0000_0020
    // Expected: paddr=0x20, pwdata=0xDEADBEEF, pwrite=1
    //           psel=1, penable=1 at WENABLE state
    // -------------------------------------------------------
    $display("\n---------- TEST 1: Single Write ----------");
    ahb_write(32'h0000_0020, 32'hDEAD_BEEF);
    repeat(2) @(posedge hclk);

    // -------------------------------------------------------
    // TEST 2: Single Read
    // Read from address 0x0000_0030
    // Peripheral returns 0x1234_5678
    // Expected: hrdata=0x12345678
    // -------------------------------------------------------
    $display("\n---------- TEST 2: Single Read ----------");
    ahb_read(32'h0000_0030, 32'h1234_5678);
    check("Single Read hrdata", hrdata, 32'h1234_5678);
    repeat(2) @(posedge hclk);

    // -------------------------------------------------------
    // TEST 3: Back-to-Back Writes
    // Write1: 0xAAAA_1111 → 0x0000_0040
    // Write2: 0xBBBB_2222 → 0x0000_0050
    // This exercises WRITE_P and WENABLE_P states
    // -------------------------------------------------------
    $display("\n---------- TEST 3: Back-to-Back Writes ----------");
    ahb_b2b_write(
      32'h0000_0040, 32'hAAAA_1111,
      32'h0000_0050, 32'hBBBB_2222
    );
    repeat(2) @(posedge hclk);

    // -------------------------------------------------------
    // TEST 4: Write then Read (different path through FSM)
    // WENABLE → READ path
    // -------------------------------------------------------
    $display("\n---------- TEST 4: Write then Read ----------");
    ahb_write(32'h0000_0060, 32'hCAFE_BABE);
    repeat(1) @(posedge hclk);
    ahb_read(32'h0000_0070, 32'hFEED_F00D);
    check("Write-then-Read hrdata", hrdata, 32'hFEED_F00D);
    repeat(2) @(posedge hclk);

    // -------------------------------------------------------
    // TEST 5: Reset During Transfer
    // Start a write, assert reset mid-way
    // Bridge must return to IDLE cleanly
    // After reset release, new transfer must work correctly
    // -------------------------------------------------------
    $display("\n---------- TEST 5: Reset During Transfer ----------");
    // Start address phase of a write
    @(negedge hclk);
    hselapb = 1;
    haddr   = 32'h0000_0080;
    hwrite  = 1;
    htrans  = 2'b10;

    // Assert reset mid-transfer
    @(negedge hclk);
    $display("[%0t] Asserting reset mid-transfer", $time);
    hresetn = 0;

    repeat(2) @(posedge hclk);
    #1;
    hresetn = 1;
    ahb_idle();
    $display("[%0t] Reset released - bridge should be in IDLE", $time);

    repeat(2) @(posedge hclk);

    // Verify bridge works correctly after reset
    $display("[%0t] Post-reset transfer test", $time);
    ahb_write(32'h0000_0090, 32'h5A5A_5A5A);
    repeat(2) @(posedge hclk);

    // -------------------------------------------------------
    // RESULTS
    // -------------------------------------------------------
    $display("\n============================================");
    $display("  TEST RESULTS");
    $display("  PASS: %0d", pass_count);
    $display("  FAIL: %0d", fail_count);
    $display("============================================\n");

    $finish;
  end

  // ===========================================================================
  // TIMEOUT WATCHDOG
  // Prevents simulation hanging if bridge gets stuck
  // ===========================================================================
  initial begin
    #10000;
    $display("[TIMEOUT] Simulation exceeded 10000ns - possible deadlock!");
    $finish;
  end

  // ===========================================================================
  // MONITOR - prints every APB transaction
  // Watches psel+penable to detect completed APB transfers
  // This is the precursor to the UVM monitor we'll build later
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