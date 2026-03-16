// =============================================================================
// File    : tb_uvm_top.sv
// Project : AHB-APB Bridge UVM Verification
//
// Description:
//   UVM testbench top module. This is the only non-class, non-package
//   file in the testbench. Everything else is a class inside a package.
//
//   Responsibilities:
//     1. Generate clock
//     2. Instantiate the SystemVerilog interface
//     3. Instantiate the DUT and connect to interface
//     4. Set virtual interface handle in uvm_config_db
//     5. Call run_test() to launch UVM
//     6. Handle initial reset
//
//   This module has NO test logic - all test logic lives in test classes.
//   run_test() picks the test class from +UVM_TESTNAME plusarg.
//
// Compile order (matches Makefile):
//   1. ahb_apb_if.sv         (interface)
//   2. ahb_apb_txn_pkg.sv    (transaction)
//   3. ahb_agent_pkg.sv      (AHB agent)
//   4. apb_agent_pkg.sv      (APB agent)
//   5. bridge_env_pkg.sv     (env + scoreboard + coverage)
//   6. bridge_test_pkg.sv    (tests + sequences)
//   7. ahb_apb_bridge.sv     (RTL)
//   8. tb_uvm_top.sv         (this file - always last)
// =============================================================================

`timescale 1ns/1ps

// Import all packages at top level
import uvm_pkg::*;
`include "uvm_macros.svh"

import ahb_apb_txn_pkg::*;
import ahb_agent_pkg::*;
import apb_agent_pkg::*;
import bridge_env_pkg::*;
import bridge_test_pkg::*;

module tb_uvm_top;

  // ===========================================================================
  // CLOCK GENERATION
  // 10ns period = 100MHz
  // Same frequency as directed testbench for consistency
  // ===========================================================================
  logic hclk;
  initial hclk = 0;
  always #5 hclk = ~hclk;

  // ===========================================================================
  // INTERFACE INSTANTIATION
  // Clock is passed as a port - interface uses it for clocking blocks
  // ===========================================================================
  ahb_apb_if dut_if (hclk);

  // ===========================================================================
  // DUT INSTANTIATION
  // All signals connected through the interface
  // This is the only place DUT ports are referenced by name
  // ===========================================================================
  ahb_apb_bridge dut (
    .hclk    (dut_if.hclk),
    .hresetn (dut_if.hresetn),
    .hselapb (dut_if.hselapb),
    .haddr   (dut_if.haddr),
    .hwrite  (dut_if.hwrite),
    .htrans  (dut_if.htrans),
    .hwdata  (dut_if.hwdata),
    .prdata  (dut_if.prdata),
    .hready  (dut_if.hready),
    .hresp   (dut_if.hresp),
    .hrdata  (dut_if.hrdata),
    .paddr   (dut_if.paddr),
    .pwdata  (dut_if.pwdata),
    .psel    (dut_if.psel),
    .penable (dut_if.penable),
    .pwrite  (dut_if.pwrite)
  );

  // ===========================================================================
  // VCD DUMP
  // ===========================================================================
  initial begin
    $dumpfile("sim_uvm.vcd");
    $dumpvars(0, tb_uvm_top);
  end

  // ===========================================================================
  // INITIAL RESET
  //
  // Drive reset before UVM starts.
  // hresetn=0 for 5 cycles then release.
  //
  // WHY handle reset here and not in a sequence?
  //   Reset must happen before ANY UVM component starts driving.
  //   The run_phase hasn't started yet during time 0 initialization.
  //   Top-level initial block runs at time 0, before UVM phases.
  //
  // All AHB inputs initialized to safe idle state during reset.
  // ===========================================================================
  initial begin
    // Safe idle state - no transfers
    dut_if.hresetn = 0;
    dut_if.hselapb = 0;
    dut_if.haddr   = 32'h0;
    dut_if.hwrite  = 0;
    dut_if.htrans  = 2'b00;
    dut_if.hwdata  = 32'h0;
    dut_if.prdata  = 32'h0;

    // Hold reset for 5 clock cycles
    repeat(5) @(posedge hclk);
    #1;

    // Release reset
    dut_if.hresetn = 1;
    `uvm_info("TOP", "Reset released - starting UVM", UVM_NONE)
  end

  // ===========================================================================
  // UVM CONFIGURATION AND LAUNCH
  //
  // uvm_config_db::set() stores the virtual interface handle
  // so ALL UVM components can retrieve it with ::get().
  //
  // Arguments:
  //   this  = null (top-level, no parent component)
  //   "*"   = wildcard path - any component can access this
  //   "vif" = the key name (must match what components use in ::get())
  //   dut_if = the actual interface instance
  //
  // run_test() launches UVM phase machine.
  // Test class selected by +UVM_TESTNAME plusarg from Makefile.
  // If no plusarg given, run_test("") fails gracefully.
  // ===========================================================================
  initial begin
    // Register virtual interface with config_db
    // All drivers, monitors, and slave models will find it here
    uvm_config_db #(virtual ahb_apb_if)::set(
      null,     // context (null = top level)
      "*",      // path (wildcard = accessible everywhere)
      "vif",    // key
      dut_if    // value
    );

    // Launch UVM - this blocks until all phases complete
    run_test();
  end

  // ===========================================================================
  // TIMEOUT WATCHDOG
  // Prevents simulation hanging if a sequence never drops its objection
  // 50us is generous for our tests - adjust if needed
  // ===========================================================================
  initial begin
    #50_000;
    `uvm_fatal("TIMEOUT",
      "Simulation exceeded 50us - possible objection not dropped")
  end

endmodule
// =============================================================================
// End of tb_uvm_top.sv
// =============================================================================