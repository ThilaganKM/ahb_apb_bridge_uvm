// =============================================================================
// File    : ahb_apb_if.sv
// Project : AHB-APB Bridge UVM Verification
//
// Description:
//   SystemVerilog interface for AHB-APB bridge.
//   Bundles all DUT signals into one object passed via virtual interface.
//
//   Modports:
//     master_mp  - used by AHB driver (drives AHB inputs to DUT)
//     slave_mp   - used by APB monitor (observes APB outputs from DUT)
//     dut_mp     - used for DUT port connection in tb_top
//
//   Clocking blocks:
//     master_cb  - synchronizes AHB driver to clock
//     monitor_cb - synchronizes monitors to clock
//
// Why clocking blocks?
//   Without clocking blocks, driver and monitor code must manually
//   handle @posedge/@negedge timing to avoid race conditions.
//   Clocking blocks define input/output skews relative to clock edge,
//   making all sampling and driving deterministic and race-free.
//   Input skew  (#1step) = sample just before clock edge
//   Output skew (1)      = drive 1 time unit after clock edge
// =============================================================================

interface ahb_apb_if (input logic hclk);

  // ===========================================================================
  // AHB Side Signals
  // ===========================================================================

  // AHB inputs to bridge (driven by AHB master / testbench)
  logic        hresetn;       // Active-low reset
  logic        hselapb;       // Slave select for APB bridge
  logic [31:0] haddr;         // AHB address
  logic        hwrite;        // 1=Write, 0=Read
  logic [1:0]  htrans;        // Transfer type (IDLE/BUSY/NONSEQ/SEQ)
  logic [31:0] hwdata;        // Write data from AHB master

  // AHB outputs from bridge (driven by DUT, observed by testbench)
  logic        hready;        // Bridge ready signal
  logic        hresp;         // Transfer response (OKAY/ERROR)
  logic [31:0] hrdata;        // Read data returned to AHB master

  // ===========================================================================
  // APB Side Signals
  // ===========================================================================

  // APB outputs from bridge (driven by DUT, observed by APB monitor)
  logic [31:0] paddr;         // APB address
  logic [31:0] pwdata;        // APB write data
  logic        psel;          // APB peripheral select
  logic        penable;       // APB enable phase
  logic        pwrite;        // APB write control

  // APB input to bridge (driven by APB peripheral model / testbench)
  logic [31:0] prdata;        // Read data from APB peripheral

  // ===========================================================================
  // CLOCKING BLOCK: AHB Master (used by AHB driver)
  //
  // output skew: #1  → drive signals 1ns AFTER posedge (setup time safe)
  // input  skew: #1step → sample signals just BEFORE posedge (hold time safe)
  //
  // Why #1step for inputs?
  //   In simulation, multiple processes update signals at posedge.
  //   #1step samples in the "postpone" region - after all active region
  //   updates - giving us the settled value. This avoids race conditions.
  // ===========================================================================
  clocking master_cb @(posedge hclk);
    default input #1step output #1;

    // Outputs from driver → DUT (AHB inputs)
    output hresetn;
    output hselapb;
    output haddr;
    output hwrite;
    output htrans;
    output hwdata;

    // Inputs to driver ← DUT (AHB outputs - driver reads these)
    input  hready;
    input  hresp;
    input  hrdata;
  endclocking

  // ===========================================================================
  // CLOCKING BLOCK: Monitor (used by both AHB and APB monitors)
  //
  // Monitor only observes - all signals are inputs
  // Same #1step sampling for race-free observation
  // ===========================================================================
  clocking monitor_cb @(posedge hclk);
    default input #1step;

    // AHB side observation
    input hresetn;
    input hselapb;
    input haddr;
    input hwrite;
    input htrans;
    input hwdata;
    input hready;
    input hresp;
    input hrdata;

    // APB side observation
    input paddr;
    input pwdata;
    input psel;
    input penable;
    input pwrite;
    input prdata;
  endclocking

  // ===========================================================================
  // CLOCKING BLOCK: APB Slave (used by APB peripheral model)
  //
  // APB peripheral drives prdata back to bridge
  // Observes psel, penable, pwrite to know when to respond
  // ===========================================================================
  clocking apb_slave_cb @(posedge hclk);
    default input #1step output #1;

    // Observe APB control from bridge
    input  psel;
    input  penable;
    input  pwrite;
    input  paddr;
    input  pwdata;

    // Drive read data back to bridge
    output prdata;
  endclocking

  // ===========================================================================
  // MODPORT: AHB Master Driver
  // Direction from driver's perspective
  // ===========================================================================
  modport master_mp (
    clocking master_cb,
    input    hclk
  );

  // ===========================================================================
  // MODPORT: Monitor (both AHB and APB monitors use this)
  // ===========================================================================
  modport monitor_mp (
    clocking monitor_cb,
    input    hclk
  );

  // ===========================================================================
  // MODPORT: APB Slave / Peripheral Model
  // ===========================================================================
  modport apb_slave_mp (
    clocking apb_slave_cb,
    input    hclk
  );

  // ===========================================================================
  // ASSERTIONS
  // Protocol checkers embedded in interface - fire during simulation
  // automatically without needing to be called from testbench
  //
  // Placing assertions in interface is better than in DUT because:
  //   1. Interface sees both sides of the connection
  //   2. Assertions survive DUT refactoring
  //   3. Reusable across different DUT implementations
  // ===========================================================================

`ifdef ASSERT_ON

  // penable must only rise after psel was high previous cycle
  property p_penable_after_psel;
    @(posedge hclk) disable iff (!hresetn)
    $rose(penable) |-> $past(psel, 1);
  endproperty
  a_penable_after_psel: assert property (p_penable_after_psel)
    else $error("[IF ASSERT] penable rose without psel high previous cycle");

  // hready must be low during APB SETUP phase
  property p_hready_low_in_setup;
    @(posedge hclk) disable iff (!hresetn)
    (psel && !penable) |-> !hready;
  endproperty
  a_hready_low_in_setup: assert property (p_hready_low_in_setup)
    else $error("[IF ASSERT] hready high during APB SETUP - protocol violation");

  // hrdata must equal prdata during read ENABLE phase
  property p_read_data_propagation;
    @(posedge hclk) disable iff (!hresetn)
    (psel && penable && !pwrite) |-> (hrdata == prdata);
  endproperty
  a_read_data_propagation: assert property (p_read_data_propagation)
    else $error("[IF ASSERT] hrdata != prdata during read ENABLE phase");

  // psel must stay high through penable
  property p_psel_stable_through_enable;
    @(posedge hclk) disable iff (!hresetn)
    $rose(penable) |-> psel;
  endproperty
  a_psel_stable_through_enable: assert property (p_psel_stable_through_enable)
    else $error("[IF ASSERT] psel dropped while penable high");

  // htrans must not be SEQ when bridge is idle (no active burst)
  // This checks for illegal burst continuation
  property p_no_seq_without_nonseq;
    @(posedge hclk) disable iff (!hresetn)
    (htrans == 2'b11 && hselapb) |-> 
    ($past(htrans == 2'b10, 1) || $past(htrans == 2'b11, 1));
  endproperty
  a_no_seq_without_nonseq: assert property (p_no_seq_without_nonseq)
    else $warning("[IF ASSERT] SEQ transfer without prior NONSEQ - possible error");

`endif

endinterface
// =============================================================================
// End of ahb_apb_if.sv
// =============================================================================