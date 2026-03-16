// =============================================================================
// File    : bridge_env_pkg.sv
// Project : AHB-APB Bridge UVM Verification
//
// Description:
//   Environment package containing:
//     - bridge_scoreboard  : compares AHB vs APB transactions
//     - bridge_coverage    : functional coverage collector
//     - bridge_env         : top-level env wiring everything together
//
// Data flow:
//   AHB agent monitor  →  ap  →  scoreboard.ahb_export
//   APB agent monitor  →  ap  →  scoreboard.apb_export
//   APB agent monitor  →  ap  →  coverage.analysis_export
//
// Scoreboard checks:
//   1. paddr  == haddr          (address translated correctly)
//   2. pwdata == hwdata         (write data passed correctly)
//   3. hrdata == prdata         (read data returned correctly)
//   4. pwrite == hwrite         (direction preserved)
//   5. hresp  == OKAY           (no unexpected errors)
// =============================================================================

package bridge_env_pkg;

  import uvm_pkg::*;
  `include "uvm_macros.svh"

  import ahb_apb_txn_pkg::*;

  // ===========================================================================
  // SCOREBOARD
  //
  // Receives transactions from BOTH AHB monitor and APB monitor.
  // Compares them to verify the bridge translated correctly.
  //
  // Uses two analysis FIFOs:
  //   ahb_fifo: stores transactions from AHB monitor (what master requested)
  //   apb_fifo: stores transactions from APB monitor (what bridge drove)
  //
  // Matching strategy:
  //   Transactions arrive in order on both FIFOs.
  //   We get one from each FIFO and compare field by field.
  //   Order-based matching works because AHB-APB bridge is in-order.
  //
  // WHY two separate FIFOs instead of one?
  //   AHB and APB transactions complete at different times.
  //   AHB monitor sees completion when hready=1.
  //   APB monitor sees completion when penable=1.
  //   These are different clock cycles - FIFOs buffer the difference.
  // ===========================================================================
  class bridge_scoreboard extends uvm_scoreboard;
    `uvm_component_utils(bridge_scoreboard)

    // Analysis FIFOs - buffer incoming transactions
    // uvm_tlm_analysis_fifo provides both analysis_export (write end)
    // and get_port (read end) in one object
    uvm_tlm_analysis_fifo #(ahb_apb_txn) ahb_fifo;
    uvm_tlm_analysis_fifo #(ahb_apb_txn) apb_fifo;

    // Counters for final report
    int unsigned total_checks;
    int unsigned passed_checks;
    int unsigned failed_checks;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      ahb_fifo = new("ahb_fifo", this);
      apb_fifo = new("apb_fifo", this);
    endfunction

    // -------------------------------------------------------------------------
    // run_phase: scoreboard comparison loop
    //
    // Runs forever, getting one transaction from each FIFO and comparing.
    // get() blocks until a transaction is available - no busy-waiting.
    // -------------------------------------------------------------------------
    task run_phase(uvm_phase phase);
      ahb_apb_txn ahb_txn, apb_txn;

      forever begin
        // Get transaction from AHB monitor (blocks until available)
        ahb_fifo.get(ahb_txn);

        // Get corresponding transaction from APB monitor
        apb_fifo.get(apb_txn);

        // Compare the two transactions
        check_transaction(ahb_txn, apb_txn);
      end
    endtask

    // -------------------------------------------------------------------------
    // check_transaction: field-by-field comparison
    // -------------------------------------------------------------------------
    task check_transaction(ahb_apb_txn ahb_txn, ahb_apb_txn apb_txn);
      bit pass = 1;
      total_checks++;

      // Check 1: Address translation
      // paddr must equal haddr - bridge must not modify address
      if (apb_txn.addr !== ahb_txn.addr) begin
        `uvm_error("SBOARD",
          $sformatf("ADDR MISMATCH: haddr=0x%08h paddr=0x%08h",
            ahb_txn.addr, apb_txn.addr))
        pass = 0;
      end

      // Check 2: Transfer direction preserved
      if (apb_txn.kind !== ahb_txn.kind) begin
        `uvm_error("SBOARD",
          $sformatf("KIND MISMATCH: ahb=%s apb=%s",
            ahb_txn.kind.name(), apb_txn.kind.name()))
        pass = 0;
      end

      // Check 3: Write data integrity
      // pwdata must equal hwdata for write transactions
      if (ahb_txn.kind == AHB_WRITE) begin
        if (apb_txn.data !== ahb_txn.data) begin
          `uvm_error("SBOARD",
            $sformatf("WDATA MISMATCH: hwdata=0x%08h pwdata=0x%08h",
              ahb_txn.data, apb_txn.data))
          pass = 0;
        end
      end

      // Check 4: Read data integrity
      // hrdata must equal prdata for read transactions
      if (ahb_txn.kind == AHB_READ) begin
        if (apb_txn.data !== ahb_txn.data) begin
          `uvm_error("SBOARD",
            $sformatf("RDATA MISMATCH: prdata=0x%08h hrdata=0x%08h",
              apb_txn.data, ahb_txn.data))
          pass = 0;
        end
      end

      // Check 5: No unexpected error response
      if (ahb_txn.resp == HRESP_ERROR) begin
        `uvm_warning("SBOARD",
          $sformatf("ERROR RESPONSE seen: addr=0x%08h", ahb_txn.addr))
      end

      // Update counters and log result
      if (pass) begin
        passed_checks++;
        `uvm_info("SBOARD",
          $sformatf("[PASS] %s addr=0x%08h data=0x%08h",
            ahb_txn.kind.name(), ahb_txn.addr, ahb_txn.data),
          UVM_MEDIUM)
      end else begin
        failed_checks++;
        `uvm_info("SBOARD",
          $sformatf("[FAIL] AHB: %s | APB: %s",
            ahb_txn.convert2string(), apb_txn.convert2string()),
          UVM_NONE)
      end
    endtask

    // -------------------------------------------------------------------------
    // report_phase: print final summary
    // Runs after simulation ends - last phase before cleanup
    // -------------------------------------------------------------------------
    function void report_phase(uvm_phase phase);
      `uvm_info("SBOARD", "==========================================", UVM_NONE)
      `uvm_info("SBOARD", "       SCOREBOARD FINAL REPORT           ", UVM_NONE)
      `uvm_info("SBOARD", "==========================================", UVM_NONE)
      `uvm_info("SBOARD",
        $sformatf("  Total Checks : %0d", total_checks), UVM_NONE)
      `uvm_info("SBOARD",
        $sformatf("  Passed       : %0d", passed_checks), UVM_NONE)
      `uvm_info("SBOARD",
        $sformatf("  Failed       : %0d", failed_checks), UVM_NONE)

      if (failed_checks == 0)
        `uvm_info("SBOARD", "  STATUS: ALL CHECKS PASSED", UVM_NONE)
      else
        `uvm_error("SBOARD",
          $sformatf("  STATUS: %0d CHECKS FAILED", failed_checks))

      `uvm_info("SBOARD", "==========================================", UVM_NONE)
    endfunction

  endclass : bridge_scoreboard

  // ===========================================================================
  // COVERAGE COLLECTOR
  //
  // Functional coverage - answers "what scenarios have we exercised?"
  //
  // Coverage groups:
  //   cg_transfer_type : read vs write coverage
  //   cg_address       : address range coverage
  //   cg_data_corners  : all-zeros, all-ones, walking patterns
  //   cg_b2b           : back-to-back transfer coverage
  //   cg_crosses       : cross coverage (kind x address range)
  //
  // Coverage is separate from checking - a test can PASS the scoreboard
  // but still have poor coverage (untested scenarios).
  // 100% functional coverage = all interesting scenarios exercised.
  // ===========================================================================
  class bridge_coverage extends uvm_subscriber #(ahb_apb_txn);
    `uvm_component_utils(bridge_coverage)

    // Transaction being sampled (set in write() method)
    ahb_apb_txn txn;

    // Previous transaction - needed for back-to-back detection
    ahb_apb_txn prev_txn;

    // -------------------------------------------------------------------------
    // COVERGROUP: Transfer Type
    // Basic read/write coverage + response type
    // -------------------------------------------------------------------------
    covergroup cg_transfer_type;
      cp_kind: coverpoint txn.kind {
        bins write_txn = {AHB_WRITE};
        bins read_txn  = {AHB_READ};
      }
      cp_resp: coverpoint txn.resp {
        bins okay  = {HRESP_OKAY};
        bins error = {HRESP_ERROR};
      }
    endgroup

    // -------------------------------------------------------------------------
    // COVERGROUP: Address Ranges
    // Verifies bridge handles different address regions
    // -------------------------------------------------------------------------
    covergroup cg_address;
      cp_addr_range: coverpoint txn.addr[7:2] {
        bins low_range  = {[0:15]};    // 0x00 - 0x3C
        bins mid_range  = {[16:47]};   // 0x40 - 0xBC
        bins high_range = {[48:63]};   // 0xC0 - 0xFC
      }
    endgroup

    // -------------------------------------------------------------------------
    // COVERGROUP: Data Corner Cases
    // All-zeros, all-ones, alternating patterns are classic bug triggers
    // -------------------------------------------------------------------------
    covergroup cg_data_corners;
      cp_data: coverpoint txn.data {
        bins all_zeros    = {32'h0000_0000};
        bins all_ones     = {32'hFFFF_FFFF};
        bins alternating1 = {32'hAAAA_AAAA};
        bins alternating2 = {32'h5555_5555};
        bins others       = default;
      }
    endgroup

    // -------------------------------------------------------------------------
    // COVERGROUP: Back-to-Back Transfers
    // Exercises pipeline states WRITE_P and WENABLE_P
    // -------------------------------------------------------------------------
    covergroup cg_b2b;
      cp_sequence: coverpoint get_sequence_type() {
        bins write_write = {"WW"};   // back-to-back writes
        bins write_read  = {"WR"};   // write then read
        bins read_write  = {"RW"};   // read then write
        bins read_read   = {"RR"};   // back-to-back reads
        bins single      = {"XX"};   // no previous transfer
      }
    endgroup

    // -------------------------------------------------------------------------
    // COVERGROUP: Cross Coverage
    // Verifies both read AND write happen across all address ranges
    // -------------------------------------------------------------------------
    covergroup cg_cross;
      cp_kind: coverpoint txn.kind {
        bins write_txn = {AHB_WRITE};
        bins read_txn  = {AHB_READ};
      }
      cp_addr: coverpoint txn.addr[7:2] {
        bins low  = {[0:15]};
        bins mid  = {[16:47]};
        bins high = {[48:63]};
      }
      // Cross: every combination of kind x address range must be hit
      cx_kind_addr: cross cp_kind, cp_addr;
    endgroup

    // -------------------------------------------------------------------------
    // COVERGROUP: Delay Between Transfers
    // Exercises both back-to-back and spaced transfers
    // -------------------------------------------------------------------------
    covergroup cg_delay;
      cp_delay: coverpoint txn.delay_cycles {
        bins b2b      = {0};          // back-to-back
        bins short    = {[1:3]};      // short gap
        bins long_gap = {[4:10]};     // longer gap
      }
    endgroup

    function new(string name, uvm_component parent);
      super.new(name, parent);
      // Instantiate all covergroups
      cg_transfer_type = new();
      cg_address       = new();
      cg_data_corners  = new();
      cg_b2b           = new();
      cg_cross         = new();
      cg_delay         = new();
    endfunction

    // -------------------------------------------------------------------------
    // write(): called automatically when analysis port sends a transaction
    // uvm_subscriber base class provides the analysis_export
    // -------------------------------------------------------------------------
    function void write(ahb_apb_txn t);
      txn = t;

      // Sample all covergroups with this transaction
      cg_transfer_type.sample();
      cg_address.sample();
      cg_data_corners.sample();
      cg_b2b.sample();
      cg_cross.sample();
      cg_delay.sample();

      // Save for next transaction's back-to-back detection
      prev_txn = t;
    endfunction

    // -------------------------------------------------------------------------
    // Helper: detect back-to-back transfer sequence type
    // Returns 2-char string: "WW", "WR", "RW", "RR", "XX"
    // -------------------------------------------------------------------------
    function string get_sequence_type();
      if (prev_txn == null) return "XX";
      case ({prev_txn.kind, txn.kind})
        {AHB_WRITE, AHB_WRITE}: return "WW";
        {AHB_WRITE, AHB_READ}:  return "WR";
        {AHB_READ,  AHB_WRITE}: return "RW";
        {AHB_READ,  AHB_READ}:  return "RR";
        default:                return "XX";
      endcase
    endfunction

    // -------------------------------------------------------------------------
    // report_phase: print coverage summary
    // -------------------------------------------------------------------------
    function void report_phase(uvm_phase phase);
      `uvm_info("COV", "==========================================", UVM_NONE)
      `uvm_info("COV", "       COVERAGE REPORT                   ", UVM_NONE)
      `uvm_info("COV", "==========================================", UVM_NONE)
      `uvm_info("COV",
        $sformatf("  Transfer Type : %.1f%%",
          cg_transfer_type.get_coverage()), UVM_NONE)
      `uvm_info("COV",
        $sformatf("  Address Range : %.1f%%",
          cg_address.get_coverage()), UVM_NONE)
      `uvm_info("COV",
        $sformatf("  Data Corners  : %.1f%%",
          cg_data_corners.get_coverage()), UVM_NONE)
      `uvm_info("COV",
        $sformatf("  B2B Transfers : %.1f%%",
          cg_b2b.get_coverage()), UVM_NONE)
      `uvm_info("COV",
        $sformatf("  Cross Coverage: %.1f%%",
          cg_cross.get_coverage()), UVM_NONE)
      `uvm_info("COV",
        $sformatf("  Delay Coverage: %.1f%%",
          cg_delay.get_coverage()), UVM_NONE)
      `uvm_info("COV", "==========================================", UVM_NONE)
    endfunction

  endclass : bridge_coverage

  // ===========================================================================
  // BRIDGE ENVIRONMENT
  //
  // Top-level container that:
  //   1. Creates ahb_agent (active)
  //   2. Creates apb_agent (passive)
  //   3. Creates scoreboard
  //   4. Creates coverage collector
  //   5. Connects analysis ports together
  //
  // Connection map:
  //   ahb_agent.ap → scoreboard.ahb_fifo.analysis_export
  //   apb_agent.ap → scoreboard.apb_fifo.analysis_export
  //   apb_agent.ap → coverage.analysis_export
  // ===========================================================================
  class bridge_env extends uvm_env;
    `uvm_component_utils(bridge_env)

    // Sub-components
    // Import agent packages at top of file
    // Declared here as base types, created via factory

    // We forward-declare using string - actual types from agent packages
    // This avoids circular package dependencies
    uvm_agent        ahb_agnt;    // cast to ahb_agent in build
    uvm_agent        apb_agnt;    // cast to apb_agent in build

    bridge_scoreboard scoreboard;
    bridge_coverage   coverage;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);

      // Create agents via factory
      // Factory allows test to override agent type for reuse
      ahb_agnt   = uvm_agent::type_id::create("ahb_agnt", this);
      apb_agnt   = uvm_agent::type_id::create("apb_agnt", this);
      scoreboard = bridge_scoreboard::type_id::create("scoreboard", this);
      coverage   = bridge_coverage::type_id::create("coverage",   this);
    endfunction

    function void connect_phase(uvm_phase phase);
      // We need handles to the actual agent types for port connections
      // Downcast from uvm_agent to specific agent type
      ahb_agent_pkg::ahb_agent ahb_a;
      apb_agent_pkg::apb_agent apb_a;

      // Downcast - $cast returns 1 on success, 0 on failure
      if (!$cast(ahb_a, ahb_agnt))
        `uvm_fatal("CAST", "Failed to cast ahb_agnt to ahb_agent")
      if (!$cast(apb_a, apb_agnt))
        `uvm_fatal("CAST", "Failed to cast apb_agnt to apb_agent")

      // Connect AHB monitor → scoreboard AHB input
      ahb_a.ap.connect(scoreboard.ahb_fifo.analysis_export);

      // Connect APB monitor → scoreboard APB input
      apb_a.ap.connect(scoreboard.apb_fifo.analysis_export);

      // Connect APB monitor → coverage collector
      apb_a.ap.connect(coverage.analysis_export);
    endfunction

  endclass : bridge_env

endpackage : bridge_env_pkg
// =============================================================================
// End of bridge_env_pkg.sv
// =============================================================================