// =============================================================================
// File    : bridge_env_pkg.sv
// Project : AHB-APB Bridge UVM Verification
// =============================================================================

package bridge_env_pkg;

  import uvm_pkg::*;
  `include "uvm_macros.svh"

  import ahb_apb_txn_pkg::*;
  import ahb_agent_pkg::*;
  import apb_agent_pkg::*;

  // ===========================================================================
  // SCOREBOARD
  // ===========================================================================
  class bridge_scoreboard extends uvm_scoreboard;
    `uvm_component_utils(bridge_scoreboard)

    uvm_tlm_analysis_fifo #(ahb_apb_txn) ahb_fifo;
    uvm_tlm_analysis_fifo #(ahb_apb_txn) apb_fifo;

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

    task run_phase(uvm_phase phase);
      ahb_apb_txn ahb_txn, apb_txn;
      forever begin
        ahb_fifo.get(ahb_txn);
        apb_fifo.get(apb_txn);
        check_transaction(ahb_txn, apb_txn);
      end
    endtask

    task check_transaction(ahb_apb_txn ahb_txn, ahb_apb_txn apb_txn);
      bit pass = 1;
      total_checks++;

      // Check 1: Address
      if (apb_txn.addr !== ahb_txn.addr) begin
        `uvm_error("SBOARD",
          $sformatf("ADDR MISMATCH: haddr=0x%08h paddr=0x%08h",
            ahb_txn.addr, apb_txn.addr))
        pass = 0;
      end

      // Check 2: Direction
      if (apb_txn.kind !== ahb_txn.kind) begin
        `uvm_error("SBOARD",
          $sformatf("KIND MISMATCH: ahb=%s apb=%s",
            ahb_txn.kind.name(), apb_txn.kind.name()))
        pass = 0;
      end

      // Check 3: Write data
      if (ahb_txn.kind == AHB_WRITE && apb_txn.data !== ahb_txn.data) begin
        `uvm_error("SBOARD",
          $sformatf("WDATA MISMATCH: hwdata=0x%08h pwdata=0x%08h",
            ahb_txn.data, apb_txn.data))
        pass = 0;
      end

      // Check 4: Read data
      if (ahb_txn.kind == AHB_READ && apb_txn.data !== ahb_txn.data) begin
        `uvm_error("SBOARD",
          $sformatf("RDATA MISMATCH: prdata=0x%08h hrdata=0x%08h",
            apb_txn.data, ahb_txn.data))
        pass = 0;
      end

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

    function void report_phase(uvm_phase phase);
      `uvm_info("SBOARD", "==========================================", UVM_NONE)
      `uvm_info("SBOARD", "       SCOREBOARD FINAL REPORT           ", UVM_NONE)
      `uvm_info("SBOARD", "==========================================", UVM_NONE)
      `uvm_info("SBOARD", $sformatf("  Total  : %0d", total_checks),  UVM_NONE)
      `uvm_info("SBOARD", $sformatf("  Passed : %0d", passed_checks), UVM_NONE)
      `uvm_info("SBOARD", $sformatf("  Failed : %0d", failed_checks), UVM_NONE)
      if (failed_checks == 0)
        `uvm_info("SBOARD",  "  STATUS : ALL CHECKS PASSED", UVM_NONE)
      else
        `uvm_error("SBOARD",
          $sformatf("  STATUS : %0d CHECKS FAILED", failed_checks))
      `uvm_info("SBOARD", "==========================================", UVM_NONE)
    endfunction

  endclass : bridge_scoreboard

  // ===========================================================================
  // COVERAGE COLLECTOR
  // ===========================================================================
  class bridge_coverage extends uvm_subscriber #(ahb_apb_txn);
    `uvm_component_utils(bridge_coverage)

    ahb_apb_txn txn;
    ahb_apb_txn prev_txn;

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

    covergroup cg_address;
      cp_addr_range: coverpoint txn.addr[7:2] {
        bins low_range  = {[0:15]};
        bins mid_range  = {[16:47]};
        bins high_range = {[48:63]};
      }
    endgroup

    covergroup cg_data_corners;
      cp_data: coverpoint txn.data {
        bins all_zeros    = {32'h0000_0000};
        bins all_ones     = {32'hFFFF_FFFF};
        bins alternating1 = {32'hAAAA_AAAA};
        bins alternating2 = {32'h5555_5555};
        bins others       = default;
      }
    endgroup

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
      cx_kind_addr: cross cp_kind, cp_addr;
    endgroup

    covergroup cg_delay;
      cp_delay: coverpoint txn.delay_cycles {
        bins b2b      = {0};
        bins short_d  = {[1:3]};
        bins long_d   = {[4:10]};
      }
    endgroup

    function new(string name, uvm_component parent);
      super.new(name, parent);
      cg_transfer_type = new();
      cg_address       = new();
      cg_data_corners  = new();
      cg_cross         = new();
      cg_delay         = new();
    endfunction

    function void write(ahb_apb_txn t);
      txn = t;
      cg_transfer_type.sample();
      cg_address.sample();
      cg_data_corners.sample();
      cg_cross.sample();
      cg_delay.sample();
      prev_txn = t;
    endfunction

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
  // FIX: Use concrete agent types directly instead of uvm_agent base class.
  // uvm_agent::type_id doesn't work in UVM-1.1d without explicit registration.
  // Declare ahb_agent and apb_agent directly - cleaner and correct.
  // ===========================================================================
  class bridge_env extends uvm_env;
    `uvm_component_utils(bridge_env)

    // FIX: Use concrete types directly - no casting needed
    ahb_agent         ahb_agnt;
    apb_agent         apb_agnt;
    bridge_scoreboard scoreboard;
    bridge_coverage   coverage;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      // FIX: Create using concrete type_id directly
      ahb_agnt   = ahb_agent::type_id::create("ahb_agnt",   this);
      apb_agnt   = apb_agent::type_id::create("apb_agnt",   this);
      scoreboard = bridge_scoreboard::type_id::create("scoreboard", this);
      coverage   = bridge_coverage::type_id::create("coverage",     this);
    endfunction

    function void connect_phase(uvm_phase phase);
      // No casting needed - already concrete types
      // Connect AHB monitor → scoreboard
      ahb_agnt.ap.connect(scoreboard.ahb_fifo.analysis_export);

      // Connect APB monitor → scoreboard
      apb_agnt.ap.connect(scoreboard.apb_fifo.analysis_export);

      // Connect APB monitor → coverage
      apb_agnt.ap.connect(coverage.analysis_export);
    endfunction

  endclass : bridge_env

endpackage : bridge_env_pkg