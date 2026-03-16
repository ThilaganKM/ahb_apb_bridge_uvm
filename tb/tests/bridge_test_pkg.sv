// =============================================================================
// File    : bridge_test_pkg.sv
// Project : AHB-APB Bridge UVM Verification
//
// Description:
//   Tests and sequences package containing:
//
//   SEQUENCES (stimulus generators):
//     - bridge_base_seq    : base sequence all others extend
//     - single_write_seq   : one write transfer
//     - single_read_seq    : one read transfer
//     - write_read_seq     : write then read to same address (data integrity)
//     - b2b_write_seq      : back-to-back writes (exercises WRITE_P/WENABLE_P)
//     - b2b_read_seq       : back-to-back reads
//     - rand_seq           : fully randomized transfers
//     - reset_seq          : triggers reset mid-transfer
//
//   TESTS (top-level test classes):
//     - bridge_base_test   : base test, creates env
//     - bridge_smoke_test  : single write + read
//     - bridge_write_test  : write-focused tests
//     - bridge_read_test   : read-focused tests
//     - bridge_b2b_test    : back-to-back pipeline tests
//     - bridge_rand_test   : fully random with coverage closure
//     - bridge_reset_test  : reset during transfer
//
// Sequence → Sequencer → Driver flow:
//   Sequence creates txn objects and calls start_item/finish_item
//   Sequencer receives them and passes to driver
//   Driver converts to pin activity
// =============================================================================

package bridge_test_pkg;

  import uvm_pkg::*;
  `include "uvm_macros.svh"

  import ahb_apb_txn_pkg::*;
  import ahb_agent_pkg::*;
  import apb_agent_pkg::*;
  import bridge_env_pkg::*;

  // ===========================================================================
  // BASE SEQUENCE
  //
  // All sequences extend this.
  // Provides common utilities:
  //   - create_write() : helper to build a write transaction
  //   - create_read()  : helper to build a read transaction
  //   - send()         : sends a transaction to the sequencer
  // ===========================================================================
  class bridge_base_seq extends uvm_sequence #(ahb_apb_txn);
    `uvm_object_utils(bridge_base_seq)

    function new(string name = "bridge_base_seq");
      super.new(name);
    endfunction

    // -------------------------------------------------------------------------
    // Helper: create a write transaction with specific addr/data
    // -------------------------------------------------------------------------
    function ahb_apb_txn create_write(
      logic [31:0] addr,
      logic [31:0] data
    );
      ahb_apb_txn txn = ahb_apb_txn::type_id::create("txn");
      txn.kind         = AHB_WRITE;
      txn.addr         = addr;
      txn.data         = data;
      txn.trans_type   = HTRANS_NONSEQ;
      txn.delay_cycles = 0;
      return txn;
    endfunction

    // -------------------------------------------------------------------------
    // Helper: create a read transaction
    // data field will be filled by driver after read completes
    // -------------------------------------------------------------------------
    function ahb_apb_txn create_read(logic [31:0] addr);
      ahb_apb_txn txn = ahb_apb_txn::type_id::create("txn");
      txn.kind         = AHB_READ;
      txn.addr         = addr;
      txn.data         = 32'h0;
      txn.trans_type   = HTRANS_NONSEQ;
      txn.delay_cycles = 0;
      return txn;
    endfunction

    // -------------------------------------------------------------------------
    // Helper: send a transaction through sequencer to driver
    // start_item: reserves the sequencer
    // finish_item: sends txn to driver and BLOCKS until driver calls item_done
    // -------------------------------------------------------------------------
    task send(ahb_apb_txn txn);
      start_item(txn);
      finish_item(txn);
    endtask

  endclass : bridge_base_seq

  // ===========================================================================
  // SINGLE WRITE SEQUENCE
  // ===========================================================================
  class single_write_seq extends bridge_base_seq;
    `uvm_object_utils(single_write_seq)

    // Randomizable fields so test can override
    rand logic [31:0] addr;
    rand logic [31:0] data;

    constraint c_addr { addr inside {[32'h0:32'hFC]}; addr[1:0] == 2'b00; }

    function new(string name = "single_write_seq");
      super.new(name);
    endfunction

    task body();
      ahb_apb_txn txn;
      `uvm_info("WRITE_SEQ",
        $sformatf("Single write: addr=0x%08h data=0x%08h", addr, data),
        UVM_MEDIUM)
      txn = create_write(addr, data);
      send(txn);
    endtask

  endclass : single_write_seq

  // ===========================================================================
  // SINGLE READ SEQUENCE
  // ===========================================================================
  class single_read_seq extends bridge_base_seq;
    `uvm_object_utils(single_read_seq)

    rand logic [31:0] addr;
    constraint c_addr { addr inside {[32'h0:32'hFC]}; addr[1:0] == 2'b00; }

    function new(string name = "single_read_seq");
      super.new(name);
    endfunction

    task body();
      ahb_apb_txn txn;
      `uvm_info("READ_SEQ",
        $sformatf("Single read: addr=0x%08h", addr),
        UVM_MEDIUM)
      txn = create_read(addr);
      send(txn);
    endtask

  endclass : single_read_seq

  // ===========================================================================
  // WRITE THEN READ SEQUENCE
  //
  // Writes a known value, then reads it back.
  // This is the most important data integrity check:
  //   1. Write 0xDEADBEEF to 0x20
  //   2. Read from 0x20
  //   3. Scoreboard checks hrdata == prdata == 0xDEADBEEF
  //
  // The APB slave memory model makes this work - it stores the write
  // and returns it on the subsequent read.
  // ===========================================================================
  class write_read_seq extends bridge_base_seq;
    `uvm_object_utils(write_read_seq)

    rand logic [31:0] addr;
    rand logic [31:0] data;

    constraint c_addr { addr inside {[32'h0:32'hFC]}; addr[1:0] == 2'b00; }

    function new(string name = "write_read_seq");
      super.new(name);
    endfunction

    task body();
      ahb_apb_txn wr_txn, rd_txn;

      `uvm_info("WR_SEQ",
        $sformatf("Write-Read: addr=0x%08h data=0x%08h", addr, data),
        UVM_MEDIUM)

      // Write phase
      wr_txn = create_write(addr, data);
      send(wr_txn);

      // Read phase - same address
      rd_txn = create_read(addr);
      send(rd_txn);
    endtask

  endclass : write_read_seq

  // ===========================================================================
  // BACK-TO-BACK WRITE SEQUENCE
  //
  // Sends N consecutive writes with no idle gap between them.
  // This exercises WRITE_P and WENABLE_P states in the bridge FSM.
  // These are the states most likely to have pipeline bugs.
  // ===========================================================================
  class b2b_write_seq extends bridge_base_seq;
    `uvm_object_utils(b2b_write_seq)

    // Number of back-to-back writes
    int unsigned num_writes = 4;

    function new(string name = "b2b_write_seq");
      super.new(name);
    endfunction

    task body();
      ahb_apb_txn txn;
      logic [31:0] base_addr = 32'h0000_0040;

      `uvm_info("B2B_WR_SEQ",
        $sformatf("Back-to-back writes: count=%0d", num_writes),
        UVM_MEDIUM)

      for (int i = 0; i < num_writes; i++) begin
        txn              = create_write(
                             base_addr + (i * 4),      // sequential addresses
                             32'hA000_0000 | i          // unique data per write
                           );
        txn.delay_cycles = 0;    // NO delay = back-to-back
        send(txn);
      end
    endtask

  endclass : b2b_write_seq

  // ===========================================================================
  // BACK-TO-BACK READ SEQUENCE
  // ===========================================================================
  class b2b_read_seq extends bridge_base_seq;
    `uvm_object_utils(b2b_read_seq)

    int unsigned num_reads = 4;

    function new(string name = "b2b_read_seq");
      super.new(name);
    endfunction

    task body();
      ahb_apb_txn txn;
      logic [31:0] base_addr = 32'h0000_0040;

      `uvm_info("B2B_RD_SEQ",
        $sformatf("Back-to-back reads: count=%0d", num_reads),
        UVM_MEDIUM)

      for (int i = 0; i < num_reads; i++) begin
        txn              = create_read(base_addr + (i * 4));
        txn.delay_cycles = 0;
        send(txn);
      end
    endtask

  endclass : b2b_read_seq

  // ===========================================================================
  // RANDOM SEQUENCE
  //
  // Fully randomized transfers - relies on constraints in ahb_apb_txn.
  // This is the workhorse sequence for coverage closure.
  // Run with a large count to hit all coverage bins.
  // ===========================================================================
  class rand_seq extends bridge_base_seq;
    `uvm_object_utils(rand_seq)

    // How many random transactions to generate
    int unsigned count = 50;

    function new(string name = "rand_seq");
      super.new(name);
    endfunction

    task body();
      ahb_apb_txn txn;

      `uvm_info("RAND_SEQ",
        $sformatf("Random sequence: count=%0d", count),
        UVM_MEDIUM)

      for (int i = 0; i < count; i++) begin
        txn = ahb_apb_txn::type_id::create("rand_txn");

        // Randomize - solver applies all constraints from txn class
        if (!txn.randomize())
          `uvm_fatal("RAND_FAIL", "Transaction randomization failed")

        `uvm_info("RAND_SEQ",
          $sformatf("[%0d/%0d] %s", i+1, count, txn.convert2string()),
          UVM_HIGH)

        send(txn);
      end
    endtask

  endclass : rand_seq

  // ===========================================================================
  // BASE TEST
  //
  // All tests extend this.
  // Creates and configures the environment.
  // Sets virtual interface in config_db so all components can access it.
  // ===========================================================================
  class bridge_base_test extends uvm_test;
    `uvm_component_utils(bridge_base_test)

    bridge_env env;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);

      // Override agent types via factory
      // This tells the factory: when someone creates a uvm_agent
      // under "ahb_agnt", use ahb_agent type
      ahb_agent_pkg::ahb_agent::type_id::set_type_override(
        ahb_agent_pkg::ahb_agent::get_type());
      apb_agent_pkg::apb_agent::type_id::set_type_override(
        apb_agent_pkg::apb_agent::get_type());

      // Create environment
      env = bridge_env::type_id::create("env", this);
    endfunction

    // -------------------------------------------------------------------------
    // end_of_elaboration_phase: print topology
    // Useful for verifying the UVM hierarchy is built correctly
    // -------------------------------------------------------------------------
    function void end_of_elaboration_phase(uvm_phase phase);
      uvm_top.print_topology();
    endfunction

    // -------------------------------------------------------------------------
    // Helper: get sequencer handle from env
    // All tests use this to start sequences
    // -------------------------------------------------------------------------
    function ahb_sequencer get_sequencer();
      ahb_agent_pkg::ahb_agent ahb_a;
      if (!$cast(ahb_a, env.ahb_agnt))
        `uvm_fatal("CAST", "Cannot get sequencer - cast failed")
      return ahb_a.seqr;
    endfunction

  endclass : bridge_base_test

  // ===========================================================================
  // SMOKE TEST
  //
  // Simplest test - one write, one read.
  // First test to run on any new environment setup.
  // If this fails, something fundamental is broken.
  // ===========================================================================
  class bridge_smoke_test extends bridge_base_test;
    `uvm_component_utils(bridge_smoke_test)

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    task run_phase(uvm_phase phase);
      single_write_seq wr_seq;
      single_read_seq  rd_seq;

      phase.raise_objection(this);

      `uvm_info("SMOKE", "Starting smoke test", UVM_NONE)

      // Single write
      wr_seq      = single_write_seq::type_id::create("wr_seq");
      wr_seq.addr = 32'h0000_0020;
      wr_seq.data = 32'hDEAD_BEEF;
      wr_seq.start(get_sequencer());

      // Single read from same address
      rd_seq      = single_read_seq::type_id::create("rd_seq");
      rd_seq.addr = 32'h0000_0020;
      rd_seq.start(get_sequencer());

      phase.drop_objection(this);
    endtask

  endclass : bridge_smoke_test

  // ===========================================================================
  // WRITE TEST
  // ===========================================================================
  class bridge_write_test extends bridge_base_test;
    `uvm_component_utils(bridge_write_test)

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    task run_phase(uvm_phase phase);
      write_read_seq wr_rd;

      phase.raise_objection(this);
      `uvm_info("WRITE_TEST", "Starting write test", UVM_NONE)

      // Write-read pairs at multiple addresses
      for (int i=0;i<5;i++) begin
        wr_rd      = write_read_seq::type_id::create("wr_rd");
        wr_rd.addr = 32'h0000_0000 + (i * 8);
        wr_rd.data = 32'hCAFE_0000 | i;
        wr_rd.start(get_sequencer());
      end

      phase.drop_objection(this);
    endtask

  endclass : bridge_write_test

  // ===========================================================================
  // READ TEST
  // ===========================================================================
  class bridge_read_test extends bridge_base_test;
    `uvm_component_utils(bridge_read_test)

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    task run_phase(uvm_phase phase);
      b2b_read_seq rd_seq;

      phase.raise_objection(this);
      `uvm_info("READ_TEST", "Starting read test", UVM_NONE)

      rd_seq           = b2b_read_seq::type_id::create("rd_seq");
      rd_seq.num_reads = 8;
      rd_seq.start(get_sequencer());

      phase.drop_objection(this);
    endtask

  endclass : bridge_read_test

  // ===========================================================================
  // BACK-TO-BACK TEST
  // Exercises pipeline states WRITE_P and WENABLE_P
  // ===========================================================================
  class bridge_b2b_test extends bridge_base_test;
    `uvm_component_utils(bridge_b2b_test)

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    task run_phase(uvm_phase phase);
      b2b_write_seq wr_seq;
      b2b_read_seq  rd_seq;

      phase.raise_objection(this);
      `uvm_info("B2B_TEST", "Starting back-to-back test", UVM_NONE)

      // Back-to-back writes
      wr_seq            = b2b_write_seq::type_id::create("wr_seq");
      wr_seq.num_writes = 8;
      wr_seq.start(get_sequencer());

      // Back-to-back reads
      rd_seq           = b2b_read_seq::type_id::create("rd_seq");
      rd_seq.num_reads = 8;
      rd_seq.start(get_sequencer());

      phase.drop_objection(this);
    endtask

  endclass : bridge_b2b_test

  // ===========================================================================
  // RANDOM TEST
  // Main coverage closure test - run with high count
  // ===========================================================================
  class bridge_rand_test extends bridge_base_test;
    `uvm_component_utils(bridge_rand_test)

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    task run_phase(uvm_phase phase);
      rand_seq rseq;

      phase.raise_objection(this);
      `uvm_info("RAND_TEST", "Starting random test", UVM_NONE)

      rseq       = rand_seq::type_id::create("rseq");
      rseq.count = 100;
      rseq.start(get_sequencer());

      phase.drop_objection(this);
    endtask

  endclass : bridge_rand_test

  // ===========================================================================
  // RESET TEST
  // Assert reset mid-transfer, verify bridge recovers cleanly
  // ===========================================================================
  class bridge_reset_test extends bridge_base_test;
    `uvm_component_utils(bridge_reset_test)

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    task run_phase(uvm_phase phase);
      single_write_seq wr_seq;
      virtual ahb_apb_if vif;

      phase.raise_objection(this);
      `uvm_info("RESET_TEST", "Starting reset test", UVM_NONE)

      // Get virtual interface to assert reset
      if (!uvm_config_db #(virtual ahb_apb_if)::get(
            this, "", "vif", vif))
        `uvm_fatal("NOVIF", "Cannot get vif for reset test")

      // Start a write
      wr_seq      = single_write_seq::type_id::create("wr_seq");
      wr_seq.addr = 32'h0000_0080;
      wr_seq.data = 32'hBEEF_CAFE;
      fork
        wr_seq.start(get_sequencer());
      join_none

      // Assert reset after 2 cycles
      repeat(2) @(posedge vif.hclk);
      `uvm_info("RESET_TEST", "Asserting reset mid-transfer", UVM_NONE)
      vif.hresetn = 0;
      repeat(3) @(posedge vif.hclk);
      vif.hresetn = 1;
      `uvm_info("RESET_TEST", "Reset released", UVM_NONE)

      // Wait for any in-flight sequence to complete
      repeat(5) @(posedge vif.hclk);

      // Verify bridge works after reset
      wr_seq      = single_write_seq::type_id::create("post_rst_wr");
      wr_seq.addr = 32'h0000_0090;
      wr_seq.data = 32'h5A5A_5A5A;
      wr_seq.start(get_sequencer());

      phase.drop_objection(this);
    endtask

  endclass : bridge_reset_test

endpackage : bridge_test_pkg
// =============================================================================
// End of bridge_test_pkg.sv
// =============================================================================

// ===========================================================================
  // DATA CORNERS SEQUENCE
  // Explicitly drives all-zeros, all-ones, alternating patterns
  // These are classic bug triggers that random testing rarely hits
  // ===========================================================================
class data_corners_seq extends bridge_base_seq;
    `uvm_object_utils(data_corners_seq)

    function new(string name = "data_corners_seq");
      super.new(name);
    endfunction

    task body();
      ahb_apb_txn txn;

      `uvm_info("CORNERS_SEQ", "Driving data corner cases", UVM_MEDIUM)

      // All zeros - tests zero propagation bugs
      txn = create_write(32'h0000_0010, 32'h0000_0000);
      send(txn);

      // All ones - tests overflow/saturation bugs
      txn = create_write(32'h0000_0014, 32'hFFFF_FFFF);
      send(txn);

      // Alternating 1010 - tests crosstalk between adjacent bits
      txn = create_write(32'h0000_0018, 32'hAAAA_AAAA);
      send(txn);

      // Alternating 0101 - complement of above
      txn = create_write(32'h0000_001C, 32'h5555_5555);
      send(txn);

      // Read back all four to verify data integrity through bridge
      txn = create_read(32'h0000_0010); send(txn);
      txn = create_read(32'h0000_0014); send(txn);
      txn = create_read(32'h0000_0018); send(txn);
      txn = create_read(32'h0000_001C); send(txn);

      `uvm_info("CORNERS_SEQ", "Data corner cases complete", UVM_MEDIUM)
    endtask

endclass : data_corners_seq

  // ===========================================================================
  // DATA CORNERS TEST
  // Runs corner case sequence first then random for full coverage closure
  // ===========================================================================
class bridge_corners_test extends bridge_base_test;
    `uvm_component_utils(bridge_corners_test)

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    task run_phase(uvm_phase phase);
      data_corners_seq corners;
      rand_seq         rseq;

      phase.raise_objection(this);
      `uvm_info("CORNERS_TEST", "Starting data corners test", UVM_NONE)

      // Step 1: hit corner cases explicitly
      corners = data_corners_seq::type_id::create("corners");
      corners.start(get_sequencer());

      // Step 2: random for everything else
      rseq       = rand_seq::type_id::create("rseq");
      rseq.count = 100;
      rseq.start(get_sequencer());

      phase.drop_objection(this);
    endtask

endclass : bridge_corners_test