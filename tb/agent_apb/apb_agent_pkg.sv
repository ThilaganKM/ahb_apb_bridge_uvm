// =============================================================================
// File    : apb_agent_pkg.sv
// Project : AHB-APB Bridge UVM Verification
//
// Description:
//   APB Agent package containing:
//     - apb_monitor      : observes APB bus, builds transactions
//     - apb_slave_model  : drives prdata response during reads
//     - apb_agent        : container (PASSIVE - no driver/sequencer)
//
// This agent is PASSIVE on the APB side - the bridge is the APB master.
// We never drive APB control signals (psel, penable, pwrite, paddr, pwdata).
// We only:
//   1. Observe what the bridge drives (monitor)
//   2. Respond with read data (slave model drives prdata)
//
// WHY a separate APB agent?
//   AHB monitor sees: what the AHB master REQUESTED
//   APB monitor sees: what the bridge actually DROVE to the peripheral
//   Scoreboard compares both - verifying correct translation
//   Example check: did paddr match haddr? did pwdata match hwdata?
// =============================================================================

package apb_agent_pkg;

  import uvm_pkg::*;
  `include "uvm_macros.svh"

  import ahb_apb_txn_pkg::*;

  // ===========================================================================
  // APB PERIPHERAL MODEL (Slave Model)
  //
  // Simulates an APB peripheral responding to bridge transactions.
  // Drives prdata when bridge performs a read (psel=1, penable=1, pwrite=0).
  //
  // Simple memory model:
  //   - 256-entry x 32-bit memory array
  //   - Write: stores pwdata at paddr[7:2] (word-addressed)
  //   - Read:  returns stored data on prdata
  //
  // This makes read-after-write tests meaningful:
  //   Write 0xDEADBEEF to 0x20
  //   Read  from 0x20 → should get 0xDEADBEEF back
  // ===========================================================================
  class apb_slave_model extends uvm_component;
    `uvm_component_utils(apb_slave_model)

    // Virtual interface
    virtual ahb_apb_if vif;

    // Simple memory model - 256 words of 32 bits
    // Index = paddr[7:2] (word address, ignoring byte offset)
    logic [31:0] mem [0:255];

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db #(virtual ahb_apb_if)::get(
            this, "", "vif", vif))
        `uvm_fatal("NOVIF",
          "apb_slave_model: virtual interface not found in config_db")

      // Initialize memory to known pattern
      // Using address-based pattern so reads return predictable data
      // even before a write has occurred
      foreach (mem[i])
        mem[i] = 32'hA5A5_0000 | i;   // e.g. mem[5] = 0xA5A50005
    endfunction

    // -------------------------------------------------------------------------
    // run_phase: respond to APB transactions
    //
    // Monitor psel+penable - this is the APB ENABLE phase when:
    //   - Write: peripheral should latch pwdata (we store in mem)
    //   - Read:  peripheral should drive prdata (we drive from mem)
    //
    // We drive prdata COMBINATIONALLY (via always_comb equivalent)
    // as soon as psel goes high - peripheral must have data ready
    // before penable, per APB spec.
    // -------------------------------------------------------------------------
    task run_phase(uvm_phase phase);
      // Initialize prdata to 0
      vif.apb_slave_cb.prdata <= 32'h0;

      // Wait for reset
      @(posedge vif.hclk);
      wait (vif.hresetn === 1'b1);

      forever begin
        @(vif.apb_slave_cb);

        // APB SETUP phase: psel=1, penable=0
        // Pre-drive prdata so it's ready before ENABLE phase
        if (vif.apb_slave_cb.psel && !vif.apb_slave_cb.penable) begin
          if (!vif.apb_slave_cb.pwrite) begin
            // Read: put data on prdata immediately in SETUP phase
            automatic logic [7:0] idx = vif.apb_slave_cb.paddr[7:2];
            vif.apb_slave_cb.prdata <= mem[idx];
            `uvm_info("APB_SLAVE",
              $sformatf("READ SETUP: paddr=0x%08h prdata=0x%08h",
                vif.apb_slave_cb.paddr, mem[idx]),
              UVM_HIGH)
          end
        end

        // APB ENABLE phase: psel=1, penable=1
        // Transfer completes here
        if (vif.apb_slave_cb.psel && vif.apb_slave_cb.penable) begin
          if (vif.apb_slave_cb.pwrite) begin
            // Write: store data in memory
            automatic logic [7:0] idx = vif.apb_slave_cb.paddr[7:2];
            mem[idx] = vif.apb_slave_cb.pwdata;
            `uvm_info("APB_SLAVE",
              $sformatf("WRITE: paddr=0x%08h pwdata=0x%08h stored at mem[%0d]",
                vif.apb_slave_cb.paddr, vif.apb_slave_cb.pwdata, idx),
              UVM_HIGH)
          end else begin
            // Read complete - data already on prdata from SETUP phase
            `uvm_info("APB_SLAVE",
              $sformatf("READ ENABLE: paddr=0x%08h prdata=0x%08h",
                vif.apb_slave_cb.paddr, vif.apb_slave_cb.prdata),
              UVM_HIGH)
          end
        end

        // Deassert prdata when bus goes idle
        if (!vif.apb_slave_cb.psel)
          vif.apb_slave_cb.prdata <= 32'h0;

      end
    endtask

    // -------------------------------------------------------------------------
    // Utility: peek at memory location (used by scoreboard)
    // -------------------------------------------------------------------------
    function logic [31:0] mem_read(logic [31:0] addr);
      return mem[addr[7:2]];
    endfunction

  endclass : apb_slave_model

  // ===========================================================================
  // APB MONITOR
  //
  // Observes APB bus signals driven by the bridge.
  // Builds transaction objects from completed APB transfers.
  // Writes to analysis port for scoreboard.
  //
  // WHEN does the monitor capture?
  //   At psel=1, penable=1 (ENABLE phase) - this is when APB transfer
  //   is complete and all signals are stable and valid.
  //
  // WHAT does it capture?
  //   paddr  → goes into txn.addr
  //   pwdata → goes into txn.data (for writes)
  //   prdata → goes into txn.data (for reads)
  //   pwrite → determines txn.kind
  // ===========================================================================
  class apb_monitor extends uvm_monitor;
    `uvm_component_utils(apb_monitor)

    // Analysis port - sends captured APB transactions to scoreboard
    uvm_analysis_port #(ahb_apb_txn) ap;

    // Virtual interface
    virtual ahb_apb_if vif;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      ap = new("ap", this);
      if (!uvm_config_db #(virtual ahb_apb_if)::get(
            this, "", "vif", vif))
        `uvm_fatal("NOVIF",
          "apb_monitor: virtual interface not found in config_db")
    endfunction

    // -------------------------------------------------------------------------
    // run_phase: monitor loop
    //
    // Simply waits for psel=1 && penable=1 on every posedge.
    // This is the APB ENABLE phase - transfer is complete.
    // Captures all relevant signals and broadcasts transaction.
    // -------------------------------------------------------------------------
    task run_phase(uvm_phase phase);
      ahb_apb_txn txn;

      // Wait for reset
      @(posedge vif.hclk);
      wait (vif.hresetn === 1'b1);

      forever begin
        @(vif.monitor_cb);

        // APB ENABLE phase detection
        if (vif.monitor_cb.psel && vif.monitor_cb.penable) begin
          txn = ahb_apb_txn::type_id::create("apb_mon_txn");

          txn.addr       = vif.monitor_cb.paddr;
          txn.trans_type = HTRANS_NONSEQ;
          txn.resp       = HRESP_OKAY;

          if (vif.monitor_cb.pwrite) begin
            // Write transaction
            txn.kind = AHB_WRITE;
            txn.data = vif.monitor_cb.pwdata;
          end else begin
            // Read transaction - capture prdata (peripheral response)
            txn.kind = AHB_READ;
            txn.data = vif.monitor_cb.prdata;
          end

          `uvm_info("APB_MON",
            $sformatf("Captured: %s", txn.convert2string()),
            UVM_MEDIUM)

          // Broadcast to scoreboard
          ap.write(txn);
        end
      end
    endtask

  endclass : apb_monitor

  // ===========================================================================
  // APB AGENT
  //
  // PASSIVE agent - contains monitor and slave model only.
  // No sequencer or driver - bridge is the APB master, not us.
  //
  // is_active = UVM_PASSIVE (set in env build_phase)
  // ===========================================================================
  class apb_agent extends uvm_agent;
    `uvm_component_utils(apb_agent)

    // Sub-components
    apb_monitor     mon;
    apb_slave_model slave;

    // Analysis port forwarded from monitor to env/scoreboard
    uvm_analysis_port #(ahb_apb_txn) ap;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      mon   = apb_monitor::type_id::create("mon",   this);
      slave = apb_slave_model::type_id::create("slave", this);
      ap    = new("ap", this);
    endfunction

    function void connect_phase(uvm_phase phase);
      // Forward monitor analysis port up to agent level
      mon.ap.connect(ap);
    endfunction

  endclass : apb_agent

endpackage : apb_agent_pkg
// =============================================================================
// End of apb_agent_pkg.sv
// =============================================================================