// =============================================================================
// File    : ahb_agent_pkg.sv
// Project : AHB-APB Bridge UVM Verification
//
// Description:
//   AHB Agent package containing:
//     - ahb_sequencer : standard UVM sequencer (transaction pipe)
//     - ahb_driver    : converts transactions to AHB pin activity
//     - ahb_monitor   : observes AHB pins, builds transactions
//     - ahb_agent     : container that builds and connects all three
//
// Agent is ACTIVE - it drives stimulus onto the DUT.
// (An PASSIVE agent would have no driver/sequencer - monitor only)
// =============================================================================

package ahb_agent_pkg;

  import uvm_pkg::*;
  `include "uvm_macros.svh"

  import ahb_apb_txn_pkg::*;

  // ===========================================================================
  // AHB SEQUENCER
  //
  // No custom logic needed - standard UVM sequencer parameterized
  // with our transaction type. The sequencer is just a FIFO pipe
  // between sequences and the driver.
  //
  // Sequence calls:    start_item(txn) → finish_item(txn)
  // Sequencer passes:  txn to driver via TLM seq_item_port
  // Driver calls:      get_next_item(txn) → item_done()
  // ===========================================================================
  class ahb_sequencer extends uvm_sequencer #(ahb_apb_txn);
    `uvm_component_utils(ahb_sequencer)

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

  endclass : ahb_sequencer

  // ===========================================================================
  // AHB DRIVER
  //
  // Receives ahb_apb_txn objects from sequencer.
  // Converts each transaction into cycle-accurate AHB pin activity.
  //
  // Key design decisions:
  //   1. Uses clocking block (master_cb) for all signal drives
  //      - ensures setup/hold times are met automatically
  //      - no manual @negedge needed
  //   2. Waits for hready=1 before completing each transaction
  //      - bridge may insert wait states (hready=0) during APB
  //      - driver must respect this or it'll send next txn too early
  //   3. Drives inter-transaction delay (txn.delay_cycles idle cycles)
  //      - exercises bridge IDLE state between transfers
  // ===========================================================================
  class ahb_driver extends uvm_driver #(ahb_apb_txn);
    `uvm_component_utils(ahb_driver)

    // Virtual interface handle
    // Set via uvm_config_db in tb_top, retrieved in connect_phase
    virtual ahb_apb_if vif;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    // -------------------------------------------------------------------------
    // build_phase: get virtual interface from config_db
    // -------------------------------------------------------------------------
    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db #(virtual ahb_apb_if)::get(
            this, "", "vif", vif))
        `uvm_fatal("NOVIF",
          "ahb_driver: virtual interface not found in config_db")
    endfunction

    // -------------------------------------------------------------------------
    // run_phase: main driver loop
    // Runs forever - gets transactions and drives them one by one
    // -------------------------------------------------------------------------
    task run_phase(uvm_phase phase);
    ahb_apb_txn txn;
    logic [31:0] saved_addr;
    logic [31:0] saved_data;   // ADD THIS
    logic        saved_write;
    logic        transfer_pending;

    transfer_pending = 0;

    @(posedge vif.hclk);
    wait (vif.hresetn === 1'b1);

    forever begin
        @(vif.monitor_cb);

        // Address phase: capture addr and write direction
        if (vif.monitor_cb.hselapb &&
            vif.monitor_cb.htrans == HTRANS_NONSEQ) begin
        saved_addr       = vif.monitor_cb.haddr;
        saved_write      = vif.monitor_cb.hwrite;
        transfer_pending = 1;
        end

        // Data phase: hwdata is valid one cycle after address phase
        // Capture it here before it gets zeroed
        if (transfer_pending && saved_write)
        saved_data = vif.monitor_cb.hwdata;  // ADD THIS

        // Completion: hready=1
        if (transfer_pending && vif.monitor_cb.hready) begin
        txn = ahb_apb_txn::type_id::create("ahb_mon_txn");
        txn.addr       = saved_addr;
        txn.kind       = saved_write ? AHB_WRITE : AHB_READ;
        txn.trans_type = HTRANS_NONSEQ;
        txn.resp       = vif.monitor_cb.hresp ? HRESP_ERROR : HRESP_OKAY;

        // FIX: use saved_data for writes, hrdata for reads
        if (saved_write)
            txn.data = saved_data;       // was: vif.monitor_cb.hwdata (already 0)
        else
            txn.data = vif.monitor_cb.hrdata;

        `uvm_info("AHB_MON",
            $sformatf("Captured: %s", txn.convert2string()),
            UVM_MEDIUM)

        ap.write(txn);
        transfer_pending = 0;
        end
    end
    endtask

    // -------------------------------------------------------------------------
    // TASK: drive_write
    //
    // AHB Write protocol:
    //   Cycle 1 (address phase): hselapb=1, haddr, hwrite=1, htrans=NONSEQ
    //   Cycle 2 (data phase):    hwdata valid, hselapb=0, htrans=IDLE
    //   Wait:                    hready=1 (bridge stalls with hready=0)
    //
    // Using clocking block: ##1 advances one clock cycle
    // Drives via master_cb: signals scheduled at output skew (#1 after posedge)
    // -------------------------------------------------------------------------
    task drive_write(ahb_apb_txn txn);
      // Address phase - drive on clocking block
      // ##1 = wait for next posedge then drive (with output skew)
      @(vif.master_cb);
      vif.master_cb.hselapb <= 1'b1;
      vif.master_cb.haddr   <= txn.addr;
      vif.master_cb.hwrite  <= 1'b1;
      vif.master_cb.htrans  <= HTRANS_NONSEQ;
      vif.master_cb.hwdata  <= 32'h0;      // hwdata not valid yet in addr phase

      // Data phase - one cycle after address phase
      @(vif.master_cb);
      vif.master_cb.hwdata  <= txn.data;   // NOW hwdata is valid
      vif.master_cb.hselapb <= 1'b0;
      vif.master_cb.htrans  <= HTRANS_IDLE;
      vif.master_cb.haddr   <= 32'h0;
      vif.master_cb.hwrite  <= 1'b0;

      // Wait for bridge to complete APB transfer
      // Bridge asserts hready=0 while APB is in progress
      // Sample hready via clocking block (input skew = #1step)
      @(vif.master_cb);
      while (!vif.master_cb.hready)
        @(vif.master_cb);

      `uvm_info("AHB_DRV",
        $sformatf("WRITE done: addr=0x%08h data=0x%08h",
          txn.addr, txn.data),
        UVM_HIGH)
    endtask

    // -------------------------------------------------------------------------
    // TASK: drive_read
    //
    // AHB Read protocol:
    //   Cycle 1 (address phase): hselapb=1, haddr, hwrite=0, htrans=NONSEQ
    //   Cycle 2:                 hselapb=0, htrans=IDLE
    //   Wait:                    hready=1 (bridge stalls during APB read)
    //   Capture:                 hrdata valid when hready=1
    //
    // Note: No data phase from master for reads (master receives data, not sends)
    // prdata is driven by APB slave model in tb_top, not here
    // -------------------------------------------------------------------------
    task drive_read(ahb_apb_txn txn);
      // Address phase
      @(vif.master_cb);
      vif.master_cb.hselapb <= 1'b1;
      vif.master_cb.haddr   <= txn.addr;
      vif.master_cb.hwrite  <= 1'b0;
      vif.master_cb.htrans  <= HTRANS_NONSEQ;
      vif.master_cb.hwdata  <= 32'h0;

      // Deassert after address phase
      @(vif.master_cb);
      vif.master_cb.hselapb <= 1'b0;
      vif.master_cb.htrans  <= HTRANS_IDLE;
      vif.master_cb.haddr   <= 32'h0;

      // Wait for hready=1 - read data is valid at this moment
      @(vif.master_cb);
      while (!vif.master_cb.hready)
        @(vif.master_cb);

      // Capture read data into transaction
      // Monitor will also capture this independently for scoreboard
      txn.data = vif.master_cb.hrdata;

      `uvm_info("AHB_DRV",
        $sformatf("READ done: addr=0x%08h hrdata=0x%08h",
          txn.addr, txn.data),
        UVM_HIGH)
    endtask

    // -------------------------------------------------------------------------
    // TASK: drive_idle - put bus in idle state
    // -------------------------------------------------------------------------
    task drive_idle();
      vif.master_cb.hselapb <= 1'b0;
      vif.master_cb.haddr   <= 32'h0;
      vif.master_cb.hwrite  <= 1'b0;
      vif.master_cb.htrans  <= HTRANS_IDLE;
      vif.master_cb.hwdata  <= 32'h0;
    endtask

    // -------------------------------------------------------------------------
    // TASK: drive_idle_cycles - drive N idle cycles between transfers
    // -------------------------------------------------------------------------
    task drive_idle_cycles(int unsigned n);
      drive_idle();
      repeat(n) @(vif.master_cb);
    endtask

  endclass : ahb_driver

  // ===========================================================================
  // AHB MONITOR
  //
  // Observes the AHB interface passively - never drives signals.
  // Builds transaction objects from observed pin activity.
  // Writes completed transactions to analysis port for scoreboard.
  //
  // WHAT does the AHB monitor observe?
  //   It watches the AHB side: haddr, hwdata, hwrite, hready, hrdata.
  //   When hready=1, a transfer has completed - capture the transaction.
  //
  // WHY monitor the AHB side separately from APB monitor?
  //   AHB monitor captures what the MASTER requested.
  //   APB monitor captures what the BRIDGE actually drove to peripheral.
  //   Scoreboard compares both - verifying the bridge translated correctly.
  // ===========================================================================
  class ahb_monitor extends uvm_monitor;
    `uvm_component_utils(ahb_monitor)

    // Analysis port - broadcasts completed transactions to scoreboard
    // Any component that connects to this port receives the transaction
    // Using write() method - non-blocking broadcast
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
          "ahb_monitor: virtual interface not found in config_db")
    endfunction

    // -------------------------------------------------------------------------
    // run_phase: monitor loop
    //
    // Strategy: sample on every posedge, detect completed transfers.
    // A transfer is complete when hready=1 AND hselapb was asserted.
    //
    // We track state across cycles using saved_addr/saved_write
    // because AHB address and data phases are in different cycles.
    // -------------------------------------------------------------------------
    task run_phase(uvm_phase phase);
      ahb_apb_txn txn;
      logic [31:0] saved_addr;
      logic        saved_write;
      logic        transfer_pending;

      transfer_pending = 0;

      // Wait for reset
      @(posedge vif.hclk);
      wait (vif.hresetn === 1'b1);

      forever begin
        @(vif.monitor_cb);

        // Detect address phase: valid transfer starting
        // hselapb=1 and htrans=NONSEQ means new transfer
        if (vif.monitor_cb.hselapb &&
            vif.monitor_cb.htrans == HTRANS_NONSEQ) begin
          saved_addr    = vif.monitor_cb.haddr;
          saved_write   = vif.monitor_cb.hwrite;
          transfer_pending = 1;
        end

        // Detect transfer completion: hready=1 with pending transfer
        // At this point all signals are settled and valid
        if (transfer_pending && vif.monitor_cb.hready) begin
          txn = ahb_apb_txn::type_id::create("ahb_mon_txn");

          txn.addr       = saved_addr;
          txn.kind       = saved_write ? AHB_WRITE : AHB_READ;
          txn.trans_type = HTRANS_NONSEQ;
          txn.resp       = vif.monitor_cb.hresp ?
                           HRESP_ERROR : HRESP_OKAY;

          // For writes: capture hwdata (valid during data phase)
          // For reads:  capture hrdata (valid when hready=1)
          if (saved_write)
            txn.data = vif.monitor_cb.hwdata;
          else
            txn.data = vif.monitor_cb.hrdata;

          `uvm_info("AHB_MON",
            $sformatf("Captured: %s", txn.convert2string()),
            UVM_MEDIUM)

          // Broadcast to scoreboard via analysis port
          ap.write(txn);

          transfer_pending = 0;
        end
      end
    endtask

  endclass : ahb_monitor

  // ===========================================================================
  // AHB AGENT
  //
  // Container class that instantiates and connects:
  //   sequencer → driver  (TLM seq_item_port connection)
  //   monitor             (independent, connected to analysis port)
  //
  // is_active = UVM_ACTIVE:  creates sequencer + driver + monitor
  // is_active = UVM_PASSIVE: creates monitor only (for reuse as passive agent)
  // ===========================================================================
  class ahb_agent extends uvm_agent;
    `uvm_component_utils(ahb_agent)

    // Sub-components
    ahb_sequencer seqr;
    ahb_driver    drv;
    ahb_monitor   mon;

    // Analysis port forwarded from monitor
    // Env connects to this port to reach scoreboard
    uvm_analysis_port #(ahb_apb_txn) ap;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    // -------------------------------------------------------------------------
    // build_phase: create sub-components based on is_active
    // -------------------------------------------------------------------------
    function void build_phase(uvm_phase phase);
      super.build_phase(phase);

      // Always create monitor
      mon = ahb_monitor::type_id::create("mon", this);
      ap  = new("ap", this);

      // Only create driver+sequencer if agent is active
      if (get_is_active() == UVM_ACTIVE) begin
        seqr = ahb_sequencer::type_id::create("seqr", this);
        drv  = ahb_driver::type_id::create("drv",  this);
      end
    endfunction

    // -------------------------------------------------------------------------
    // connect_phase: wire up TLM connections
    // -------------------------------------------------------------------------
    function void connect_phase(uvm_phase phase);
      // Forward monitor's analysis port to agent's analysis port
      mon.ap.connect(ap);

      // Connect driver to sequencer (only if active)
      if (get_is_active() == UVM_ACTIVE)
        drv.seq_item_port.connect(seqr.seq_item_export);
    endfunction

  endclass : ahb_agent

endpackage : ahb_agent_pkg
// =============================================================================
// End of ahb_agent_pkg.sv
// =============================================================================