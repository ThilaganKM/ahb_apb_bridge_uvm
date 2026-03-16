`timescale 1ns/1ps

package ahb_agent_pkg;

  import uvm_pkg::*;
  `include "uvm_macros.svh"
  import ahb_apb_txn_pkg::*;

  //===========================================================================
  // AHB SEQUENCER
  //===========================================================================
  class ahb_sequencer extends uvm_sequencer #(ahb_apb_txn);
    `uvm_component_utils(ahb_sequencer)
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
  endclass : ahb_sequencer

  //===========================================================================
  // AHB DRIVER
  //===========================================================================
  class ahb_driver extends uvm_driver #(ahb_apb_txn);
    `uvm_component_utils(ahb_driver)

    virtual ahb_apb_if vif;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db #(virtual ahb_apb_if)::get(this, "", "vif", vif))
        `uvm_fatal("NOVIF", "ahb_driver: vif not found")
    endfunction

    task run_phase(uvm_phase phase);
      // Initialize bus to idle
      vif.master_cb.hselapb <= 0;
      vif.master_cb.haddr   <= 0;
      vif.master_cb.hwrite  <= 0;
      vif.master_cb.htrans  <= HTRANS_IDLE;
      vif.master_cb.hwdata  <= 0;

      // Wait for reset release
      @(posedge vif.hclk);
      wait (vif.hresetn === 1'b1);
      @(posedge vif.hclk);

      forever begin
        ahb_apb_txn txn;
        seq_item_port.get_next_item(txn);

        `uvm_info("AHB_DRV",
          $sformatf("Driving: %s", txn.convert2string()), UVM_HIGH)

        if (txn.kind == AHB_WRITE)
          drive_write(txn);
        else
          drive_read(txn);

        if (txn.delay_cycles > 0) begin
          vif.master_cb.hselapb <= 0;
          vif.master_cb.htrans  <= HTRANS_IDLE;
          vif.master_cb.haddr   <= 0;
          vif.master_cb.hwdata  <= 0;
          repeat(txn.delay_cycles) @(vif.master_cb);
        end

        seq_item_port.item_done();
      end
    endtask

    task drive_write(ahb_apb_txn txn);
      // Address phase
      @(vif.master_cb);
      vif.master_cb.hselapb <= 1;
      vif.master_cb.haddr   <= txn.addr;
      vif.master_cb.hwrite  <= 1;
      vif.master_cb.htrans  <= HTRANS_NONSEQ;
      vif.master_cb.hwdata  <= 0;

      // Data phase
      @(vif.master_cb);
      vif.master_cb.hwdata  <= txn.data;
      vif.master_cb.hselapb <= 0;
      vif.master_cb.htrans  <= HTRANS_IDLE;
      vif.master_cb.haddr   <= 0;
      vif.master_cb.hwrite  <= 0;

      // Wait for hready
      @(vif.master_cb);
      while (!vif.master_cb.hready) @(vif.master_cb);

      `uvm_info("AHB_DRV",
        $sformatf("WRITE done: addr=0x%08h data=0x%08h", txn.addr, txn.data),
        UVM_HIGH)
    endtask

    task drive_read(ahb_apb_txn txn);
      // Address phase
      @(vif.master_cb);
      vif.master_cb.hselapb <= 1;
      vif.master_cb.haddr   <= txn.addr;
      vif.master_cb.hwrite  <= 0;
      vif.master_cb.htrans  <= HTRANS_NONSEQ;
      vif.master_cb.hwdata  <= 0;

      // Deassert
      @(vif.master_cb);
      vif.master_cb.hselapb <= 0;
      vif.master_cb.htrans  <= HTRANS_IDLE;
      vif.master_cb.haddr   <= 0;

      // Wait for hready - hrdata valid here
      @(vif.master_cb);
      while (!vif.master_cb.hready) @(vif.master_cb);

      txn.data = vif.master_cb.hrdata;

      `uvm_info("AHB_DRV",
        $sformatf("READ done: addr=0x%08h hrdata=0x%08h", txn.addr, txn.data),
        UVM_HIGH)
    endtask

  endclass : ahb_driver

  //===========================================================================
  // AHB MONITOR
  //
  // Sampling strategy:
  //   - Detect address phase: hselapb=1, htrans=NONSEQ → save addr, write
  //   - Save hwdata one cycle AFTER address phase (AHB data phase)
  //   - Detect completion: hready=1 → build and send transaction
  //===========================================================================
  class ahb_monitor extends uvm_monitor;
    `uvm_component_utils(ahb_monitor)

    uvm_analysis_port #(ahb_apb_txn) ap;
    virtual ahb_apb_if vif;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      ap = new("ap", this);
      if (!uvm_config_db #(virtual ahb_apb_if)::get(this, "", "vif", vif))
        `uvm_fatal("NOVIF", "ahb_monitor: vif not found")
    endfunction

    task run_phase(uvm_phase phase);
      ahb_apb_txn  txn;
      logic [31:0] saved_addr;
      logic [31:0] saved_data;
      logic        saved_write;
      logic        addr_phase_seen;
      logic        data_phase_seen;

      addr_phase_seen = 0;
      data_phase_seen = 0;

      @(posedge vif.hclk);
      wait (vif.hresetn === 1'b1);

      forever begin
        @(posedge vif.hclk);
        #1; // settle

        // Step 1: detect address phase
        if (vif.hselapb && vif.htrans == HTRANS_NONSEQ) begin
          saved_addr      = vif.haddr;
          saved_write     = vif.hwrite;
          addr_phase_seen = 1;
          data_phase_seen = 0;
          saved_data      = 0;
        end

        // Step 2: one cycle after address phase = data phase
        // hwdata is valid here for writes
        else if (addr_phase_seen && !data_phase_seen) begin
          if (saved_write)
            saved_data = vif.hwdata;
          data_phase_seen = 1;
        end

        // Step 3: hready=1 means transfer complete
        if (addr_phase_seen && vif.hready) begin
          txn            = ahb_apb_txn::type_id::create("ahb_mon_txn");
          txn.addr       = saved_addr;
          txn.kind       = saved_write ? AHB_WRITE : AHB_READ;
          txn.trans_type = HTRANS_NONSEQ;
          txn.resp       = vif.hresp ? HRESP_ERROR : HRESP_OKAY;

          if (saved_write)
            txn.data = saved_data;
          else
            txn.data = vif.hrdata;

          `uvm_info("AHB_MON",
            $sformatf("Captured: %s", txn.convert2string()), UVM_MEDIUM)

          ap.write(txn);
          addr_phase_seen = 0;
          data_phase_seen = 0;
        end
      end
    endtask

  endclass : ahb_monitor

  //===========================================================================
  // AHB AGENT
  //===========================================================================
  class ahb_agent extends uvm_agent;
    `uvm_component_utils(ahb_agent)

    ahb_sequencer seqr;
    ahb_driver    drv;
    ahb_monitor   mon;
    uvm_analysis_port #(ahb_apb_txn) ap;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      mon = ahb_monitor::type_id::create("mon", this);
      ap  = new("ap", this);
      if (get_is_active() == UVM_ACTIVE) begin
        seqr = ahb_sequencer::type_id::create("seqr", this);
        drv  = ahb_driver::type_id::create("drv",  this);
      end
    endfunction

    function void connect_phase(uvm_phase phase);
      mon.ap.connect(ap);
      if (get_is_active() == UVM_ACTIVE)
        drv.seq_item_port.connect(seqr.seq_item_export);
    endfunction

  endclass : ahb_agent

endpackage : ahb_agent_pkg