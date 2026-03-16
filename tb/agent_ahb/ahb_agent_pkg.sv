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
  //
  // Drives signals directly on negedge (like directed TB) to avoid
  // clocking block output skew fighting the RTL's posedge sampling.
  // Clocking block #1 output skew means signal arrives 1ns AFTER posedge
  // which is too late for the RTL to latch correctly.
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
      ahb_apb_txn txn;

      // Initialize bus to idle on negedge
      @(negedge vif.hclk);
      vif.hselapb <= 0;
      vif.haddr   <= 0;
      vif.hwrite  <= 0;
      vif.htrans  <= HTRANS_IDLE;
      vif.hwdata  <= 0;

      // Wait for reset release
      @(posedge vif.hclk);
      wait (vif.hresetn === 1'b1);
      @(posedge vif.hclk);

      forever begin
        seq_item_port.get_next_item(txn);

        `uvm_info("AHB_DRV",
          $sformatf("Driving: %s", txn.convert2string()), UVM_HIGH)

        if (txn.kind == AHB_WRITE)
          drive_write(txn);
        else
          drive_read(txn);

        // Inter-transaction idle gap
        if (txn.delay_cycles > 0) begin
          @(negedge vif.hclk);
          vif.hselapb <= 0;
          vif.htrans  <= HTRANS_IDLE;
          vif.haddr   <= 0;
          vif.hwdata  <= 0;
          repeat(txn.delay_cycles - 1) @(negedge vif.hclk);
        end

        seq_item_port.item_done();
      end
    endtask

    // -------------------------------------------------------------------------
    // drive_write: drive on negedge, RTL samples on posedge
    // -------------------------------------------------------------------------
    task drive_write(ahb_apb_txn txn);
      // Address phase - drive on negedge, sampled by RTL on next posedge
      @(negedge vif.hclk);
      vif.hselapb <= 1;
      vif.haddr   <= txn.addr;
      vif.hwrite  <= 1;
      vif.htrans  <= HTRANS_NONSEQ;
      vif.hwdata  <= 0;         // hwdata not valid yet in address phase

      // Data phase - hwdata valid one negedge after address phase
      @(negedge vif.hclk);
      vif.hwdata  <= txn.data;
      vif.hselapb <= 0;
      vif.htrans  <= HTRANS_IDLE;
      vif.haddr   <= 0;
      vif.hwrite  <= 0;

      // Wait for hready=1 on posedge (bridge releases AHB master)
      @(posedge vif.hclk);
      while (!vif.hready) @(posedge vif.hclk);

      `uvm_info("AHB_DRV",
        $sformatf("WRITE done: addr=0x%08h data=0x%08h",
          txn.addr, txn.data), UVM_HIGH)
    endtask

    // -------------------------------------------------------------------------
    // drive_read
    // -------------------------------------------------------------------------
    task drive_read(ahb_apb_txn txn);
      // Address phase
      @(negedge vif.hclk);
      vif.hselapb <= 1;
      vif.haddr   <= txn.addr;
      vif.hwrite  <= 0;
      vif.htrans  <= HTRANS_NONSEQ;
      vif.hwdata  <= 0;

      // Deassert after address phase
      @(negedge vif.hclk);
      vif.hselapb <= 0;
      vif.htrans  <= HTRANS_IDLE;
      vif.haddr   <= 0;

      // Wait for hready=1 - hrdata valid at this posedge
      @(posedge vif.hclk);
      while (!vif.hready) @(posedge vif.hclk);

      txn.data = vif.hrdata;

      `uvm_info("AHB_DRV",
        $sformatf("READ done: addr=0x%08h hrdata=0x%08h",
          txn.addr, txn.data), UVM_HIGH)
    endtask

  endclass : ahb_driver

  //===========================================================================
  // AHB MONITOR
  //
  // Samples directly from interface signals at posedge+#1 settle time.
  // Three-step state machine:
  //   Step 1: detect address phase (hselapb=1, htrans=NONSEQ)
  //   Step 2: capture hwdata one cycle later (data phase)
  //   Step 3: build txn when hready=1
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
      saved_data      = 0;

      @(posedge vif.hclk);
      wait (vif.hresetn === 1'b1);

      forever begin
        @(posedge vif.hclk);
        #1; // small settle after posedge

        // Step 1: address phase detection
        if (vif.hselapb && (vif.htrans == HTRANS_NONSEQ)) begin
          saved_addr      = vif.haddr;
          saved_write     = vif.hwrite;
          addr_phase_seen = 1;
          data_phase_seen = 0;
          saved_data      = 0;
        end
        // Step 2: data phase - one cycle after address phase
        else if (addr_phase_seen && !data_phase_seen) begin
          if (saved_write)
            saved_data = vif.hwdata;
          data_phase_seen = 1;
        end

        // Step 3: completion - hready=1
        if (addr_phase_seen && vif.hready) begin
          txn            = ahb_apb_txn::type_id::create("ahb_mon_txn");
          txn.addr       = saved_addr;
          txn.kind       = saved_write ? AHB_WRITE : AHB_READ;
          txn.trans_type = HTRANS_NONSEQ;
          txn.resp       = vif.hresp ? HRESP_ERROR : HRESP_OKAY;
          txn.data       = saved_write ? saved_data : vif.hrdata;

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