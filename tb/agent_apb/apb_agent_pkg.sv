`timescale 1ns/1ps

package apb_agent_pkg;

  import uvm_pkg::*;
  `include "uvm_macros.svh"
  import ahb_apb_txn_pkg::*;

  //===========================================================================
  // APB SLAVE MODEL
  // Drives prdata during reads. Has NO analysis port.
  //===========================================================================
  class apb_slave_model extends uvm_component;
    `uvm_component_utils(apb_slave_model)

    virtual ahb_apb_if vif;
    logic [31:0] mem [0:255];

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db #(virtual ahb_apb_if)::get(this, "", "vif", vif))
        `uvm_fatal("NOVIF", "apb_slave_model: vif not found")
      foreach (mem[i])
        mem[i] = 32'hA5A5_0000 | i;
    endfunction

    task run_phase(uvm_phase phase);
      @(posedge vif.hclk);
      wait (vif.hresetn === 1'b1);

      forever begin
        @(posedge vif.hclk);

        // APB SETUP phase: pre-drive prdata for reads
        if (vif.psel && !vif.penable && !vif.pwrite) begin
          vif.prdata <= mem[vif.paddr[7:2]];
          `uvm_info("APB_SLAVE",
            $sformatf("READ SETUP: paddr=0x%08h driving prdata=0x%08h",
              vif.paddr, mem[vif.paddr[7:2]]), UVM_HIGH)
        end

        // APB ENABLE phase: latch write data
        if (vif.psel && vif.penable && vif.pwrite) begin
          mem[vif.paddr[7:2]] = vif.pwdata;
          `uvm_info("APB_SLAVE",
            $sformatf("WRITE: paddr=0x%08h pwdata=0x%08h",
              vif.paddr, vif.pwdata), UVM_HIGH)
        end

        // Bus idle: clear prdata
        if (!vif.psel)
          vif.prdata <= 32'h0;
      end
    endtask

    function logic [31:0] mem_read(logic [31:0] addr);
      return mem[addr[7:2]];
    endfunction

  endclass : apb_slave_model

  //===========================================================================
  // APB MONITOR
  // Observes APB bus, builds transactions, writes to analysis port.
  //===========================================================================
  class apb_monitor extends uvm_monitor;
    `uvm_component_utils(apb_monitor)

    uvm_analysis_port #(ahb_apb_txn) ap;
    virtual ahb_apb_if vif;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      ap = new("ap", this);
      if (!uvm_config_db #(virtual ahb_apb_if)::get(this, "", "vif", vif))
        `uvm_fatal("NOVIF", "apb_monitor: vif not found")
    endfunction

    task run_phase(uvm_phase phase);
      ahb_apb_txn txn;

      @(posedge vif.hclk);
      wait (vif.hresetn === 1'b1);

      txn = null;

      forever begin
        @(posedge vif.hclk);

        // APB SETUP phase: capture address/control
        if (vif.psel === 1'b1 && vif.penable === 1'b0) begin
          txn = ahb_apb_txn::type_id::create("apb_mon_txn", this);
          txn.addr       = vif.paddr;
          txn.trans_type = HTRANS_NONSEQ;
          txn.resp       = HRESP_OKAY;

          if (vif.pwrite)
            txn.kind = AHB_WRITE;
          else
            txn.kind = AHB_READ;

          `uvm_info("APB_MON_DBG",
            $sformatf("SETUP: paddr=0x%08h pwrite=%0b pwdata=0x%08h prdata=0x%08h",
                      vif.paddr, vif.pwrite, vif.pwdata, vif.prdata),
            UVM_HIGH)
        end

        // APB ENABLE phase: capture data and publish
        if (vif.psel === 1'b1 && vif.penable === 1'b1 && txn != null) begin
          if (txn.kind == AHB_WRITE)
            txn.data = vif.pwdata;
          else
            txn.data = vif.prdata;

          `uvm_info("APB_MON",
            $sformatf("Captured: %s", txn.convert2string()),
            UVM_MEDIUM)

          ap.write(txn);
          txn = null;
        end

        // Safety clear when bus goes idle
        if (vif.psel === 1'b0)
          txn = null;
      end
    endtask

  endclass : apb_monitor

  //===========================================================================
  // APB AGENT - PASSIVE
  //===========================================================================
  class apb_agent extends uvm_agent;
    `uvm_component_utils(apb_agent)

    apb_monitor     mon;
    apb_slave_model slave;
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
      mon.ap.connect(ap);
    endfunction

  endclass : apb_agent

endpackage : apb_agent_pkg