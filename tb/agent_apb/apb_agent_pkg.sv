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
      // Initialize prdata
      @(posedge vif.hclk);
      wait (vif.hresetn === 1'b1);

      forever begin
        @(posedge vif.hclk);
        #1;

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

      forever begin
        // Sample at posedge + small delta for signal settling
        @(posedge vif.hclk);
        #1;

        // Capture at ENABLE phase - psel=1, penable=1
        if (vif.psel && vif.penable) begin
          txn            = ahb_apb_txn::type_id::create("apb_mon_txn");
          txn.addr       = vif.paddr;
          txn.trans_type = HTRANS_NONSEQ;
          txn.resp       = HRESP_OKAY;

          if (vif.pwrite) begin
            txn.kind = AHB_WRITE;
            txn.data = vif.pwdata;
          end else begin
            txn.kind = AHB_READ;
            txn.data = vif.prdata;
          end

          `uvm_info("APB_MON",
            $sformatf("Captured: %s", txn.convert2string()), UVM_MEDIUM)

          ap.write(txn);
        end
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