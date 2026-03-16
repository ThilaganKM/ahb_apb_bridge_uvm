// =============================================================================
// File    : ahb_apb_txn_pkg.sv
// Project : AHB-APB Bridge UVM Verification
// =============================================================================

package ahb_apb_txn_pkg;

  import uvm_pkg::*;
  `include "uvm_macros.svh"

  // ===========================================================================
  // ENUMERATIONS
  // ===========================================================================

  typedef enum logic {
    AHB_READ  = 1'b0,
    AHB_WRITE = 1'b1
  } ahb_kind_e;

  typedef enum logic [1:0] {
    HTRANS_IDLE   = 2'b00,
    HTRANS_BUSY   = 2'b01,
    HTRANS_NONSEQ = 2'b10,
    HTRANS_SEQ    = 2'b11
  } ahb_trans_e;

  typedef enum logic {
    HRESP_OKAY  = 1'b0,
    HRESP_ERROR = 1'b1
  } ahb_resp_e;

  // ===========================================================================
  // TRANSACTION CLASS
  // FIX: Fields declared BEFORE uvm_object_utils block
  // UVM-1.1d macros need fields visible at macro expansion time
  // ===========================================================================

  class ahb_apb_txn extends uvm_sequence_item;

    // =========================================================================
    // FIELDS - declared first, before utils block
    // =========================================================================
    rand ahb_kind_e  kind;
    rand logic [31:0] addr;
    rand logic [31:0] data;
    rand ahb_trans_e trans_type;
         ahb_resp_e  resp;
    rand int unsigned delay_cycles;

    // =========================================================================
    // FACTORY + FIELD MACROS
    // Fields must already be declared above for macros to work in UVM-1.1d
    // =========================================================================
    `uvm_object_utils_begin(ahb_apb_txn)
      `uvm_field_enum(ahb_kind_e,  kind,         UVM_ALL_ON)
      `uvm_field_int (addr,                       UVM_ALL_ON | UVM_HEX)
      `uvm_field_int (data,                       UVM_ALL_ON | UVM_HEX)
      `uvm_field_enum(ahb_trans_e, trans_type,    UVM_ALL_ON)
      `uvm_field_enum(ahb_resp_e,  resp,          UVM_ALL_ON)
      `uvm_field_int (delay_cycles,               UVM_ALL_ON)
    `uvm_object_utils_end

    // =========================================================================
    // CONSTRAINTS
    // =========================================================================

    constraint c_addr {
      addr inside {[32'h0000_0000 : 32'h0000_00FC]};
      addr[1:0] == 2'b00;
    }

    constraint c_kind {
      kind dist {
        AHB_WRITE := 60,
        AHB_READ  := 40
      };
    }

    constraint c_trans_type {
      trans_type dist {
        HTRANS_NONSEQ := 90,
        HTRANS_IDLE   := 10
      };
    }

    constraint c_delay {
      delay_cycles dist {
        0      := 50,
        [1:3]  := 30,
        [4:10] := 20
      };
    }

    // =========================================================================
    // CONSTRUCTOR
    // =========================================================================
    function new(string name = "ahb_apb_txn");
      super.new(name);
      resp = HRESP_OKAY;
    endfunction

    // =========================================================================
    // DISPLAY
    // =========================================================================
    function string convert2string();
      return $sformatf(
        "TXN: %s addr=0x%08h data=0x%08h trans=%s resp=%s delay=%0d",
        kind.name(), addr, data,
        trans_type.name(), resp.name(),
        delay_cycles
      );
    endfunction

  endclass : ahb_apb_txn

endpackage : ahb_apb_txn_pkg