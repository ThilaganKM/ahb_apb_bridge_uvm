// =============================================================================
// File    : ahb_apb_txn_pkg.sv
// Project : AHB-APB Bridge UVM Verification
//
// Description:
//   Transaction package containing:
//     - ahb_apb_txn  : main transaction object (one complete transfer)
//     - Enumerations : transfer kind, htrans type, response type
//
// A transaction is the ABSTRACT representation of one bridge transfer.
// It hides all the cycle-by-cycle signal toggling behind a clean object.
//
// Flow:
//   Sequence creates txn → Driver converts txn to pin wiggles
//   Monitor  observes pin wiggles → builds txn → Scoreboard compares
// =============================================================================

package ahb_apb_txn_pkg;

  // Import UVM base classes and macros
  import uvm_pkg::*;
  `include "uvm_macros.svh"

  // ===========================================================================
  // ENUMERATIONS
  // ===========================================================================

  // Transfer direction
  typedef enum logic {
    AHB_READ  = 1'b0,
    AHB_WRITE = 1'b1
  } ahb_kind_e;

  // AHB htrans values
  // These mirror the AMBA spec encoding exactly
  typedef enum logic [1:0] {
    HTRANS_IDLE   = 2'b00,  // No transfer
    HTRANS_BUSY   = 2'b01,  // Master busy (burst stall)
    HTRANS_NONSEQ = 2'b10,  // New transfer (single or burst start)
    HTRANS_SEQ    = 2'b11   // Burst continuation
  } ahb_trans_e;

  // AHB hresp values
  typedef enum logic {
    HRESP_OKAY  = 1'b0,
    HRESP_ERROR = 1'b1
  } ahb_resp_e;

  // ===========================================================================
  // TRANSACTION CLASS: ahb_apb_txn
  //
  // Extends uvm_sequence_item - the base class for all UVM transactions.
  // uvm_sequence_item provides:
  //   - print()      : formatted display
  //   - copy()       : deep copy
  //   - compare()    : field comparison (used by scoreboard)
  //   - pack/unpack  : for TLM transport
  //
  // `uvm_object_utils_begin/end registers the class with the UVM factory.
  // `uvm_field_* macros auto-generate copy/compare/print for each field.
  // ===========================================================================

  class ahb_apb_txn extends uvm_sequence_item;

    // Register class with UVM factory
    `uvm_object_utils_begin(ahb_apb_txn)
      `uvm_field_enum(ahb_kind_e,  kind,       UVM_ALL_ON)
      `uvm_field_int (addr,                    UVM_ALL_ON | UVM_HEX)
      `uvm_field_int (data,                    UVM_ALL_ON | UVM_HEX)
      `uvm_field_enum(ahb_trans_e, trans_type, UVM_ALL_ON)
      `uvm_field_enum(ahb_resp_e,  resp,       UVM_ALL_ON)
      `uvm_field_int (delay_cycles,            UVM_ALL_ON)
    `uvm_object_utils_end

    // =========================================================================
    // TRANSACTION FIELDS
    // =========================================================================

    // Transfer direction - READ or WRITE
    rand ahb_kind_e kind;

    // 32-bit address
    // Constrained to APB-typical peripheral address range
    rand logic [31:0] addr;

    // 32-bit data
    // For WRITE: data driven by master onto hwdata
    // For READ:  data returned by peripheral on prdata (set by APB slave model)
    rand logic [31:0] data;

    // Transfer type - controls valid/invalid scenarios
    // Default NONSEQ for normal transfers
    rand ahb_trans_e trans_type;

    // Response expected from bridge
    // Scoreboard checks this against actual hresp
    ahb_resp_e resp;

    // Inter-transaction delay in clock cycles
    // Adds idle cycles between transfers for realistic bus behavior
    // 0 = back-to-back, >0 = idle gap
    rand int unsigned delay_cycles;

    // =========================================================================
    // CONSTRAINTS
    // =========================================================================

    // Keep address in a reasonable peripheral range
    // APB peripherals typically mapped in lower address space
    // Aligned to 4-byte boundary (word-aligned as per AMBA spec)
    constraint c_addr {
      addr inside {[32'h0000_0000 : 32'h0000_00FF]};
      addr[1:0] == 2'b00;   // word aligned
    }

    // Weighted distribution: more writes than reads (typical bus traffic)
    // 60% writes, 40% reads
    constraint c_kind {
      kind dist {
        AHB_WRITE := 60,
        AHB_READ  := 40
      };
    }

    // Normal transfers use NONSEQ
    // Occasionally use IDLE to test bridge idle behavior
    // BUSY and SEQ left for specific burst test sequences
    constraint c_trans_type {
      trans_type dist {
        HTRANS_NONSEQ := 90,
        HTRANS_IDLE   := 10
      };
    }

    // Delay: mostly back-to-back (0 cycles) with occasional gaps
    // This stresses the pipeline states
    constraint c_delay {
      delay_cycles dist {
        0       := 50,    // 50% back-to-back
        [1:3]   := 30,    // 30% short gap
        [4:10]  := 20     // 20% longer gap
      };
    }

    // =========================================================================
    // CONSTRUCTOR
    // Standard UVM constructor - always takes string name argument
    // =========================================================================
    function new(string name = "ahb_apb_txn");
      super.new(name);
      resp = HRESP_OKAY;    // default expected response is OKAY
    endfunction

    // =========================================================================
    // DISPLAY FUNCTION
    // convert2string() is called by UVM's built-in print infrastructure
    // Override to give meaningful output in transcript
    // =========================================================================
    function string convert2string();
      return $sformatf(
        "TXN: %s addr=0x%08h data=0x%08h trans=%s resp=%s delay=%0d",
        kind.name(),
        addr,
        data,
        trans_type.name(),
        resp.name(),
        delay_cycles
      );
    endfunction

  endclass : ahb_apb_txn

endpackage : ahb_apb_txn_pkg
// =============================================================================
// End of ahb_apb_txn_pkg.sv
// =============================================================================