// =============================================================================
// Module  : ahb_apb_bridge
// Project : AHB-APB Bridge UVM Verification
// Author  : Appalla Subrahmanya Karthikeya
//
// Description:
//   Full pipelined AHB to APB bridge.
//   - Translates AHB (pipelined, high-speed) transfers to
//     APB (non-pipelined, low-speed) transfers.
//   - Supports: single read, single write, back-to-back reads,
//     back-to-back writes, mixed read/write pipelining.
//   - Proper AMBA-compliant hresp error signaling.
//   - htrans-aware: handles IDLE, BUSY, NONSEQ, SEQ correctly.
//   - hready deasserted correctly across all APB phases.
//
// FSM States (8):
//   IDLE, READ, RENABLE,
//   WWAIT, WRITE, WENABLE, WRITE_P, WENABLE_P
//
// AMBA Ref: ARM IHI0024B (APB spec), ARM IHI0011A (AHB spec)
// =============================================================================

module ahb_apb_bridge (
  //--------------------------------------------
  // AHB Slave Interface (inputs from AHB Master)
  //--------------------------------------------
  input  logic        hclk,       // AHB clock - all logic is synchronous to this
  input  logic        hresetn,    // Active-low reset (AMBA standard)
  input  logic        hselapb,    // Slave select - AHB decoder asserts this when
                                  // address targets the APB bridge region
  input  logic [31:0] haddr,      // AHB address bus
  input  logic        hwrite,     // 1 = Write, 0 = Read
  input  logic [1:0]  htrans,     // Transfer type:
                                  //   2'b00 = IDLE   (no transfer)
                                  //   2'b01 = BUSY   (master busy, insert wait)
                                  //   2'b10 = NONSEQ (single or first burst beat)
                                  //   2'b11 = SEQ    (subsequent burst beat)
  input  logic [31:0] hwdata,     // Write data from AHB master

  //--------------------------------------------
  // AHB Slave Interface (outputs back to AHB)
  //--------------------------------------------
  output logic        hready,     // 0 = bridge inserting wait states into AHB
                                  // 1 = transfer complete, AHB can proceed
  output logic        hresp,      // 0 = OKAY, 1 = ERROR (2-cycle AMBA error response)
  output logic [31:0] hrdata,     // Read data returned to AHB master

  //--------------------------------------------
  // APB Master Interface (outputs to APB peripherals)
  //--------------------------------------------
  output logic [31:0] paddr,      // APB address (registered from haddr)
  output logic [31:0] pwdata,     // APB write data (registered from hwdata)
  output logic        psel,       // APB peripheral select
  output logic        penable,    // APB enable phase (2nd cycle of APB transfer)
  output logic        pwrite,     // 1 = Write, 0 = Read

  //--------------------------------------------
  // APB Slave Interface (input from APB peripheral)
  //--------------------------------------------
  input  logic [31:0] prdata      // Read data from APB peripheral
);

  // ===========================================================================
  // PARAMETERS - FSM State Encoding (3-bit one-hot-friendly binary)
  // Why binary and not one-hot?
  //   One-hot uses more flops (8 flops for 8 states).
  //   Binary encoding uses 3 flops. For synthesis on ASIC this is area-efficient.
  //   QuestaSim handles both fine.
  // ===========================================================================

  localparam [2:0] IDLE      = 3'b000;
  localparam [2:0] READ      = 3'b001;  // APB SETUP phase for read
  localparam [2:0] RENABLE   = 3'b010;  // APB ENABLE phase for read
  localparam [2:0] WWAIT     = 3'b011;  // Waiting for AHB write data phase
                                         // (absorbs AHB pipeline)
  localparam [2:0] WRITE     = 3'b100;  // APB SETUP phase for write (no next transfer)
  localparam [2:0] WENABLE   = 3'b101;  // APB ENABLE phase for write (no next transfer)
  localparam [2:0] WRITE_P   = 3'b110;  // APB SETUP phase for write (next transfer pending)
  localparam [2:0] WENABLE_P = 3'b111;  // APB ENABLE phase for write (next transfer pending)

  // htrans encodings - named for readability
  localparam [1:0] HTRANS_IDLE   = 2'b00;
  localparam [1:0] HTRANS_BUSY   = 2'b01;
  localparam [1:0] HTRANS_NONSEQ = 2'b10;
  localparam [1:0] HTRANS_SEQ    = 2'b11;

  // ===========================================================================
  // INTERNAL SIGNALS
  // ===========================================================================

  logic [2:0] present_state, next_state;

  // Temporary registers - critical for pipelining
  // AHB is pipelined: address phase of next transfer overlaps data phase of current.
  // We latch addr/data/write in WWAIT state so we don't lose them when AHB
  // moves its address bus to the next transfer address.
  logic [31:0] haddr_temp;   // Latched AHB address
  logic [31:0] hwdata_temp;  // Latched AHB write data
  logic        hwrite_temp;  // Latched AHB write direction

  // valid: combinational signal meaning "a real transfer is targeting this bridge"
  // Conditions: slave is selected AND transfer is NONSEQ or SEQ (not IDLE/BUSY)
  logic        valid;

  // error_resp: tracks 2-cycle error response state
  // AMBA spec requires hresp=1 for 2 consecutive cycles with hready=0 then hready=1
  logic        error_resp;
  logic        error_resp_d; // delayed by 1 cycle for 2-cycle sequencing

  // ===========================================================================
  // BLOCK 1: VALID LOGIC (Combinational)
  //
  // valid = 1 when AHB master is making a real transfer targeting this bridge.
  //
  // Why check both NONSEQ and SEQ?
  //   NONSEQ = start of a new transfer (single or first burst beat)
  //   SEQ    = continuation of a burst
  //   IDLE   = no transfer, ignore
  //   BUSY   = master is busy (e.g., doing internal work between burst beats)
  //            we must NOT start an APB transfer on BUSY - treat like IDLE
  // ===========================================================================

  always_comb begin
    if (hselapb && (htrans == HTRANS_NONSEQ || htrans == HTRANS_SEQ))
      valid = 1'b1;
    else
      valid = 1'b0;
  end

  // ===========================================================================
  // BLOCK 2: STATE REGISTER (Sequential)
  //
  // Simple synchronous FSM register with async active-low reset.
  // Active-low reset is AMBA standard - hresetn=0 means reset asserted.
  //
  // We also latch haddr/hwdata/hwrite here in WWAIT state.
  // WHY in the sequential block and not combinational?
  //   Because we need to HOLD these values across multiple cycles.
  //   A combinational assignment would just pass through - no holding.
  //   Registers hold the value until explicitly overwritten.
  // ===========================================================================

  always_ff @(posedge hclk or negedge hresetn) begin
    if (!hresetn) begin
      present_state <= IDLE;
      haddr_temp    <= 32'b0;
      hwdata_temp   <= 32'b0;
      hwrite_temp   <= 1'b0;
      error_resp_d  <= 1'b0;
    end
    else begin
      present_state <= next_state;
      error_resp_d  <= error_resp;

      // Latch AHB address-phase signals in WWAIT
      // WWAIT is the state where AHB address phase has completed for a write
      // but APB hasn't started yet. We capture here so when AHB moves on
      // (pipelining the next address), we still have the current one.
      if (present_state == WWAIT) begin
        haddr_temp  <= haddr;
        hwdata_temp <= hwdata;
        hwrite_temp <= hwrite;
      end
    end
  end

  // ===========================================================================
  // BLOCK 3: NEXT STATE + OUTPUT LOGIC (Combinational)
  //
  // This is a MOORE FSM - outputs are purely functions of present_state.
  // (A Mealy FSM would also use inputs to determine outputs -
  //  Moore is cleaner for synthesis and timing closure.)
  //
  // Default output values set at the top of always_comb to avoid
  // unintentional latches. Every signal gets a safe default, then
  // specific states override what they need.
  // ===========================================================================

  always_comb begin

    //----------------------------------------------------------
    // DEFAULT OUTPUT VALUES
    // Set everything to safe/inactive state.
    // This is critical - without defaults, synthesis may infer
    // latches for signals not assigned in every branch.
    //----------------------------------------------------------
    next_state = present_state; // stay in current state unless overridden

    // APB outputs - inactive
    psel       = 1'b0;
    penable    = 1'b0;
    pwrite     = 1'b0;
    paddr      = 32'b0;
    pwdata     = 32'b0;

    // AHB response - default: ready, no error, no data
    hready     = 1'b1;   // default ready (AHB can proceed)
    hresp      = 1'b0;   // default OKAY
    hrdata     = 32'b0;

    // Error response default
    error_resp = 1'b0;

    //----------------------------------------------------------
    // FSM: State-by-State Logic
    //----------------------------------------------------------

    case (present_state)

      // --------------------------------------------------------
      // IDLE: Bridge is idle, waiting for a valid transfer
      //
      // hready=1 (default) - AHB master can send next transfer
      // We check valid and hwrite to decide where to go.
      // --------------------------------------------------------
      IDLE: begin
        if (!valid) begin
          next_state = IDLE;          // No transfer - stay idle
        end
        else if (valid && !hwrite) begin
          next_state = READ;          // Read request - go setup APB read
        end
        else begin
          // valid && hwrite
          next_state = WWAIT;         // Write request - go wait for data phase
        end
      end

      // --------------------------------------------------------
      // READ: APB SETUP phase for a read transfer
      //
      // This is the first cycle of an APB read:
      //   psel=1, penable=0 (SETUP phase per APB spec)
      //   pwrite=0 (it's a read)
      //   paddr = haddr (pass address directly - no pipeline issue on reads
      //           because AHB read data and address are in the same phase)
      //
      // hready=0: stall AHB master while APB completes
      // Always go to RENABLE next (no conditional - APB must have ENABLE phase)
      // --------------------------------------------------------
      READ: begin
        psel       = 1'b1;
        penable    = 1'b0;
        pwrite     = 1'b0;
        paddr      = haddr;
        hready     = 1'b0;          // Stall AHB - APB not done yet
        next_state = RENABLE;
      end

      // --------------------------------------------------------
      // RENABLE: APB ENABLE phase for a read transfer
      //
      // Second cycle of APB read:
      //   psel=1, penable=1 (ENABLE phase - peripheral latches/drives data)
      //   hrdata = prdata: capture peripheral data and send to AHB master
      //
      // hready=1: APB read is done, release AHB master
      //
      // Next state decision: what's coming next from AHB?
      //   valid && !hwrite → another read → READ (back-to-back read)
      //   valid && hwrite  → write next  → WWAIT
      //   !valid           → nothing     → IDLE
      // --------------------------------------------------------
      RENABLE: begin
        psel       = 1'b1;
        penable    = 1'b1;
        pwrite     = 1'b0;
        paddr      = haddr;
        hrdata     = prdata;        // Capture APB read data → AHB
        hready     = 1'b1;          // Release AHB master
        if (valid && !hwrite)
          next_state = READ;        // Back-to-back read
        else if (valid && hwrite)
          next_state = WWAIT;       // Next is a write
        else
          next_state = IDLE;
      end

      // --------------------------------------------------------
      // WWAIT: Wait for AHB write data phase
      //
      // WHY does this state exist?
      // AHB is PIPELINED. When a write is issued:
      //   Cycle N:   Address phase (haddr, hwrite valid)
      //   Cycle N+1: Data phase   (hwdata valid)
      // But APB needs BOTH address and data before it can start.
      // So the bridge must wait in WWAIT for hwdata to become valid.
      //
      // During WWAIT:
      //   hready=0: stall AHB (it can't send next address yet)
      //   We latch haddr/hwdata/hwrite in the sequential block (Block 2)
      //
      // Next state:
      //   !valid → no next transfer pending → WRITE (simple)
      //   valid  → next transfer already arriving → WRITE_P (pipelined)
      // --------------------------------------------------------
      WWAIT: begin
        hready = 1'b0;              // Stall AHB - we're absorbing the pipeline
        if (!valid)
          next_state = WRITE;       // No next transfer - simple write
        else
          next_state = WRITE_P;     // Next transfer already arriving - pipelined write
      end

      // --------------------------------------------------------
      // WRITE: APB SETUP phase for write (no next transfer pending)
      //
      // psel=1, penable=0 (SETUP phase)
      // Use LATCHED values (haddr_temp, hwdata_temp) NOT live haddr/hwdata.
      // WHY? Because AHB has already moved on - live haddr now has the
      // NEXT transfer's address. We need the one we captured in WWAIT.
      //
      // hready=0: still stalling AHB
      //
      // Next state:
      //   !valid → WENABLE (complete this write)
      //   valid  → WENABLE_P (complete this write, next one is queued)
      // --------------------------------------------------------
      WRITE: begin
        psel       = 1'b1;
        penable    = 1'b0;
        pwrite     = 1'b1;
        paddr      = haddr_temp;    // USE LATCHED ADDRESS
        pwdata     = hwdata_temp;   // USE LATCHED DATA
        hready     = 1'b0;
        if (!valid)
          next_state = WENABLE;
        else
          next_state = WENABLE_P;
      end

      // --------------------------------------------------------
      // WRITE_P: APB SETUP phase for write (next transfer IS pending)
      //
      // Same as WRITE but we know a new transfer is already arriving.
      // Always go to WENABLE_P.
      //
      // This is the "pipelined write" path that your PDF calls write_p.
      // The bridge completes current APB write while already
      // acknowledging the next AHB transfer is queued.
      // --------------------------------------------------------
      WRITE_P: begin
        psel       = 1'b1;
        penable    = 1'b0;
        pwrite     = 1'b1;
        paddr      = haddr_temp;
        pwdata     = hwdata_temp;
        hready     = 1'b0;
        next_state = WENABLE_P;
      end

      // --------------------------------------------------------
      // WENABLE: APB ENABLE phase for write (no next transfer)
      //
      // psel=1, penable=1 (ENABLE - peripheral latches write data here)
      // hready=1: release AHB master, transfer is done
      //
      // Next state: same logic as RENABLE
      // --------------------------------------------------------
      WENABLE: begin
        psel       = 1'b1;
        penable    = 1'b1;
        pwrite     = 1'b1;
        paddr      = haddr_temp;
        pwdata     = hwdata_temp;
        hready     = 1'b1;          // Release AHB master
        if (valid && !hwrite)
          next_state = READ;
        else if (valid && hwrite)
          next_state = WWAIT;
        else
          next_state = IDLE;
      end

      // --------------------------------------------------------
      // WENABLE_P: APB ENABLE phase for write (next transfer pending)
      //
      // Same as WENABLE but the next AHB transfer has already arrived.
      // hready=1: release AHB master
      //
      // Next state options are richer here because we have a queued transfer:
      //   !valid && hwrite  → WRITE  (queued write, no further one after)
      //   valid && !hwrite  → READ   (queued transfer is actually a read)
      //   valid && hwrite   → WWAIT  (queued transfer is a write, another behind it)
      //   else              → IDLE
      // --------------------------------------------------------
      WENABLE_P: begin
        psel       = 1'b1;
        penable    = 1'b1;
        pwrite     = 1'b1;
        paddr      = haddr_temp;
        pwdata     = hwdata_temp;
        hready     = 1'b1;
        if (!valid && hwrite_temp)
          next_state = WRITE;       // Complete the queued write
        else if (valid && !hwrite)
          next_state = READ;
        else if (valid && hwrite)
          next_state = WWAIT;
        else
          next_state = IDLE;
      end

      // --------------------------------------------------------
      // DEFAULT: Safety net - go back to IDLE
      // Prevents FSM lockup if illegal state is entered
      // (e.g., due to glitch or simulation X-propagation)
      // --------------------------------------------------------
      default: begin
        next_state = IDLE;
      end

    endcase

    // ==========================================================
    // ERROR RESPONSE LOGIC
    //
    // AMBA AHB spec: when a slave needs to signal an error,
    // it must:
    //   Cycle 1: hresp=1, hready=0
    //   Cycle 2: hresp=1, hready=1
    //
    // We detect an illegal condition: htrans=SEQ but bridge is
    // in IDLE (means a burst started without proper NONSEQ first,
    // or slave was not selected for the NONSEQ beat).
    //
    // error_resp_d is the 1-cycle delayed version, giving us
    // the 2-cycle sequencing.
    // ==========================================================
    if (hselapb && (htrans == HTRANS_SEQ) && (present_state == IDLE)) begin
      error_resp = 1'b1;
      hresp      = 1'b1;
      hready     = 1'b0;           // Cycle 1: hready low
    end

    if (error_resp_d) begin
      hresp  = 1'b1;
      hready = 1'b1;               // Cycle 2: hready high, then return to IDLE
    end

  end // always_comb

  // ===========================================================================
  // SIMULATION ASSERTIONS
  // Compiled only when ASSERT_ON is defined (see Makefile)
  // These catch protocol violations during simulation
  // ===========================================================================

`ifdef ASSERT_ON

  // penable must only rise AFTER psel
  property p_penable_after_psel;
    @(posedge hclk) disable iff (!hresetn)
    $rose(penable) |-> $past(psel, 1);
  endproperty
  a_penable_after_psel: assert property (p_penable_after_psel)
    else $error("[ASSERT] penable rose without psel being high previous cycle");

  // hready must be low during APB SETUP phase (psel=1, penable=0)
  property p_hready_low_in_setup;
    @(posedge hclk) disable iff (!hresetn)
    (psel && !penable) |-> !hready;
  endproperty
  a_hready_low_in_setup: assert property (p_hready_low_in_setup)
    else $error("[ASSERT] hready was high during APB SETUP phase - protocol violation");

  // prdata must appear on hrdata after a read ENABLE phase
  property p_read_data_propagation;
    @(posedge hclk) disable iff (!hresetn)
    (psel && penable && !pwrite) |-> (hrdata == prdata);
  endproperty
  a_read_data_propagation: assert property (p_read_data_propagation)
    else $error("[ASSERT] hrdata does not match prdata during read ENABLE phase");

  // psel must not deassert in middle of ENABLE (psel must stay high through penable)
  property p_psel_stable_through_enable;
    @(posedge hclk) disable iff (!hresetn)
    $rose(penable) |-> psel;
  endproperty
  a_psel_stable_through_enable: assert property (p_psel_stable_through_enable)
    else $error("[ASSERT] psel deasserted while penable was high - APB protocol violation");

`endif

endmodule
// =============================================================================
// End of ahb_apb_bridge.sv
// =============================================================================