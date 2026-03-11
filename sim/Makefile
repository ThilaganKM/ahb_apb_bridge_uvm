#--------------------------------------------------
# AHB-APB Bridge UVM Verification
# Tool: QuestaSim 2019
#--------------------------------------------------

SIM  = vsim
VLOG = vlog

#--------------------------------------------------
# UVM Path
#--------------------------------------------------

UVM_HOME = /home/cad/eda/Questa_2019/questasim/verilog_src/uvm-1.1d/src

#--------------------------------------------------
# Global Defines
#--------------------------------------------------

DEFINES = +define+ASSERT_ON +define+COVERAGE_ON

#--------------------------------------------------
# RTL Files
#--------------------------------------------------

RTL = \
  ../rtl/ahb_apb_bridge.sv

#--------------------------------------------------
# Interface Files
#--------------------------------------------------

INTERFACES = \
  ../tb/interfaces/ahb_apb_if.sv

#--------------------------------------------------
# Transaction Package
#--------------------------------------------------

TXN = \
  ../tb/txn/ahb_apb_txn_pkg.sv

#--------------------------------------------------
# Testbench Package (env + agents + tests)
#--------------------------------------------------

TBPKG = \
  ../tb/agent_ahb/ahb_agent_pkg.sv \
  ../tb/agent_apb/apb_agent_pkg.sv \
  ../tb/env/bridge_env_pkg.sv \
  ../tb/tests/bridge_test_pkg.sv

#--------------------------------------------------
# Top Module
#--------------------------------------------------

TOP = \
  ../tb/top/tb_top.sv

#--------------------------------------------------
# Compile - RTL only (use during RTL development)
#--------------------------------------------------

compile_rtl:
	vlib work
	$(VLOG) -sv $(RTL)

#--------------------------------------------------
# Compile - Full UVM TB (use after RTL is stable)
#--------------------------------------------------

compile:
	vlib work
	$(VLOG) -sv +incdir+$(UVM_HOME) $(INTERFACES)
	$(VLOG) -sv +incdir+$(UVM_HOME) $(TXN)
	$(VLOG) -sv +incdir+$(UVM_HOME) $(TBPKG)
	$(VLOG) -sv +incdir+$(UVM_HOME) $(RTL)
	$(VLOG) -sv +incdir+$(UVM_HOME) $(TOP)

#--------------------------------------------------
# Compile with Coverage (use for final sign-off)
#--------------------------------------------------

compile_cov:
	vlib work
	$(VLOG) -sv +cover=sbceft -coveropt 3 $(DEFINES) +incdir+$(UVM_HOME) $(INTERFACES)
	$(VLOG) -sv +cover=sbceft -coveropt 3 $(DEFINES) +incdir+$(UVM_HOME) $(TXN)
	$(VLOG) -sv +cover=sbceft -coveropt 3 $(DEFINES) +incdir+$(UVM_HOME) $(TBPKG)
	$(VLOG) -sv +cover=sbceft -coveropt 3 $(DEFINES) +incdir+$(UVM_HOME) $(RTL)
	$(VLOG) -sv +cover=sbceft -coveropt 3 $(DEFINES) +incdir+$(UVM_HOME) $(TOP)

#--------------------------------------------------
# Run Tests (no GUI - CLI only for MobaXterm)
#--------------------------------------------------

# Smoke test - single write + read
smoke:
	$(SIM) -c tb_top +UVM_TESTNAME=bridge_smoke_test \
	+UVM_VERBOSITY=UVM_MEDIUM \
	-do "run -all; quit"

# Write test - single and back to back writes
write_test:
	$(SIM) -c tb_top +UVM_TESTNAME=bridge_write_test \
	+UVM_VERBOSITY=UVM_MEDIUM \
	-do "run -all; quit"

# Read test
read_test:
	$(SIM) -c tb_top +UVM_TESTNAME=bridge_read_test \
	+UVM_VERBOSITY=UVM_MEDIUM \
	-do "run -all; quit"

# Back to back pipelined transfers
b2b_test:
	$(SIM) -c tb_top +UVM_TESTNAME=bridge_b2b_test \
	+UVM_VERBOSITY=UVM_MEDIUM \
	-do "run -all; quit"

# Reset during transfer
reset_test:
	$(SIM) -c tb_top +UVM_TESTNAME=bridge_reset_test \
	+UVM_VERBOSITY=UVM_MEDIUM \
	-do "run -all; quit"

# Full random test with coverage
rand_test:
	$(SIM) -c -coverage tb_top +UVM_TESTNAME=bridge_rand_test \
	+UVM_VERBOSITY=UVM_MEDIUM \
	-do "run -all; coverage save -codeAll bridge.ucdb; quit"

#--------------------------------------------------
# Waveform dump - run with VCD for GTKWave
# Usage: make wave TEST=smoke_test
#--------------------------------------------------

wave:
	$(SIM) -c tb_top +UVM_TESTNAME=bridge_$(TEST) \
	+UVM_VERBOSITY=UVM_HIGH \
	-do "vcd file sim.vcd; vcd add -r /*; run -all; quit"
	gtkwave sim.vcd &

#--------------------------------------------------
# Coverage Report
#--------------------------------------------------

cov_report:
	vcover report bridge.ucdb

#--------------------------------------------------
# Clean
#--------------------------------------------------

clean:
	rm -rf work transcript vsim.wlf *.ucdb *.vcd