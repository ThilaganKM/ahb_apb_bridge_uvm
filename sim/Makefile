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
# RTL
#--------------------------------------------------

RTL = ../rtl/ahb_apb_bridge.sv

#--------------------------------------------------
# Directed TB (no UVM)
#--------------------------------------------------

DIRECTED_TOP = ../tb/top/tb_top.sv

#--------------------------------------------------
# UVM TB Files - ORDER IS CRITICAL
#--------------------------------------------------

INTERFACE = ../tb/interfaces/ahb_apb_if.sv

TXN       = ../tb/txn/ahb_apb_txn_pkg.sv

AHB_AGENT = ../tb/agent_ahb/ahb_agent_pkg.sv

APB_AGENT = ../tb/agent_apb/apb_agent_pkg.sv

ENV       = ../tb/env/bridge_env_pkg.sv

TESTS     = ../tb/tests/bridge_test_pkg.sv

UVM_TOP   = ../tb/top/tb_uvm_top.sv

#--------------------------------------------------
# STEP 1: Compile RTL only
#--------------------------------------------------

compile_rtl:
	vlib work
	$(VLOG) -sv $(RTL)

#--------------------------------------------------
# STEP 2: Compile RTL + Directed TB
#--------------------------------------------------

compile_directed:
	vlib work
	$(VLOG) -sv $(DEFINES) $(RTL)
	$(VLOG) -sv $(DEFINES) $(DIRECTED_TOP)

#--------------------------------------------------
# STEP 3: Compile Full UVM TB
# ORDER: interface → txn → agents → env → tests → RTL → top
#--------------------------------------------------

compile:
	vlib work
	$(VLOG) -sv +incdir+$(UVM_HOME) $(INTERFACE)
	$(VLOG) -sv +incdir+$(UVM_HOME) $(TXN)
	$(VLOG) -sv +incdir+$(UVM_HOME) $(AHB_AGENT)
	$(VLOG) -sv +incdir+$(UVM_HOME) $(APB_AGENT)
	$(VLOG) -sv +incdir+$(UVM_HOME) $(ENV)
	$(VLOG) -sv +incdir+$(UVM_HOME) $(TESTS)
	$(VLOG) -sv +incdir+$(UVM_HOME) $(RTL)
	$(VLOG) -sv +incdir+$(UVM_HOME) $(UVM_TOP)

#--------------------------------------------------
# Compile with Coverage
#--------------------------------------------------

compile_cov:
	vlib work
	$(VLOG) -sv +cover=sbceft -coveropt 3 $(DEFINES) +incdir+$(UVM_HOME) $(INTERFACE)
	$(VLOG) -sv +cover=sbceft -coveropt 3 $(DEFINES) +incdir+$(UVM_HOME) $(TXN)
	$(VLOG) -sv +cover=sbceft -coveropt 3 $(DEFINES) +incdir+$(UVM_HOME) $(AHB_AGENT)
	$(VLOG) -sv +cover=sbceft -coveropt 3 $(DEFINES) +incdir+$(UVM_HOME) $(APB_AGENT)
	$(VLOG) -sv +cover=sbceft -coveropt 3 $(DEFINES) +incdir+$(UVM_HOME) $(ENV)
	$(VLOG) -sv +cover=sbceft -coveropt 3 $(DEFINES) +incdir+$(UVM_HOME) $(TESTS)
	$(VLOG) -sv +cover=sbceft -coveropt 3 $(DEFINES) +incdir+$(UVM_HOME) $(RTL)
	$(VLOG) -sv +cover=sbceft -coveropt 3 $(DEFINES) +incdir+$(UVM_HOME) $(UVM_TOP)

#--------------------------------------------------
# Directed Test
#--------------------------------------------------

directed:
	$(SIM) -c tb_top \
	-voptargs="+acc" \
	-do "run -all; quit"

#--------------------------------------------------
# UVM Tests
#--------------------------------------------------

smoke:
	$(SIM) -c tb_uvm_top \
	+UVM_TESTNAME=bridge_smoke_test \
	+UVM_VERBOSITY=UVM_MEDIUM \
	-voptargs="+acc" \
	-do "run -all; quit"

write_test:
	$(SIM) -c tb_uvm_top \
	+UVM_TESTNAME=bridge_write_test \
	+UVM_VERBOSITY=UVM_MEDIUM \
	-voptargs="+acc" \
	-do "run -all; quit"

read_test:
	$(SIM) -c tb_uvm_top \
	+UVM_TESTNAME=bridge_read_test \
	+UVM_VERBOSITY=UVM_MEDIUM \
	-voptargs="+acc" \
	-do "run -all; quit"

b2b_test:
	$(SIM) -c tb_uvm_top \
	+UVM_TESTNAME=bridge_b2b_test \
	+UVM_VERBOSITY=UVM_MEDIUM \
	-voptargs="+acc" \
	-do "run -all; quit"

reset_test:
	$(SIM) -c tb_uvm_top \
	+UVM_TESTNAME=bridge_reset_test \
	+UVM_VERBOSITY=UVM_MEDIUM \
	-voptargs="+acc" \
	-do "run -all; quit"

rand_test:
	$(SIM) -c -coverage tb_uvm_top \
	+UVM_TESTNAME=bridge_rand_test \
	+UVM_VERBOSITY=UVM_MEDIUM \
	-voptargs="+acc" \
	-do "run -all; coverage save -codeAll bridge.ucdb; quit"

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