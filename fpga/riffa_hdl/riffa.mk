# ----------------------------------------------------------------------
# Copyright (c) 2016, The Regents of the University of California All
# rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
# 
#     * Redistributions in binary form must reproduce the above
#       copyright notice, this list of conditions and the following
#       disclaimer in the documentation and/or other materials provided
#       with the distribution.
# 
#     * Neither the name of The Regents of the University of California
#       nor the names of its contributors may be used to endorse or
#       promote products derived from this software without specific
#       prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL REGENTS OF THE
# UNIVERSITY OF CALIFORNIA BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
# TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
# USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.
# ----------------------------------------------------------------------
$(VENDOR)_HDL=$(VENDOR).vh translation_$(VENDOR).v

ENG_TYPES=c r

$(TYPE)_ENGINE_HDL= tx_engine_$(TYPE).v rx_engine_$(TYPE).v \
	rxc_engine_$(TYPE).v rxr_engine_$(TYPE).v \
	txc_engine_$(TYPE).v txr_engine_$(TYPE).v
ENGINE_HDL = tx_alignment_pipeline.v tx_data_fifo.v tx_data_pipeline.v \
	tx_data_shift.v tx_engine.v tx_engine_selector.v tx_hdr_fifo.v \
	$($(TYPE)_ENGINE_HDL)


classic_HDL=tlp.vh $(CLASSIC_ENGINE_HDL)
ultrascale_HDL=ultrascale.vh

xilinx_classic_HDL += rxr_engine_128.v 

RIFFA_$(WIDTH)_HDL=channel_$(WIDTH).v tx_port_$(WIDTH).v rx_port_$(WIDTH).v \
	tx_port_buffer_$(WIDTH).v tx_port_channel_gate_$(WIDTH).v \
	tx_multiplexer_$(WIDTH).v tx_port_monitor_$(WIDTH).v \
	sg_list_reader_$(WIDTH).v fifo_packer_$(WIDTH).v 

RIFFA_HDL= types.vh widths.vh trellis.vh schedules.vh functions.vh riffa.vh \
        tx_port_writer.v tx_multiplexer.v syncff.v \
	async_fifo.v async_fifo_fwft.v channel.v chnl_tester.v counter.v \
	cross_domain_signal.v demux.v engine_layer.v ff.v fifo.v \
	interrupt.v interrupt_controller.v mux.v offset_flag_to_one_hot.v \
	offset_to_mask.v one_hot_mux.v pipeline.v ram_1clk_1w_1r.v \
	ram_2clk_1w_1r.v recv_credit_flow_ctrl.v register.v registers.v \
	reorder_queue.v reorder_queue_input.v reorder_queue_output.v \
	reset_controller.v reset_extender.v riffa.v rotate.v \
	rx_port_channel_gate.v rx_port_reader.v rx_port_requester_mux.v \
	scsdpram.v sg_list_requester.v shiftreg.v sync_fifo.v \
	$(ENGINE_HDL) $(RIFFA_$(WIDTH)_HDL) $($(TYPE)_HDL) $($(VENDOR)_HDL)






