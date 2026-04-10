/* 
Copyright (c) 2019, IIT Madras All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted
provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions
  and the following disclaimer.  
* Redistributions in binary form must reproduce the above copyright notice, this list of 
  conditions and the following disclaimer in the documentation and/or other materials provided 
  with the distribution.  
* Neither the name of IIT Madras  nor the names of its contributors may be used to endorse or 
  promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS
OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--------------------------------------------------------------------------------------------------

Author: Neel Gala
Email id: neelgala@gmail.com
Details:

--------------------------------------------------------------------------------------------------
*/
package mixed_cluster;
	import AXI4_Lite_Types:: *;
	import AXI4_Lite_Fabric:: *;
  import err_slave::*;
  import Connectable:: *;
  import GetPut:: *;
  
  import i2c :: * ;
  import gptimer::*;
  import gpio :: * ;
  import clint :: * ;
  import plic :: * ;
  import pinmux :: * ;
  import Clocks::*;
  import BUtils::*;
  import pinmux_axi4lite :: * ;
  `include "Soc.defines"

  interface Ifc_mixed_cluster;
     interface AXI4_Lite_Master_IFC#(`paddr,32,0) axi4lite_master;
		method I2C_out i2c0_out;									//I2c IO interface
		method I2C_out i2c1_out;									//I2c IO interface
    method Bit#(2) sb_ext_interrupt;
    (*always_ready, always_enabled*)
    interface GPIO#(32) gpio_io;						//GPIO IO interface
    
    (*always_ready,always_enabled*)
	  interface Ifc_gptimer_io gptimer0_io;
    (*always_ready,always_enabled*)
	  interface Ifc_gptimer_io gptimer1_io;
    (*always_ready,always_enabled*)
	  interface Ifc_gptimer_io gptimer2_io;  
    (*always_ready,always_enabled*)
	  interface Ifc_gptimer_io gptimer3_io;

    interface IOCellSide pinmuxtop_iocell_side;
    interface PeripheralSide pinmuxtop_peripheral_side;
    (*always_ready, always_enabled*)
		method Action interrupts(Bit#(14) inp);
    interface AXI4_Lite_Slave_IFC#(`paddr, `buswidth, `USERSPACE) slave;
    interface AXI4_Lite_Master_IFC#(`paddr, `buswidth, `USERSPACE) xadc_master;
  endinterface

  (*synthesize*)
  module mki2c (Ifc_i2c_axi4lite#(`paddr, `buswidth, `USERSPACE));
	  let core_clock<-exposeCurrentClock;
  	let core_reset<-exposeCurrentReset;
    let ifc();
    mki2c_axi4lite#(core_clock, core_reset) _temp(ifc);
    return ifc;
  endmodule

  (*synthesize*)
module mkgptimer#(Clock ext_clk)(Ifc_gptimer_axi4lite#(`paddr, `buswidth, `USERSPACE,32)); 
  let ifc();
  mkgptimer_axi4lite#(ext_clk) _temp(ifc);
  return ifc;
endmodule

  (*synthesize*)
  module mkgpio(Ifc_gpio_axi4lite#(`paddr, `buswidth, `USERSPACE, 32));
    let ifc();
    mkgpio_axi4lite _temp(ifc);
    return ifc;
  endmodule

  (*synthesize*)
  module mkplic(Ifc_plic_axi4lite#(`paddr, `buswidth, `USERSPACE, 36, 2, 7));
    let ifc();
    mkplic_axi4lite#(`PLICBase)_temp(ifc);
    return ifc;
  endmodule

  (*synthesize*)
  module mkpinmuxtop(Ifc_pinmux_axi4lite#(`paddr, `buswidth, `USERSPACE));
    let ifc();
    mkpinmux_axi4lite _temp(ifc);
    return ifc;
  endmodule

  function Bit#(TLog#(`MixedCluster_Num_Slaves)) fn_slave_map (Bit#(`paddr) addr);
    Bit#(TLog#(`MixedCluster_Num_Slaves)) slave_num = 0;
    if(addr>= `GPIOBase && addr<= `GPIOEnd)
      slave_num = `GPIO_slave_num;
    else if(addr>= `PLICBase && addr<= `PLICEnd)
      slave_num = `PLIC_slave_num;
    else if(addr>= `I2C1Base && addr<= `I2C1End)
      slave_num = `I2C1_slave_num;
    else if(addr>= `I2C0Base && addr<= `I2C0End)
      slave_num = `I2C0_slave_num;
    else if(addr>= `GPTimer0Base && addr<= `GPTimer0End)
			slave_num = `GPTimer0_slave_num;
    else if(addr>= `GPTimer1Base && addr<= `GPTimer1End)
		  slave_num = `GPTimer1_slave_num;
    else if(addr>= `AXI4LiteBase && addr<= `AXI4LiteEnd)
		  slave_num = `AXI4Lite_slave_num;	  	  
    else if(addr>= `GPTimer2Base && addr<= `GPTimer2End)
		  slave_num = `GPTimer2_slave_num;
    else if(addr>= `GPTimer3Base && addr<= `GPTimer3End)
		  slave_num = `GPTimer3_slave_num;
    else if(addr>= `XADCBase && addr<= `XADCEnd)
      slave_num = `XADC_slave_num;
    else if(addr >= `PinmuxBase && addr <= `PinmuxEnd)
      slave_num = `Pinmux_slave_num;
    else
      slave_num = `MixedCluster_err_slave_num;
      
    return slave_num;
  endfunction:fn_slave_map

  (*synthesize*)
  module mkmixed_cluster#(Clock ext_clk)(Ifc_mixed_cluster);
	
		AXI4_Lite_Master_Xactor_IFC #(`paddr, `buswidth, `USERSPACE) c2m_xactor <- mkAXI4_Lite_Master_Xactor;
		AXI4_Lite_Slave_Xactor_IFC #(`paddr, `buswidth, `USERSPACE) c2s_xactor <- mkAXI4_Lite_Slave_Xactor;
    AXI4_Lite_Fabric_IFC #(`MixedCluster_Num_Masters, `MixedCluster_Num_Slaves, `paddr, `buswidth,`USERSPACE) 
                                                    fabric <- mkAXI4_Lite_Fabric(fn_slave_map);
    
    
    let curr_clk<- exposeCurrentClock;
    let curr_rst <- exposeCurrentReset;
    let i2c0 <- mki2c;
    let i2c1 <- mki2c;
    let gpio <- mkgpio();
    let plic <- mkplic();
    let gptimer0 <- mkgptimer(ext_clk);
    let gptimer1 <- mkgptimer(ext_clk);
    let gptimer2 <- mkgptimer(ext_clk);
    let gptimer3 <- mkgptimer(ext_clk);
    let pinmuxtop <- mkpinmuxtop();
    Ifc_err_slave_axi4lite#(`paddr, `buswidth, `USERSPACE ) err_slave <- mkerr_slave_axi4lite;
		Wire#(Bit#(14)) wr_external_interrupts <- mkDWire('d0);
    Wire#(Bit#(2)) wr_sb_ext_interrupt <- mkDWire(0);

		//Rule to connect PLIC interrupt to the core's sideband
		rule rl_core_plic_connection;
			let x =  plic.sb_to_targets;
			wr_sb_ext_interrupt <= {pack(x[1]), pack(x[0])};
		endrule

    rule rl_connect_plic_connections;
			let tmp <- gpio.sb_gpio_to_plic.get;
			Bit#(16) lv_gpio_intr= truncate(pack(tmp));
			Bit#(36) plic_inputs= {wr_external_interrupts[13], wr_external_interrupts[12:6], i2c1.isint, i2c0.isint, gptimer3.sb_interrupt, gptimer2.sb_interrupt, gptimer1.sb_interrupt, gptimer0.sb_interrupt, lv_gpio_intr, wr_external_interrupts[5:0]};
			plic.sb_frm_sources(plic_inputs);
		endrule

   	mkConnection(c2m_xactor.axi_side, fabric.v_from_masters[0]);

    mkConnection(c2s_xactor.o_wr_addr,c2m_xactor.i_wr_addr);
    mkConnection(c2s_xactor.o_wr_data,c2m_xactor.i_wr_data);
    mkConnection(c2m_xactor.o_wr_resp,c2s_xactor.i_wr_resp);
    mkConnection(c2s_xactor.o_rd_addr,c2m_xactor.i_rd_addr);
    mkConnection(c2m_xactor.o_rd_data,c2s_xactor.i_rd_data);
		
   	mkConnection (fabric.v_to_slaves [`I2C0_slave_num ],		i2c0.slave);
   	mkConnection (fabric.v_to_slaves [`I2C1_slave_num ],		i2c1.slave);
     mkConnection (fabric.v_to_slaves [`GPTimer0_slave_num ],gptimer0.slave);
     mkConnection (fabric.v_to_slaves [`GPTimer1_slave_num ],gptimer1.slave);
     mkConnection (fabric.v_to_slaves [`GPTimer2_slave_num ],gptimer2.slave);
     mkConnection (fabric.v_to_slaves [`GPTimer3_slave_num ],gptimer3.slave);
		mkConnection (fabric.v_to_slaves [`PLIC_slave_num ], plic.slave);
		mkConnection (fabric.v_to_slaves [`GPIO_slave_num ], gpio.slave);
		mkConnection (fabric.v_to_slaves [`Pinmux_slave_num ], pinmuxtop.slave);
    mkConnection (fabric.v_to_slaves [`MixedCluster_err_slave_num ] , err_slave.slave);
		
    method I2C_out i2c0_out= i2c0.io;
    method I2C_out i2c1_out= i2c1.io;
    interface gptimer0_io = gptimer0.io;
    interface gptimer1_io = gptimer1.io;
    interface gptimer2_io = gptimer2.io;
    interface gptimer3_io = gptimer3.io;
   

    method sb_ext_interrupt = wr_sb_ext_interrupt;
    interface gpio_io= gpio.io;
		method Action interrupts(Bit#(14) inp);
			wr_external_interrupts<= inp;
		endmethod
    interface pinmuxtop_iocell_side = pinmuxtop.pinmuxaxi4lite_iocell_side;
    interface pinmuxtop_peripheral_side = pinmuxtop.pinmuxaxi4lite_peripheral_side;
    interface slave= c2s_xactor.axi_side;
    interface axi4lite_master = fabric.v_to_slaves[`AXI4Lite_slave_num];
    interface xadc_master = fabric.v_to_slaves[`XADC_slave_num];
  endmodule
endpackage
