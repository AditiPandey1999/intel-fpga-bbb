//
// Copyright (c) 2020, Intel Corporation
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
//
// Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// Neither the name of the Intel Corporation nor the names of its contributors
// may be used to endorse or promote products derived from this software
// without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

`include "ofs_plat_if.vh"
`include "afu_json_info.vh"

//
// CCI-P version of add two numbers AFU example.
//

module ofs_plat_afu
   (
    // All platform wires, wrapped in one interface.
    ofs_plat_if plat_ifc
    );

    // ====================================================================
    //
    //  Get a CCI-P port from the platform.
    //
    // ====================================================================

    // Instance of a CCI-P interface. The interface wraps usual CCI-P
    // sRx and sTx structs as well as the associated clock and reset.
    ofs_plat_host_ccip_if host_ccip();

    // Use the platform-provided module to map the primary host interface
    // to CCI-P. The "primary" interface is the port that includes the
    // main OPAE-managed MMIO connection. This primary port is always
    // index 0 of plat_ifc.host_chan.ports, indepedent of the platform
    // and the native protocol of the host channel.
    ofs_plat_host_chan_as_ccip primary_ccip
       (
        .to_fiu(plat_ifc.host_chan.ports[0]),
        .to_afu(host_ccip),

        // These ports would be used if the PIM is told to cross to
        // a different clock. In this example, host_ccip is instantiated
        // with the native pClk.
        .afu_clk(),
        .afu_reset_n()
        );


    // Each interface names its associated clock and reset.
    logic clk;
    assign clk = host_ccip.clk;
    logic reset_n;
    assign reset_n = host_ccip.reset_n;


    // ====================================================================
    //
    //  Tie off unused ports.
    //
    // ====================================================================

    // The PIM ties off unused devices, controlled by the AFU indicating
    // which devices it is using. This way, an AFU must know only about
    // the devices it uses. Tie-offs are thus portable, with the PIM
    // managing devices unused by and unknown to the AFU.
    ofs_plat_if_tie_off_unused
      #(
        // Host channel group 0 port 0 is connected. The mask is a
        // bit vector of indices used by the AFU.
        .HOST_CHAN_IN_USE_MASK(1)
        )
        tie_off(plat_ifc);


    // =========================================================================
    //
    //   CSR (MMIO) handling.
    //
    // =========================================================================

    // The AFU ID is a unique ID for a given program.  Here we generated
    // one with the "uuidgen" program and stored it in the AFU's JSON file.
    // ASE and synthesis setup scripts automatically invoke afu_json_mgr
    // to extract the UUID into afu_json_info.vh.
    logic [127:0] afu_id = `AFU_ACCEL_UUID;

    //
    // A valid AFU must implement a device feature list, starting at MMIO
    // address 0.  Every entry in the feature list begins with 5 64-bit
    // words: a device feature header, two AFU UUID words and two reserved
    // words.
    /*a feature list structure that creates a linked list of feature headers within MMIO
    space, thus providing an extensible way of adding features. The software can walk through the feature
    headers to enumerate the following:
     AFUs
     Basic Building Blocks (BBBs)
     Private features  */
    //

    // Is a CSR read request active this cycle?
    logic is_csr_read;
    assign is_csr_read = host_ccip.sRx.c0.mmioRdValid;

    

    // Is a CSR write request active this cycle?
    logic is_csr_write;
    assign is_csr_write = host_ccip.sRx.c0.mmioWrValid;

    // The MMIO request header is overlayed on the normal c0 memory read
    // response data structure.  Cast the c0Rx header to an MMIO request
    // header.
    t_ccip_c0_ReqMmioHdr mmio_req_hdr;
    assign mmio_req_hdr = t_ccip_c0_ReqMmioHdr'(host_ccip.sRx.c0.hdr);


    //
    // Implement the device feature list by responding to MMIO reads.
    //

    always_ff @(posedge clk)
    begin
        if (!reset_n)
        begin
            host_ccip.sTx.c2.mmioRdValid <= 1'b0;
        end
        else
        begin
            // Always respond with something for every read request
            host_ccip.sTx.c2.mmioRdValid <= is_csr_read;

            // The unique transaction ID matches responses to requests
            host_ccip.sTx.c2.hdr.tid <= mmio_req_hdr.tid;

            // Addresses are of 32-bit objects in MMIO space.  Addresses
            // of 64-bit objects are thus multiples of 2.
            case (mmio_req_hdr.address)
              0: // AFU DFH (device feature header)
                begin
                    // Here we define a trivial feature list.  In this
                    // example, our AFU is the only entry in this list.
                    host_ccip.sTx.c2.data <= t_ccip_mmioData'(0);
                    // Feature type is AFU
                    host_ccip.sTx.c2.data[63:60] <= 4'h1;
                    // End of list (last entry in list)
                    host_ccip.sTx.c2.data[40] <= 1'b1;
                end

              // AFU_ID_L
              2: host_ccip.sTx.c2.data <= afu_id[63:0];

              // AFU_ID_H
              4: host_ccip.sTx.c2.data <= afu_id[127:64];

              // DFH_RSVD0
              6: host_ccip.sTx.c2.data <= t_ccip_mmioData'(0);

              // DFH_RSVD1
              8: host_ccip.sTx.c2.data <= t_ccip_mmioData'(0);

              default: host_ccip.sTx.c2.data <= t_ccip_mmioData'(0);
            endcase
        end
    end

    logic is_mem_addr_csr_write;
    assign is_mem_addr_csr_write = is_csr_write &&
                                   (mmio_req_hdr.address == t_ccip_mmioAddr'(0));

    // Memory address to which this AFU will write.
    t_ccip_clAddr mem_addr;

    always_ff @(posedge clk)
    begin
        if (is_mem_addr_csr_write)
        begin
            mem_addr <= t_ccip_clAddr'(host_ccip.sRx.c0.data);
        end
    end
    
    // Construct a memory read request header. 
     t_ccip_c0_ReqMemHdr rd_hdr;
    always_comb
    begin
        rd_hdr = t_ccip_c0_ReqMemHdr'(0);
        // Read request type
        //rd_hdr.req_type = t_ccip_clLen(4'h0);
        // Virtual address (MPF virtual addressing is enabled)
        rd_hdr.address = mem_addr;
        // Let the FIU pick the channel
        //rd_hdr.vc_sel = t_ccip_vc(2'h0);

        //rd_hdr.cl_len = t_ccip_clLen(2'h0);
    end

    


    // Construct a memory write request header.  For this AFU it is always
    // the same, since we write to only one address.
    t_ccip_c1_ReqMemHdr wr_hdr;
    always_comb
    begin
        // Zero works for most write request header fields in this example
        wr_hdr = t_ccip_c1_ReqMemHdr'(0);
        // Set the write address
        wr_hdr.address = mem_addr;
        // Start of packet is always set for single beat writes
        wr_hdr.sop = 1'b1;
    end


    // =========================================================================
    //
    //   Main AFU logic
    //
    // =========================================================================

    //
    // States in our simple example.
    //


    logic [7:0] res;
    logic [7:0] a;
    logic [7:0] b;


    typedef enum logic [3:0]
    {
        STATE_IDLE,
        STATE_SEND_READ_REQUEST,
        STATE_READ_RESPONSE,
        STATE_NUM,
        STATE_WRITE
    }
    t_state;

    t_state state;

    //
    // State machine
    //
    t_ccip_c0_RspMemHdr rsp_hdr;////
    //logic  mem_read_data[31:0];///
    t_ccip_clData mem_read_data;


    always_ff @(posedge clk)
    begin
        if (!reset_n)
        begin
            state <= STATE_IDLE;
            host_ccip.sTx.c1.valid <= 1'b0;
            host_ccip.sTx.c0.valid <= 1'b0;
        end
        else
        begin
             case (state)
                STATE_IDLE:
                    // Trigger the AFU when mem_addr is set above.  (When the CPU
                    // tells us the address to which the FPGA should write a message.)
                    if (is_mem_addr_csr_write)// you have the address to which you have to write, and therefore corresp read addresses
                    begin
                        host_ccip.sTx.c0.hdr <= rd_hdr;
                        //state <= STATE_SEND_READ_REQUEST;
                        state <= STATE_SEND_READ_REQUEST;
                        $display("AFU sending read request..."); //for reading first and second number //1
                    end

                    
                    // Trigger the AFU when mem_addr is set above, when the CPU tells us the address to which the FPGA should write a message.
                    STATE_SEND_READ_REQUEST:
                    begin    
                        // Control logic for memory read request 
                        
                        host_ccip.sTx.c0.valid <= 1'b1;
                        host_ccip.sTx.c1.valid <= 1'b0;
                        state <= STATE_READ_RESPONSE;
                        $display("Waiting for AFU receiving response...");
                
                    end

                    STATE_READ_RESPONSE:
                    begin
                        //Memory Read Response Header
                        if(host_ccip.sRx.c0.rspValid)
                        begin
                            $display(" AFU received response...");
                            rsp_hdr <= t_ccip_c0_RspMemHdr'(0);
                            mem_read_data <= t_ccip_clData'(host_ccip.sRx.c0.data);
                            $display(" num 1 %d, num 2 %d", mem_read_data[15:8], mem_read_data[23:16]);
                            state <= STATE_NUM;
                        end
                    end

                    STATE_NUM:
                    begin
                        /*
                        a <= mem_read_data[15:8];
                        b <= mem_read_data[23:16];
                        $display(" num 1 %d, num 2 %d", a, b) */
                        host_ccip.sTx.c1.data <= t_ccip_clData'(50);//2 
                        state <= STATE_WRITE;
                    end

                    // The AFU completes its task by writing a single line.  When
                    // the line is written return to idle.  The write will happen
                    // as long as the request channel is not full.


                    STATE_WRITE:
                    begin
                        // Control logic for memory writes
                        // Request the write as long as the channel isn't full.
                        host_ccip.sTx.c1.hdr <= wr_hdr;
                        //host_ccip.sTx.c1.data <= t_ccip_clData'(a);
                        host_ccip.sTx.c1.valid <= (! host_ccip.sRx.c1TxAlmFull);
                        host_ccip.sTx.c0.valid <= 1'b0;
                            
                        state <= STATE_IDLE;
                        $display("AFU done...");
                    
                    end
             endcase
        end
    end

endmodule

/*

CPU --> Write mem-addr (read inputs and write output) --> FPGA (S1)
FPGA --> i received a read (MMioRdValid), mem_addr = what i got from cpu
num1 and num2 ?
FPGA -> read contents of mem_addr -- CCIP read req from FPGA to CPU
FPGA --> waits for resp
*/









