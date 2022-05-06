//Expt .sv 

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

// Wrapper interface for passing all top-level interfaces into an AFU.
// Every platform must provide this interface.
// https://github.com/OPAE/ofs-platform-afu-bbb/blob/master/plat_if_develop/ofs_plat_if/src/rtl/ofs_plat_if.vh
`include "ofs_plat_if.vh"
`include "afu_json_info.vh"

//
// CCI-P version of hello world AFU example.
//

module ofs_plat_afu
   (
    // All platform wires, wrapped in one interface.
    ofs_plat_if plat_ifc,

    //////////////////////

    //// CSR connections
    //app_csrs.app csrs,
    //tracks outstanding requests.  These will be true as long as
    // reads or unacknowledged writes are still in flight.
    input  logic c0NotEmpty,
    input  logic c1NotEmpty
    //
    // Convert between byte addresses and line addresses.  The conversion
    // is simple: adding or removing low zero bits.
    //

    localparam CL_BYTE_IDX_BITS = 6;
    typedef logic [$bits(t_cci_clAddr) + CL_BYTE_IDX_BITS - 1 : 0] t_byteAddr;

    function automatic t_cci_clAddr byteAddrToClAddr(t_byteAddr addr);
        return addr[CL_BYTE_IDX_BITS +: $bits(t_cci_clAddr)];
    endfunction

    function automatic t_byteAddr clAddrToByteAddr(t_cci_clAddr addr);
        return {addr, CL_BYTE_IDX_BITS'(0)};
    endfunction

    //////////////////////
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
         //fpga interface unit, bridge between platform interfaces like PCIe and Afu-side interfaces like CCIP
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

    //////////////
    // logic [127:0] afu_id = `AFU_ACCEL_UUID;

    ///////////////////
    logic [15:0] cnt_list_length;
    logic [15:0] cnt_data_entries;
    cnt_list_length=2;
    cnt_data_entries=0;

    always_comb
    begin
        // The AFU ID is a unique ID for a given program.  Here we generated
        // one with the "uuidgen" program and stored it in the AFU's JSON file.
        // ASE and synthesis setup scripts automatically invoke afu_json_mgr
        // to extract the UUID into afu_json_info.vh.
        csrs.afu_id = `AFU_ACCEL_UUID;

        // Default
        for (int i = 0; i < NUM_APP_CSRS; i = i + 1)
        begin
            csrs.cpu_rd_csrs[i].data = 64'(0);
        end

        // Exported counters.  The simple csrs interface used here has
        // no read request.  It expects the current CSR value to be
        // available every cycle.
        csrs.cpu_rd_csrs[0].data = 64'(cnt_list_length);
        csrs.cpu_rd_csrs[1].data = 64'(cnt_data_entries);
    end

    ///////////////////
    //
    // A valid AFU must implement a device feature list, starting at MMIO
    // address 0.  Every entry in the feature list begins with 5 64-bit
    // words: a device feature header, two AFU UUID words and two reserved
    // words.
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


    //
    // CSR write handling.  Host software must tell the AFU the memory address
    // to which it should be writing.  The address is set by writing a CSR.
    //

    // We use MMIO address 0 to set the memory address.  The read and
    // write MMIO spaces are logically separate so we are free to use
    // whatever we like.  This may not be good practice for cleanly
    // organizing the MMIO address space, but it is legal.
   
   
    logic is_mem_addr_csr_write;
    assign is_mem_addr_csr_write = is_csr_write &&
                                   (mmio_req_hdr.address == t_ccip_mmioAddr'(0));

    // Memory address to which this AFU will write.
    t_ccip_clAddr mem_write_addr;


    /////////////////////

    logic start_traversal;
    t_ccip_clAddr start_traversal_addr;
    t_ccip_clAddr addr_next;

    always_ff @(posedge clk)
    begin
        if (csrs.cpu_wr_csrs[0].en)
        begin
            result_addr <= byteAddrToClAddr(csrs.cpu_wr_csrs[0].data);
        end

        start_traversal <= csrs.cpu_wr_csrs[1].en;
        start_traversal_addr <= byteAddrToClAddr(csrs.cpu_wr_csrs[1].data);
        addr_next <= start_traversal_addr+1;
    end

     
    //////////////////////

    always_ff @(posedge clk)
    begin
        if (is_mem_addr_csr_write)
        begin
            mem_write_addr <= t_ccip_clAddr'(host_ccip.sRx.c0.data);
        end
    end
   


    // =========================================================================
    //
    //   Main AFU logic
    //
    // =========================================================================

    //
    // States in our simple example.
    //
    typedef enum logic [0:0]
    {
        STATE_IDLE,
        STATE_READ,
        STATE_WRITE,
    }
    t_state;

    t_state state;

    //
    // State machine
    //
    always_ff @(posedge clk)
    begin
        if (!reset_n)
        begin
            state <= STATE_IDLE;
        end
        else
        begin
           
           // Trigger the AFU when mem_addr is set above, when the CPU tells us the address which the FPGA should read a message.
            case (state)
            STATE_IDLE:
                begin
                    if (start_traversal && is_mem_addr_csr_read)
                    begin
                        state <= STATE_READ;
                    $display("AFU reading...");
                    end
                end
            // Trigger the AFU when mem_addr is set above, when the CPU tells us the address to which the FPGA should write a message.
            STATE_READ:
                begin    
                    if (is_mem_addr_csr_write)
                    begin
                        state <= STATE_WRITE;
                    $display("AFU writing...");
                end

                // The AFU completes its task by writing a single line.  When
                // the line is written return to idle.  The write will happen
                // as long as the request channel is not full.

            STATE_WRITE:
                begin
                    if ((state == STATE_WRITE) && ! host_ccip.sRx.c1TxAlmFull)
                    begin
                        state <= STATE_IDLE;
                        $display("AFU done...");
                    end
                end
            endcase
        end
    end

    // READ REQUEST
    //

    // Since back pressure may prevent an immediate read request, we must
    // record whether a read is needed and hold it until the request can
    // be sent to the FIU.
    //
    t_cci_clAddr rd_addr;
    logic rd_needed;
    

    always_ff @(posedge clk)
    begin
        if (reset)
        begin
            rd_needed <= 1'b0;
        end
        else
        begin
            // If reads are allowed this cycle then we can safely clear
            // any previously requested reads.  This simple AFU has only
            // one read in flight at a time since it is walking a pointer
            // chain.
            if (rd_needed)
            begin
                rd_needed <= cp2af_sRx.c0TxAlmFull;
            end
            else
            begin
                // Need a read under two conditions:
                //   - Starting a new walk
                //   - A read response just arrived from a line containing
                //     a next pointer.
                rd_needed <= (start_traversal);
                rd_addr <= (start_traversal ? start_traversal_addr : addr_next);
            end
        end
    end

    // Send read requests to the FIU
    always_ff @(posedge clk)
    begin
        if (reset)
        begin
            af2cp_sTx.c0.valid <= 1'b0;
        end
        else
        begin
            // Generate a read request when needed and the FIU isn't full
            af2cp_sTx.c0.valid <= (rd_needed && ! cp2af_sRx.c0TxAlmFull);
            af2cp_sTx.c0.hdr <= rd_hdr;

            if (rd_needed && ! cp2af_sRx.c0TxAlmFull)
            begin
                $display("  Reading from VA 0x%x", clAddrToByteAddr(rd_addr));
            end
        end
    end
//Read response handling

////////////////////

    //
    // Write "Hello world!" to memory when in STATE_RUN.
    //

    // Construct a memory read request header. 
   
    always_comb
    begin
        rd_hdr = t_cci_c0_ReqMemHdr'(0);
        // Read request type
        rd_hdr.req_type = eREQ_RDLINE_I;
        // Virtual address (MPF virtual addressing is enabled)
        rd_hdr.address = rd_addr;
        // Let the FIU pick the channel
        rd_hdr.vc_sel = eVC_VA;
    end

   // Send read requests to the FIU
    always_ff @(posedge clk)
    begin
        if (reset)
        begin
            af2cp_sTx.c0.valid <= 1'b0;
            cnt_list_length <= 0;
        end
        else
        begin
            // Generate a read request when needed and the FIU isn't full
            af2cp_sTx.c0.valid <= (rd_needed && ! cp2af_sRx.c0TxAlmFull);
            af2cp_sTx.c0.hdr <= rd_hdr;

            if (rd_needed && ! cp2af_sRx.c0TxAlmFull)
            begin
                cnt_list_length <= cnt_list_length + 1;
                $display("  Reading from VA 0x%x", clAddrToByteAddr(rd_addr));
            end
        end
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
        // Let the FIU pick the channel
        wr_hdr.vc_sel = eVC_VA;
        // Start of packet is always set for single beat writes
        wr_hdr.sop = 1'b1;
    end

    // Data to write to memory: little-endian ASCII encoding of "Hello world!"
    //assign host_ccip.sTx.c1.data = t_ccip_clData'('h0021646c726f77206f6c6c6548);
   logic [7:0] res;
   logic [7:0] a;
   logic [7:0] b;
   assign a=t_ccip_clData(rd_hdr1.address);
   assign b=t_ccip_clData(rd_hdr2.address);
   assign res= a+b;
   assign host_ccip.sTx.c1.data = t_ccip_clData(res);

    // Control logic for memory writes
    always_ff @(posedge clk)
    begin
        if (!reset_n)
        begin
            host_ccip.sTx.c1.valid <= 1'b0;
        end
        else
        begin
            // Request the write as long as the channel isn't full.
            host_ccip.sTx.c1.valid <= ((state == STATE_RUN) &&
                                       ! host_ccip.sRx.c1TxAlmFull);
        end

        host_ccip.sTx.c1.hdr <= wr_hdr;
    end


    //
    // This AFU never makes a read request.
    //
    //assign host_ccip.sTx.c0.valid = 1'b0;

endmodule