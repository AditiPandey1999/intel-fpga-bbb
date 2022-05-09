//ADD 2 NUMBERS


// Copyright (c) 2017, Intel Corporation
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

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <assert.h>
#include <uuid/uuid.h>

#include <opae/fpga.h>

// State from the AFU's JSON file, extracted using OPAE's afu_json_mgr script
#include "afu_json_info.h"

#define CACHELINE_BYTES 64
#define CL(x) ((x) * CACHELINE_BYTES)


//
// Search for an accelerator matching the requested UUID and connect to it.
//
static fpga_handle connect_to_accel(const char *accel_uuid)
{
    fpga_properties filter = NULL;
    fpga_guid guid;
    fpga_token accel_token;
    uint32_t num_matches;
    fpga_handle accel_handle;
    fpga_result r;

    // Don't print verbose messages in ASE by default
    setenv("ASE_LOG", "0", 0);

    // Set up a filter that will search for an accelerator
  /*
    fpga_resultfpgaGetProperties(fpga_tokentoken, fpga_properties *prop)
    Create a fpga_properties object, Initializes the memory pointed at by prop to represent a properties object, and populates it with the properties of the resource referred to by token.
    Individual properties can then be queried using fpgaPropertiesGet*() accessor functions
*/
    fpgaGetProperties(NULL, &filter);
  
  /*
Set the object type of a resource, i.e. the Properties object to modify
 Currently supported object types are FPGA_DEVICE and FPGA_ACCELERATOR.
*/
  
    fpgaPropertiesSetObjectType(filter, FPGA_ACCELERATOR);

    // Add the desired UUID to the filter
  /*
  The uuid_parse() function converts the UUID string specified by in to the internal uuid_t format. 
  int uuid_parse(char *in, uuid_t uu);
  If the input string is parsed successfully, 0 is returned and the UUID is stored in the location pointed to by uu. Otherwise -1 is returned.
  */
    uuid_parse(accel_uuid, guid);
    fpgaPropertiesSetGUID(filter, guid);

    // Do the search across the available FPGA contexts
    num_matches = 1;
    fpgaEnumerate(&filter, 1, &accel_token, 1, &num_matches);

    // Not needed anymore
    fpgaDestroyProperties(&filter);

    if (num_matches < 1)
    {
        fprintf(stderr, "Accelerator %s not found!\n", accel_uuid);
        return 0;
    }

    // Open accelerator
    r = fpgaOpen(accel_token, &accel_handle, 0);
    assert(FPGA_OK == r);

    // Done with token
    fpgaDestroyToken(&accel_token);

    return accel_handle;
}


//
// Allocate a buffer in I/O memory, shared with the FPGA.
//
static volatile void* alloc_buffer(fpga_handle accel_handle,
                                   ssize_t size,
                                   uint64_t *wsid,
                                   uint64_t *io_addr)
{
    fpga_result r;
    volatile void* buf;

    r = fpgaPrepareBuffer(accel_handle, size, (void*)&buf, wsid, 0);
    if (FPGA_OK != r) return NULL;

    // Get the physical address of the buffer in the accelerator
    r = fpgaGetIOAddress(accel_handle, *wsid, io_addr);
    assert(FPGA_OK == r);

    return buf;
}

//CPU program which provides memory address of 2 numbers as input, and expects sum as output in provided address
int main(int argc, char *argv[])
{
    //Search for an accelerator matching the requested UUID and connect to it.
    int a = 10;
    int b = 25;
    fpga_handle accel_handle;
    volatile char *buf;
    uint64_t wsid; //uniquely identify the buffer once the buffer is created (or "prepared")
    uint64_t buf_pa; //physical i/o address where we want to allocate buffer
  
    // Find and connect to the accelerator
    //CPU gets a handle for FPGA to talk to it
    accel_handle = connect_to_accel(AFU_ACCEL_UUID);

    // Allocate a single page memory buffer in I/O memory, shared with the FPGA.
    // buf is a pointer to the buffer
    buf = (volatile char*)alloc_buffer(accel_handle, getpagesize(),
                                       &wsid, &buf_pa);
    
    assert(NULL != buf);
    
    buf[1]= a;
    buf[2]= b;
      
   
    // Set the low byte of the shared buffer to 0.  The FPGA will write
    // a non-zero value to it.
   
    buf[0] = 0;

    // Tell the accelerator the address of the buffer using cache line
    // addresses.  The accelerator will respond by writing to the buffer.
    // calls an API to tell FPGA which address of buffer it is listening on
  
    fpgaWriteMMIO64(accel_handle, 0, 0, buf_pa / CL(1));

    ////////////

    /*uint8_t bit512[64];//64 numbers of 8 (64 bits) bytes each
    memcpy(bit512, &buf_pa, 8);//8 bytes aka 64 bit pa
    bit512[8] = 10;//9th byte
    bit512[9] = 25;//10th byte
    if (FPGA_OK != fpgaWriteMMIO512(accel_handle, 0, 0, (void *)&bit512)) {
        printf ("FPGA Write failed error %x\n", errno);
    }*/
    
    printf("buf_pa %x\n", (int)buf_pa);
   

    // Spin, waiting for the value in memory to change to something non-zero.
    // Keeps waiting for non-null char in buffer to see if fpga has written something

    int i = 0;
    while ( 0 == buf[0] && i != 200)
    {
	if (i%25 == 0)
	    printf("Still waiting %d\n", i);
        // A well-behaved program would use _mm_pause(), nanosleep() or
        // equivalent to save power here.
        i++;
    };
   
    //Once non-null is seen on memory location, prints contents
    // Print the result written by the FPGA
   
   

    printf("buf0 %d\n", buf[0]);
    printf("buf1 %d\n", buf[1]);
    printf("buf2 %d\n", buf[2]);
    printf("buf3 %d\n", buf[3]);
    printf("buf4 %d\n", buf[4]);
    printf("buf5 %d\n", buf[5]);
    printf("buf6 %d\n", buf[6]);
    printf("buf7 %d\n", buf[7]);
    printf("buf8 %d\n", buf[8]);
    printf("buf9 %d\n", buf[9]);

    

    // Done
    fpgaReleaseBuffer(accel_handle, wsid);
    fpgaClose(accel_handle);

    return 0;
}