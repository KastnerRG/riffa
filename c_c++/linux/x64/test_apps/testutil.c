// ----------------------------------------------------------------------
// Copyright (c) 2016, The Regents of the University of California All
// rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
// 
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
// 
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
// 
//     * Neither the name of The Regents of the University of California
//       nor the names of its contributors may be used to endorse or
//       promote products derived from this software without specific
//       prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL REGENTS OF THE
// UNIVERSITY OF CALIFORNIA BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
// OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
// TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
// USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.
// ----------------------------------------------------------------------

#include <pthread.h>
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include "timer.h"
#include "riffa.h"
#define NUM_TESTS 100

struct thread_info {    /* Used as argument to thread_start() */

	// please refer to API of fpga_send() and fpga_recv() at http://riffa.ucsd.edu/node/10 or https://github.com/KastnerRG/riffa/blob/master/driver/linux/riffa.c#L84-L111
	fpga_t * fpga;       
	unsigned int chnl;     
	unsigned int * buffer;
	unsigned int len;
	unsigned int offset;
	unsigned int last;
	long long timeout;
};

int main(int argc, char** argv) {
	fpga_t * fpga;
	fpga_info_list info;
	int option;
	unsigned int i;
	int id;
	int chnl;
	unsigned int numWords;
	unsigned int * sendBuffer;
	unsigned int * recvBuffer;

	GET_TIME_INIT(3);

	if (argc < 2) {
		printf("Usage: %s <option>\n", argv[0]);
		return -1;
	}

	option = atoi(argv[1]);

	if (option == 0) {
		// List FPGA info
		// Populate the fpga_info_list struct
		if (fpga_list(&info) != 0) {
			printf("Error populating fpga_info_list\n");
			return -1;
		}
		printf("Number of devices: %d\n", info.num_fpgas);
		for (i = 0; i < (unsigned int)info.num_fpgas; i++) {
			printf("%d: id:%d\n", i, info.id[i]);
			printf("%d: num_chnls:%d\n", i, info.num_chnls[i]);
			printf("%d: name:%s\n", i, info.name[i]);
			printf("%d: vendor id:%04X\n", i, info.vendor_id[i]);
			printf("%d: device id:%04X\n", i, info.device_id[i]);
		}
	}
	else if (option == 1) { // Reset FPGA
		if (argc < 3) {
			printf("Usage: %s %d <fpga id>\n", argv[0], option);
			return -1;
		}

		id = atoi(argv[2]);

		// Get the device with id
		fpga = fpga_open(id);
		if (fpga == NULL) {
			printf("Could not get FPGA %d\n", id);
			return -1;
		}

		// Reset
		fpga_reset(fpga);

		// Done with device
		fpga_close(fpga);
	}
	else if (option == 2) { // Send data, receive data
		if (argc < 5) {
			printf("Usage: %s %d <fpga id> <chnl> <num words to transfer>\n", argv[0], option);
			return -1;
		}

		unsigned int maxWords, minWords;
		id = atoi(argv[2]);
		chnl = atoi(argv[3]);
		minWords = 4; // Must be at least 4 for the channel tester app
		maxWords = atoi(argv[4]);
		printf("Running bandwidth test from %d up to %d words\n", minWords, maxWords);

		// Get the device with id
		fpga = fpga_open(id);
		if (fpga == NULL) {
			printf("Could not get FPGA %d\n", id);
			return -1;
		}

		// Malloc the arrays
		sendBuffer = (unsigned int *)malloc(maxWords*4);
		if (sendBuffer == NULL) {
			printf("Could not malloc memory for sendBuffer\n");
			fpga_close(fpga);
			return -1;
		}
		recvBuffer = (unsigned int *)malloc(maxWords*4 + 4);
		recvBuffer +=1;
		if (recvBuffer == NULL) {
			printf("Could not malloc memory for recvBuffer\n");
			free(sendBuffer);
			fpga_close(fpga);
			return -1;
		}

		for (numWords = minWords; numWords <= maxWords; numWords += (2*numWords <= maxWords) ? numWords : (maxWords-numWords)) { // adaptively change the buffer size for the final iteration, and double the transaction size for every iteration
			//int j;
			//for (j = 0; j < NUM_TESTS + 1; ++j) {
				// Initialize the data
				for (i = 0; i < numWords; i++) {
					sendBuffer[i] = i+1;
					recvBuffer[i] = 0;
				}

				int NTH = 2;  // number of threads : one fpga_recv() thread, one fpga_send() thread

				pthread_t tid[NTH];
				struct thread_info tinfo[NTH];
				/*
				struct thread_info *tinfo;
				// Allocate memory for pthread_create() arguments 
				tinfo = calloc(NTH, sizeof(struct thread_info));
							if (tinfo == NULL)
				exit(1);
				*/

				unsigned int ret_val[NTH]; // retval[0] is number of words sent;  retval[1] is number of words received

				int loop = 0;  // for pthread_join()

				//printf("\n Going to create threads \n");
				/** Creation of threads*/
				/*	for(loop=0; loop<NTH; loop++) {
				pthread_create(&tid[loop], NULL, &sample, &value[loop]);
				printf("\n value of loop = %d\n", loop);
				}
				*/

				unsigned int offset_for_sending = 0;
				unsigned int is_last_send = 1;
				unsigned int chnl_timeout = 5;  // timeout is 5ms, for low-latency purpose, please see the description at http://xillybus.com/doc/xillybus-latency

				assert(tinfo != NULL); // null check

				tinfo[0].fpga = fpga;
				tinfo[0].chnl = chnl;
				tinfo[0].buffer = sendBuffer;
				tinfo[0].len = numWords;  // try to send all words before thread switching due to timeout
				tinfo[0].offset = offset_for_sending;
				tinfo[0].last = is_last_send;
				tinfo[0].timeout = chnl_timeout;  

				pthread_create(&tid[0], NULL, &fpga_send, &tinfo[0]);
				//printf("\n value of loop = %d\n", 0);

				tinfo[1].fpga = fpga;
				tinfo[1].chnl = chnl;
				tinfo[1].buffer = recvBuffer;
				tinfo[1].len = numWords;  // try to receive all words that have been sent before thread switching due to timeout
				tinfo[1].timeout = chnl_timeout;  

				pthread_create(&tid[1], NULL, &fpga_recv, &tinfo[1]);
				//printf("\n value of loop = %d\n", 1);

				/** Synch of threads in order to exit normally*/
				GET_TIME_VAL(0);

				for(loop=0; loop<NTH; loop++) {
					pthread_join(tid[loop], (void**)&ret_val[loop]);
				}

				GET_TIME_VAL(1);

				const double MILLI_CONVERSION = 1000.0;  // converts milliseconds to seconds
				const unsigned int BIRECTION = 2; // two ways, so total number of data transferred is doubled
				double total_execution_time = ((TIME_VAL_TO_MS(1) - TIME_VAL_TO_MS(0)) / MILLI_CONVERSION);   // in seconds

				printf("number of words sent = %d\n\r", ret_val[0]);
				printf("number of words recv = %d\n\r", ret_val[1]);

				printf("Total execution time = %f s\n\r", total_execution_time);

				if(ret_val[1] == numWords) 	// number of words sent == number of words received
				{
					const int GIGA_CONVERSION = 1000*1000*1000;  // converts Bps to GBps
					const int BYTES_PER_WORD = 4;	// 32-bit = 4 bytes
					const int WORDS_PER_TRANSACTION = 4;  // we are using 128-bit PCIe interface. therefore there are 4 32-bit words in each transaction

					// check the data
					for (i = WORDS_PER_TRANSACTION; i < numWords; i++) {  // the first 4 32-bit words are always corrupted, please refer to explanation given at https://pergamos.lib.uoa.gr/uoa/dl/frontend/file/lib/default/data/1326221/theFile#page=38
						if (recvBuffer[i] != sendBuffer[i]) {
							printf("recvBuffer[%d]: %d, expected %d\n", i, recvBuffer[i], sendBuffer[i]);
							return -1;
						}
					}
					printf("Overall bandwidth: %f GBps\n\n", (double)BIRECTION*numWords*(double)BYTES_PER_WORD/(double)GIGA_CONVERSION/total_execution_time);
				}
				
				if(numWords == maxWords) break; // last iteration, so exit the loop
		}

		// Done with device
	    fpga_close(fpga);
	}

	return 0;
}
