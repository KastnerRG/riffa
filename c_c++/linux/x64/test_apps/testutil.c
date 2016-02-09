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
#include <stdlib.h>
#include <stdio.h>
#include "timer.h"
#include "riffa.h"
#define NUM_TESTS 100

int main(int argc, char** argv) {
	fpga_t * fpga;
	fpga_info_list info;
	int option;
	int i;
	int id;
	int chnl;
	size_t numWords;
	int sent;
	int recvd;
	int failure = 0;
	unsigned int * sendBuffer;
	unsigned int * recvBuffer;
	int err;
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
		for (i = 0; i < info.num_fpgas; i++) {
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

		size_t maxWords, minWords;
		id = atoi(argv[2]);
		chnl = atoi(argv[3]);
		minWords = 4; // Must be at least 4 for the channel tester app
		maxWords = atoi(argv[4]);
		printf("Running bandwidth test from %zu up to %zu words\n", minWords, maxWords);

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

		int numWords;
		for (numWords = minWords; numWords <= maxWords; numWords = numWords*2) {
			int j;
			for (j = 0; j < NUM_TESTS + 1; ++j) {
				// Initialize the data
				for (i = 0; i < numWords; i++) {
					sendBuffer[i] = i+1;
					recvBuffer[i] = 0;
				}

				GET_TIME_VAL(0);

				// Send the data
				sent = fpga_send(fpga, chnl, sendBuffer, numWords, 0, 1, 25000);
				printf("Test %d: words sent: %d\n", j, sent);

				GET_TIME_VAL(1);

				if (sent != 0) {
					// Recv the data
					recvd = fpga_recv(fpga, chnl, recvBuffer, numWords, 25000);
					printf("Test %d: words recv: %d\n", j, recvd);
				}

				GET_TIME_VAL(2);

				// Check the data
				if (recvd != 0) {
					for (i = 4; i < recvd; i++) {
						if (recvBuffer[i] != sendBuffer[i]) {
							printf("recvBuffer[%d]: %d, expected %d\n", i, recvBuffer[i], sendBuffer[i]);
							return;
						}
					}

					if (j > 0)
						printf("send bw: %f\n",
							sent*4.0/1000/1000/((TIME_VAL_TO_MS(1) - TIME_VAL_TO_MS(0))/1000.0)); //,

					if (j > 0)
						printf("recv bw: %f\n",
							recvd*4.0/1000/1000/((TIME_VAL_TO_MS(2) - TIME_VAL_TO_MS(1))/1000.0)); //,
				}
			}
		}
		// Done with device
	        fpga_close(fpga);
	}
	else if (option == 3) { // Send data, receive data
		if (argc < 5) {
			printf("Usage: %s %d <fpga id> <chnl> <num words to transfer>\n", argv[0], option);
			return -1;
		}

		size_t maxWords, minWords;
		id = atoi(argv[2]);
		chnl = atoi(argv[3]);
		minWords = 4; // Must be at least 4 for the channel tester app
		maxWords = atoi(argv[4]);
		printf("Running receive offset test from %zu up to %zu words\n", minWords, maxWords);

		// Get the device with id
		fpga = fpga_open(id);
		if (fpga == NULL) {
			printf("Could not get FPGA %d\n", id);
			return -1;
		}

		// Malloc the arrays (page aligned) 
		printf("Asked for %zu bytes\n",((maxWords*sizeof(unsigned int)*2 + 4096)/4096)*4096 + 4096);
		err = posix_memalign((void **)&sendBuffer, 4096, ((maxWords*sizeof(unsigned int)*2 + 4096)/4096)*4096 + 4096);
		if (sendBuffer == NULL) {
			printf("Could not malloc memory for sendBuffer\n");
			fpga_close(fpga);
			return -1;
		}
		err = posix_memalign((void **)&recvBuffer, 4096, ((maxWords*sizeof(unsigned int)*2 + 4096)/4096)*4096 + 4096);

		recvBuffer = (unsigned int *)malloc(((maxWords*sizeof(unsigned int)*2 + 4096)/4096)*4096 + 4096);
		if (recvBuffer == NULL) {
			printf("Could not malloc memory for recvBuffer\n");
			free(sendBuffer);
			fpga_close(fpga);
			return -1;
		}

		int numWords;
		for (numWords = minWords; numWords <= maxWords; numWords = numWords*2) {
			int j;
			for (j = 0; j < 4096/sizeof(unsigned int); j++) {
				int rxOffset = j;
				// Initialize the data
				for (i = 0; i < numWords; i++) {
					sendBuffer[i+ rxOffset] = i+1;
					recvBuffer[i] = 0;
				}

				GET_TIME_VAL(0);

				// Send the data
				sent = fpga_send(fpga, chnl, &sendBuffer[rxOffset], numWords, 0, 1, 25000);
				printf("Test %d: words sent from address %p: %d\n", j, &sendBuffer[rxOffset], sent);

				GET_TIME_VAL(1);

				if (sent != 0) {
					// Recv the data
					recvd = fpga_recv(fpga, chnl, recvBuffer, numWords, 25000);
					printf("Test %d: words recv: %d\n", j, recvd);
				}

				GET_TIME_VAL(2);

				// Check the data
				if (recvd != 0) {
					for (i = 4; i < recvd; i++) {
						if (recvBuffer[i] != sendBuffer[i + rxOffset]) {
							printf("recvBuffer[%d]: %d, expected %d\n", i, recvBuffer[i], sendBuffer[i + rxOffset]);
							failure = 1;
						}
					}


					if (j > 0)
						printf("send bw: %f\n",
							sent*4.0/1000/1000/((TIME_VAL_TO_MS(1) - TIME_VAL_TO_MS(0))/1000.0)); //,

					if (j > 0)
						printf("recv bw: %f\n",
							recvd*4.0/1000/1000/((TIME_VAL_TO_MS(2) - TIME_VAL_TO_MS(1))/1000.0)); //,
					if(failure) 
						return;
				}
			}
		}
		// Done with device
	        fpga_close(fpga);
	}	
	else if (option == 4) { // Send data, receive data
		if (argc < 5) {
			printf("Usage: %s %d <fpga id> <chnl> <num words to transfer>\n", argv[0], option);
			return -1;
		}

		size_t maxWords, minWords;
		id = atoi(argv[2]);
		chnl = atoi(argv[3]);
		minWords = 4; // Must be at least 4 for the channel tester app
		maxWords = atoi(argv[4]);
		printf("Running tx offset test from %zu up to %zu words\n", minWords, maxWords);

		// Get the device with id
		fpga = fpga_open(id);
		if (fpga == NULL) {
			printf("Could not get FPGA %d\n", id);
			return -1;
		}

		// Malloc the arrays (page aligned) 
		printf("Asked for %zu bytes\n",((maxWords*sizeof(unsigned int)*2 + 4096)/4096)*4096 + 4096);

		err = posix_memalign((void **)&sendBuffer, 4096, ((maxWords*sizeof(unsigned int)*2 + 4096)/4096)*4096 + 4096);
		if (err) {
			printf("Could not malloc memory for sendBuffer\n");
			fpga_close(fpga);
			return -1;
		}
		err = posix_memalign((void **)&recvBuffer, 4096, ((maxWords*sizeof(unsigned int)*2 + 4096)/4096)*4096 + 4096);

		if (err) {
			printf("Could not malloc memory for recvBuffer\n");
			free(sendBuffer);
			fpga_close(fpga);
			return -1;
		}

		int numWords;
		for (numWords = minWords; numWords <= maxWords; numWords = numWords*2) {
			int j;
			for (j = 0; j < 4096/sizeof(unsigned int); ++j) {
				int txOffset = j;
				// Initialize the data
				for (i = 0; i < numWords; i++) {
					sendBuffer[i] = i+1;
					recvBuffer[i + txOffset] = 0;
				}

				GET_TIME_VAL(0);

				// Send the data
				sent = fpga_send(fpga, chnl, sendBuffer, numWords, 0, 1, 25000);
				printf("Test %d: words sent: %d (Address %p) \n", j, sent, sendBuffer);

				GET_TIME_VAL(1);

				if (sent != 0) {
					// Recv the data
					recvd = fpga_recv(fpga, chnl, &recvBuffer[txOffset], numWords, 25000);
					printf("Test %d: words recv: %d (Address %p) \n", j, recvd, &recvBuffer[txOffset]);
				}

				GET_TIME_VAL(2);

				// Check the data
				if (recvd != 0) {
					for (i = 4; i < recvd; i++) {
						if (recvBuffer[i + txOffset] != sendBuffer[i]) {
							printf("recvBuffer[%d]: %d, expected %d\n", i, recvBuffer[i + txOffset], sendBuffer[i]);
							failure = 1;
						}
					}

					if (j > 0)
						printf("send bw: %f\n",
							sent*4.0/1000/1000/((TIME_VAL_TO_MS(1) - TIME_VAL_TO_MS(0))/1000.0)); //,

					if (j > 0)
						printf("recv bw: %f\n",
							recvd*4.0/1000/1000/((TIME_VAL_TO_MS(2) - TIME_VAL_TO_MS(1))/1000.0)); //,
					if(failure) 
						return;

				}
			}
		}
		// Done with device
	        fpga_close(fpga);
	}
	else if (option == 5) { // Send data, receive data
		if (argc < 7) {
			printf("Usage: %s %d <fpga id> <chnl> <offset> <num words to transfer> <number of iterations>\n", argv[0], option);
			return -1;
		}
		size_t offset;
		size_t numWords;
		unsigned int numIter;
		id = atoi(argv[2]);
		chnl = atoi(argv[3]);
		offset = atoi(argv[4]) % (4096 / sizeof(unsigned int));
		if(numWords < 4) {
			printf("Must transfer at least 4 words %d\n", id);
			return -1;
		}
		numWords = atoi(argv[5]);
		numIter = atoi(argv[6]);
		printf("Running single test with %zu words, from host-page offset %zu \n", numWords, offset);

		// Get the device with id
		fpga = fpga_open(id);
		if (fpga == NULL) {
			printf("Could not get FPGA %d\n", id);
			return -1;
		}

		// Malloc the arrays (page aligned) 
		printf("Asked for %zu bytes\n",((numWords*sizeof(unsigned int) + 4096)/4096)*4096 + 4096);
		err = posix_memalign((void **)&sendBuffer, 4096, ((numWords*sizeof(unsigned int)*2 + 4096)/4096)*4096 + 4096);
		if (err) {
			printf("Could not malloc memory for sendBuffer\n");
			fpga_close(fpga);
			return -1;
		}

		err = posix_memalign((void **)&recvBuffer, 4096, ((numWords*sizeof(unsigned int)*2 + 4096)/4096)*4096 + 4096);
		if (err) {
			printf("Could not malloc memory for recvBuffer\n");
			free(sendBuffer);
			fpga_close(fpga);
			return -1;
		}

		int j;
		for (j = 0; j < numIter; ++j) {
			for (i = 0; i < numWords; i++) {
				sendBuffer[i + offset] = i+1;
				recvBuffer[i] = 0;
			}

			GET_TIME_VAL(0);

			// Send the data
			sent = fpga_send(fpga, chnl, &sendBuffer[offset], numWords, 0, 1, 25000);
			printf("words sent: %d\n", sent);

			GET_TIME_VAL(1);

			if (sent != 0) {
				// Recv the data
				recvd = fpga_recv(fpga, chnl, recvBuffer, numWords, 25000);
				printf("words recv: %d\n", recvd);
			}

			GET_TIME_VAL(2);

			// Check the data
			if (recvd != 0) {
				for (i = 4; i < recvd; i++) {
					if (recvBuffer[i] != sendBuffer[i + offset]) {
						printf("recvBuffer[%d]: %d, expected %d\n", i, recvBuffer[i], sendBuffer[i + offset]);
						failure = 1;
					}
				}

				printf("send bw: %f\n",
					sent*4.0/1000/1000/((TIME_VAL_TO_MS(1) - TIME_VAL_TO_MS(0))/1000.0)); //,

				printf("recv bw: %f\n",
					recvd*4.0/1000/1000/((TIME_VAL_TO_MS(2) - TIME_VAL_TO_MS(1))/1000.0)); //,
				if(failure) 
					return;

			}
		}
		// Done with device
	        fpga_close(fpga);
	}
	else if (option == 6) { // Send data, receive data
		if (argc < 7) {
			printf("Usage: %s %d <fpga id> <chnl> <offset> <num words to transfer> <number of iterations>\n", argv[0], option);
			return -1;
		}
		size_t offset;
		size_t numWords;
		unsigned int numIter;

		id = atoi(argv[2]);
		chnl = atoi(argv[3]);
		offset = atoi(argv[4]) % (4096 / sizeof(unsigned int));
		if(numWords < 4) {
			printf("Must transfer at least 4 words %d\n", id);
			return -1;
		}
		numWords = atoi(argv[5]);
		numIter = atoi(argv[6]);
		printf("Running single test with %zu words, to host-page offset %zu \n", numWords, offset);

		// Get the device with id
		fpga = fpga_open(id);
		if (fpga == NULL) {
			printf("Could not get FPGA %d\n", id);
			return -1;
		}

		// Malloc the arrays (page aligned) 
		printf("Asked for %zu bytes\n",((numWords*sizeof(unsigned int) + 4096)/4096)*4096 + 4096);
		err = posix_memalign((void **)&sendBuffer, 4096, ((numWords*sizeof(unsigned int)*2 + 4096)/4096)*4096 + 4096);
		if (err) {
			printf("Could not malloc memory for sendBuffer\n");
			fpga_close(fpga);
			return -1;
		}

		err = posix_memalign((void **)&recvBuffer, 4096, ((numWords*sizeof(unsigned int)*2 + 4096)/4096)*4096 + 4096);
		if (err) {
			printf("Could not malloc memory for recvBuffer\n");
			free(sendBuffer);
			fpga_close(fpga);
			return -1;
		}

		int j;
		for (j = 0; j < numIter; ++j) {
			for (i = 0; i < numWords; i++) {
				sendBuffer[i] = i+1;
				recvBuffer[i + offset] = 0;
			}

			GET_TIME_VAL(0);

			// Send the data
			sent = fpga_send(fpga, chnl, sendBuffer, numWords, 0, 1, 25000);
			printf("test %d: words sent: %d\n", j, sent);

			GET_TIME_VAL(1);

			if (sent != 0) {
				// Recv the data
				recvd = fpga_recv(fpga, chnl, &recvBuffer[offset], numWords, 25000);
				printf("test %d: words recv: %d (Address %p %p)\n", j, recvd, &recvBuffer[offset], &recvBuffer[offset+numWords]);
			}

			GET_TIME_VAL(2);

			// Check the data
			if (recvd != 0) {
				for (i = 4; i < recvd; i++) {
					if (recvBuffer[i + offset] != sendBuffer[i]) {
						printf("recvBuffer[%d]: %d, expected %d\n", i, recvBuffer[i + offset], sendBuffer[i]);
						failure = 1;
					}
				}

				printf("send bw: %f\n",
					sent*4.0/1000/1000/((TIME_VAL_TO_MS(1) - TIME_VAL_TO_MS(0))/1000.0)); //,

				printf("recv bw: %f\n",
					recvd*4.0/1000/1000/((TIME_VAL_TO_MS(2) - TIME_VAL_TO_MS(1))/1000.0)); //,
				if(failure) 
					return;

			}
		}
		// Done with device
	        fpga_close(fpga);
	}
	return 0;
}
