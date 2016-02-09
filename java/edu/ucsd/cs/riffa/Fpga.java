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

/*
 * Filename: Fpga.java
 * Version: 2.0
 * Description: Java API for RIFFA.
 * Author: Matthew Jacobsen
 * History: @mattj: Initial release. Version 2.0.
 */

/**
 * Represents a FPGA accessible by RIFFA. The usage pattern is:
 *    Fpga f = Fpga.open(...);
 *    f.send(...);
 *    f.recv(...);
 *    f.close(...);
 *
 * The static method list can be used to get a listing of the FPGAs installed in
 * the system and their ids. You'll need the FPGA id to pass to the open method.
 * If only 1 FPGA is installed in the system, it's id will always be 0.
 *
 * In the send and recv methods below use java.nio.ByteBuffer instead of a
 * byte[] to represent the data for sending and buffer for receiving data. The
 * java.nio.ByteBuffer class is used because the underlying byte[] can easily be
 * accessed. But more importantly, it has methods to reinterpret the underlying
 * byte[] into other primitive array types (e.g. int[]) without copying the
 * contents of the array. Copying arrays (especially large arrays) will reduce
 * throughput considerably and is best to be avoided. Use the allocateDirect
 * method on ByteBuffer to create a new instance. Then use one of the asXXXBufer
 * methods to acquire the appropriate java.nio.Buffer subclass. The example
 * below illustrates this:
 *    java.nio.ByteBuffer bb = java.nio.ByteBuffer.allocateDirect(NUM_INTS*4);
 *    java.nio.IntBuffer ib = bb.asIntBuffer();
 *    for (i = 0; i < NUM_INTS; i++)
 *        ib.put(i, ...)
 *    Fpga f = Fpga.open(0);
 *    fpga.send(0, bb, NUM_INTS, 0, true, 0L);
 */
package edu.ucsd.cs.riffa;

import java.nio.ByteBuffer;

public class Fpga {
	private Long ptr;

	private native int jFpgaList(FpgaInfo info);
	private native long jFpgaOpen(int id);
	private native void jFpgaClose(long fpga);
	private native int jFpgaSend(long fpga, int chnl, ByteBuffer data, int len, int destoff, int last, long timeout);
	private native int jFpgaRecv(long fpga, int chnl, ByteBuffer data, long timeout);
	private native void jFpgaReset(long fpga);

	/* Expects the JNI library to be libjriffa.so or libjriffa.dll */
	static {
		NativeLibLoader.loadLibrary("jriffa");
	}

	/**
	 * Default constructor.
	 */
	private Fpga() {
		ptr = null;
	}

	/**
	 * Populates and returns a FpgaInfo object with all FPGAs registered in the
	 * system. Returns a FpgaInfo object on success. Returns null on error.
	 *
	 * @returns A FpgaInfo object on success or null.
	 */
	public static FpgaInfo list() {
		FpgaInfo info = new FpgaInfo();
		Fpga fpga = new Fpga();
		if (fpga.jFpgaList(info) == 0)
			return info;
		else
			return null;
	}

	/**
	 * Initializes the FPGA specified by id. On success, returns a Fpga object.
	 * On error, returns null. Each FPGA must be opened before any channels can
	 * be accessed. Once opened, any number of threads can use the Fpga object.
	 *
 	 * @param id - Identifier for the FPGA (in single FPGA installations, this
	 * is always 0).
	 * @returns Fpga object or null.
	 */
	public static Fpga open(int id) {
		Fpga fpga = new Fpga();
		fpga.ptr = new Long(fpga.jFpgaOpen(id));
		return fpga;
	}

	/**
	 * Cleans up memory/resources for the FPGA represented by this instance.
	 */
	public void close() {
		this.jFpgaClose(this.ptr.longValue());
		ptr = null;
	}

	/**
	 * Sends len words (4 byte words) from data to FPGA channel chnl on the FPGA
	 * represented by this Fpga object. The FPGA channel will be sent len,
	 * destoff, and last. The value of destoff is used to support sending data
	 * across multiple send transactions. Note that only the low 31 bits of
	 * offset are sent. If last is true the channel should interpret the end of
	 * this send as the end of a transaction. If last is false, the channel
	 * should wait for additional sends before the end of the transaction. If
	 * timeout is non-zero, this call will send data and wait up to timeout ms
	 * for the FPGA to respond (between packets) before timing out. If timeout
	 * is zero, this call may block indefinitely. Multiple threads sending on
	 * the same channel may result in corrupt data or error. This function is
	 * thread safe across channels. Returns the number of words sent.
	 *
 	 * @param chnl - Channel number over which to communicate.
 	 * @param data - java.nio.ByteBuffer holding the byte[] send. Note that the
	 * data transfer unit is a 32 bit word.
 	 * @param len - Length of data to send, in (32 bit) words. Thus a value of 4
	 * means send 16 bytes.
 	 * @param destoff - Value sent to FPGA core to indicate where to start
	 * writing this data. Only the least significant 31 bits are sent (not all
	 * 32).
 	 * @param last - If true, this transfer is the last in a sequence of
	 * transfers. If false, this transfer is not the last in a sequence of
	 * transfers(more transfers to come).
 	 * @param timeout - Timeout value in ms. If 0, no timeout is specified.
	 * Otherwise, the PC will wait up to timeout ms in between PC/FPGA
	 * communications.
	 * @returns The number of words sent.
	 */
	public int send(int chnl, ByteBuffer data, int len, int destoff, boolean last, long timeout) {
		int s = this.jFpgaSend(this.ptr.longValue(), chnl, data, len, destoff, (last ? 1 : 0), timeout);
		if (s == -1)
			throw new RuntimeException("Unble to access ByteBuffer direct memory.");
		return s;
	}

	/**
	 * Receives data from the FPGA channel chnl to the java.nio.ByteBuffer
	 * object, on the FPGA represented by this Fpga object. The FPGA channel can
	 * send any amount of data, so the java.nio.ByteBuffer should be large
	 * enough to accommodate. The FPGA will specify an offset value which will
	 * determine where received data will start being written. If the amount of
	 * data plus offset exceed the size of the data array, then the additional
	 * data will be discarded. If timeout is non-zero, this call will wait up to
	 * timeout ms for the FPGA to respond (between packets) before timing out.
	 * If timeout is zero, this call may block indefinitely. Multiple threads
	 * receiving on the same channel may result in corrupt data or error. This
	 * function is thread safe across channels. Returns the number of words
	 * written to the java.nio.ByteBuffer.
	 *
 	 * @param chnl - Channel number over which to communicate.
 	 * @param data - java.nio.ByteBuffer into which received data will be
	 * written.
 	 * @param timeout - Timeout value in ms. If 0, no timeout is specified.
	 * Otherwise, the PC will wait up to timeout ms in between PC/FPGA
	 * communications.
	 * @returns The number of words written to the java.nio.ByteBuffer.
	 */
	public int recv(int chnl, ByteBuffer data, long timeout) {
		int r = this.jFpgaRecv(this.ptr.longValue(), chnl, data, timeout);
		if (r == -1)
			throw new RuntimeException("Unble to access ByteBuffer direct memory.");
		return r;
	}

	/**
	 * Resets the state of the FPGA and all channels. This is meant to be used
	 * as an alternative to rebooting if an error occurs while sending or
	 * receiving. Calling this function while other threads are sending or
	 * receiving will result in unexpected behavior.
	 */
	public void reset() {
		this.jFpgaReset(this.ptr.longValue());
	}
}

