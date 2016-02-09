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
 * Filename: FpgaInfo.java
 * Version: 2.0
 * Description: Java API for RIFFA.
 * Author: Matthew Jacobsen
 * History: @mattj: Initial release. Version 2.0.
 */

/**
 * Value object to hold information about all the installed FPGA accessible by 
 * RIFFA.
 */
package edu.ucsd.cs.riffa;

public class FpgaInfo {
	private static final int NUM_FPGAS = 5;
	private int numFpgas;
	private int[] numChannels;
	private int[] id;
	private int[] vendorId;
	private int[] deviceId;
	private String[] name;

	/**
	 * Default constructor.
	 */
	public FpgaInfo() {
		this.numFpgas = 0;
		this.name = new String[NUM_FPGAS];
		this.id = new int[NUM_FPGAS];
		this.numChannels = new int[NUM_FPGAS];
		this.deviceId = new int[NUM_FPGAS];
		this.vendorId = new int[NUM_FPGAS];
	}
	
	/**
	 * Returns the number of RIFFA accessible FPGAs installed in the system.
	 *
	 * @returns Number of RIFFA accessible FPGAs installed in the system.
	 */
	public int getNumFpgas() {
		return this.numFpgas;
	}
	
	/**
	 * Sets the number of RIFFA accessible FPGAs installed in the system.
	 *
	 * @param val - Number of RIFFA accessible FPGAs installed in the system.
	 */
	public void setNumFpgas(int val) {
		this.numFpgas = val;		
	}

	/**
	 * Returns the number of RIFFA channels configured on the FPGA at position 
	 * pos.
	 *
	 * @returns Number of RIFFA channels configured on the FPGA at position pos.
	 */
	public int getNumChannels(int pos) {
		return this.numChannels[pos];
	}
	
	/**
	 * Sets the number of RIFFA channels configured on the FPGA at position pos.
	 *
	 * @param pos - Position of FPGA.
	 * @param val - Number of RIFFA channels configured on the FPGA at position 
	 * pos.
	 */
	public void setNumChannels(int pos, int val) {
		this.numChannels[pos] = val;		
	}

	/**
	 * Returns the FPGA id at position pos. This id is used to open the FPGA on 
	 * the Fpga's open method.
	 *
	 * @returns FPGA id at position pos..
	 */
	public int getId(int pos) {
		return this.id[pos];
	}
	
	/**
	 * Sets the FPGA id at position pos.
	 *
	 * @param pos - Position of FPGA.
	 * @param val - FPGA id at position pos.
	 */
	public void setId(int pos, int val) {
		this.id[pos] = val;		
	}

	/**
	 * Returns the name of the FPGA at position pos. This is typically the PCIe 
	 * bus and slot number.
	 *
	 * @returns Name of the FPGA at position pos.
	 */
	public String getName(int pos) {
		return this.name[pos];
	}
	
	/**
	 * Sets the name of the FPGA at position pos.
	 *
	 * @param pos - Position of FPGA.
	 * @param val - Name of the FPGA at position pos.
	 */
	public void setName(int pos, String val) {
		this.name[pos] = val;		
	}

	/**
	 * Returns the FPGA vendor id at position pos.
	 *
	 * @returns The FPGA vendor id at position pos.
	 */
	public int getVendorId(int pos) {
		return this.vendorId[pos];
	}
	
	/**
	 * Sets the FPGA vendor id at position pos.
	 *
	 * @param pos - Position of FPGA.
	 * @param val - The FPGA vendor id at position pos.
	 */
	public void setVendorId(int pos, int val) {
		this.vendorId[pos] = val;		
	}

	/**
	 * Returns the FPGA device id at position pos.
	 *
	 * @returns The FPGA device id at position pos.
	 */
	public int getDeviceId(int pos) {
		return this.deviceId[pos];
	}
	
	/**
	 * Sets the FPGA device id at position pos.
	 *
	 * @param pos - Position of FPGA.
	 * @param val - The FPGA device id at position pos.
	 */
	public void setDeviceId(int pos, int val) {
		this.deviceId[pos] = val;		
	}

	/**
	 * Returns a nicely formatted listing of all the RIFFA FPGAs detected.
	 * 
	 * @returns a nicely formatted String of all the RIFFA FPGAs detected.
	 */
	public String toString() {
		StringBuffer buffy = new StringBuffer();
		String eol = System.getProperty("line.separator");
		buffy.append("num fpgas: " + this.numFpgas + eol);
		for (int i=0; i < this.numFpgas; i++) {
			buffy.append("id: " + this.id[i] + eol);
			buffy.append("name: " + this.name[i] + eol);
			buffy.append("num channels: " + this.numChannels[i] + eol);
			buffy.append("vendor id: " + Integer.toHexString(this.vendorId[i]) + eol);
			buffy.append("device id: " + Integer.toHexString(this.deviceId[i]) + eol);
		}
		return buffy.toString();
	}
}
