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
 * Filename: jriffa.c
 * Version: 2.0
 * Description: Java JNI library for calling RIFFA.
 * Author: Matthew Jacobsen
 * History: @mattj: Initial release. Version 2.0.
 */

#include <limits.h>
#include <riffa.h>
#include "jriffa.h"
#include <stdio.h>

JNIEXPORT jint JNICALL Java_edu_ucsd_cs_riffa_Fpga_jFpgaList(JNIEnv *env, jobject obj, jobject info) {
	int r;
	int i;
	fpga_info_list list;
	jclass cls;
	jstring name;
	jmethodID setNumFpgas;
	jmethodID setName;
	jmethodID setId;
	jmethodID setNumChannels;
	jmethodID setVendorId;
	jmethodID setDeviceId;
	if ((r = fpga_list(&list)) == 0) {
		cls = (*env)->FindClass(env, "edu/ucsd/cs/riffa/FpgaInfo");
		setNumFpgas = (*env)->GetMethodID(env, cls, "setNumFpgas", "(I)V");
		(*env)->CallVoidMethod(env, info, setNumFpgas, list.num_fpgas);
		for (i = 0; i < list.num_fpgas; i++) {
			setName = (*env)->GetMethodID(env, cls, "setName", "(ILjava/lang/String;)V");
			name = (*env)->NewStringUTF(env, list.name[i]);
			(*env)->CallVoidMethod(env, info, setName, i, name);
			setNumChannels = (*env)->GetMethodID(env, cls, "setNumChannels", "(II)V");
			(*env)->CallVoidMethod(env, info, setNumChannels, i, list.num_chnls[i]);
			setId = (*env)->GetMethodID(env, cls, "setId", "(II)V");
			(*env)->CallVoidMethod(env, info, setId, i, list.id[i]);
			setVendorId = (*env)->GetMethodID(env, cls, "setVendorId", "(II)V");
			(*env)->CallVoidMethod(env, info, setVendorId, i, list.vendor_id[i]);
			setDeviceId = (*env)->GetMethodID(env, cls, "setDeviceId", "(II)V");
			(*env)->CallVoidMethod(env, info, setDeviceId, i, list.device_id[i]);
		}
	}
	return r;
}

JNIEXPORT jlong JNICALL Java_edu_ucsd_cs_riffa_Fpga_jFpgaOpen(JNIEnv *env, jobject obj, jint id) {
	fpga_t * fpga = fpga_open((int)id);
	return (jlong)fpga;
}

JNIEXPORT void JNICALL Java_edu_ucsd_cs_riffa_Fpga_jFpgaClose(JNIEnv *env, jobject obj, jlong fpga) {
	fpga_close((fpga_t *)fpga);
}

JNIEXPORT jint JNICALL Java_edu_ucsd_cs_riffa_Fpga_jFpgaSend(JNIEnv *env, jobject obj, jlong fpga,
	jint chnl, jobject buffer, jint len, jint destoff, jint last, jlong timeout) {
	jlong datalen;
	unsigned int ulen;
	unsigned int lgt;
	void * data;
	data = (*env)->GetDirectBufferAddress(env, buffer);
	datalen = (*env)->GetDirectBufferCapacity(env, buffer);
	if (data == NULL || datalen == -1)
		return -1; // Means cannot get access to ByteBuffer underlying pointer
	ulen = (unsigned int)len;
	lgt = (ulen > (datalen>>2) ? (datalen>>2) : ulen);
	return fpga_send((fpga_t *)fpga, chnl, data, lgt, destoff, last, timeout);
}

JNIEXPORT jint JNICALL Java_edu_ucsd_cs_riffa_Fpga_jFpgaRecv(JNIEnv *env, jobject obj, jlong fpga,
	jint chnl, jobject buffer, jlong timeout) {
	jlong datalen;
	unsigned int lgt;
	void * data;
	data = (*env)->GetDirectBufferAddress(env, buffer);
	datalen = (*env)->GetDirectBufferCapacity(env, buffer);
	if (data == NULL || datalen == -1)
		return -1; // Means cannot get access to ByteBuffer underlying pointer
	lgt = (datalen>>2);
	return fpga_recv((fpga_t *)fpga, chnl, data, lgt, timeout);
}

JNIEXPORT void JNICALL Java_edu_ucsd_cs_riffa_Fpga_jFpgaReset(JNIEnv *env, jobject obj, jlong fpga) {
	fpga_reset((fpga_t *)fpga);
}

