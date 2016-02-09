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
 * Filename: NativeLibLoader.java
 * Version: 2.0
 * Description: Java API for RIFFA.
 * Author: Matthew Jacobsen
 * History: @mattj: Initial release. Version 2.0.
 */

/**
 * Utility to load the JNI RIFFA library.
 */
package edu.ucsd.cs.riffa;

import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.URL;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Enumeration;
import java.util.List;
import java.util.jar.JarEntry;
import java.util.jar.JarFile;

public class NativeLibLoader {

	/**
	 * Default constructor.
	 */
	private NativeLibLoader() { }

	/**
	 * Returns a String representing the platform. Format: ${os.arch}/${os.name}
	 *
	 * @returns A String representing the platform.
	 */
	public static String getCurrentPlatformIdentifier() {
		String osName = System.getProperty("os.name");
		if (osName.toLowerCase().indexOf("windows") > -1) {
			osName = "Windows";
		}
		return System.getProperty("os.arch") + "/" + osName;
	}

	/**
	 * Loads a library from the same ClassLoader as this class at the path:
	 * /native/${os.arch}/${os.path}/lib[libName].[so|dll]
	 * If no such path is found, attemts to load the library by name using the
	 * System.loadLibrary function which searches the java.library.path system
	 * property.
	 *
	 * @param libName - Library name, without the lib prefix or any suffix.
	 */
	public static void loadLibrary(String libName) {
		boolean usingEmbedded = false;
		// Attempt to locate embedded native library within JAR at:
		// /native/${os.arch}/${os.name}/lib[libName].[so|dll]
		String[] exts = new String[] { "so", "dll" };
		StringBuffer url = new StringBuffer();
		url.append("/native/");
		url.append(getCurrentPlatformIdentifier());
		url.append("/lib");
		url.append(libName);
		url.append(".");
		URL nativeLibraryUrl = null;
		// Loop through extensions, stopping after finding first one
		for (int i=0; i < exts.length; i++) {
			nativeLibraryUrl = NativeLibLoader.class.getResource(url.toString() + exts[i]);
			if (nativeLibraryUrl != null)
				break;
		}

		if (nativeLibraryUrl != null) {
			// Native library found within JAR, extract and load
			try {
				final File libfile = File.createTempFile(libName + "-", ".lib");
				libfile.deleteOnExit(); // just in case

				final InputStream in = nativeLibraryUrl.openStream();
				final OutputStream out = new BufferedOutputStream(new FileOutputStream(libfile));

				int len = 0;
				byte[] buffer = new byte[8192];
				while ((len = in.read(buffer)) > -1)
					out.write(buffer, 0, len);
				out.close();
				in.close();

				System.load(libfile.getAbsolutePath());

				libfile.delete();

				usingEmbedded = true;

			} catch (IOException x) {
				// mission failed, do nothing
			}
		}

		// If not loaded, try loading using the java.library.path
		if (!usingEmbedded)
			System.loadLibrary(libName);
	}
}
