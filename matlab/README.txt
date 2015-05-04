To use the Matlab bindings simply include the Riffa.m class in your Matlab
working directory.

Note that the riffa driver and C/C++ library must already be installed on the 
system.

On Windows platforms you'll need to have the riffa.h header file in the working
directory as well (as there is no default include directory on Windows).

The first time you load the RIFFA bindings, several auto-generated files will be
produced. This will require that Matlab has access to a C compiler. This is
often taken care of during Matlab install.
