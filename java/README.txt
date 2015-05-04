To use the Java bindings simply include the riffa.jar in your classpath while
compiling and running. Everything has already been included in this jar.

To compile the sample application:

javac -cp riffa.jar SampleApp.java


To run the sample sample application:

java -cp riffa.jar;. SampleApp

or for Linux:

java -cp riffa.jar:. SampleApp


Note that the riffa driver and C/C++ library must already be installed on the 
system.

