This directory contains 3 python scripts for graphing, short term logging and long term logging. 
Functionality is further described in the scripts. They are highly non-optimised.
Neither of these will work if the com port is connected via putty/other due to how the port can only communicate with one software at a time.
Due to additional characters in the python output which is removed prior to printing /graphing, any printf statement from the MCU (several prints combined (ie in a loop) or one in total (separate line print)) should include carriage return and newline at the end in that order ("\r\n"), ie printf("Print Content\r\n"); or printf("Start: A, ");printf("B, ");printf("C, end\r\n");, to allow for consistent removal of resulting characters from start and end of output.


csv_logger_long.py
	Long term logging. Most reliable logger, but is slower than the burst logger. Recommeded for use for any logging with logging periods over a few seconds and intervals over ~50ms.

csv_logger_short_burst.py
	Short term logging. Less reliable logger, but potentially unreliable with computer faults. Potential for logging file not to be saved/finalised, and data lost, if fault with computer occurs.

grapher.py
	Constantly graphs arrays of flots or integers form the serial terminal.