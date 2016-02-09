% ----------------------------------------------------------------------
% Copyright (c) 2016, The Regents of the University of California All
% rights reserved.
% 
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are
% met:
% 
%     * Redistributions of source code must retain the above copyright
%       notice, this list of conditions and the following disclaimer.
% 
%     * Redistributions in binary form must reproduce the above
%       copyright notice, this list of conditions and the following
%       disclaimer in the documentation and/or other materials provided
%       with the distribution.
% 
%     * Neither the name of The Regents of the University of California
%       nor the names of its contributors may be used to endorse or
%       promote products derived from this software without specific
%       prior written permission.
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
% "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
% LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
% A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL REGENTS OF THE
% UNIVERSITY OF CALIFORNIA BE LIABLE FOR ANY DIRECT, INDIRECT,
% INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
% BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
% OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
% ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
% TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
% USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
% DAMAGE.
% ----------------------------------------------------------------------
classdef Riffa < handle
    % Riffa RIFFA 2.0 interface for Matlab R2008a or greater.

    properties (Hidden)
        fpga_t;
        libname;		
    end

    properties (SetAccess = private)
        id;
        closed; %If this value is non-zero then the FPGA is closed
    end
    

    methods 
        function obj = Riffa(id_num)
            % Constructs a Riffa instance for FPGA with id id_num.
            if (nargin ~= 1)
                error('You must specify the FPGA id number.');
            end
            
			obj.libname = Riffa.load_lib();
            obj.fpga_t = calllib(obj.libname,'fpga_open',id_num);
            if isNull(obj.fpga_t)
                obj.closed = -1;
                error('While attempting to access FPGA.')
			end
            obj.id = id_num;
            obj.closed = 0;
        end
        
        
        function close(obj)
            % Closes the Riffa instance. Once closed, this instance is no longer
			% usable. 
            if (obj.closed ~= 0)
                error('This Riffa instance is closed.');
            end
            if isNull(obj.fpga_t)
                obj.closed = -1;
                error('Riffa FPGA handle is NULL!')
            end
            calllib(obj.libname,'fpga_close',obj.fpga_t);
            obj.closed = 1;
        end
        
        
        function sent = send(obj, chnl, data, length, offset, last, timeout)
			% Sends length words (4 byte words) from data to FPGA channel chnl.  
			% The FPGA channel will be sent length, destoff, and last. If last 
			% is 1, the channel should interpret the end of this send as the end
			% of a transaction. If last is 0, the channel should wait for 
			% additional sends before the end of the transaction. If timeout is 
			% non-zero, this call will send data and wait up to timeout ms for 
			% the FPGA to respond (between packets) before timing out. If
			% timeout is zero, this call may block indefinitely. Returns the 
			% number of words sent. 
            if obj.closed ~= 0
                error('This Riffa instance is closed.');
            end
            if isNull(obj.fpga_t)
                obj.closed = -1;
                error('Riffa FPGA handle is NULL!')
            end
            if nargin ~= 7
                error('Usage: words_sent = send(chnl, data, length, offset, last, timeout)')
            end
            if length < 0
                error('Length (arg 3) must be non-negative');
            end
            if offset < 0
                error('Offset (arg 4) must be non-negative');
            end
            if timeout < 0
                error('Timeout (arg 6) must be non-negative');
            end
			fpgaptr = libpointer('fpga_tPtr', obj.fpga_t);
            sent = calllib(obj.libname, 'fpga_send', fpgaptr, int32(chnl), ...
				data, int32(length), int32(offset), int32(last), int64(timeout));
        end
        
                
        function [recvd, data] = recv(obj, chnl, length, timeout)
			% Receives data from the FPGA channel chnl and returns it in the 
			% data variable. The FPGA channel can send any amount of data, but
			% only length words will be returned. The FPGA channel will specify 
			% an offset which will determine where in the returned data object
			% the data will start being written. If the amount of data (plus 
			% offset) exceed the length specified, then additional data will be 
			% discarded. If timeout is non-zero, this call will wait up to 
			% timeout ms for the FPGA to respond (between packets) before 
			% timing out. If timeout is zero, this call may block indefinitely. 
			% Returns the number of words received to the data variable. 
            if obj.closed ~= 0
                error('This Riffa instance is closed.');
            end
            if isNull(obj.fpga_t)
                obj.closed = -1;
                error('Riffa FPGA handle is NULL!')
            end
            if nargin ~= 4 
                error('Usage: words_recvd = recv(chnl, length, timeout)')
            end
            if length < 0
                error('Length (arg 3) must be positive');
            end
            if timeout < 0
                error('Timeout (arg 4) must be positive');
            end
			dataptr = libpointer('voidPtr', int32(zeros(1, length)));
			fpgaptr = libpointer('fpga_tPtr', obj.fpga_t);
            recvd = calllib(obj.libname, 'fpga_recv', fpgaptr, int32(chnl), ...
				dataptr, int32(length), int64(timeout));
			recvd = min(recvd, length);
			data = dataptr.Value;
			data = data(1:recvd);
        end
        
        
        function reset(obj)
			% Resets all FPGA channels connected to this Riffa instance.
            if obj.closed ~= 0
                error('This Riffa instance is closed.');
            end
            if isNull(obj.fpga_t)
                obj.closed = -1;
                error('Riffa FPGA handle is NULL!')
            end
            calllib(obj.libname,'fpga_reset',obj.fpga_t);
            
        end
    end 

    
    methods (Static) 
        function fpga_info_list = list()
			% Returns a fpga_info_list with all FPGAs registered in the system.
			libname = Riffa.load_lib();
            mstruct_list = struct('num_fpgas', 0,...
                'id', [0,0,0,0,0],...
                'num_chnls', [0,0,0,0,0],...
                'name', int8(zeros(1,80)),...
                'vendor_id',[0,0,0,0,0],...
                'device_id',[0,0,0,0,0]);
            
            cstruct_list = libstruct('fpga_info_list',mstruct_list);
            rc = calllib(libname,'fpga_list',cstruct_list);
            fpga_info_list = get(cstruct_list);
            if rc ~= 0
                fpga_info_list.num_fpgas = 0;
                error('Error populating the fpga_info_list struct.');
            end

            for i=1:fpga_info_list.num_fpgas
                fpga_info_list.names{i} = char(fpga_info_list.name(((i-1)*16)+1:(i*16)));
            end
            
			fpga_info_list.id = fpga_info_list.id(1:fpga_info_list.num_fpgas);
			fpga_info_list.num_chnls = fpga_info_list.num_chnls(1:fpga_info_list.num_fpgas);
			fpga_info_list.vendor_id = fpga_info_list.vendor_id(1:fpga_info_list.num_fpgas);
			fpga_info_list.device_id = fpga_info_list.device_id(1:fpga_info_list.num_fpgas);
        end
    end


    methods (Static,Hidden,Access=private) 
        function libname = load_lib()
			% Loads the system library and returns the library name.
            if ispc == 1
                %disp('Running Windows platform');
                sharedlibrary = 'C://Windows/System32/riffa.dll';
				header = 'riffa.h';
				libname = 'riffa';
            elseif isunix == 1
                %disp ('Running Unix platform');
                sharedlibrary = '/usr/local/lib/libriffa.so';
				header = '/usr/local/include/riffa.h';
				libname = 'libriffa';
            else
                error('Not running a compatible OS.')
            end              
                        
            if not (libisloaded(libname))
                protoname = 'riffa_proto';
                protoext = '.m';
                if exist(strcat(protoname, protoext), 'file') ~= 2
                    disp('Creating initial RIFFA library prototype for MATLAB.');
                    loadlibrary(sharedlibrary, header, 'mfilename', protoname);
                    buf = {};
                    d = 0;
                    searchexp = '''name''\s*,\s*''int8#''';
                    replaceexp = '''name'', ''int8#80''';
                    fd = fopen(strcat(protoname, protoext), 'r');
                    while ~feof(fd)
                       s = fgetl(fd);
                       r = regexprep(s, searchexp, replaceexp);
                       buf = vertcat(buf, r);
                       d = or(d, ~isequal(r, s));                       
                    end
                    fclose(fd);
                    if d == 1
                        fd = fopen(strcat(protoname, protoext), 'w+');
                        for i=1:size(buf, 1)
                           fprintf(fd,'%s\n', buf{i});
                        end
                        fclose(fd);
                        unloadlibrary(libname);
                        disp('RIFFA library prototype created.');
                        disp('This version of MATLAB has a bug that requires ');
                        disp('you exit and restart MATLAB before continuing.');
                        disp('You should only have to do this once, ever.');
                        error('Exit MATLAB and restart please.');
                    end
                end
                disp('Loading RIFFA library.')
                loadlibrary(sharedlibrary, @riffa_proto);
            end
    	end
	end
    
end

