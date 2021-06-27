function [dout]=loadadc5ch(filnam)
    % load  5-channel analog logger data file
    % file is structure is:
    %  32-bit unsigned integer unix seconds
    %  16-bit unsigned integer spare (used for microseconds between samples)
    %  5 x 16-bit unsigned integer ADC counts
    %  2/5/21  MJB
    
    if nargin == 0
        [raw_name,temp,filterindex]=uigetfile('*.dat','Load ADC 5-channel Logger File');
        filnam=[temp raw_name];
    else 
        filnam = [filnam];  % handy for functions that scan a directory
    end
    
    disp(sprintf('Reading data from %s',raw_name));
    fid = fopen(filnam,'r');
    fseek(fid,0,'eof'); % move to end of file
    pos2 = ftell(fid); % pos2 is overall length of file
    frewind(fid); % move back to beginning of file
    
    %  each record is 16 bytes as described above
    
    nrecords=floor((pos2)/16); % number of records
    disp(sprintf('%d records of data in the file',nrecords));
    
    
    %read unix seconds
    fseek(fid,0,'bof');
    dout.secs = fread(fid,nrecords,'uint32',12); % skip  12 bytes
    
    fseek(fid,4,'bof'); % rewind to beginning of spare field
    % next read spare
    dout.spare = fread(fid,nrecords,'uint16',14); % skip  14
    
    fseek(fid,6,'bof'); % start at beginning of CH0
    dout.ch0 = fread(fid,nrecords,'uint16',14);
    
    fseek(fid,8,'bof'); % start at beginning of CH1
    dout.ch1 = fread(fid,nrecords,'uint16',14);
    
    fseek(fid,10,'bof'); % start at beginning of CH2
    dout.ch2 = fread(fid,nrecords,'uint16',14);
    
    fseek(fid,12,'bof'); % start at beginning of CH3
    dout.ch3 = fread(fid,nrecords,'uint16',14);
    
    fseek(fid,14,'bof'); % start at beginning of CH4
    dout.ch4 = fread(fid,nrecords,'uint16',14);
    
    % Now convert to uncalibrated volts
    adcref = 3.32;   % assumed value of adc reference
    % in 12-bit mode max output count is 4095
    dout.ch0  = dout.ch0 * (adcref/4095.0); % for testing, 2.5V precision Ref.
    dout.ch1  = dout.ch1 * (adcref/4095.0); % ch0 divided down to ~1.0V
    dout.ch2  = dout.ch2 * (adcref/4095.0); % Alkaline AA cell
    dout.ch3  = dout.ch3 * (adcref/4095.0); % sine wave ~100Hz
    dout.ch4  = dout.ch4 * (adcref/4095.0); % sine wave ~100Hz
    fclose(fid);
    plot(dout.ch0); % check noise in 2.5V reference sample
    
    % now we determine the sample rate by seeing how many samples 
    % occur between changes in the seconds value
    tp = diff(dout.secs);  %  should be zero or one
    tidx = find(tp); % get the indices of non-zero values in tp
    samprate  = mean(diff(tidx)); %
    disp(sprintf('Recovered sample rate is %6.2f samples per second',samprate));
    dout.samprate = samprate;
    end