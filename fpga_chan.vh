//	Parameter register addresses
localparam PAR_SMASK = 0;  // summ mask
localparam PAR_TMASK = 1;  // trigger mask
localparam PAR_STMASK = 2; // selftrigger mask
localparam PAR_STHR = 3;	// summ send threshold (for the summ of 16 channels)
localparam PAR_ZTHR = 4; 	// trigger send threshold - master trigger zero suppression
localparam PAR_STTHR = 5; 	// self trigger threshold
localparam PAR_MTTHR = 6; 	// 64-channels sum trigger threshold (main trigger)
localparam PAR_STPRC = 7;	// selftrigger prescale
localparam PAR_WINBEG = 8;	// master trigger window begin
localparam PAR_WINLEN = 9; // window length
localparam PAR_SWINBEG = 10; // self trigger window begin
localparam PAR_INVMASK = 11; // mask for waveform inversion
