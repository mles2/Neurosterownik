function signal=siggen(t,sqr_amp,sqr_frq,sin_amp,sin_frq,dc,Nlevel,stp_lvl,stp_t)
%
% Signal generator
%
% The function combines square signals with sine wawes and
% adds a DC-level and a step function.
%
% INPUT:    t       - The time in seconds
%           sqr_amp - Amplitude of square signals (row-vector)
%           sqr_frq - Frequency of square signals (column-vector)
%           sin_amp - Amplitude of sine signals (row-vector)
%           sin_frq - Frequency of sine signals (column-vector)
%           dc      - [DC off-set level] (optional)
%           stp_lvl - Step level
%           stp_t   - Step time
% OUTPUT:   signal  - The combined signal
%
if nargin<=7, stp_lvl=0.0; stp_t=0.0; end;
if nargin<=6, Nlevel = 0.0;end
if nargin<=5, dc=0.0; end; 
if nargin<=4, sin_amp=0.0; sin_frq=0.0; end;

signal=0.0;
if t>=stp_t, signal=stp_lvl; end;
signal=signal + sqr_amp*sign(sin(2*pi*sqr_frq*t));
signal=signal + sin_amp*sin(2*pi*sin_frq*t) + dc + Nlevel*randn;

