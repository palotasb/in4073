    clf
    clear,clc

    fs = 100;              % frec. sampling (Hz)
    Z = 50;                 % impedancia (Ohmios)
    T = 1/fs;               % tiempo de muestreo: inversa de fs
 
%	Load file with data sensors

    load file;
    
%	Graphic with Time and Frequency Signals from "file"

    figure(1)
    time_dep(file,T,Z,'SINUSOIDE','log'); 
 
%	Butterworth filter 7-Order

    fc = 5;% Max frequency according to Nyquist fc < fs/2 (49Hz max.in this case)
    [b,a] = butter(7,fc/(fs/2));
    figure(2)
    freqz (b,a); %Frequency response of digital filter
    
 % (8) Filter "file"
 
    file_filtered = filter(b,a,file);
    figure(3)
    time_dep(file_filtered,T,Z,'SINUSOIDE','log');
    
% Fin