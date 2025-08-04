clear
close all
matpath = 'Data\';
%filepath = '../../../SeaControl/';
filepath = 'Csv\'

files = dir([filepath '*.csv']);
filelist = files;

N = 6;%time + 5 var
%N = 7;%time + 6 var

for i=1:length(filelist)
filelist(i).name
fid = fopen([filepath filelist(i).name]); 
DATA = textscan(fid,'%f %f %f %f %f %f %f %f %f');

D = cell2struct(DATA(1,1),'A',1);
TimeSec = struct2array(D);

D = cell2struct(DATA(1,2),'A',1);
ActiveCurrentA = struct2array(D);

D = cell2struct(DATA(1,3),'A',1);
CurrentCommand = struct2array(D);

D = cell2struct(DATA(1,4),'A',1);
Position = struct2array(D);

D = cell2struct(DATA(1,5),'A',1);
Velocity = struct2array(D);

D = cell2struct(DATA(1,6),'A',1);
pVelocity = struct2array(D);

D = cell2struct(DATA(1,7),'A',1);
AuxiliaryPosition = struct2array(D);

D = cell2struct(DATA(1,8),'A',1);
AuxiliaryVelocity = struct2array(D);

D = cell2struct(DATA(1,9),'A',1);
pAuxiliaryVelocity = struct2array(D);


figure(1)
subplot(N,1,1)
plot(TimeSec,ActiveCurrentA)
subplot(N,1,2)
plot(TimeSec,CurrentCommand)
subplot(N,1,3)
plot(TimeSec,Position)
subplot(N,1,4)
plot(TimeSec,Velocity,TimeSec,pVelocity)
subplot(N,1,5)
plot(TimeSec,AuxiliaryPosition)
subplot(N,1,6)
plot(TimeSec,AuxiliaryVelocity,TimeSec,pAuxiliaryVelocity)

pause
eval(['save ' matpath '' strrep(filelist(i).name,'csv','mat') ' ActiveCurrentA CurrentCommand Position Velocity pVelocity AuxiliaryPosition AuxiliaryVelocity pAuxiliaryVelocity TimeSec']);

fclose(fid);
end



