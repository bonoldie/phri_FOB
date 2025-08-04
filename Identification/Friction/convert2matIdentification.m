clear
close all
matpath = 'DataLoad/';
filepath = 'CsvLoad/'

files = dir([filepath '**.csv']);
filelist = files;



for i=1:length(filelist)
filelist(i).name
fid = fopen([filepath filelist(i).name]); 
DATA = textscan(fid,'%f %f %f %f %f %f %f %f %f %f');

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

D = cell2struct(DATA(1,10),'A',1);
Torque = struct2array(D);

N = 5;

figure(1)
subplot(N,1,1)
plot(TimeSec,ActiveCurrentA,TimeSec,CurrentCommand )
legend('active','command')
subplot(N,1,2)
plot(TimeSec,Position)
subplot(N,1,3)
% plot(TimeSec,Velocity,TimeSec,pVelocity)
plot(TimeSec,pVelocity)
subplot(N,1,4)
plot(TimeSec,AuxiliaryPosition)
subplot(N,1,5)
plot(TimeSec,AuxiliaryVelocity,TimeSec,pAuxiliaryVelocity)
plot(TimeSec,pAuxiliaryVelocity)
% subplot(N,1,6)
% plot(TimeSec,Torque,TimeSec,Position-AuxiliaryPosition)
% legend('torque','displacement')


pause
eval(['save ' matpath '' strrep(filelist(i).name,'csv','mat') ' ActiveCurrentA CurrentCommand Position Velocity pVelocity AuxiliaryPosition AuxiliaryVelocity pAuxiliaryVelocity Torque TimeSec']);

fclose(fid);
end


