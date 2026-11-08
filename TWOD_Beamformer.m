colSp = 0.5*wavelength;
rowSp = 0.4*wavelength;
ura = phased.URA('Size',[10 5],'ElementSpacing',[rowSp colSp]);
ura.Element.FrequencyRange = [90e5 110e6];

x = collectPlaneWave(ura,s,inputAngle,carrierFreq);
noise = sqrt(noisePwr/2)*(randn(rs,size(x))+1i*randn(rs,size(x)));

s1 = 10*randn(rs,nSamp,1);
s2 = 10*randn(rs,nSamp,1);

%interference at [30; 10] and at [50; -5]
interference = collectPlaneWave(ura,[s1 s2],[30 50; 10 -5],carrierFreq);
rxInt = interference + noise;                 % total interference + noise
rxSignal = x + rxInt;                % total received signal


mvdrbeamformer = phased.MVDRBeamformer('SensorArray',ura,...
    'Direction',inputAngle,'OperatingFrequency',carrierFreq,...
    'TrainingInputPort',true,'WeightsOutputPort',true);

[yURA,w]= mvdrbeamformer(rxSignal,rxInt);

plot(t,abs(yURA)); axis tight;
title('Output of MVDR Beamformer for URA');
xlabel('Time (s)');ylabel('Magnitude (V)');

subplot(2,1,1);
pattern(ura,carrierFreq,-180:180,-5,'Weights',w,'Type','powerdb',...
    'PropagationSpeed',physconst('LightSpeed'),'Normalize',false,...
    'CoordinateSystem','rectangular');
title('Response Pattern at -5 Degrees Elevation');
axis([-90 90 -60 -5]);
subplot(2,1,2);
pattern(ura,carrierFreq,-180:180,10,'Weights',w,'Type','powerdb',...
    'PropagationSpeed',physconst('LightSpeed'),'Normalize',false,...
    'CoordinateSystem','rectangular');
title('Response Pattern at 10 Degrees Elevation');
axis([-90 90 -60 -5]);
