psbeamformer = phased.PhaseShiftBeamformer('SensorArray',ula,...
    'OperatingFrequency',carrierFreq,'Direction',inputAngle,...
    'WeightsOutputPort', true);

[yCbf,w] = psbeamformer(rxSignal);
% Plot the output
clf;
plot(t,abs(yCbf)); axis tight;
title('Output of Phase Shift Beamformer');

% Plot array response with weighting
pattern(ula,carrierFreq,-180:180,0,'Weights',w,'Type','powerdb',...
    'PropagationSpeed',physconst('LightSpeed'),'Normalize',false,...
    'CoordinateSystem','rectangular');
axis([-90 90 -60 0]);
xlabel('Time (s)');ylabel('Magnitude (V)');