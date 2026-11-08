% Define the MVDR beamformer
mvdrbeamformer = phased.MVDRBeamformer('SensorArray',ula,...
    'Direction',inputAngle,'OperatingFrequency',carrierFreq,...
    'WeightsOutputPort',true);
mvdrbeamformer.TrainingInputPort = true;

[yMVDR, wMVDR] = mvdrbeamformer(rxSignal,rxInt);