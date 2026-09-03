function result = evaluateSynchronizedMotion( ...
        result, profile, initialState, options)
%% Section 0: Header & Readme
% SYNTAX
%   result = ruckigEngine.evaluateSynchronizedMotion( ...
%       result, profile, initialState, options)
%**************************************************************************
% PURPOSE
%   - Evaluate the synchronized polynomial into stable sampled motion fields.
%**************************************************************************
% INPUTS
%   - result (scalar empty Ruckig-engine result)
%       Stable request and diagnostic record to complete.
%   - profile (scalar exact switching-profile struct)
%       Successful synchronized polynomial and jerk controls.
%   - initialState, options (scalar structs)
%       Normalized initial state and resolved sampling controls.
%**************************************************************************
% OUTPUTS
%   - result (scalar Ruckig-engine result)
%       Sampled motion fields; the following checkResult stage sets status.
%**************************************************************************
% UNITS
%   - Units are caller-defined and consistent across motion derivatives.
%**************************************************************************

%% Section 1: Evaluate The Synchronized Motion

% Synchronization changes axis timing, so reconstruct the complete polynomial
% at uniform and switching times rather than combining per-axis samples.

result.FinalTime = profile.FinalTime;
result.Duration = profile.FinalTime - initialState.time;
result.ControlJerk = profile.ControlJerk;
result.Polynomial = profile.Polynomial;
result.IntegratedSquaredJerk = profile.IntegratedSquaredJerk;
uniformTime = (initialState.time:options.SampleTime:profile.FinalTime).';
sampleTime = unique([uniformTime; ...
    profile.Polynomial.SegmentStartTime; profile.FinalTime]);
[sampleTime, position, velocity, acceleration, jerk] = ...
    ruckigEngine.internal.evaluatePolynomial( ...
    profile.Polynomial, sampleTime);
result.time = sampleTime;
result.position = position;
result.velocity = velocity;
result.acceleration = acceleration;
result.jerk = jerk;
end
