function handles = plotAzElMotion(result, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = plotAzElMotion()
%   handles = plotAzElMotion(result)
%   handles = plotAzElMotion(result, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Preserve the public plotting API while delegating all rendering to the
%     dedicated azElPlotting package.
%**************************************************************************
% INPUTS
%   - result (scalar planner result)
%       Stable success-or-failure record returned by planAzElMotion.
%   - optionOverrides (scalar struct, optional; default struct())
%       Partial display options accepted by azElPlotting.plotMotion.
%**************************************************************************
% OUTPUTS
%   - handles (scalar struct)
%       Figure and axes handles for motion or failure diagnostics.
%   - options (scalar struct, zero-input call)
%       Fully resolved plotting defaults.
%**************************************************************************
% UNITS
%   - Spatial axes use degrees; time uses seconds; derivatives use deg/s,
%     deg/s^2, and deg/s^3.
%**************************************************************************

%% Section 1: Delegate To The Plotting Package

if nargin == 0
    handles = azElPlotting.plotMotion();
elseif nargin == 1
    handles = azElPlotting.plotMotion(result);
else
    handles = azElPlotting.plotMotion(result, optionOverrides);
end
end
