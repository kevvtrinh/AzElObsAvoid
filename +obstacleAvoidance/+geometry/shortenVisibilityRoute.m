function [cleanedRoute_deg, record] = shortenVisibilityRoute( ...
        route_deg, visibilityFunction, signatureFunction, requiredSignature)
%% Section 0: Header & Readme
% SYNTAX
%   [cleanedRoute_deg, record] = ...
%       obstacleAvoidance.geometry.shortenVisibilityRoute( ...
%       route_deg, visibilityFunction, signatureFunction, requiredSignature)
%**************************************************************************
% PURPOSE
%   - Shorten a polyline using only visible chords that preserve its supplied
%     topological signature, while recording every evaluated decision.
%**************************************************************************
% INPUTS
%   - route_deg (N-by-2 numeric matrix)
%       Ordered route positions in [azimuth elevation] coordinates.
%   - visibilityFunction (scalar function handle)
%       Returns true only when its two 1-by-2 endpoints form a clear chord.
%   - signatureFunction (scalar function handle)
%       Returns the topological signature of a candidate route.
%   - requiredSignature (numeric row vector)
%       Signature that every accepted shortcut must preserve exactly.
%**************************************************************************
% OUTPUTS
%   - cleanedRoute_deg (M-by-2 numeric matrix)
%       Greedily shortened route; unchanged when no shortcut is accepted.
%   - record (scalar struct)
%       Evaluated, rejected, accepted, and achieved-reduction counts.
%**************************************************************************
% UNITS
%   - Route coordinates and length reduction are degrees.
%**************************************************************************

%% Section 1: Select Decision-Faithful Shortcuts

cleanedRoute_deg = route_deg;
initialLength_deg = routeLength(route_deg);
record = struct("CandidateCount", 0, "VisibilityRejectedCount", 0, ...
    "HomologyRejectedCount", 0, "AcceptedCount", 0, "LengthReduction_deg", 0);
if size(route_deg, 1) < 3 || ...
        ~isequal(signatureFunction(route_deg), requiredSignature)
    return;
end
while size(cleanedRoute_deg, 1) >= 3
    currentLength_deg = routeLength(cleanedRoute_deg);
    lengthTolerance_deg = max(1e-12, 1e-12 * currentLength_deg);
    bestReduction_deg = 0;
    bestRoute_deg = cleanedRoute_deg;
    for firstIndex = 1:size(cleanedRoute_deg, 1) - 2
        for secondIndex = firstIndex + 2:size(cleanedRoute_deg, 1)
            record.CandidateCount = record.CandidateCount + 1;
            reduction_deg = routeLength(cleanedRoute_deg(firstIndex:secondIndex, :)) - ...
                norm(cleanedRoute_deg(secondIndex, :) - cleanedRoute_deg(firstIndex, :));
            if reduction_deg <= lengthTolerance_deg
                continue;
            end
            if ~visibilityFunction(cleanedRoute_deg(firstIndex, :), ...
                    cleanedRoute_deg(secondIndex, :))
                record.VisibilityRejectedCount = record.VisibilityRejectedCount + 1;
                continue;
            end
            candidate_deg = [cleanedRoute_deg(1:firstIndex, :); ...
                cleanedRoute_deg(secondIndex:end, :)];
            if ~isequal(signatureFunction(candidate_deg), requiredSignature)
                record.HomologyRejectedCount = record.HomologyRejectedCount + 1;
                continue;
            end
            if reduction_deg <= bestReduction_deg + lengthTolerance_deg
                continue;
            end
            bestReduction_deg = reduction_deg;
            bestRoute_deg = candidate_deg;
        end
    end
    if bestReduction_deg <= lengthTolerance_deg
        break;
    end
    cleanedRoute_deg = bestRoute_deg;
    record.AcceptedCount = record.AcceptedCount + 1;
end
record.LengthReduction_deg = initialLength_deg - routeLength(cleanedRoute_deg);
end

%% Section 2: Local Functions

function length_deg = routeLength(route_deg)
% Measure Euclidean polyline length for strict shortcut comparisons.
length_deg = sum(vecnorm(diff(route_deg, 1, 1), 2, 2));
end
