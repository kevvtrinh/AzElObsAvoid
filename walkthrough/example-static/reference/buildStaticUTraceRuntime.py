"""Make an ignored diagnostic copy of BMTP; add records without changing inputs.

Production source is left untouched. Rebuild this runtime whenever sources
change. The teaching recorder compares its candidate to the public planner.
"""
from pathlib import Path
import sys

project = Path(sys.argv[1]).resolve()
runtime = project / 'output/static-u/.steps09-16-runtime'
destination = runtime / '+staticULessonEngine'


def add_before_main_end(text, block):
    # The main ends just before the local-function section, or at EOF.
    local = text.find('\nfunction ', 1)
    end = text.rfind('\nend', 0, local if local >= 0 else len(text))
    return text[:end] + '\n' + block + text[end:]


for original in (project / 'trajectory/+bmtpEngine').rglob('*.m'):
    relative = original.relative_to(project / 'trajectory/+bmtpEngine')
    text = original.read_text(encoding='utf-8').replace('bmtpEngine.', 'staticULessonEngine.')
    name = relative.as_posix()
    if name == 'solve.m':
        anchor = 'startingCurve = staticULessonEngine.createStartingCurve(solverRequest);'
        text = text.replace(anchor, anchor + '\nglobal staticULessonTrace\nstaticULessonTrace.Request = solverRequest;\nstaticULessonTrace.StartingCurve = startingCurve;')
        anchor = 'requiredSeparation_units = maximumNormalLength * options.CollisionClearanceTolerance_units + roundoffReserve_units;'
        text = text.replace(anchor, anchor + '\nstaticULessonTrace.SeparationTarget_units = requiredSeparation_units;\nstaticULessonTrace.RoundoffReserve_units = roundoffReserve_units;')
    if name == '+optimization/solveTrajectoryStep.m':
        anchor = 'validateattributes(trajectoryStep,'
        text = text.replace(anchor, "global staticULessonTrace\nstepRecord = struct('Input', trajectoryStep, 'Rounds', {cell(0, 1)});\n" + anchor, 1)
        anchor = '    solveCount = solveCount + 1;'
        text = text.replace(anchor, "    stepRecord.Rounds{end + 1} = struct('Problem', solverProblem, ...\n        'Values', attemptedSolverValues, 'ExitFlag', attemptedExitFlag);\n" + anchor)
        text = add_before_main_end(text, "stepRecord.Controls_units = controlPoint_units;\nstepRecord.Times_s = segmentTime_s;\nstepRecord.ExitFlag = exitFlag;\nstepRecord.Output = solverOutput;\nstepRecord.SharedConstraints = savedTrajectoryConstraints;\nstaticULessonTrace.Steps{end + 1} = stepRecord;")
    if name == '+separation/solveMaximumMarginLine.m':
        anchor = 'degree                ='
        text = text.replace(anchor, "global staticULessonTrace\nlineRecordIndex = numel(staticULessonTrace.Lines) + 1;\nstaticULessonTrace.Lines{lineRecordIndex} = struct( ...\n    'Controls_units', controlPoint_units, 'Region_units', vertices_units);\n" + anchor, 1)
        anchor = 'plane.ExitFlag           = exitFlag;'
        text = text.replace(anchor, anchor + '\nstaticULessonTrace.Lines{lineRecordIndex}.ExitFlag = exitFlag;\nstaticULessonTrace.Lines{lineRecordIndex}.Plane = plane;\nstaticULessonTrace.Lines{lineRecordIndex}.SolverValues = solverValues;\nstaticULessonTrace.Lines{lineRecordIndex}.ObjectiveWeights = objectiveWeights;')
        text = add_before_main_end(text, 'staticULessonTrace.Lines{lineRecordIndex}.Plane = plane;')
    if name == '+optimization/solveActivePairTrajectory.m':
        anchor = 'segmentCount          ='
        text = text.replace(anchor, 'global staticULessonTrace\n' + anchor, 1)
        anchor = '    diagnostics.IterationCount = iterationIndex;'
        text = text.replace(anchor, anchor + "\n    staticULessonTrace.Iterations{iterationIndex} = struct( ...\n        'BestBefore_s', bestDuration_s, 'PlanesBefore', separatingPlanes, 'LineIndices', []);")
        anchor = '        diagnostics.PlaneSocpCount ='
        text = text.replace(anchor, '        staticULessonTrace.Iterations{iterationIndex}.LineIndices(end + 1) = numel(staticULessonTrace.Lines);\n        staticULessonTrace.Lines{end}.SegmentIndex = segmentIndex;\n        staticULessonTrace.Lines{end}.RegionIndex = regionIndex;\n' + anchor, 1)
        anchor = '    trialWasRetained   = false;'
        text = text.replace(anchor, anchor + "\n    staticULessonTrace.Iterations{iterationIndex}.Controls_units = trialControl_units;\n    staticULessonTrace.Iterations{iterationIndex}.Times_s = trialSegmentTime_s;\n    staticULessonTrace.Iterations{iterationIndex}.CollisionPairs = collisionPairs;\n    staticULessonTrace.Iterations{iterationIndex}.NewPairs = newConstraintPairs;\n    staticULessonTrace.Iterations{iterationIndex}.EncounteredPairs = encounteredPairs;\n    staticULessonTrace.Iterations{iterationIndex}.Prepared = trialPreparedMotion;\n    staticULessonTrace.Iterations{iterationIndex}.Proof = trialMotionCheck;")
        anchor = '        if improvementReachedTolerance'
        text = text.replace(anchor, "        staticULessonTrace.Iterations{iterationIndex}.Retained = trialWasRetained;\n        staticULessonTrace.Iterations{iterationIndex}.Improvement_s = arrivalImprovement_s;\n        staticULessonTrace.Iterations{iterationIndex}.Stop = improvementReachedTolerance;\n" + anchor, 1)
        text = add_before_main_end(text, 'staticULessonTrace.Optimization = result;\nstaticULessonTrace.OptimizationDiagnostics = diagnostics;')
    if name == '+optimization/refineTravel.m':
        anchor = 'retainedControl_units ='
        text = text.replace(anchor, 'global staticULessonTrace\nstaticULessonTrace.RetainedBeforeRefinement = retainedResult;\nstaticULessonTrace.RetainedPreparation = retainedPreparedMotion;\nstaticULessonTrace.RetainedProof = retainedProof;\n' + anchor, 1)
        text = add_before_main_end(text, 'staticULessonTrace.RefinementResult = result;\nstaticULessonTrace.RefinementDiagnostics = diagnostics;')
    if name == '+motion/createMotion.m':
        anchor = '% First make the controls match'
        text = text.replace(anchor, "global staticULessonTrace\npreparationRecord = struct('Controls_units', controlPoint_units, 'Times_s', segmentTime_s, ...\n    'Split', splitSegment, 'SplitProgress', splitProgress, 'GivenPower_units', suppliedPowerCoefficients_units);\n\n" + anchor, 1)
        text = add_before_main_end(text, 'preparationRecord.Prepared = preparedMotion;\nstaticULessonTrace.Preparations{end + 1} = preparationRecord;')
    target = destination / relative
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(text, encoding='utf-8')
print(runtime)
