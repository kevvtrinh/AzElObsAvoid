# Az/El plotting

`azElPlotting.plotMotion` is the public plotting API and owns all rendering for
successful trajectories and failed-search diagnostics. It consumes only the
stable planner result and does not rerun planning or reconstruct missing search
traces. Plotting uses the same prepared obstacle geometry and goal
interpolation as validation.
