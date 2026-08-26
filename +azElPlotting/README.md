# Az/El plotting

`azElPlotting.plotMotion` owns all rendering for successful trajectories and
failed-search diagnostics. It consumes only the stable planner result and does
not rerun planning or reconstruct missing search traces.

The root `plotAzElMotion` function remains the supported public API and
delegates here so existing examples and callers stay compatible. Plotting uses
the same prepared obstacle geometry and goal interpolation as validation.
