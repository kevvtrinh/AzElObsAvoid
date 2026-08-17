# Azimuth-Elevation RP Retimer Model

`azElRpRetimerAgent.mat` was trained with MATLAB R2024b and Reinforcement
Learning Toolbox. The saved metadata records 5,000 episodes and random seed
41. Training finished all 5,000 one-step episodes. The final episode reward
was -1.011212182105. The final average reward was -1.165428483246. The mean
episode reward was -1.322924666097.

The training set contains 384 deterministic labels from the production
retimer. The independent validation set contains 128 labels and uses seed
104770. The promoted policy has a mean absolute radius-fraction error of
0.253133286285. The best fixed-radius baseline error is 0.333593750000. The
previous saved policy error is 0.333783158427 on the same validation set.
The new policy reduces the previous error by 24.2 percent.

The model is a DDPG contextual radius proposer. Training labels are the
radius fractions selected by the obstacle-free production retimer. Its
10-by-1 dimensionless
observation has this order:

1. Turn deflection divided by pi.
2. Minimum adjacent-segment length divided by maximum adjacent-segment
   length.
3. Bounded logarithm of the minimum adjacent-segment length.
4. Candidate turn radius divided by the minimum adjacent-segment length.
5. Incoming azimuth direction component.
6. Incoming elevation direction component.
7. Outgoing azimuth direction component.
8. Outgoing elevation direction component.
9. Acceleration time scale divided by the reference time scale.
10. Jerk time scale divided by the reference time scale.

The scalar action is in `[-1, 1]`. Inference maps it to a radius fraction in
`[0, 1]` with `(action + 1) / 2`.

The policy has proposal-only authority. It does not receive obstacle data. It
cannot select a route, accept a curve, change a safety margin, or certify a
trajectory. The production retimer evaluates the proposal with its
deterministic candidate search. Kinematic and continuous collision
certificates select or reject the final result.

From the repository root, regenerate the checked-in model with this command:

```powershell
matlab -batch "training=trainAzElRpRetimer(struct('MaximumEpisodes',5000,'RandomSeed',41,'Verbose',false,'ShowTrainingPlot',false));"
```

The training result summary for this file is:

- Model format: `AzElRpRetimerAgent`, version 2.
- Episodes: 5,000.
- Environment steps: 5,000.
- Production training cases: 384.
- Held-out production validation cases: 128.
- Random seed: 41.
- MATLAB release: R2024b.
- Action range: `[-1, 1]`.
- Radius-fraction range: `[0, 1]`.
- Obstacle inputs used for training: false.

The deterministic production search remains strong. On 20 new random
multi-corner routes and four mixed sharp-and-shallow routes, the promoted
policy and the policy-disabled retimer returned equal minimum duration. The
policy added one evaluated candidate in each random case. These checks show
better target prediction, but they do not show a complete-route duration
gain.
