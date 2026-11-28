model = "MTE484Lab3";
open_system(model)

ST0 = slTuner(model, "Outer loop controller");

addPoint(ST0, {"r(t)", "u(t)", "y(t)", "v(t)"});

Vmax = 6;
A    = 0.15;     % largest step in r(t), in the model's units

OverH    = TuningGoal.Overshoot("r(t)", "y(t)", 45);
Margins = TuningGoal.Margins("u(t)", 6, 40);
SpeedH   = TuningGoal.Poles(0.57, 0, Inf);
SpeedS   = TuningGoal.Poles(0.7, 0, Inf);

Volt = TuningGoal.Gain("r(t)", "v(t)", Vmax/A);

[ST2, ~, gHard] = systune(ST0, [SpeedS Margins], [Volt OverH SpeedH]);
gHard
showTunable(ST2)
stepinfo(getIOTransfer(ST2, "r(t)", "y(t)"))