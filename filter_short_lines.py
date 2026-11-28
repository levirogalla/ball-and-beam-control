#!/usr/bin/env python3

import os
import sys
import tempfile


def filter_file_in_place(path: str, minimum_length: int = 10) -> None:
    file_mode = os.stat(path).st_mode
    directory = os.path.dirname(os.path.abspath(path)) or "."

    with open(path, "r", encoding="utf-8") as source, tempfile.NamedTemporaryFile(
        "w", encoding="utf-8", dir=directory, delete=False
    ) as destination:
        temporary_path = destination.name
        for line in source:
            if len(line.rstrip("\r\n")) >= minimum_length:
                destination.write(line)

    try:
        os.chmod(temporary_path, file_mode)
        os.replace(temporary_path, path)
    except Exception:
        os.unlink(temporary_path)
        raise


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit(f"Usage: {sys.argv[0]} FILE")

    filter_file_in_place(sys.argv[1])


    % inner controller vars
syms C1 C2 C3

% motor plant vars
syms P1 P2

% physics var
syms K2 K3

% frequency domain
syms s

innerController = (C1*s + C2) / (s + C3);
motorPlant = P1 / (P2*s^2 + s);

innerPlant = ...
    ( innerController*motorPlant / (1 + innerController*motorPlant) ) * ...
    ( K2 ) * ...
    ( K3 / s^2 );

% outer controller vars
syms Ca Cb Cc Cd Ce

outerController = (Ca*s^2 + Cb*s + Cc) / ...
                  (s^2 + Cd*s + Ce);

[Np, Dp] = numden(simplifyFraction(innerPlant));

c = coeffs(Dp, s, 'All');     % c(1) is the leading coefficient
Np = vpa(expand(Np/c(1)), 5);
Dp = vpa(expand(Dp/c(1)), 5);

[Nc, Dc] = numden(outerController);
num = collect(simplify(Nc*Np), s);
den = collect(simplify(Dc*Dp + Nc*Np), s);

syms Wn zeta nondp1 nondp2 a0 a1
p = 10*zeta*Wn;

Wd = Wn*sqrt(1-zeta^2);
charDesired = (s^2 + 2*zeta*Wn*s + Wn^2) * (s + p)^3 * (s^2 + a1*s + a0);
charDesired = collect(expand(charDesired), s);

den
charDesired
c1 = coeffs(den, s, "All");
c2 = coeffs(charDesired, s, "All");

eqns = c1 == c2;
sol = solve(eqns, [Ca, Cb, Cc, Cd, Ce, a1, a0]);


plantVals = [-2.452 -135.4 24.95 -1.8906 0.018107 0.0609 -4.25];

% charDesired with the solved a1, a0 plugged in
charSol = subs(charDesired, [a1 a0], [sol.a1 sol.a0]);
charSol = subs(charSol, [C1 C2 C3 P1 P2 K2 K3], plantVals);
cf = matlabFunction(coeffs(charSol, s, 'All'), 'Vars', {zeta, Wn});

inRegion = @(r) all(real(r) < 0);        % replace with your region
isGood   = @(z, wn) inRegion(roots(cf(z, wn)));

w = logspace(-1, 3, 400);

% ---- vary zeta, Wn fixed ----
zv  = linspace(0.4, 1.0, 60);
Wn0 = 2;
[W, Z] = meshgrid(w, zv);
M = 20*log10(abs(H(1j*W, Z, Wn0)));

good = arrayfun(@(z) isGood(z, Wn0), zv);
plotMasked(W, Z, M, good)
xlabel('\omega (rad/s)'), ylabel('\zeta'), zlabel('Magnitude (dB)')
title(sprintf('Varying \\zeta  (\\omega_n = %g)', Wn0))

% ---- vary Wn, zeta fixed ----
wnv   = linspace(0.5, 20, 60);
zeta0 = 0.7;
[W, N] = meshgrid(w, wnv);
M = 20*log10(abs(H(1j*W, zeta0, N)));

good = arrayfun(@(wn) isGood(zeta0, wn), wnv);
plotMasked(W, N, M, good)
xlabel('\omega (rad/s)'), ylabel('\omega_n'), zlabel('Magnitude (dB)')
title(sprintf('Varying \\omega_n  (\\zeta = %g)', zeta0))

w   = logspace(-1, 3, 80);
zv  = linspace(0.4, 1.0, 30);
wnv = linspace(0.5, 20, 30);

[Wg, Zg, Ng] = meshgrid(w, zv, wnv);        % dims: zeta x omega x Wn
M = 20*log10(abs(H(1j*Wg, Zg, Ng)));

% stability for each (zeta, Wn), stretched along the frequency axis
[Zs, Ns] = ndgrid(zv, wnv);
good   = arrayfun(isGood, Zs, Ns);
stable = repmat(permute(good, [1 3 2]), 1, numel(w), 1);

keep = stable & M < 0;

figure
scatter3(log10(Wg(keep)), Zg(keep), Ng(keep), 8, M(keep), 'filled')
xlabel('log_{10} \omega (rad/s)'), ylabel('\zeta'), zlabel('\omega_n')
title('Stable and magnitude < 0 dB')
colorbar, grid on, view(-35, 25)

% ---- local function (keep at the bottom of the script) ----
function plotMasked(X, Y, M, good)
mask  = repmat(good(:), 1, size(M, 2));   % one row per zeta/Wn value
Mgood = M;  Mgood(~mask) = NaN;
Mbad  = M;  Mbad(mask)   = NaN;

figure
surf(X, Y, Mgood, 'EdgeColor', 'none'), hold on
surf(X, Y, Mbad, 'FaceColor', 'k', 'EdgeColor', 'none')
set(gca, 'XScale', 'log')
caxis([min(Mgood(:)) max(Mgood(:))])      % keep colours set by the stable part
colorbar, view(-35, 30)
end
















G = outerController * innerPlant / (1 + outerController*innerPlant);

G = subs(G, [Ca Cb Cc Cd Ce a0 a1], [sol.Ca sol.Cb sol.Cc sol.Cd sol.Ce sol.a0 sol.a1]);
G = subs(G, [C1 C2 C3 P1 P2 K2 K3], [-2.452 -135.4 24.95 -1.8906 0.018107 0.0609 -4.25]);

simplify(G)
symvar(G)
syms s zeta Wn
H = matlabFunction(G, 'Vars', {s, zeta, Wn});
w = logspace(-1, 3, 400);                 % rad/s

% ---- vary zeta, Wn fixed ----
zv  = linspace(0.4, 1.0, 60);
Wn0 = 2;
[W, Z] = meshgrid(w, zv);
M = 20*log10(abs(H(1j*W, Z, Wn0)));

figure
surf(W, Z, M, 'EdgeColor', 'none')
set(gca, 'XScale', 'log')
xlabel('\omega (rad/s)'), ylabel('\zeta'), zlabel('Magnitude (dB)')
title(sprintf('Varying \\zeta  (\\omega_n = %g)', Wn0))
colorbar, view(-35, 30)

% ---- vary Wn, zeta fixed ----
wnv   = linspace(0.5, 4, 60);
zeta0 = 0.7;
[W, N] = meshgrid(w, wnv);
M = 20*log10(abs(H(1j*W, zeta0, N)));

figure
surf(W, N, M, 'EdgeColor', 'none')
set(gca, 'XScale', 'log')
xlabel('\omega (rad/s)'), ylabel('\omega_n'), zlabel('Magnitude (dB)')
title(sprintf('Varying \\omega_n  (\\zeta = %g)', zeta0))
colorbar, view(-35, 30)

G = subs(G, [Wn zeta], [13.9754 0.60339]);
G = simplifyFraction(G);
[num, den] = numden(G);
c = coeffs(collect(den, s));

Ng = vpa(expand(num/c(1)), 5)
Dg = vpa(expand(den/c(1)), 5)

coeffs(Ng, s, "All")
coeffs(Dg, s, "All")