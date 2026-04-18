function qOut = clamp_to_limits(qIn, limits)
% clamp_to_limits:
% Clamp joint vector to numeric limits.

qOut = qIn;
n = min(numel(qOut), size(limits, 1));
epsLim = 1e-6;

for i = 1:n
    lo = limits(i, 1);
    hi = limits(i, 2);
    if isfinite(lo)
        qOut(i) = max(qOut(i), lo + epsLim);
    end
    if isfinite(hi)
        qOut(i) = min(qOut(i), hi - epsLim);
    end
end
end

