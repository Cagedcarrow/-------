function qOut = nearest_equivalent_to_ref(qIn, qRef, limits)
% nearest_equivalent_to_ref:
% Resolve 2*pi periodic ambiguity and choose nearest equivalent to qRef.

qOut = qIn;
n = min([numel(qIn), numel(qRef), size(limits, 1)]);

for i = 1:n
    cands = qIn(i) + 2*pi*(-2:2);
    lo = limits(i, 1);
    hi = limits(i, 2);

    if isfinite(lo)
        cands = cands(cands >= lo);
    end
    if isfinite(hi)
        cands = cands(cands <= hi);
    end

    if isempty(cands)
        qOut(i) = qIn(i);
    else
        [~, idx] = min(abs(cands - qRef(i)));
        qOut(i) = cands(idx);
    end
end
end

