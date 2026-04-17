function angDeg = rotm_geodesic_deg(Ra, Rb)
% rotm_geodesic_deg:
% Geodesic rotation distance in degrees.

Rrel = Ra * Rb';
c = (trace(Rrel) - 1) / 2;
c = min(max(c, -1), 1);
angDeg = rad2deg(acos(c));
end

