
function V = computeAirspeed(u, v, w)
    V = sqrt(u.^2 + v.^2 + w.^2);
end
