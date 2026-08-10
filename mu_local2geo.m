function geo = mu_local2geo(local, geoRef)
% mu_local2geo : local cartesian (m) -> lat/lon/alt (deg, m)
%   geo = mu_local2geo(local, geoRef)
%   local : Nx3 [x y z] meters, Z up, origin = city center
%   geoRef: struct with originLat/Lon/Alt, heading (deg), k (scale)
%   geo   : Nx3 [lat lon alt]
% Helmholtz 2D planar approx: rotate local frame by heading into N-E, then convert.
if ~isfield(geoRef,'heading'), geoRef.heading = 0; end
if ~isfield(geoRef,'k'), geoRef.k = 1; end
lat0 = geoRef.originLat; lon0 = geoRef.originLon; alt0 = geoRef.originAlt;
hdg  = deg2rad(geoRef.heading); k = geoRef.k;
x = local(:,1); y = local(:,2); z = local(:,3);
xe =  cos(hdg)*x + sin(hdg)*y;
yn = -sin(hdg)*x + cos(hdg)*y;
mPerDegLat = 111320;
mPerDegLon = 111320*cos(deg2rad(lat0));
dLat = (yn*k) / mPerDegLat;
dLon = (xe*k) / mPerDegLon;
geo = [lat0 + dLat, lon0 + dLon, alt0 + z];
end
