function local = mu_geo2local(geo, geoRef)
% mu_geo2local : lat/lon/alt -> local cartesian (m). Inverse of mu_local2geo.
%   local = mu_geo2local(geo, geoRef)
%   geo   : Nx3 [lat lon alt]
%   geoRef: struct with originLat/Lon/Alt, heading (deg), k (scale)
%   local : Nx3 [x y z] meters, Z up
if ~isfield(geoRef,'heading'), geoRef.heading = 0; end
if ~isfield(geoRef,'k'), geoRef.k = 1; end
lat0 = geoRef.originLat; lon0 = geoRef.originLon; alt0 = geoRef.originAlt;
hdg  = deg2rad(geoRef.heading); k = geoRef.k;
lat = geo(:,1); lon = geo(:,2); zalt = geo(:,3);
yn = (lat - lat0) * 111320 / k;
xe = (lon - lon0) * 111320*cos(deg2rad(lat0)) / k;
x =  cos(hdg)*xe - sin(hdg)*yn;
y =  sin(hdg)*xe + cos(hdg)*yn;
local = [x, y, zalt - alt0];
end
