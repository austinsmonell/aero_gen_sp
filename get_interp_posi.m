function [posi,dir] = get_interp_posi(gamma, flight_path1)

if gamma > pi
gamma = gamma - 2*pi;
elseif gamma < -pi
gamma = gamma+2*pi;
end

posi = [interp1(flight_path1(:, 8),flight_path1(:, 1), gamma), ...
        interp1(flight_path1(:, 8),flight_path1(:, 2), gamma), ...
        interp1(flight_path1(:, 8),flight_path1(:, 3), gamma)];
dir = [interp1(flight_path1(:, 8),flight_path1(:, 4), gamma), ...
       interp1(flight_path1(:, 8),flight_path1(:, 5), gamma), ...
       interp1(flight_path1(:, 8),flight_path1(:, 6), gamma)];

end