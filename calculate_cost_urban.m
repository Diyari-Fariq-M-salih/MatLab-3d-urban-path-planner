function occ = calculate_cost_urban(xg, yg, zg, scenario)
%build 3-D occupancy grid from building data.
%for instance: occ(x,y,z) = true means occupied.

[nx,ny,nz] = deal(numel(xg), numel(yg), numel(zg));
occ = false(nx,ny,nz);

% Mark building volumes
if isfield(scenario, 'buildings') && ~isempty(scenario.buildings)
    B = scenario.buildings;
    for i = 1:size(B,1)
        x0 = B(i,1); y0 = B(i,2); z0 = B(i,3);
        w  = B(i,4); d  = B(i,5); h  = B(i,6);

        x1 = x0;      x2 = x0 + w;
        y1 = y0;      y2 = y0 + d;
        z1 = z0;      z2 = z0 + h;

        xi = find(xg>=x1 & xg<=x2);
        yi = find(yg>=y1 & yg<=y2);
        zi = find(zg>=z1 & zg<=z2);

        if ~isempty(xi) && ~isempty(yi) && ~isempty(zi)
            occ(xi, yi, zi) = true;
        end
    end
end

end
