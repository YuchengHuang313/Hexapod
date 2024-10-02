legAngleOffsets = [120, 180, 240, 300, 0, 60];

for c = 1:length(legAngleOffsets)
    angle = deg2rad(legAngleOffsets(c));
    matrix = [cos(angle) -sin(angle) 0; sin(angle) cos(angle) 0; 0 0 1];
    disp(matrix);

end
    