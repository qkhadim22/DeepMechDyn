function VisualizeSystem(sys)

figure; hold on; grid on; axis equal;
xlabel('X'); ylabel('Y');

for i = 1:length(sys.bodies)

    b = sys.bodies{i};

    % SAFE CHECK
    if isfield(b,'graphics') && ~isempty(b.graphics) && isfield(b.graphics,'vertices')

        g = b.graphics;

        patch(g.vertices(:,1), g.vertices(:,2), ...
            [0.8 0.8 0.8]);

    else
        % fallback: just COM
        plot(b.position(1), b.position(2), 'ro', 'MarkerSize', 8);
    end

    text(b.position(1), b.position(2), sprintf('B%d', b.id));

end

axis equal;
end