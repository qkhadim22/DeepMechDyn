function body = AddMarker(body, marker)

if isempty(body.markers)
    body.markers                = marker;
    body.markers(1).id          = 1;
else
    marker.id                   = length(body.markers) + 1;
    body.markers(end+1)         = marker;
end

end