function sys = AddBody(sys, body)

body.id             = length(sys.bodies) + 1;
sys.bodies{end+1}   = body;

end