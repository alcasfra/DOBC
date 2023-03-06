function [ddw] = ddw(t)
    
    global a0 b0 a1 b1 a2 b2 w0 w1 w2

    ddw = -a0*w0^2*sin(w0*t) - a1*w1^2*sin(w1*t) - a2*w2^2*sin(w2*t) - ...
        b0*w0^2*cos(w0*t) - b1*w1^2*cos(w1*t) - b2*w2^2*cos(w2*t);
    
end